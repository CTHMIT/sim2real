import sys
from pathlib import Path
import asyncio
import threading
import time
import concurrent.futures
from queue import Queue, Empty, Full
import cv2
import numpy as np
from typing import Dict, Optional, List, Any, Callable
import os
from pydantic import BaseModel, Field

import cosysairsim as airsim  # type: ignore
from utils.logger import LOGGER  # type: ignore

PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.append(str(PROJECT_ROOT))


# =============================================================================
# CONFIGURATION MODELS
# =============================================================================


class ConnectionConfig(BaseModel):
    """Connection configuration parameters"""

    ip_address: str = Field(
        default="172.19.160.1", description="AirSim server IP address"
    )
    vehicle_name: str = Field(default="PX4", description="Vehicle name")


class CameraConfig(BaseModel):
    """Camera configuration parameters"""

    camera_name: str = Field(default="front_center", description="Camera name")
    image_type: int = Field(default=airsim.ImageType.Scene, description="Image type")
    pixels_as_float: bool = Field(default=False, description="Return pixels as float")
    compress: bool = Field(default=False, description="Compress image data")
    width: int = Field(default=960, description="Requested width (0 for default)")
    height: int = Field(default=540, description="Requested height (0 for default)")


class MultiCameraConfig(BaseModel):
    """Configuration for multiple cameras"""

    enabled: bool = Field(default=True, description="Enable multiple camera views")
    cameras: Dict[str, CameraConfig] = Field(
        default_factory=lambda: {
            "front_center": CameraConfig(camera_name="front_center"),
            "side_left": CameraConfig(camera_name="side_left"),
            "side_right": CameraConfig(camera_name="side_right"),
        },
        description="Camera configurations by name",
    )
    # Default active camera (the one controlled by gimbal)
    active_camera: str = Field(
        default="front_center", description="Currently active camera"
    )

    def get_camera_config(self, camera_name: str) -> Optional[CameraConfig]:
        """Get configuration for a specific camera"""
        return self.cameras.get(camera_name)


class DisplayConfig(BaseModel):
    """Display configuration parameters"""

    window_name: str = Field(
        default="AirSim Camera View", description="Display window name"
    )
    framerate_hz: float = Field(
        default=60.0,
        ge=10.0,
        le=240.0,
        description="Display framerate target in Hz (10-240)",
    )
    convert_to_rgb: bool = Field(default=False, description="Convert BGR to RGB")
    optimize_for_realtime: bool = Field(
        default=True, description="Optimize for realtime display"
    )
    use_threading: bool = Field(default=True, description="Use threaded image fetching")
    fetch_queue_size: int = Field(default=16, description="Raw image fetch queue size")
    thread_count: int = Field(
        default=3, ge=1, le=4, description="Number of background threads (1-6)"
    )
    queue_size: int = Field(default=1, description="Image queue size")
    waitKey_delay: int = Field(default=1, description="CV2 waitKey delay in ms (1-10)")
    resize_output: bool = Field(
        default=True, description="Resize output for performance"
    )
    output_width: int = Field(default=960, description="Output width if resizing")
    output_height: int = Field(default=540, description="Output height if resizing")

    @property
    def update_rate(self) -> float:
        """Convert framerate in Hz to update interval in seconds"""
        return 1.0 / self.framerate_hz


class ProcessingConfig(BaseModel):
    """Image processing configuration parameters"""

    enable_parallel_processing: bool = Field(
        default=True, description="Enable parallel image processing"
    )
    skip_frames_if_busy: bool = Field(
        default=True, description="Skip frames if processing is busy"
    )
    downscale_factor: float = Field(
        default=1.0,
        ge=0.1,
        le=1.0,
        description="Downscale factor for processing (0.1-1.0)",
    )


class AirSimCameraConfig(BaseModel):
    """Overall configuration for AirSim camera client"""

    # AirSim Parameters
    connection: ConnectionConfig = Field(default_factory=ConnectionConfig)
    camera: CameraConfig = Field(default_factory=CameraConfig)
    multi_camera: MultiCameraConfig = Field(default_factory=MultiCameraConfig)
    display: DisplayConfig = Field(default_factory=DisplayConfig)
    processing: ProcessingConfig = Field(default_factory=ProcessingConfig)
    timeout: int = Field(default=10, description="Connection timeout (seconds)")

    # Gimbal Control Parameters
    initial_pitch: float = Field(
        -90.0, description="Initial gimbal pitch angle in degrees"
    )
    min_pitch_deg: float = Field(-90.0, description="Minimum gimbal pitch angle")
    max_pitch_deg: float = Field(30.0, description="Maximum gimbal pitch angle")
    min_roll_deg: float = Field(-45.0, description="Minimum gimbal roll angle")
    max_roll_deg: float = Field(45.0, description="Maximum gimbal roll angle")
    min_yaw_deg: float = Field(-180.0, description="Minimum gimbal yaw angle")
    max_yaw_deg: float = Field(180.0, description="Maximum gimbal yaw angle")


# =============================================================================
# GIMBAL STATE
# =============================================================================


class GimbalState(BaseModel):
    """Current state of the gimbal"""

    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0

    # Optional rates for smoother movement
    pitch_rate: float = 0.0
    roll_rate: float = 0.0
    yaw_rate: float = 0.0

    class Config:
        arbitrary_types_allowed = True


# =============================================================================
# EVENT BUS FOR DECOUPLED COMMUNICATION
# =============================================================================


class EventBus:
    """
    Event bus for decoupled communication between components
    Replaces global queues with a publish-subscribe pattern
    """

    def __init__(self):
        self._subscribers = {}
        self._queues = {}

    def subscribe(self, event_type: str, callback: Callable):
        """Subscribe to an event type with a callback function"""
        if event_type not in self._subscribers:
            self._subscribers[event_type] = []
        self._subscribers[event_type].append(callback)

    def publish(self, event_type: str, data=None):
        """Publish an event to all subscribers"""
        if event_type in self._subscribers:
            for callback in self._subscribers[event_type]:
                callback(data)

    def create_queue(self, queue_name: str, maxsize: int = 0) -> Queue:
        """Create a named queue that can be accessed by components"""
        queue: Queue = Queue(maxsize=maxsize)
        self._queues[queue_name] = queue
        return queue

    def get_queue(self, queue_name: str) -> Optional[Queue]:
        """Get a queue by name"""
        return self._queues.get(queue_name)


# =============================================================================
# IMAGE PROCESSING STRATEGIES
# =============================================================================


class ImageProcessingStrategy:
    """Base class for image processing strategies"""

    def process_image(self, img_data, height, width):
        """Process raw image data into displayable format"""
        pass


class DefaultImageProcessingStrategy(ImageProcessingStrategy):
    """Default image processing implementation"""

    def __init__(
        self,
        convert_to_rgb=True,
        resize_output=False,
        output_width=640,
        output_height=480,
    ):
        self.convert_to_rgb = convert_to_rgb
        self.resize_output = resize_output
        self.output_width = output_width
        self.output_height = output_height

    def process_image(self, img_data, height, width):
        """Process raw image data into displayable format"""
        if not img_data:
            return np.zeros(
                (
                    self.output_height if self.resize_output else height,
                    self.output_width if self.resize_output else width,
                    3,
                ),
                dtype=np.uint8,
            )
        try:
            img_bgr = np.frombuffer(img_data, dtype=np.uint8).reshape(height, width, 3)
            img_display = img_bgr

            if self.resize_output:
                img_display = cv2.resize(
                    img_display,
                    (self.output_width, self.output_height),
                    interpolation=cv2.INTER_LINEAR,
                )

            if self.convert_to_rgb:
                img_display = cv2.cvtColor(img_display, cv2.COLOR_BGR2RGB)

            return img_display
        except Exception as e:
            LOGGER.error(f"Error processing image: {e}")
            return np.zeros(
                (
                    self.output_height if self.resize_output else height,
                    self.output_width if self.resize_output else width,
                    3,
                ),
                dtype=np.uint8,
            )


class HighPerformanceProcessingStrategy(ImageProcessingStrategy):
    """High performance image processing with minimal operations"""

    def __init__(
        self,
        convert_to_rgb=True,
        resize_output=False,
        output_width=1920,
        output_height=1080,
    ):
        self.convert_to_rgb = convert_to_rgb
        self.resize_output = resize_output
        self.output_width = output_width
        self.output_height = output_height

    def process_image(self, img_data, height, width):
        """Process image with minimal operations for maximum speed"""
        try:
            img = np.frombuffer(img_data, dtype=np.uint8).reshape(height, width, 3)

            if self.resize_output:
                if not hasattr(self, "resized_buffer") or self.resized_buffer.shape != (
                    self.output_height,
                    self.output_width,
                    3,
                ):
                    self.resized_buffer = np.zeros(
                        (self.output_height, self.output_width, 3), dtype=np.uint8
                    )

                cv2.resize(
                    img,
                    (self.output_width, self.output_height),
                    dst=self.resized_buffer,
                    interpolation=cv2.INTER_NEAREST,
                )
                img = self.resized_buffer

            if self.convert_to_rgb:
                if (
                    not hasattr(self, "rgb_buffer")
                    or self.rgb_buffer.shape != img.shape
                ):
                    self.rgb_buffer = np.zeros_like(img)

                cv2.cvtColor(img, cv2.COLOR_BGR2RGB, dst=self.rgb_buffer)
                return self.rgb_buffer

            return img
        except Exception as e:
            LOGGER.error(f"Image processing error: {e}")
            return np.zeros(
                (
                    self.output_height if self.resize_output else height,
                    self.output_width if self.resize_output else width,
                    3,
                ),
                dtype=np.uint8,
            )


# =============================================================================
# AIRSIM CAMERA CLIENT
# =============================================================================


class AirSimCameraClient:
    """Client for AirSim camera image capture and processing"""

    def __init__(
        self,
        config: AirSimCameraConfig,
        gimbal_state: Optional[GimbalState] = None,
        event_bus: Optional[EventBus] = None,
        image_strategy: Optional[ImageProcessingStrategy] = None,
    ):
        self.config = config
        self.gimbal_state = gimbal_state or GimbalState(pitch=config.initial_pitch)
        self.event_bus = event_bus or EventBus()
        self.client = None

        # Create queues through event_bus instead of using global variables
        self.camera_pose_queue = self.event_bus.create_queue(
            "camera_pose_queue", maxsize=10
        )
        self.display_image_queue = self.event_bus.create_queue(
            "display_image_queue", maxsize=2
        )

        # Set up image processing strategy
        if image_strategy:
            self.image_strategy = image_strategy
        elif config.display.optimize_for_realtime:
            self.image_strategy = HighPerformanceProcessingStrategy(
                convert_to_rgb=config.display.convert_to_rgb,
                resize_output=config.display.resize_output,
                output_width=config.display.output_width,
                output_height=config.display.output_height,
            )
        else:
            self.image_strategy = DefaultImageProcessingStrategy(
                convert_to_rgb=config.display.convert_to_rgb,
                resize_output=config.display.resize_output,
                output_width=config.display.output_width,
                output_height=config.display.output_height,
            )

        self.running = False
        self.exit_flag = False
        self.fetch_times: List[float] = []
        self.raw_image_queue: Queue = Queue(maxsize=config.display.fetch_queue_size)
        self.fetch_thread: Optional[threading.Thread] = None
        self.thread_clients: List[Any] = []
        self.fetch_stats: Dict = {"fetch_times": [], "process_times": []}

        # Create configurations for all available cameras
        self.camera_configs = {}
        for camera_name, cam_config in config.multi_camera.cameras.items():
            self.camera_configs[camera_name] = {
                "camera_name": cam_config.camera_name,
                "image_type": cam_config.image_type,
                "pixels_as_float": cam_config.pixels_as_float,
                "compress": cam_config.compress,
                "width": cam_config.width,
                "height": cam_config.height,
            }

        self.process_pool = None
        if config.processing.enable_parallel_processing:
            self.process_pool = concurrent.futures.ThreadPoolExecutor(
                max_workers=os.cpu_count() or 2
            )

        self.opencv_display_thread: Optional[threading.Thread] = None
        self.camera_pose_thread: Optional[threading.Thread] = None

    def connect(self) -> bool:
        """Connect to AirSim server using a thread to avoid asyncio conflicts"""
        LOGGER.info(
            f"Attempting to connect to AirSim at: {self.config.connection.ip_address}"
        )
        result = {"success": False, "client": None, "error": None}
        connection_complete = threading.Event()

        def connect_thread():
            nonlocal result
            try:
                self.client = airsim.MultirotorClient(
                    ip=self.config.connection.ip_address
                )
                self.client.confirmConnection()
                result["success"] = True
                result["client"] = self.client
                # Get additional info if needed
                try:
                    vehicles = self.client.listVehicles()
                    LOGGER.info(f"AirSim available vehicles: {vehicles}")
                    if self.config.connection.vehicle_name not in vehicles:
                        LOGGER.warning(
                            f"Vehicle '{self.config.connection.vehicle_name}' not found in AirSim list."
                        )
                except Exception as e_info:
                    LOGGER.warning(f"Could not list AirSim vehicles: {e_info}")

            except Exception as e:
                result["error"] = str(e)
                LOGGER.error(f"Failed to connect to AirSim in thread: {e}")
            finally:
                connection_complete.set()

        conn_thread = threading.Thread(target=connect_thread, daemon=True)
        conn_thread.start()

        if not connection_complete.wait(timeout=self.config.timeout):
            LOGGER.error(
                f"AirSim connection timeout after {self.config.timeout} seconds"
            )
            return False

        if not result["success"]:
            LOGGER.error(
                f"Failed to connect to AirSim: {result.get('error', 'Unknown error')}"
            )
            return False

        LOGGER.info(
            f"Connected to AirSim vehicle: {self.config.connection.vehicle_name}"
        )
        return True

    def get_image(
        self, client_instance: airsim.MultirotorClient, camera_config=None
    ) -> Optional[Dict]:
        """獲取單個原始圖像幀。由獲取線程運行。修復事件循環問題。"""
        client_to_use = client_instance or self.client

        # 如果沒有提供攝像頭配置，使用默認配置
        if camera_config is None:
            camera_config = {
                "camera_name": self.config.camera.camera_name,
                "image_type": self.config.camera.image_type,
                "pixels_as_float": self.config.camera.pixels_as_float,
                "compress": self.config.camera.compress,
                "width": self.config.camera.width,
                "height": self.config.camera.height,
            }

        try:
            start_time = time.time()

            # 大多數 AirSim API 不需要事件循環，但模擬運行在異步環境中
            # 如果 camera_config 中的攝像頭名稱是 front_center 且需要調整云台姿態
            if (
                camera_config["camera_name"] == "front_center"
                and hasattr(self, "state")
                and (
                    self.state.gimbal_pitch != 0
                    or self.state.gimbal_roll != 0
                    or self.state.gimbal_yaw != 0
                )
            ):
                # 在非主線程中設置相機姿態時，需要處理事件循環
                try:
                    # 使用標準 AirSim API 而不依賴於事件循環
                    quat = airsim.euler_to_quaternion(
                        self.state.gimbal_roll * np.pi / 180,
                        self.state.gimbal_pitch * np.pi / 180,
                        self.state.gimbal_yaw * np.pi / 180,
                    )

                    client_to_use.simSetCameraPose(
                        camera_config["camera_name"],
                        airsim.Pose(airsim.Vector3r(0, 0, 0.2), quat),
                        vehicle_name=self.config.connection.vehicle_name,
                    )
                except Exception as e:
                    LOGGER.warning(f"Could not set camera pose: {e}")

            # 設置圖像請求
            request = airsim.ImageRequest(
                camera_name=camera_config["camera_name"],
                image_type=camera_config["image_type"],
                pixels_as_float=camera_config["pixels_as_float"],
                compress=camera_config["compress"],
            )

            if camera_config["width"] > 0 and camera_config["height"] > 0:
                request.width = camera_config["width"]
                request.height = camera_config["height"]

            # 獲取圖像
            responses = client_to_use.simGetImages(
                [request], vehicle_name=self.config.connection.vehicle_name
            )

            # 檢查響應是否有效
            if not responses or not responses[0].image_data_uint8:
                return None

            response = responses[0]

            # 記錄獲取時間
            fetch_time = time.time() - start_time
            if len(self.fetch_times) > 100:
                self.fetch_times.pop(0)
            self.fetch_times.append(fetch_time)

            # 返回圖像信息
            return {
                "data": response.image_data_uint8,
                "width": response.width,
                "height": response.height,
                "timestamp": time.time(),
                "fetch_time": fetch_time,
                "camera_name": camera_config["camera_name"],
            }
        except Exception as e:
            LOGGER.error(
                f"Failed to get image from {camera_config['camera_name']}: {e}"
            )
            return None

    def _setup_thread_event_loop(self):
        """Set up a new event loop for the thread if needed (primarily for airsim calls)"""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            return True
        except Exception as e:
            LOGGER.error(f"Failed to set up event loop in thread: {e}")
            return False

    def _create_thread_client(self):
        """Create a thread-specific client instance that ensures each thread has a proper event loop"""
        try:
            # Set up a new event loop in the thread
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            # Create the client instance
            thread_client = airsim.MultirotorClient(
                ip=self.config.connection.ip_address
            )
            thread_client.confirmConnection()
            self.thread_clients.append(thread_client)
            LOGGER.info(
                f"AirSim client created successfully for thread {threading.current_thread().name}"
            )
            return thread_client
        except Exception as e:
            LOGGER.error(
                f"Failed to connect thread client in {threading.current_thread().name}: {e}"
            )
            return None

    def _fetch_images_worker(self):
        """Background thread for getting raw images from all cameras. Handles event loop issues."""
        thread_name = threading.current_thread().name
        LOGGER.info(f"Starting image fetch worker: {thread_name}")

        thread_client = self._create_thread_client()
        if not thread_client:
            LOGGER.error(
                f"Could not create AirSim client for {thread_name}, terminating worker."
            )
            return

        # Create dedicated threads for each camera
        def camera_fetcher(camera_name, camera_config):
            # Create a new event loop for each subthread
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                LOGGER.info(
                    f"Created new event loop for {threading.current_thread().name}"
                )
            except Exception as e:
                LOGGER.error(
                    f"Failed to set up event loop in thread {threading.current_thread().name}: {e}"
                )
                return

            # Create a separate client instance to avoid thread contention
            try:
                camera_client = airsim.MultirotorClient(
                    ip=self.config.connection.ip_address
                )
                camera_client.confirmConnection()
                LOGGER.info(f"Created dedicated client for camera {camera_name}")
            except Exception as e:
                LOGGER.error(f"Failed to create client for camera {camera_name}: {e}")
                return

            while self.running and not self.exit_flag:
                if not self.raw_image_queue.full():

                    try:
                        # Get the image using the thread-specific client
                        image_info = self.get_image(camera_client, camera_config)

                        if image_info:
                            try:
                                self.raw_image_queue.put_nowait(image_info)
                            except Full:
                                try:
                                    # If queue is full, prioritize newer frames
                                    self.raw_image_queue.get_nowait()  # Discard oldest
                                    self.raw_image_queue.put_nowait(
                                        image_info
                                    )  # Put newest
                                except (Full, Empty):
                                    pass
                        else:
                            # If image fetch failed, wait a moment before retrying
                            time.sleep(0.05)
                    except Exception as e:
                        LOGGER.error(f"Error fetching image from {camera_name}: {e}")
                        time.sleep(0.1)  # Brief sleep after error
                else:
                    time.sleep(0.01)

                # Short sleep to avoid CPU overload
                time.sleep(0.001)

        # Create a dedicated thread for each camera
        camera_threads = []
        for camera_name, camera_config in self.camera_configs.items():
            thread = threading.Thread(
                target=camera_fetcher,
                args=(camera_name, camera_config),
                name=f"CameraFetch_{camera_name}",
                daemon=True,
            )
            thread.start()
            camera_threads.append(thread)
            # Let each thread have time to initialize its event loop, avoid race conditions
            time.sleep(0.1)

        # Monitor thread status
        while self.running and not self.exit_flag:
            alive_threads = [t for t in camera_threads if t.is_alive()]
            if len(alive_threads) < len(camera_threads):
                LOGGER.warning(
                    f"Some camera threads have died! Alive: {len(alive_threads)}/{len(camera_threads)}"
                )
            time.sleep(1.0)

        LOGGER.info(f"Image fetch worker {thread_name} finished.")

    def start_image_fetcher(self):
        """Start the background image fetching thread."""
        if not self.config.display.use_threading:
            LOGGER.warning(
                "Threading is disabled, image fetching will be synchronous (not recommended)."
            )
            return

        if self.fetch_thread and self.fetch_thread.is_alive():
            LOGGER.warning("Image fetcher thread already running.")
            return

        self.running = True
        self.fetch_thread = threading.Thread(
            target=self._fetch_images_worker, name="ImageFetchThread", daemon=True
        )
        self.fetch_thread.start()
        LOGGER.info("Image fetcher thread started.")

    def start_camera_pose_thread(self):
        """Start the camera pose worker thread"""
        if self.camera_pose_thread and self.camera_pose_thread.is_alive():
            LOGGER.warning("Camera pose thread already running")
            return

        self.camera_pose_thread = threading.Thread(
            target=self.camera_pose_worker, name="CameraPoseWorker", daemon=True
        )
        self.camera_pose_thread.start()
        LOGGER.info("Camera pose worker thread started")

    def camera_pose_worker(self):
        """Worker thread to update camera poses from queue. Runs in its own thread."""
        thread_name = threading.current_thread().name
        LOGGER.info(f"{thread_name} started.")

        pose_client = self._create_thread_client()
        if not pose_client:
            LOGGER.error(
                f"Failed to create AirSim client for {thread_name}. Worker exiting."
            )
            return

        try:
            while not self.exit_flag:
                try:
                    pose_data = self.camera_pose_queue.get(timeout=0.5)

                    if pose_data == "EXIT":
                        LOGGER.info(f"{thread_name} received exit signal.")
                        break

                    if isinstance(pose_data, dict):
                        try:
                            # Create a quaternion from Euler angles
                            quat = airsim.euler_to_quaternion(
                                self.gimbal_state.roll * np.pi / 180,
                                self.gimbal_state.pitch * np.pi / 180,
                                self.gimbal_state.yaw * np.pi / 180,
                            )

                            pose_client.simSetCameraPose(
                                camera_name=pose_data.get(
                                    "camera_name", self.config.camera.camera_name
                                ),
                                pose=airsim.Pose(airsim.Vector3r(0, 0, 0.2), quat),
                                vehicle_name=pose_data.get(
                                    "vehicle_name", self.config.connection.vehicle_name
                                ),
                            )
                            LOGGER.debug(
                                f"Set camera pose: R={pose_data.get('roll', 0):.1f}, P={pose_data.get('pitch', 0):.1f}, Y={pose_data.get('yaw', 0):.1f}"
                            )
                        except Exception as e:
                            if "Connection" not in str(e) and "timeout" not in str(e):
                                LOGGER.error(f"Error setting camera pose: {e}")
                            time.sleep(0.5)

                except Empty:
                    continue
                except Exception as e:
                    LOGGER.error(f"Error in {thread_name}: {e}")
                    time.sleep(0.1)

        except Exception as e:
            LOGGER.error(f"Fatal error in {thread_name}: {e}", exc_info=True)
        finally:
            LOGGER.info(f"{thread_name} exiting.")

    def _process_and_publish_image(self, image_info):
        """Process the image and publish to the event bus for GUI to use"""
        if not image_info or not isinstance(image_info, dict):
            LOGGER.warning("Invalid image information received")
            return None

        try:
            process_start_time = time.time()

            if "data" not in image_info or not image_info["data"]:
                LOGGER.warning("Image data is empty")
                return None

            if "height" not in image_info or "width" not in image_info:
                LOGGER.warning("Missing image dimension information")
                return None

            # Get the camera name from the image info
            camera_name = image_info.get("camera_name", "front_center")

            # Process the image using the selected strategy
            processed_img = self.image_strategy.process_image(
                image_info["data"], image_info["height"], image_info["width"]
            )
            process_time = time.time() - process_start_time

            if len(self.fetch_stats["process_times"]) > 100:
                self.fetch_stats["process_times"].pop(0)
            self.fetch_stats["process_times"].append(process_time)

            if processed_img is None:
                LOGGER.warning("Image processing failed, returned None")
                return None

            if processed_img.size == 0:
                LOGGER.warning("Processed image size is 0")
                return None

            LOGGER.debug(
                f"Processed {camera_name} image: shape={processed_img.shape}, process time={process_time*1000:.1f}ms"
            )

            # Publish processed image to event bus
            self.event_bus.publish(
                "camera_image_updated",
                {
                    "image": processed_img,
                    "timestamp": image_info.get("timestamp", time.time()),
                    "process_time": process_time,
                    "camera_name": camera_name,
                },
            )

            return processed_img
        except Exception as e:
            LOGGER.error(f"Error processing image: {e}", exc_info=True)
            return None

    def _opencv_display_worker(self):
        """Dedicated thread for displaying images using OpenCV."""
        thread_name = threading.current_thread().name
        LOGGER.info(f"Starting image processing worker: {thread_name}")
        window_created = False
        use_opencv_window = True

        try:
            while not self.exit_flag:
                processed_img = None
                try:
                    image_info = self.raw_image_queue.get(timeout=0.1)

                    if image_info == "STOP":
                        LOGGER.info("Display worker received STOP signal.")
                        break

                    if not isinstance(image_info, dict):
                        LOGGER.warning(
                            f"Unexpected item in raw image queue: {type(image_info)}"
                        )
                        continue

                    processed_img = self._process_and_publish_image(image_info)

                except Empty:
                    time.sleep(0.005)
                    continue
                except Exception as q_err:
                    LOGGER.error(f"Error getting from raw image queue: {q_err}")
                    time.sleep(0.1)
                    continue

                if (
                    use_opencv_window
                    and processed_img is not None
                    and processed_img.size > 0
                ):
                    try:
                        if not window_created:
                            cv2.namedWindow(
                                self.config.display.window_name, cv2.WINDOW_NORMAL
                            )
                            cv2.resizeWindow(
                                self.config.display.window_name,
                                self.config.display.output_width,
                                self.config.display.output_height,
                            )
                            LOGGER.info(
                                f"OpenCV window '{self.config.display.window_name}' created."
                            )
                            window_created = True

                        cv2.imshow(self.config.display.window_name, processed_img)

                        key = cv2.waitKey(self.config.display.waitKey_delay) & 0xFF
                        if key == 27 or key == ord("q") or key == ord("Q"):
                            LOGGER.info("Exit requested via OpenCV window (ESC/Q).")
                            self.exit_flag = True
                            break

                    except Exception as cv_err:
                        if "NULL window" not in str(cv_err):
                            LOGGER.error(f"OpenCV display error: {cv_err}")
                        self.exit_flag = True
                        break
                else:
                    time.sleep(0.01)

        except Exception as e:
            LOGGER.error(f"Fatal error in image processing worker: {e}", exc_info=True)
            self.exit_flag = True
        finally:
            LOGGER.info(f"Image processing worker {thread_name} cleaning up...")
            if window_created and use_opencv_window:
                try:
                    cv2.destroyWindow(self.config.display.window_name)
                    cv2.destroyAllWindows()
                    LOGGER.info("OpenCV window destroyed.")
                except Exception as destroy_err:
                    LOGGER.error(f"Error destroying OpenCV window: {destroy_err}")
            LOGGER.info(f"Image processing worker {thread_name} finished.")

    def start_display_thread(self):
        """Starts the dedicated OpenCV display thread."""
        if self.opencv_display_thread and self.opencv_display_thread.is_alive():
            LOGGER.warning("OpenCV display thread already running.")
            return

        self.opencv_display_thread = threading.Thread(
            target=self._opencv_display_worker, name="DisplayThread", daemon=True
        )
        self.opencv_display_thread.start()
        LOGGER.info("OpenCV display thread started.")

    def stop_threads(self):
        """Stop all background threads managed by this client."""
        LOGGER.info("Stopping AirSimCameraClient threads...")
        self.running = False  # Signal threads to stop
        self.exit_flag = True

        self.event_bus.publish("camera_image_updated", {"exit": True})

        # Stop fetcher thread
        if self.fetch_thread and self.fetch_thread.is_alive():
            LOGGER.debug("Waiting for image fetcher thread to join...")
            self.fetch_thread.join(timeout=1.0)
            if self.fetch_thread.is_alive():
                LOGGER.warning("Image fetcher thread did not stop in time.")
            else:
                LOGGER.info("Image fetcher thread stopped.")
        self.fetch_thread = None

        # Stop display thread
        if self.opencv_display_thread and self.opencv_display_thread.is_alive():
            LOGGER.debug("Waiting for OpenCV display thread to join...")
            try:
                self.raw_image_queue.put_nowait("STOP")  # Sentinel value
            except Full:
                LOGGER.warning("Display queue full, cannot add STOP sentinel.")

            self.opencv_display_thread.join(timeout=2.0)
            if self.opencv_display_thread.is_alive():
                LOGGER.warning("OpenCV display thread did not stop in time.")
            else:
                LOGGER.info("OpenCV display thread stopped.")
        self.opencv_display_thread = None

        # Stop camera pose thread
        if self.camera_pose_thread and self.camera_pose_thread.is_alive():
            LOGGER.debug("Waiting for camera pose thread to join...")
            try:
                self.camera_pose_queue.put_nowait("EXIT")  # Sentinel value
            except Full:
                LOGGER.warning("Camera pose queue full, cannot add EXIT sentinel.")

            self.camera_pose_thread.join(timeout=1.0)
            if self.camera_pose_thread.is_alive():
                LOGGER.warning("Camera pose thread did not stop in time.")
            else:
                LOGGER.info("Camera pose thread stopped.")
        self.camera_pose_thread = None

        # Shutdown parallel processing pool
        if self.process_pool:
            LOGGER.debug("Shutting down process pool...")
            self.process_pool.shutdown(wait=True)  # Wait for tasks to complete
            self.process_pool = None
            LOGGER.info("Process pool shut down.")

        # Clean up thread-specific clients
        self.thread_clients.clear()
        LOGGER.info("AirSimCameraClient threads stopped.")

    def get_performance_stats(self) -> Dict:
        """Get performance statistics"""
        stats = {}

        def avg(data):
            return sum(data) / len(data) if data else 0

        stats["avg_fetch_time_ms"] = avg(self.fetch_stats.get("fetch_times", [])) * 1000
        stats["avg_process_time_ms"] = (
            avg(self.fetch_stats.get("process_times", [])) * 1000
        )
        stats["raw_queue_size"] = self.raw_image_queue.qsize()
        stats["display_queue_size"] = (
            self.display_image_queue.qsize() if self.display_image_queue else 0
        )

        # Calculate effective FPS based on display loop potential (limited by waitKey)
        display_wait_s = self.config.display.waitKey_delay / 1000.0
        potential_display_fps = (
            1.0 / display_wait_s if display_wait_s > 0 else float("inf")
        )
        stats["potential_display_fps"] = potential_display_fps

        return stats

    def set_gimbal_pose(self, pitch: float, roll: float, yaw: float):
        """Set the gimbal pose to a specific orientation"""
        # Update the gimbal state
        self.gimbal_state.pitch = np.clip(
            pitch, self.config.min_pitch_deg, self.config.max_pitch_deg
        )
        self.gimbal_state.roll = np.clip(
            roll, self.config.min_roll_deg, self.config.max_roll_deg
        )
        self.gimbal_state.yaw = (yaw + 180.0) % 360.0 - 180.0  # Wrap to [-180, 180]

        # Queue this update for the pose worker thread
        pose_data = {
            "pitch": self.gimbal_state.pitch,
            "roll": self.gimbal_state.roll,
            "yaw": self.gimbal_state.yaw,
            "camera_name": self.config.multi_camera.active_camera,
            "vehicle_name": self.config.connection.vehicle_name,
        }

        try:
            if not self.camera_pose_queue.full():
                self.camera_pose_queue.put_nowait(pose_data)
            else:
                LOGGER.warning("Camera pose queue full, cannot update gimbal pose")
        except Exception as e:
            LOGGER.error(f"Error queuing gimbal pose: {e}")

        # Publish event for subscribers
        self.event_bus.publish("gimbal_update", pose_data)

        return True


# =============================================================================
# EXAMPLE USAGE
# =============================================================================


def example_usage():
    """Example of how to use the AirSimCameraClient"""
    # Create configuration
    config = AirSimCameraConfig()
    config.connection.ip_address = "127.0.0.1"  # Change to your AirSim IP
    config.camera.width = 1280
    config.camera.height = 720
    config.display.output_width = 1280
    config.display.output_height = 720

    # Create event bus
    event_bus = EventBus()

    # Create gimbal state
    gimbal_state = GimbalState(pitch=-15.0)  # Initial pitch

    # Create camera client
    camera_client = AirSimCameraClient(config, gimbal_state, event_bus)

    # Connect to AirSim
    if not camera_client.connect():
        LOGGER.error("Failed to connect to AirSim")
        return

    # Subscribe to image updates
    def on_image_updated(data):
        if data.get("exit", False):
            return

        if "image" in data:
            camera_name = data.get("camera_name", "unknown")
            image = data["image"]
            if image is not None:
                LOGGER.info(
                    f"Received image from {camera_name}: shape={image.shape}, timestamp={data['timestamp']}"
                )

    event_bus.subscribe("camera_image_updated", on_image_updated)

    # Start threads
    camera_client.start_image_fetcher()
    camera_client.start_camera_pose_thread()
    camera_client.start_display_thread()

    try:
        LOGGER.info("Running - press Ctrl+C to exit")
        LOGGER.info("Moving gimbal in a pattern...")

        # Example: Move gimbal in a pattern
        for i in range(10):
            # Move up and down
            camera_client.set_gimbal_pose(-30 + i * 5, 0, 0)
            time.sleep(0.5)

        # Reset to initial position
        camera_client.set_gimbal_pose(-15, 0, 0)

        # Keep running until Ctrl+C
        while not camera_client.exit_flag:
            time.sleep(0.1)

    except KeyboardInterrupt:
        LOGGER.info("Interrupted by user")
    finally:
        LOGGER.info("Stopping camera client...")
        camera_client.stop_threads()
        cv2.destroyAllWindows()
        LOGGER.info("Camera client stopped")


if __name__ == "__main__":
    example_usage()
