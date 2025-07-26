import sys
from pathlib import Path
import asyncio
import tkinter as tk
import threading
import time
import concurrent.futures
from queue import Queue, Empty, Full
from PIL import Image, ImageTk
import PIL
import cv2
import numpy as np
from typing import Set, Dict, Optional, List, Any, Callable
from abc import ABC, abstractmethod
import argparse
import signal
import os
from pydantic import BaseModel, Field

import cosysairsim as airsim  # type: ignore
from mavsdk import System  # type: ignore
from mavsdk.telemetry import FlightMode  # type: ignore
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed  # type: ignore
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


class DroneConfig(BaseModel):
    """Overall configuration parameters"""

    # AirSim Parameters
    connection: ConnectionConfig = Field(default_factory=ConnectionConfig)
    camera: CameraConfig = Field(default_factory=CameraConfig)
    multi_camera: MultiCameraConfig = Field(default_factory=MultiCameraConfig)
    display: DisplayConfig = Field(default_factory=DisplayConfig)
    processing: ProcessingConfig = Field(default_factory=ProcessingConfig)
    timeout: int = Field(default=10, description="Connection timeout (seconds)")

    # MAVSDK Connection
    system_address: str = Field("udp://:14540", description="MAVSDK connection URL")
    default_takeoff_altitude: float = Field(
        5.0, description="Default altitude for takeoff in meters"
    )
    connection_timeout: float = Field(
        10.0, description="Timeout for drone connection attempt in seconds"
    )

    # Control Parameters
    speed_increment: float = Field(0.5, description="Drone speed increment (m/s)")
    yaw_increment: float = Field(30.0, description="Drone yaw speed increment (deg/s)")
    max_speed: float = Field(5.0, description="Maximum forward/right/down speed (m/s)")
    max_yaw_speed: float = Field(45.0, description="Maximum yaw speed (deg/s)")
    max_gimbal_rate: float = Field(
        30.0, description="Maximum gimbal angular rate (deg/s)"
    )
    gimbal_angle_increment: float = Field(
        2.0, description="Gimbal angle increment per key press (degrees)"
    )
    acceleration_factor: float = Field(1.0, description="Control acceleration factor")
    decay_factor: float = Field(
        0.8, description="Control decay factor when keys are released"
    )
    zero_threshold: float = Field(0.05, description="Threshold to zero out values")

    initial_pitch: float = Field(
        -90.0, description="Initial gimbal pitch angle in degrees"
    )
    min_pitch_deg: float = Field(-90.0, description="Minimum gimbal pitch angle")
    max_pitch_deg: float = Field(30.0, description="Maximum gimbal pitch angle")
    min_roll_deg: float = Field(-45.0, description="Minimum gimbal roll angle")
    max_roll_deg: float = Field(45.0, description="Maximum gimbal roll angle")
    min_yaw_deg: float = Field(-180.0, description="Minimum gimbal yaw angle")
    max_yaw_deg: float = Field(180.0, description="Maximum gimbal yaw angle")


class ControlState(BaseModel):
    """Current control state of the drone and gimbal"""

    # Drone Velocity State
    velocity_forward: float = 0.0
    velocity_right: float = 0.0
    velocity_down: float = 0.0
    yawspeed: float = 0.0

    # Gimbal State
    gimbal_pitch: float = 0
    gimbal_roll: float = 0
    gimbal_yaw: float = 0

    gimbal_pitch_rate: float = 0.0
    gimbal_roll_rate: float = 0.0
    gimbal_yaw_rate: float = 0.0

    # System State
    exit_flag: bool = False
    speed_multiplier: float = 1.0
    pressed_keys: Set[str] = Field(default_factory=set)

    # Telemetry Data
    last_position: dict = Field(default_factory=dict)
    last_attitude: dict = Field(default_factory=dict)

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


event_bus = EventBus()

# =============================================================================
# IMAGE PROCESSING STRATEGIES
# =============================================================================


class ImageProcessingStrategy(ABC):
    """Base class for image processing strategies"""

    @abstractmethod
    def process_image(self, img_data, height, width):
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
# AIRSIM CLIENT
# =============================================================================


class SimulationDroneClient:
    """AirSim client implementation"""

    def __init__(
        self,
        config: DroneConfig,
        state: ControlState,
        event_bus: EventBus,
        image_strategy: Optional[ImageProcessingStrategy] = None,
    ):
        self.config = config
        self.state = state
        self.event_bus = event_bus
        self.client: airsim.MultirotorClient

        # Create queues through event_bus instead of using global variables
        self.camera_pose_queue = event_bus.create_queue("camera_pose_queue", maxsize=10)
        self.display_image_queue = event_bus.create_queue(
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
        self.fetch_times: List[float] = []
        self.raw_image_queue: Queue = Queue(maxsize=config.display.fetch_queue_size)
        self.fetch_thread: Optional[threading.Thread] = None
        self.thread_clients: List[airsim.MultirotorClient] = []
        self.fetch_stats: Dict = {"fetch_times": [], "process_times": []}

        self.camera_configs = {
            "front_center": {
                "camera_name": "front_center",
                "image_type": airsim.ImageType.Scene,
                "pixels_as_float": False,
                "compress": False,
                "width": config.camera.width,
                "height": config.camera.height,
            },
            "side_left": {
                "camera_name": "side_left",
                "image_type": airsim.ImageType.Scene,
                "pixels_as_float": False,
                "compress": False,
                "width": config.camera.width,
                "height": config.camera.height,
            },
            "side_right": {
                "camera_name": "side_right",
                "image_type": airsim.ImageType.Scene,
                "pixels_as_float": False,
                "compress": False,
                "width": config.camera.width,
                "height": config.camera.height,
            },
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

    def _create_thread_client(self) -> Optional[airsim.MultirotorClient]:
        """創建線程專用的客戶端實例，確保每個線程有適當的事件循環"""
        try:
            # 在線程中設置新的事件循環
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            # 創建客戶端實例
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
        """背景線程用於從所有攝像頭獲取原始圖像。處理事件循環問題。"""
        thread_name = threading.current_thread().name
        LOGGER.info(f"Starting image fetch worker: {thread_name}")

        thread_client = self._create_thread_client()
        if not thread_client:
            LOGGER.error(
                f"Could not create AirSim client for {thread_name}, terminating worker."
            )
            return

        # 為每個攝像頭創建專門的線程
        def camera_fetcher(camera_name, camera_config):
            # 為每個子線程創建新的事件循環
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

            # 創建單獨的 client 實例以避免線程競爭
            try:
                camera_client = airsim.MultirotorClient(
                    ip=self.config.connection.ip_address
                )
                camera_client.confirmConnection()
                LOGGER.info(f"Created dedicated client for camera {camera_name}")
            except Exception as e:
                LOGGER.error(f"Failed to create client for camera {camera_name}: {e}")
                return

            while self.running and not self.state.exit_flag:
                if not self.raw_image_queue.full():

                    try:
                        # 使用線程專用客戶端獲取圖像
                        image_info = self.get_image(camera_client, camera_config)

                        if image_info:
                            try:
                                self.raw_image_queue.put_nowait(image_info)
                            except Full:
                                try:
                                    # 如果隊列已滿，優先處理較新的幀
                                    self.raw_image_queue.get_nowait()  # 丟棄最舊的
                                    self.raw_image_queue.put_nowait(
                                        image_info
                                    )  # 放入最新的
                                except (Full, Empty):
                                    pass
                        else:
                            # 如果獲取失敗，稍等片刻再嘗試
                            time.sleep(0.05)
                    except Exception as e:
                        LOGGER.error(f"Error fetching image from {camera_name}: {e}")
                        time.sleep(0.1)  # 出錯後略微休眠
                else:
                    time.sleep(0.01)

                # 短暫休眠以避免 CPU 過載
                time.sleep(0.001)

        # 為每個攝像頭創建一個專門的線程
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
            # 讓每個線程有時間初始化事件循環，避免競爭
            time.sleep(0.1)

        # 監視線程狀態
        while self.running and not self.state.exit_flag:
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
            while not self.state.exit_flag:
                try:
                    pose_data = self.camera_pose_queue.get(timeout=0.5)

                    if pose_data == "EXIT":
                        LOGGER.info(f"{thread_name} received exit signal.")
                        break

                    if isinstance(pose_data, dict):
                        try:
                            quat = airsim.euler_to_quaternion(
                                self.state.gimbal_roll * np.pi / 180,
                                self.state.gimbal_pitch * np.pi / 180,
                                self.state.gimbal_yaw * np.pi / 180,
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
        use_opencv_window = False

        try:
            while not self.state.exit_flag:
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
                            self.state.exit_flag = True
                            break

                    except Exception as cv_err:
                        if "NULL window" not in str(cv_err):
                            LOGGER.error(f"OpenCV display error: {cv_err}")
                        self.state.exit_flag = True
                        break
                else:
                    time.sleep(0.01)

        except Exception as e:
            LOGGER.error(f"Fatal error in image processing worker: {e}", exc_info=True)
            self.state.exit_flag = True
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
        LOGGER.info("Stopping SimulationDroneClient threads...")
        self.running = False  # Signal threads to stop

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
        LOGGER.info("SimulationDroneClient threads stopped.")

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


# =============================================================================
# DRONE CONTROLLER
# =============================================================================


class DroneController:
    """Controller for MAVSDK drone operations with improved async handling"""

    def __init__(self, config: DroneConfig, event_bus: EventBus):
        self.config = config
        self.event_bus = event_bus
        self.drone = System()
        self._telemetry_task: Any = None

        # Events to monitor drone state
        self.connected_event = asyncio.Event()
        self.armed_event = asyncio.Event()
        self.in_air_event = asyncio.Event()
        self.offboard_started_event = asyncio.Event()

        # Subscribe to relevant events
        event_bus.subscribe("drone_exit", self._handle_exit_request)

    async def _handle_exit_request(self, data):
        """Handle exit request from other components"""
        LOGGER.info("Drone controller received exit request")
        # Start landing sequence
        await self.land_drone()

    async def is_connected(self) -> bool:
        """Check if drone is connected"""
        connected = False
        async for state in self.drone.core.connection_state():
            connected = state.is_connected
            break
        return connected

    async def connect_drone(self) -> bool:
        """Connect to drone using MAVSDK"""
        LOGGER.info(f"Connecting to drone at {self.config.system_address}...")
        try:
            await self.drone.connect(system_address=self.config.system_address)

            LOGGER.info("Waiting for drone connection...")
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    LOGGER.info("Drone connected!")
                    self.connected_event.set()
                    self.event_bus.publish("drone_connected", True)
                    return True

            LOGGER.error("Drone connection loop finished unexpectedly.")
            return False

        except asyncio.TimeoutError:
            LOGGER.error(
                f"Drone connection timed out after {self.config.connection_timeout} seconds"
            )
            return False
        except Exception as e:
            LOGGER.error(f"Drone connection error: {e}")
            return False

    async def check_drone_health(self) -> bool:
        """Check drone health status"""
        if not await self.is_connected():
            LOGGER.error("Health check failed: Drone not connected.")
            return False

        try:
            LOGGER.info("Checking drone health...")
            async for health in self.drone.telemetry.health():
                is_healthy = (
                    health.is_gyrometer_calibration_ok
                    or health.is_accelerometer_calibration_ok
                    or health.is_magnetometer_calibration_ok
                )
                is_armable = health.is_armable

                LOGGER.info(
                    f"Health: GyroOK={health.is_gyrometer_calibration_ok}, "
                    f"AccelOK={health.is_accelerometer_calibration_ok}, "
                    f"MagOK={health.is_magnetometer_calibration_ok}, "
                    f"LocalPosOK={health.is_local_position_ok}, "
                    f"GlobalPosOK={health.is_global_position_ok}, "
                    f"Armable={is_armable}"
                )

                self.event_bus.publish(
                    "drone_health",
                    {
                        "is_healthy": is_healthy,
                        "is_armable": is_armable,
                        "health_data": {
                            "gyro_ok": health.is_gyrometer_calibration_ok,
                            "accel_ok": health.is_accelerometer_calibration_ok,
                            "mag_ok": health.is_magnetometer_calibration_ok,
                            "local_pos_ok": health.is_local_position_ok,
                            "global_pos_ok": health.is_global_position_ok,
                        },
                    },
                )

                if is_healthy and is_armable:
                    LOGGER.info("Drone health OK and armable!")
                    return True
                if is_healthy and not is_armable:
                    LOGGER.warning(
                        "Drone sensors OK, but currently not armable. Check pre-arm conditions."
                    )
                    return True
                await asyncio.sleep(1)

            LOGGER.error("Health telemetry stream ended unexpectedly.")
            return False

        except asyncio.TimeoutError:
            LOGGER.error("Timeout waiting for health telemetry.")
            return False
        except Exception as e:
            LOGGER.error(f"Error during health check: {e}")
            return False

    async def arm_drone(self) -> bool:
        """Arm the drone"""
        if not await self.is_connected():
            LOGGER.error("Arming failed: Drone not connected.")
            return False
        try:
            LOGGER.info("Arming drone...")
            await self.drone.action.arm()
            LOGGER.info("Drone armed!")
            self.armed_event.set()
            self.event_bus.publish("drone_armed", True)
            return True
        except Exception as e:
            LOGGER.error(f"Arming failed: {e}")
            return False

    async def takeoff_drone(self) -> bool:
        """Takeoff to specified altitude"""
        if not await self.is_connected():
            LOGGER.error("Takeoff failed: Drone not connected.")
            return False

        altitude = self.config.default_takeoff_altitude
        try:
            LOGGER.info(f"Setting takeoff altitude to {altitude} meters...")
            await self.drone.action.set_takeoff_altitude(altitude)

            LOGGER.info("Commanding takeoff...")
            await self.drone.action.takeoff()

            # Monitor altitude to confirm takeoff
            takeoff_timeout = 30  # seconds
            start_time = asyncio.get_event_loop().time()
            LOGGER.info("Monitoring takeoff progress...")

            async for position in self.drone.telemetry.position():
                current_alt = position.relative_altitude_m
                LOGGER.info(f"Current altitude: {current_alt:.2f} m")

                # Publish altitude info for GUI
                self.event_bus.publish("drone_altitude", current_alt)

                if current_alt >= altitude * 0.90:  # Allow slightly lower
                    LOGGER.info("Target altitude reached!")
                    self.in_air_event.set()
                    self.event_bus.publish("drone_in_air", True)
                    return True

                if asyncio.get_event_loop().time() - start_time > takeoff_timeout:
                    LOGGER.warning(
                        f"Takeoff timeout after {takeoff_timeout}s at altitude {current_alt:.2f}m. Proceeding anyway."
                    )
                    self.in_air_event.set()
                    self.event_bus.publish("drone_in_air", True)
                    return True  # Proceed even if full altitude wasn't confirmed

            LOGGER.warning(
                "Position telemetry stream ended before confirming takeoff altitude."
            )
            return True

        except asyncio.TimeoutError:
            LOGGER.error("Timeout during takeoff sequence (waiting for telemetry).")
            return False
        except Exception as e:
            LOGGER.error(f"Takeoff failed: {e}")
            return False

    async def land_drone(self):
        """Land the drone"""
        if not await self.is_connected():
            LOGGER.warning("Cannot land: Drone not connected.")
            return

        try:
            # Check if already landed/disarmed
            is_armed = await self.drone.telemetry.armed().__aiter__().__anext__()
            in_air = await self.drone.telemetry.in_air().__aiter__().__anext__()

            if not is_armed or not in_air:
                LOGGER.info("Drone already on ground or disarmed. No landing needed.")
                self.in_air_event.clear()
                self.event_bus.publish("drone_landed", True)
                return

            LOGGER.info("Commanding drone to land...")
            await self.drone.action.land()
            LOGGER.info("Drone landing command sent. Monitoring landing...")
            self.event_bus.publish("drone_landing", True)

            # Monitor until disarmed or timeout
            land_timeout = 60  # seconds
            start_time = asyncio.get_event_loop().time()

            async for armed_status in self.drone.telemetry.armed():
                if not armed_status:
                    LOGGER.info("Drone disarmed. Landing complete.")
                    self.in_air_event.clear()
                    self.armed_event.clear()
                    self.event_bus.publish("drone_landed", True)
                    return  # Success

                if asyncio.get_event_loop().time() - start_time > land_timeout:
                    LOGGER.warning(
                        f"Landing timeout after {land_timeout}s. Drone might still be armed."
                    )
                    return  # Timeout

            LOGGER.warning("Armed telemetry stream ended before confirming disarm.")

        except asyncio.TimeoutError:
            LOGGER.error("Timeout during landing sequence (waiting for telemetry).")
        except Exception as e:
            LOGGER.error(f"Landing error: {e}")

    async def _telemetry_updater(self, state: ControlState):
        """Continuously updates telemetry in the background."""
        LOGGER.info("Starting telemetry updater task.")
        try:
            # Subscribe to multiple streams concurrently
            async def subscribe_position():
                async for position in self.drone.telemetry.position():
                    if state.exit_flag:
                        break

                    position_data = {
                        "latitude_deg": position.latitude_deg,
                        "longitude_deg": position.longitude_deg,
                        "absolute_altitude_m": position.absolute_altitude_m,
                        "relative_altitude_m": position.relative_altitude_m,
                    }

                    state.last_position = position_data
                    self.event_bus.publish("drone_position", position_data)

            async def subscribe_attitude():
                async for attitude in self.drone.telemetry.attitude_euler():
                    if state.exit_flag:
                        break

                    attitude_data = {
                        "roll_deg": attitude.roll_deg,
                        "pitch_deg": attitude.pitch_deg,
                        "yaw_deg": attitude.yaw_deg,
                    }

                    state.last_attitude = attitude_data
                    self.event_bus.publish("drone_attitude", attitude_data)

            # Add flight mode subscription
            async def subscribe_flight_mode():
                async for flight_mode in self.drone.telemetry.flight_mode():
                    if state.exit_flag:
                        break
                    self.event_bus.publish("drone_flight_mode", flight_mode)

            await asyncio.gather(
                subscribe_position(), subscribe_attitude(), subscribe_flight_mode()
            )

        except asyncio.CancelledError:
            LOGGER.info("Telemetry updater task cancelled.")
        except Exception as e:
            LOGGER.error(f"Error in telemetry updater: {e}", exc_info=True)
        finally:
            LOGGER.info("Telemetry updater task finished.")

    async def start_telemetry_updates(self, state: ControlState):
        """Start the background telemetry update task."""
        if self._telemetry_task and not self._telemetry_task.done():
            LOGGER.warning("Telemetry task already running.")
            return
        self._telemetry_task = asyncio.create_task(self._telemetry_updater(state))

    async def stop_telemetry_updates(self):
        """Stop the background telemetry update task."""
        if self._telemetry_task and not self._telemetry_task.done():
            LOGGER.info("Stopping telemetry updater task...")
            self._telemetry_task.cancel()
            try:
                await asyncio.wait_for(self._telemetry_task, timeout=1.0)
            except asyncio.TimeoutError:
                LOGGER.warning("Timeout waiting for telemetry task to cancel.")
            except asyncio.CancelledError:
                pass  # Expected
            LOGGER.info("Telemetry updater task stopped.")
        self._telemetry_task = None

    async def run_manual_control_loop(self, state: ControlState):
        """Run the main drone control loop. Reads state updated by GUI."""
        if not await self.is_connected():
            LOGGER.error("Cannot run control loop: drone not connected")
            return

        LOGGER.info("Preparing for flight control...")
        offboard_started = False

        try:
            # Start background telemetry updates
            await self.start_telemetry_updates(state)

            # --- Start Offboard Mode ---
            LOGGER.info("Priming Offboard setpoints...")
            # Send initial setpoints before starting
            initial_velocity_cmd = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            await self.drone.offboard.set_velocity_body(initial_velocity_cmd)
            await asyncio.sleep(0.1)  # Short delay

            try:
                LOGGER.info("Starting Offboard mode...")
                await self.drone.offboard.start()
                offboard_started = True
                self.offboard_started_event.set()
                self.event_bus.publish("offboard_started", True)
                LOGGER.info("Offboard mode started successfully.")
            except OffboardError as e:
                LOGGER.error(f"Failed to start Offboard mode: {e}")
                LOGGER.info("Attempting to land.")
                await self.land_drone()
                return  # Cannot continue without Offboard

            # --- Main Control Loop ---
            LOGGER.info("\n--- Flight Control Active ---")
            LOGGER.info("Use Tkinter GUI window for control.")
            LOGGER.info("Press ESC in GUI or Q/ESC in OpenCV window to stop.\n")

            last_print_time = time.monotonic()
            control_error_count = 0
            max_control_errors = 10
            offboard_check_interval = 2.0  # Check if still in offboard every 2s
            last_offboard_check_time = time.monotonic()

            while not state.exit_flag:
                current_time_monotonic = time.monotonic()

                # --- Check if still in Offboard Mode periodically ---
                if (
                    current_time_monotonic - last_offboard_check_time
                    > offboard_check_interval
                ):
                    try:
                        current_mode = (
                            await self.drone.telemetry.flight_mode()
                            .__aiter__()
                            .__anext__()
                        )
                        if current_mode != FlightMode.OFFBOARD:  # Use mavsdk enum
                            LOGGER.error(
                                f"Drone left Offboard mode! Current mode: {current_mode}. Landing."
                            )
                            state.exit_flag = True
                            self.event_bus.publish("offboard_lost", current_mode)
                            break  # Exit control loop
                        last_offboard_check_time = current_time_monotonic
                    except Exception as mode_err:
                        LOGGER.warning(f"Could not verify flight mode: {mode_err}")

                # --- Send Drone Velocity Command ---
                try:
                    await self.drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(
                            state.velocity_forward,
                            state.velocity_right,
                            state.velocity_down,
                            state.yawspeed,
                        )
                    )
                    control_error_count = 0  # Reset error count on success

                except OffboardError as e:
                    control_error_count += 1
                    LOGGER.warning(
                        f"Offboard command error ({control_error_count}/{max_control_errors}): {e}"
                    )
                    if control_error_count >= max_control_errors:
                        LOGGER.error(
                            "Too many consecutive Offboard errors, stopping control loop."
                        )
                        state.exit_flag = True  # Signal exit
                        self.event_bus.publish(
                            "offboard_error", f"Too many errors: {e}"
                        )
                    await asyncio.sleep(0.1)  # Wait before retrying after error
                    continue  # Skip rest of loop iteration
                except Exception as e:
                    control_error_count += 1
                    LOGGER.warning(
                        f"Control command error ({control_error_count}/{max_control_errors}): {e}"
                    )
                    if control_error_count >= max_control_errors:
                        LOGGER.error(
                            "Too many consecutive control errors, stopping control loop."
                        )
                        state.exit_flag = True
                        self.event_bus.publish("control_error", f"Too many errors: {e}")
                    await asyncio.sleep(0.1)
                    continue

                # --- Periodic Logging ---
                if current_time_monotonic - last_print_time >= 1.0:
                    log_msg = (
                        f"Vel (F,R,D,Y): {state.velocity_forward:+.1f}, {state.velocity_right:+.1f}, "
                        f"{state.velocity_down:+.1f}, {state.yawspeed:+.1f} | "
                        f"Gimbal Sim (P,R,Y): {state.gimbal_pitch:+.1f}, {state.gimbal_roll:+.1f}, {state.gimbal_yaw:+.1f}"
                    )
                    LOGGER.debug(log_msg)  # Use debug level for frequent logs
                    last_print_time = current_time_monotonic

                await asyncio.sleep(0.02)

        except asyncio.CancelledError:
            LOGGER.info("Control loop task cancelled.")
        except Exception as e:
            LOGGER.error(f"Unhandled error during flight control: {e}", exc_info=True)
            state.exit_flag = True  # Ensure exit on error
            self.event_bus.publish("drone_control_error", str(e))
        finally:
            LOGGER.info("Exiting control loop and cleaning up...")

            # Stop telemetry updates first
            await self.stop_telemetry_updates()

            # Stop Offboard mode if it was started
            if offboard_started:
                LOGGER.info("Stopping Offboard mode...")
                try:
                    # Send zero velocity before stopping
                    await self.drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                    )
                    await asyncio.sleep(0.1)
                    await self.drone.offboard.stop()
                    self.offboard_started_event.clear()
                    self.event_bus.publish("offboard_stopped", True)
                    LOGGER.info("Offboard mode stopped.")
                except OffboardError as e:
                    LOGGER.error(f"Error stopping offboard mode: {e}")
                except Exception as e:
                    LOGGER.error(f"Unexpected error stopping offboard mode: {e}")

            # Landing is now handled in the main finally block
            LOGGER.info("Control loop cleanup complete.")


class DroneControlGUI:
    """Enhanced GUI for drone and gimbal control with visualization indicators"""

    def __init__(
        self, root: tk.Tk, config: DroneConfig, state: ControlState, event_bus: EventBus
    ):
        self.root = root
        self.config = config
        self.state = state
        self.event_bus = event_bus
        self.active_key_labels: Dict[str, tk.Label] = {}  # Type hint

        # Subscribe to events
        event_bus.subscribe("drone_position", self._handle_position_update)
        event_bus.subscribe("drone_attitude", self._handle_attitude_update)
        event_bus.subscribe("drone_flight_mode", self._handle_flight_mode_update)
        event_bus.subscribe("drone_connected", self._handle_connection_status)
        event_bus.subscribe("drone_armed", self._handle_armed_status)
        event_bus.subscribe("drone_in_air", self._handle_in_air_status)
        event_bus.subscribe("offboard_started", self._handle_offboard_status)
        event_bus.subscribe("offboard_error", self._handle_error_message)
        event_bus.subscribe("control_error", self._handle_error_message)
        event_bus.subscribe("drone_control_error", self._handle_error_message)
        event_bus.subscribe("camera_image_updated", self._handle_image_update)

        self.root.title("Drone Control Interface")
        self.root.geometry("1920x1080")
        self.root.configure(bg="#f0f0f0")

        # Fonts
        self.title_font = ("Arial", 16, "bold")
        self.header_font = ("Arial", 12, "bold")
        self.text_font = ("Arial", 10)
        self.button_font = ("Arial", 11, "bold")

        # Colors
        self.title_color = "#2C3E50"
        self.status_active_color = "#27AE60"
        self.status_warning_color = "#F39C12"
        self.status_error_color = "#E74C3C"

        self.main_container = tk.Frame(root, bg="#f0f0f0", padx=15, pady=15)
        self.main_container.pack(fill="both", expand=True)

        self.create_header()
        self.create_main_layout()
        self.create_camera_display()

        self.create_control_reference()
        self.create_status_section()
        self.create_visual_indicators()
        self.create_control_buttons()

        # Status message label at the bottom
        self.status_message_label = tk.Label(
            self.main_container, text="", font=("Arial", 10), bg="#f0f0f0", fg="#333333"
        )
        self.status_message_label.pack(fill="x", pady=(5, 0))

        self.last_key_process_time = time.monotonic()
        self.last_telemetry_update_time = time.monotonic()

        # Initialize image related variables
        self.last_image = None
        self.image_update_count = 0
        self.last_frame_time = None

        # Ensure GUI has focus initially
        self.root.focus_force()

        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.exit_control)

        # Start the UI update loop
        self._update_gui_loop_id = self.root.after(50, self.update_gui_loop)

    def _handle_image_update(self, data):
        """Handle image updates from the event bus"""
        if data.get("exit", False):
            return

        if not self.root.winfo_exists():
            return

        try:
            if "image" in data and data["image"] is not None:
                # Get the camera name from the data or default to front_center
                camera_name = data.get("camera_name", "front_center")

                if camera_name not in self.camera_views:
                    LOGGER.warning(f"Received image for unknown camera: {camera_name}")
                    # Default to front_center if the camera is unknown
                    camera_name = "front_center"

                camera_view = self.camera_views[camera_name]

                img_shape = (
                    data["image"].shape if hasattr(data["image"], "shape") else "無效"
                )
                LOGGER.debug(
                    f"Received image update for {camera_name}: shape = {img_shape}, "
                    f"update count = {camera_view['update_count']}"
                )

                camera_view["last_image"] = data["image"]
                camera_view["update_count"] += 1

                # Update the specific camera display
                self.update_specific_camera_display(camera_name)

        except Exception as e:
            LOGGER.error(f"Error handling image update: {e}", exc_info=True)

    def update_specific_camera_display(self, camera_name):
        """Update a specific camera display in the GUI with fixed size"""
        if camera_name not in self.camera_views:
            LOGGER.error(f"Cannot update unknown camera: {camera_name}")
            return

        camera_view = self.camera_views[camera_name]
        camera_label = camera_view["label"]
        camera_info_label = camera_view["info_label"]

        if not camera_label.winfo_exists():
            LOGGER.error(f"Camera label for {camera_name} does not exist")
            return

        if camera_view["last_image"] is None:
            LOGGER.debug(f"No image available for {camera_name}")
            return

        try:
            if camera_view["last_image"].size == 0:
                LOGGER.warning(f"Empty image data for {camera_name}")
                return

            LOGGER.debug(
                f"Processing image for {camera_name}: shape={camera_view['last_image'].shape}, "
                f"type={type(camera_view['last_image'])}"
            )

            display_img = camera_view["last_image"].copy()
            if len(display_img.shape) != 3 or display_img.shape[2] != 3:
                LOGGER.warning(
                    f"Unexpected image format for {camera_name}: {display_img.shape}"
                )
                return

            # Use the fixed size from our camera_views dictionary
            fixed_width = camera_view["width"]
            fixed_height = camera_view["height"]

            img_height, img_width = display_img.shape[:2]

            if img_width == 0 or img_height == 0:
                LOGGER.warning(f"Invalid image dimensions for {camera_name}")
                return

            # Calculate aspect ratio preserving scale
            width_ratio = fixed_width / img_width
            height_ratio = fixed_height / img_height

            scale_factor = min(width_ratio, height_ratio)

            # Resize to fit within the fixed frame, maintaining aspect ratio
            new_width = int(img_width * scale_factor)
            new_height = int(img_height * scale_factor)

            LOGGER.debug(
                f"Scaling {camera_name} image to: {new_width}x{new_height}, scale factor: {scale_factor}"
            )

            resized_img = cv2.resize(
                display_img, (new_width, new_height), interpolation=cv2.INTER_LINEAR
            )

            # Create a black canvas of the fixed size
            canvas = np.zeros((fixed_height, fixed_width, 3), dtype=np.uint8)

            # Center the resized image on the canvas
            y_offset = (fixed_height - new_height) // 2
            x_offset = (fixed_width - new_width) // 2

            # Place the resized image in the center of the canvas
            canvas[
                y_offset : y_offset + new_height, x_offset : x_offset + new_width
            ] = resized_img

            # Convert to PIL image for Tkinter
            pil_img = Image.fromarray(canvas)
            tk_img = ImageTk.PhotoImage(image=pil_img)

            camera_label.configure(image=tk_img)
            camera_label.image = (
                tk_img  # Keep a reference to prevent garbage collection
            )

            fps = 0
            if camera_view["last_frame_time"]:
                elapsed = time.time() - camera_view["last_frame_time"]
                fps = 1.0 / elapsed if elapsed > 0 else 0

            camera_view["last_frame_time"] = time.time()

            if camera_info_label.winfo_exists():
                camera_info_label.config(
                    text=f"{camera_name} | Fixed Size: {fixed_width}x{fixed_height} | FPS: {fps:.1f}"
                )

            LOGGER.debug(f"Successfully updated {camera_name} image")

        except Exception as e:
            LOGGER.error(f"Error updating {camera_name} display: {e}", exc_info=True)

    def update_camera_display(self):
        """Update all camera displays in the GUI"""
        # Update each camera individually
        for camera_name in self.camera_views:
            self.update_specific_camera_display(camera_name)

    def _handle_position_update(self, data):
        """Handle position update event"""
        if hasattr(self, "altitude_label"):
            alt = data.get("relative_altitude_m", 0.0)
            self.altitude_label.config(text=f"{alt:.1f} m")

    def _handle_attitude_update(self, data):
        """Handle attitude update event"""
        if hasattr(self, "attitude_label"):
            roll = data.get("roll_deg", 0.0)
            pitch = data.get("pitch_deg", 0.0)
            yaw = data.get("yaw_deg", 0.0)
            self.attitude_label.config(
                text=f"R:{roll:+.1f}, P:{pitch:+.1f}, Y:{yaw:+.1f} deg"
            )

    def _handle_flight_mode_update(self, mode):
        """Handle flight mode update event"""
        self.update_status_message(f"Flight mode: {mode}", "info")

    def _handle_connection_status(self, connected):
        """Handle connection status update event"""
        if connected and hasattr(self, "conn_status"):
            self.conn_status.config(text="● CONNECTED", fg=self.status_active_color)

    def _handle_armed_status(self, armed):
        """Handle armed status update event"""
        if armed:
            self.update_status_message("Drone armed", "info")

    def _handle_in_air_status(self, in_air):
        """Handle in-air status update event"""
        if in_air:
            self.update_status_message("Drone in air", "info")
        else:
            self.update_status_message("Drone landed", "info")

    def _handle_offboard_status(self, offboard):
        """Handle offboard status update event"""
        if offboard:
            self.update_status_message("Offboard mode active", "info")
        else:
            self.update_status_message("Offboard mode inactive", "info")

    def _handle_error_message(self, message):
        """Handle error message event"""
        self.update_status_message(f"Error: {message}", "error")

    def update_status_message(self, message: str, level: str = "info"):
        """Updates the status message label at the bottom."""
        if not self.root.winfo_exists():
            return
        color = "#333333"  # Default info color
        if level == "warning":
            color = self.status_warning_color
        elif level == "error":
            color = self.status_error_color
        self.status_message_label.config(text=message, fg=color)

    def create_header(self):
        """Create the header with title and connection status"""
        header_frame = tk.Frame(self.main_container, bg="#f0f0f0")
        header_frame.pack(fill="x", pady=(0, 10))
        title_label = tk.Label(
            header_frame,
            text="Drone Control System",
            font=self.title_font,
            fg=self.title_color,
            bg="#f0f0f0",
        )
        title_label.pack(side=tk.LEFT, pady=5)
        self.conn_status = tk.Label(
            header_frame,
            text="● INITIALIZING",
            font=self.text_font,
            fg=self.status_warning_color,
            bg="#f0f0f0",
        )
        self.conn_status.pack(side=tk.RIGHT, pady=5, padx=10)

    def create_main_layout(self):
        """Create the main layout, divided into the left control area and the right camera area"""
        # The main layout container uses horizontal split
        self.main_layout = tk.PanedWindow(
            self.main_container,
            orient=tk.HORIZONTAL,
            bg="#f0f0f0",
            sashwidth=4,
            sashrelief=tk.RAISED,
        )
        self.main_layout.pack(fill="both", expand=True, pady=5)

        # Left panel - contains the controls section
        self.left_panel = tk.Frame(self.main_layout, bg="#f0f0f0", padx=5, pady=5)
        self.main_layout.add(self.left_panel, width=640)

        # Right panel - for camera display
        self.right_panel = tk.Frame(self.main_layout, bg="#f0f0f0", padx=5, pady=5)
        self.main_layout.add(self.right_panel, width=800)

        self.left_top = tk.Frame(self.left_panel, bg="#f0f0f0")
        self.left_top.pack(fill="both", expand=True)
        self.left_bottom = tk.Frame(self.left_panel, bg="#f0f0f0")
        self.left_bottom.pack(fill="x", pady=(10, 0))

    def create_camera_display(self):
        """Create a camera display area in the right panel with multiple views of fixed size"""
        # Main container for all cameras
        camera_frame = tk.LabelFrame(
            self.right_panel,
            text="Camera Views",
            font=self.header_font,
            bg="#f0f0f0",
            fg=self.title_color,
        )
        camera_frame.pack(fill="both", expand=True, padx=5, pady=5)

        # Define fixed sizes for camera displays
        self.front_camera_width = self.config.camera.width
        self.front_camera_height = self.config.camera.height
        self.side_camera_width = self.config.camera.width
        self.side_camera_height = self.config.camera.height

        # Top frame for front_center camera
        self.front_camera_frame = tk.Frame(camera_frame, bg="#f0f0f0")
        self.front_camera_frame.pack(fill="x", padx=5, pady=5)

        # Label for front camera
        tk.Label(
            self.front_camera_frame,
            text="Front Camera",
            font=self.text_font,
            bg="#f0f0f0",
            anchor="w",
        ).pack(anchor="w", padx=5)

        # Create a fixed-size frame to contain the front camera
        front_size_frame = tk.Frame(
            self.front_camera_frame,
            width=self.front_camera_width,
            height=self.front_camera_height,
            bg="black",
        )
        front_size_frame.pack(pady=5)
        front_size_frame.pack_propagate(
            False
        )  # Prevent frame from resizing with its contents

        # Front camera display inside the fixed frame
        self.front_camera_label = tk.Label(front_size_frame, bg="black")
        self.front_camera_label.pack(fill="both", expand=True)

        # Front camera info label
        self.front_camera_info_label = tk.Label(
            self.front_camera_frame,
            text="Waiting for front camera feed...",
            font=self.text_font,
            bg="#f0f0f0",
            anchor="w",
        )
        self.front_camera_info_label.pack(fill="x", padx=5, pady=(0, 5))

        # Bottom frame for side cameras
        self.side_cameras_frame = tk.Frame(camera_frame, bg="#f0f0f0")
        self.side_cameras_frame.pack(fill="x", padx=5, pady=5)

        # Side cameras container (horizontal layout)
        side_cameras_container = tk.Frame(self.side_cameras_frame, bg="#f0f0f0")
        side_cameras_container.pack(fill="x", padx=5, pady=5)

        # Left side camera frame
        left_side_frame = tk.Frame(side_cameras_container, bg="#f0f0f0")
        left_side_frame.pack(side=tk.LEFT, fill="x", expand=True, padx=(0, 5))

        # Label for left side camera
        tk.Label(
            left_side_frame,
            text="Left Side Camera",
            font=self.text_font,
            bg="#f0f0f0",
            anchor="w",
        ).pack(anchor="w", padx=5)

        # Create a fixed-size frame for left camera
        left_size_frame = tk.Frame(
            left_side_frame,
            width=self.side_camera_width,
            height=self.side_camera_height,
            bg="black",
        )
        left_size_frame.pack(pady=5)
        left_size_frame.pack_propagate(False)  # Prevent frame from resizing

        # Left side camera display inside the fixed frame
        self.left_camera_label = tk.Label(left_size_frame, bg="black")
        self.left_camera_label.pack(fill="both", expand=True)

        # Left camera info label
        self.left_camera_info_label = tk.Label(
            left_side_frame,
            text="Waiting for left camera feed...",
            font=self.text_font,
            bg="#f0f0f0",
            anchor="w",
        )
        self.left_camera_info_label.pack(fill="x", padx=5, pady=(0, 5))

        # Right side camera frame
        right_side_frame = tk.Frame(side_cameras_container, bg="#f0f0f0")
        right_side_frame.pack(side=tk.RIGHT, fill="x", expand=True, padx=(5, 0))

        # Label for right side camera
        tk.Label(
            right_side_frame,
            text="Right Side Camera",
            font=self.text_font,
            bg="#f0f0f0",
            anchor="w",
        ).pack(anchor="w", padx=5)

        # Create a fixed-size frame for right camera
        right_size_frame = tk.Frame(
            right_side_frame,
            width=self.side_camera_width,
            height=self.side_camera_height,
            bg="black",
        )
        right_size_frame.pack(pady=5)
        right_size_frame.pack_propagate(False)  # Prevent frame from resizing

        # Right side camera display inside the fixed frame
        self.right_camera_label = tk.Label(right_size_frame, bg="black")
        self.right_camera_label.pack(fill="both", expand=True)

        # Right camera info label
        self.right_camera_info_label = tk.Label(
            right_side_frame,
            text="Waiting for right camera feed...",
            font=self.text_font,
            bg="#f0f0f0",
            anchor="w",
        )
        self.right_camera_info_label.pack(fill="x", padx=5, pady=(0, 5))

        # For backward compatibility
        self.camera_label = self.front_camera_label
        self.camera_info_label = self.front_camera_info_label

        # Dictionary to keep track of camera views
        self.camera_views = {
            "front_center": {
                "label": self.front_camera_label,
                "info_label": self.front_camera_info_label,
                "last_image": None,
                "last_frame_time": None,
                "update_count": 0,
                "width": self.front_camera_width,
                "height": self.front_camera_height,
            },
            "side_left": {
                "label": self.left_camera_label,
                "info_label": self.left_camera_info_label,
                "last_image": None,
                "last_frame_time": None,
                "update_count": 0,
                "width": self.side_camera_width,
                "height": self.side_camera_height,
            },
            "side_right": {
                "label": self.right_camera_label,
                "info_label": self.right_camera_info_label,
                "last_image": None,
                "last_frame_time": None,
                "update_count": 0,
                "width": self.side_camera_width,
                "height": self.side_camera_height,
            },
        }

    def create_control_reference(self):
        """Create the control reference panel"""
        controls_frame = tk.LabelFrame(
            self.left_top,
            text="Control Reference",
            font=self.header_font,
            bg="#f0f0f0",
            fg=self.title_color,
            padx=10,
            pady=5,
        )
        controls_frame.pack(fill="x", pady=5)
        control_categories = {
            "Movement (Body Frame)": [
                ("W / S", "Forward / Backward", ("w", "s")),
                ("A / D", "Left / Right", ("a", "d")),
                ("R / F", "Up / Down", ("r", "f")),
                ("Q / E", "Yaw Left / Yaw Right", ("q", "e")),
            ],
            "Gimbal Control (Sim)": [
                ("I / K", "Pitch Up / Down", ("i", "k")),
                ("J / L", "Yaw Left / Right", ("j", "l")),
                ("U / O", "Roll Left / Right", ("u", "o")),
            ],
            "Speed Control": [
                ("Z / X", "Increase / Decrease Multiplier", ("z", "x")),
                ("C", "Reset Multiplier (1.0x)", ("c",)),
            ],
            "System Control": [
                ("ESC / Close Window", "Emergency Stop & Land", ("escape",))
            ],
        }
        row = 0
        for category, controls in control_categories.items():
            category_label = tk.Label(
                controls_frame,
                text=category,
                font=("Arial", 10, "bold"),
                bg="#f0f0f0",
                fg=self.title_color,
                anchor="w",
            )
            category_label.grid(
                row=row, column=0, columnspan=3, sticky="w", pady=(5, 0)
            )
            row += 1
            for keys_text, description, key_codes in controls:
                key_label = tk.Label(
                    controls_frame,
                    text=keys_text,
                    font=self.text_font,
                    bg="#e0e0e0",
                    width=18,
                    anchor="w",
                    relief=tk.GROOVE,
                    borderwidth=1,
                    padx=5,
                )
                key_label.grid(row=row, column=0, sticky="w", padx=2, pady=1)
                tk.Label(
                    controls_frame,
                    text=description,
                    font=self.text_font,
                    bg="#f0f0f0",
                    anchor="w",
                ).grid(row=row, column=1, columnspan=2, sticky="w", padx=5)
                for key_code in key_codes:
                    self.active_key_labels[key_code] = key_label
                row += 1
            tk.Label(controls_frame, text="", bg="#f0f0f0").grid(row=row, column=0)
            row += 1

    def create_status_section(self):
        """Create the status section with telemetry data"""
        status_frame = tk.LabelFrame(
            self.left_top,
            text="System Status",
            font=self.header_font,
            bg="#f0f0f0",
            fg=self.title_color,
            padx=10,
            pady=5,
        )
        status_frame.pack(fill="x", pady=5)

        def add_status_row(parent, label_text):
            row_frame = tk.Frame(parent, bg="#f0f0f0")
            row_frame.pack(fill="x", pady=2)
            tk.Label(
                row_frame,
                text=label_text,
                font=self.text_font,
                bg="#f0f0f0",
                width=15,
                anchor="w",
            ).pack(side=tk.LEFT)
            value_label = tk.Label(
                row_frame, text="N/A", font=self.text_font, bg="#f0f0f0", anchor="w"
            )
            value_label.pack(side=tk.LEFT, expand=True, fill="x")
            return value_label

        self.speed_label = add_status_row(status_frame, "Speed Multiplier:")
        self.velocity_label = add_status_row(status_frame, "Drone Velocity:")
        self.yaw_label = add_status_row(status_frame, "Yaw Speed:")
        self.gimbal_status_label = add_status_row(status_frame, "Gimbal Status:")
        self.gimbal_angle_label = add_status_row(status_frame, "Gimbal Angles:")
        self.altitude_label = add_status_row(status_frame, "Altitude (Rel):")
        self.attitude_label = add_status_row(status_frame, "Attitude:")

        # Set initial values
        self.speed_label.config(
            text=f"{self.state.speed_multiplier:.1f}x", fg="#0066CC"
        )
        self.velocity_label.config(text="F: +0.0, R: +0.0, D: +0.0")
        self.yaw_label.config(text="+0.0 deg/s")
        self.gimbal_status_label.config(
            text="Active (Simulated)", fg=self.status_active_color
        )
        self.gimbal_angle_label.config(
            text=f"P:{self.state.gimbal_pitch:+.1f}, R:{self.state.gimbal_roll:+.1f}, Y:{self.state.gimbal_yaw:+.1f}"
        )
        self.altitude_label.config(text="0.0 m")
        self.attitude_label.config(text="R: +0.0, P: +0.0, Y: +0.0")

    def create_visual_indicators(self):
        """Create visual indicators for drone and gimbal movement"""
        visual_frame = tk.LabelFrame(
            self.left_top,
            text="Visual Indicators",
            font=self.header_font,
            bg="#f0f0f0",
            fg=self.title_color,
            padx=10,
            pady=5,
        )
        visual_frame.pack(fill="both", expand=True, pady=5)

        # Movement indicator (arrows showing direction)
        movement_frame = tk.Frame(visual_frame, bg="#f0f0f0")
        movement_frame.pack(
            side=tk.LEFT, fill="both", expand=True, padx=5
        )  # Pack side by side
        tk.Label(
            movement_frame,
            text="Movement",
            font=self.text_font,
            bg="#f0f0f0",
            anchor="center",
        ).pack(anchor="n")
        self.movement_canvas = tk.Canvas(
            movement_frame,
            width=120,
            height=120,
            bg="white",
            highlightthickness=1,
            highlightbackground="#cccccc",
        )
        self.movement_canvas.pack(pady=5, expand=True)
        cx, cy = 60, 60  # Center
        self.movement_canvas.create_line(cx, 0, cx, 120, fill="#cccccc")  # Vertical
        self.movement_canvas.create_line(0, cy, 120, cy, fill="#cccccc")  # Horizontal
        self.movement_canvas.create_oval(
            cx - 3, cy - 3, cx + 3, cy + 3, fill="#555555", outline=""
        )  # Center dot
        self.movement_indicator = self.movement_canvas.create_polygon(
            cx, cy, cx, cy, cx, cy, fill="#4CAF50", outline="#4CAF50"
        )  # Placeholder triangle

        # Gimbal angle visualization
        gimbal_frame = tk.Frame(visual_frame, bg="#f0f0f0")
        gimbal_frame.pack(
            side=tk.RIGHT, fill="both", expand=True, padx=5
        )  # Pack side by side
        tk.Label(
            gimbal_frame,
            text="Gimbal Sim",
            font=self.text_font,
            bg="#f0f0f0",
            anchor="center",
        ).pack(anchor="n")
        self.gimbal_canvas = tk.Canvas(
            gimbal_frame,
            width=120,
            height=80,
            bg="white",
            highlightthickness=1,
            highlightbackground="#cccccc",
        )
        self.gimbal_canvas.pack(pady=5, expand=True)
        gx, gy_base = 60, 70  # Gimbal base center-bottom
        self.gimbal_canvas.create_rectangle(
            gx - 15, gy_base - 10, gx + 15, gy_base, fill="#aaaaaa", outline=""
        )  # Base rectangle
        self.gimbal_line = self.gimbal_canvas.create_line(
            gx, gy_base - 5, gx, gy_base - 45, fill="#0066CC", width=3, arrow=tk.LAST
        )  # Initial line (pitch only)

    def show_diagnostics(self):
        """Displays comprehensive diagnostic information for debugging camera display issues"""
        if (
            hasattr(self, "diagnostics_window")
            and self.diagnostics_window.winfo_exists()
        ):
            # If window already exists, just update its content and bring to front
            self.diagnostics_window.focus_force()
            self.update_diagnostics_info()
            return

        # Create new diagnostics window
        self.diagnostics_window = tk.Toplevel(self.root)
        self.diagnostics_window.title("Camera Display Diagnostics")
        self.diagnostics_window.geometry("700x600")
        self.diagnostics_window.configure(bg="#f0f0f0")

        # Add a title
        title_label = tk.Label(
            self.diagnostics_window,
            text="Camera Display Diagnostics",
            font=self.title_font,
            fg=self.title_color,
            bg="#f0f0f0",
            pady=10,
        )
        title_label.pack(fill="x")

        # Create a frame for the text content
        info_frame = tk.Frame(self.diagnostics_window, bg="#f0f0f0", padx=15, pady=10)
        info_frame.pack(fill="both", expand=True)

        # Create a scrollable text widget for diagnostic info
        self.diag_text = tk.Text(
            info_frame,
            wrap=tk.WORD,
            bg="white",
            font=("Courier", 10),
            padx=10,
            pady=10,
            height=25,
        )
        scrollbar = tk.Scrollbar(info_frame, command=self.diag_text.yview)
        self.diag_text.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.diag_text.pack(side=tk.LEFT, fill="both", expand=True)

        # Button frame
        button_frame = tk.Frame(self.diagnostics_window, bg="#f0f0f0", padx=15, pady=15)
        button_frame.pack(fill="x")

        # Add buttons for various diagnostic actions
        refresh_btn = tk.Button(
            button_frame,
            text="Refresh Information",
            command=self.update_diagnostics_info,
            bg="#3498DB",
            fg="white",
            font=self.button_font,
            padx=10,
            pady=5,
        )
        refresh_btn.pack(side=tk.LEFT, padx=10, fill="x", expand=True)

        update_btn = tk.Button(
            button_frame,
            text="Force Update Camera Views",
            command=self.update_camera_display,
            bg="#2ECC71",
            fg="white",
            font=self.button_font,
            padx=10,
            pady=5,
        )
        update_btn.pack(side=tk.LEFT, padx=10, fill="x", expand=True)

        close_btn = tk.Button(
            button_frame,
            text="Close Diagnostics",
            command=lambda: self.diagnostics_window.destroy(),
            bg="#E74C3C",
            fg="white",
            font=self.button_font,
            padx=10,
            pady=5,
        )
        close_btn.pack(side=tk.LEFT, padx=10, fill="x", expand=True)

        # Fill the diagnostic info
        self.update_diagnostics_info()

        # Handle window close event
        self.diagnostics_window.protocol(
            "WM_DELETE_WINDOW", lambda: self.diagnostics_window.destroy()
        )

    def update_diagnostics_info(self):
        """Updates the diagnostic information in the text widget"""
        if (
            not hasattr(self, "diagnostics_window")
            or not self.diagnostics_window.winfo_exists()
        ):
            return

        if not hasattr(self, "diag_text") or not self.diag_text.winfo_exists():
            return

        # Clear current text
        self.diag_text.delete(1.0, tk.END)

        # Get current time
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")

        # Gather system information
        system_info = [
            "==== DIAGNOSTIC INFORMATION ====",
            f"Time: {current_time}",
            "",
            "==== SYSTEM STATUS ====",
            f"Tkinter Version: {tk.TkVersion}",
            f"Python Version: {sys.version.split()[0]}",
            f"OpenCV Version: {cv2.__version__}",
            f"PIL/Pillow Available: {hasattr(PIL, '__version__')}",
            f"PIL Version: {getattr(PIL, '__version__', 'N/A')}",
            "",
            "==== UI COMPONENT STATUS ====",
            f"Root Window Exists: {self.root.winfo_exists()}",
            f"Root Window Size: {self.root.winfo_width()}x{self.root.winfo_height()}",
            f"Left Panel Size: {self.left_panel.winfo_width()}x{self.left_panel.winfo_height()}",
            f"Right Panel Size: {self.right_panel.winfo_width()}x{self.right_panel.winfo_height()}",
            "",
        ]

        # Camera info
        camera_info = [
            "==== CAMERA STATUS ====",
        ]

        for camera_name, camera_view in self.camera_views.items():
            camera_label = camera_view["label"]
            label_exists = camera_label.winfo_exists() if camera_label else False
            label_size = (
                f"{camera_label.winfo_width()}x{camera_label.winfo_height()}"
                if label_exists
                else "N/A"
            )

            last_img = camera_view["last_image"]
            img_shape = (
                str(last_img.shape)
                if last_img is not None and hasattr(last_img, "shape")
                else "None"
            )
            update_count = camera_view["update_count"]
            last_update = (
                time.time() - camera_view["last_frame_time"]
                if camera_view["last_frame_time"]
                else "Never"
            )
            last_update_str = (
                f"{last_update:.2f} seconds ago"
                if isinstance(last_update, float)
                else last_update
            )

            camera_info.extend(
                [
                    f"Camera: {camera_name}",
                    f"  - Label Exists: {label_exists}",
                    f"  - Label Size: {label_size}",
                    f"  - Last Image: {img_shape}",
                    f"  - Update Count: {update_count}",
                    f"  - Last Update: {last_update_str}",
                    "",
                ]
            )

        # Performance metrics
        performance_info = []
        if hasattr(self, "airsim_client") and self.airsim_client:
            stats = self.airsim_client.get_performance_stats()
            performance_info = [
                "==== PERFORMANCE METRICS ====",
                f"Average Fetch Time: {stats.get('avg_fetch_time_ms', 'N/A'):.2f} ms",
                f"Average Process Time: {stats.get('avg_process_time_ms', 'N/A'):.2f} ms",
                f"Raw Queue Size: {stats.get('raw_queue_size', 'N/A')}",
                f"Display Queue Size: {stats.get('display_queue_size', 'N/A')}",
                f"Potential Display FPS: {stats.get('potential_display_fps', 'N/A'):.1f}",
                "",
            ]

        # Control state info
        control_info = [
            "==== CONTROL STATE ====",
            f"Speed Multiplier: {self.state.speed_multiplier:.1f}x",
            f"Velocity: Forward={self.state.velocity_forward:.2f}, Right={self.state.velocity_right:.2f}, Down={self.state.velocity_down:.2f}",
            f"Yaw Speed: {self.state.yawspeed:.2f} deg/s",
            f"Gimbal Angles: Pitch={self.state.gimbal_pitch:.2f}, Roll={self.state.gimbal_roll:.2f}, Yaw={self.state.gimbal_yaw:.2f}",
            f"Active Keys: {', '.join(self.state.pressed_keys) if self.state.pressed_keys else 'None'}",
            f"Exit Flag: {self.state.exit_flag}",
        ]

        # Thread status
        thread_info = [
            "",
            "==== THREAD STATUS ====",
        ]

        for t in threading.enumerate():
            thread_info.append(
                f"Thread: {t.name}, daemon={t.daemon}, alive={t.is_alive()}"
            )

        # Combine all sections
        all_info = (
            system_info + camera_info + performance_info + control_info + thread_info
        )

        # Insert into text widget
        self.diag_text.insert(tk.END, "\n".join(all_info))

        # Scroll to top
        self.diag_text.see("1.0")

    def create_control_buttons(self):
        """Create control buttons for common actions"""
        button_frame = tk.Frame(self.left_bottom, bg="#f0f0f0")
        button_frame.pack(fill="x", pady=5)

        self.stop_button = tk.Button(
            button_frame,
            text="STOP & LAND (ESC)",
            command=self.exit_control,
            bg="#E74C3C",
            fg="white",
            font=self.button_font,
            padx=10,
            pady=5,
            relief=tk.RAISED,
            borderwidth=2,
        )
        self.stop_button.pack(side=tk.LEFT, padx=5, fill="x", expand=True)

        self.reset_speed_button = tk.Button(
            button_frame,
            text="Reset Speed (C)",
            command=self.reset_speed,
            bg="#3498DB",
            fg="white",
            font=self.button_font,
            padx=10,
            pady=5,
        )
        self.reset_speed_button.pack(side=tk.LEFT, padx=5, fill="x", expand=True)

        self.center_gimbal_button = tk.Button(
            button_frame,
            text="Center Gimbal",
            command=self.center_gimbal_cmd,
            bg="#2ECC71",
            fg="white",
            font=self.button_font,
            padx=10,
            pady=5,
        )
        self.center_gimbal_button.pack(side=tk.LEFT, padx=5, fill="x", expand=True)

        self.diag_button = tk.Button(
            button_frame,
            text="Diagnose",
            command=self.show_diagnostics,
            bg="#9B59B6",
            fg="white",
            font=self.button_font,
            padx=10,
            pady=5,
        )
        self.diag_button.pack(side=tk.LEFT, padx=5, fill="x", expand=True)

    def on_key_press(self, event):
        """Handle key press events"""
        if not self.root.winfo_exists():
            return
        key = event.keysym.lower()
        is_special_key = False

        if key == "escape":
            self.exit_control()
            return
        elif key == "z":
            self.increase_speed()
            is_special_key = True
        elif key == "x":
            self.decrease_speed()
            is_special_key = True
        elif key == "c":
            self.reset_speed()
            is_special_key = True

        if not is_special_key and key in self.active_key_labels:
            self.state.pressed_keys.add(key)
            # Update highlighting immediately
            self.active_key_labels[key].config(
                bg="#FFD700", relief=tk.SUNKEN
            )  # Highlight in gold, sunken

    def on_key_release(self, event):
        """Handle key release events"""
        if not self.root.winfo_exists():
            return
        key = event.keysym.lower()
        if key in self.state.pressed_keys:
            self.state.pressed_keys.remove(key)

        # Restore key highlighting only if it exists in our map
        if key in self.active_key_labels:
            self.active_key_labels[key].config(
                bg="#e0e0e0", relief=tk.GROOVE
            )  # Restore background, groove relief

    def center_gimbal_state(self):
        """Logic to center gimbal state variables."""
        self.state.gimbal_pitch = self.config.initial_pitch
        self.state.gimbal_yaw = 0.0
        self.state.gimbal_roll = 0.0
        # Also reset rates
        self.state.gimbal_pitch_rate = 0.0
        self.state.gimbal_yaw_rate = 0.0
        self.state.gimbal_roll_rate = 0.0
        LOGGER.info("Gimbal state centered.")

        # Notify via event bus that gimbal has been centered
        self.event_bus.publish(
            "gimbal_centered",
            {
                "pitch": self.state.gimbal_pitch,
                "roll": self.state.gimbal_roll,
                "yaw": self.state.gimbal_yaw,
            },
        )

    def center_gimbal_cmd(self):
        """Command to center the gimbal, updates state and UI."""
        self.center_gimbal_state()
        # Update UI immediately
        self.update_gimbal_indicator()
        self.gimbal_angle_label.config(
            text=f"P:{self.state.gimbal_pitch:+.1f}, R:{self.state.gimbal_roll:+.1f}, Y:{self.state.gimbal_yaw:+.1f}"
        )
        self.update_status_message("Gimbal centered.")

    def process_keys(self):
        """Process keyboard inputs held down and update control state. Called periodically."""
        if not self.root.winfo_exists() or self.state.exit_flag:
            return

        cfg = self.config
        state = self.state
        keys = state.pressed_keys  # Get currently pressed keys
        speed_mult = state.speed_multiplier
        accel = 1.0 - cfg.decay_factor  # Amount to move towards target per cycle
        decay = cfg.decay_factor  # Amount to retain from previous velocity per cycle

        # --- Target Velocities based on currently pressed keys ---
        target_forward = 0.0
        if "w" in keys:
            target_forward = cfg.max_speed * speed_mult
        elif "s" in keys:
            target_forward = -cfg.max_speed * speed_mult

        target_right = 0.0
        if "d" in keys:
            target_right = cfg.max_speed * speed_mult
        elif "a" in keys:
            target_right = -cfg.max_speed * speed_mult

        target_down = 0.0
        if "f" in keys:
            target_down = cfg.max_speed * speed_mult  # F = Down
        elif "r" in keys:
            target_down = -cfg.max_speed * speed_mult  # R = Up

        target_yawspeed = 0.0
        drone_yaw_command = None
        if "e" in keys:
            target_yawspeed = cfg.max_yaw_speed * speed_mult  # Clockwise
            drone_yaw_command = "right"
        elif "q" in keys:
            target_yawspeed = -cfg.max_yaw_speed * speed_mult  # Counter-Clockwise
            drone_yaw_command = "left"

        # --- Apply Smoothing  ---
        state.velocity_forward = state.velocity_forward * decay + target_forward * accel
        state.velocity_right = state.velocity_right * decay + target_right * accel
        state.velocity_down = state.velocity_down * decay + target_down * accel
        state.yawspeed = state.yawspeed * decay + target_yawspeed * accel

        # --- Apply Zero Threshold ---
        if target_forward == 0.0 and abs(state.velocity_forward) < cfg.zero_threshold:
            state.velocity_forward = 0.0
        if target_right == 0.0 and abs(state.velocity_right) < cfg.zero_threshold:
            state.velocity_right = 0.0
        if target_down == 0.0 and abs(state.velocity_down) < cfg.zero_threshold:
            state.velocity_down = 0.0
        if target_yawspeed == 0.0 and abs(state.yawspeed) < cfg.zero_threshold:
            state.yawspeed = 0.0

        # --- Gimbal Control ---
        pitch_change = 0.0
        yaw_change = 0.0
        roll_change = 0.0
        gimbal_increment = (
            cfg.gimbal_angle_increment * speed_mult
        )  # Scale gimbal speed too

        if "i" in keys:
            pitch_change = gimbal_increment  # Pitch Up
        elif "k" in keys:
            pitch_change = -gimbal_increment  # Pitch Down

        if drone_yaw_command == "right" or "l" in keys:
            yaw_change = gimbal_increment  # Yaw Right
        elif drone_yaw_command == "left" or "j" in keys:
            yaw_change = -gimbal_increment  # Yaw Left

        if "o" in keys:
            roll_change = gimbal_increment  # Roll Right
        elif "u" in keys:
            roll_change = -gimbal_increment  # Roll Left

        # Update angles directly
        state.gimbal_pitch += pitch_change
        state.gimbal_yaw += yaw_change
        state.gimbal_roll += roll_change

        # Clamp angles to limits
        state.gimbal_pitch = max(
            cfg.min_pitch_deg, min(cfg.max_pitch_deg, state.gimbal_pitch)
        )
        state.gimbal_roll = max(
            cfg.min_roll_deg, min(cfg.max_roll_deg, state.gimbal_roll)
        )
        # Wrap yaw angle between -180 and 180
        state.gimbal_yaw = (state.gimbal_yaw + 180.0) % 360.0 - 180.0

        gimbal_changed = pitch_change != 0.0 or yaw_change != 0.0 or roll_change != 0.0
        if gimbal_changed:
            LOGGER.debug(
                f"Gimbal changed to P:{state.gimbal_pitch:.1f}, R:{state.gimbal_roll:.1f}, Y:{state.gimbal_yaw:.1f}"
            )

            self.event_bus.publish(
                "gimbal_update",
                {
                    "pitch": state.gimbal_pitch,
                    "roll": state.gimbal_roll,
                    "yaw": state.gimbal_yaw,
                    "vehicle_name": self.config.connection.vehicle_name,
                    "camera_name": self.config.camera.camera_name,
                },
            )

        target_gimbal_pitch_rate = 0.0
        if "i" in keys:
            target_gimbal_pitch_rate = cfg.max_gimbal_rate
        elif "k" in keys:
            target_gimbal_pitch_rate = -cfg.max_gimbal_rate

        target_gimbal_yaw_rate = 0.0
        if "l" in keys:
            target_gimbal_yaw_rate = cfg.max_gimbal_rate
        elif "j" in keys:
            target_gimbal_yaw_rate = -cfg.max_gimbal_rate

        target_gimbal_roll_rate = 0.0
        if "o" in keys:
            target_gimbal_roll_rate = cfg.max_gimbal_rate
        elif "u" in keys:
            target_gimbal_roll_rate = -cfg.max_gimbal_rate

        # Apply smoothing to rates
        state.gimbal_pitch_rate = (
            state.gimbal_pitch_rate * decay + target_gimbal_pitch_rate * accel
        )
        state.gimbal_yaw_rate = (
            state.gimbal_yaw_rate * decay + target_gimbal_yaw_rate * accel
        )
        state.gimbal_roll_rate = (
            state.gimbal_roll_rate * decay + target_gimbal_roll_rate * accel
        )

        # Zero threshold for rates
        if (
            target_gimbal_pitch_rate == 0.0
            and abs(state.gimbal_pitch_rate) < cfg.zero_threshold
        ):
            state.gimbal_pitch_rate = 0.0
        if (
            target_gimbal_yaw_rate == 0.0
            and abs(state.gimbal_yaw_rate) < cfg.zero_threshold
        ):
            state.gimbal_yaw_rate = 0.0
        if (
            target_gimbal_roll_rate == 0.0
            and abs(state.gimbal_roll_rate) < cfg.zero_threshold
        ):
            state.gimbal_roll_rate = 0.0

        # --- Visual Indicators ---
        if hasattr(self, "update_movement_indicator"):
            self.update_movement_indicator()
        if hasattr(self, "update_gimbal_indicator"):
            self.update_gimbal_indicator()

    def update_movement_indicator(self):
        """Update the movement direction indicator on canvas"""
        if (
            not hasattr(self, "movement_canvas")
            or not self.movement_canvas.winfo_exists()
        ):
            return

        # Use current state velocities
        max_vel = self.config.max_speed  # Base max speed
        # Normalize based on potential max speed (ignoring multiplier for indicator range)
        forward_norm = np.clip(self.state.velocity_forward / (max_vel + 0.01), -1, 1)
        right_norm = np.clip(self.state.velocity_right / (max_vel + 0.01), -1, 1)

        cx, cy = 60, 60  # Canvas center
        scale = 45  # Max distance from center

        indicator_x = cx + right_norm * scale
        indicator_y = cy - forward_norm * scale  # Y is inverted

        # Update movement indicator (arrow shape)
        if abs(forward_norm) > 0.05 or abs(right_norm) > 0.05:
            angle_rad = np.arctan2(-forward_norm, right_norm)  # Angle of movement
            arrow_len = 15
            arrow_width = 10

            # Tip of the arrow
            x_tip = indicator_x
            y_tip = indicator_y

            # Base points (perpendicular to the direction vector)
            angle_left = angle_rad + np.pi / 2
            angle_right = angle_rad - np.pi / 2
            base_offset = arrow_len * 0.7  # Move base back from tip

            x_base_mid = x_tip - base_offset * np.cos(angle_rad)
            y_base_mid = y_tip - base_offset * np.sin(angle_rad)

            x_base_l = x_base_mid + arrow_width / 2 * np.cos(angle_left)
            y_base_l = y_base_mid + arrow_width / 2 * np.sin(angle_left)
            x_base_r = x_base_mid + arrow_width / 2 * np.cos(angle_right)
            y_base_r = y_base_mid + arrow_width / 2 * np.sin(angle_right)

            self.movement_canvas.coords(
                self.movement_indicator,
                x_tip,
                y_tip,
                x_base_l,
                y_base_l,
                x_base_r,
                y_base_r,
            )

            # Color based on speed multiplier or magnitude
            speed_magnitude = np.sqrt(
                self.state.velocity_forward**2 + self.state.velocity_right**2
            )
            norm_speed_mag = speed_magnitude / (max_vel + 0.01)

            if norm_speed_mag > 0.7:
                fill_color = "#E74C3C"  # Red
            elif norm_speed_mag > 0.3:
                fill_color = "#F39C12"  # Orange
            else:
                fill_color = "#2ECC71"  # Green

            self.movement_canvas.itemconfig(
                self.movement_indicator, fill=fill_color, outline=fill_color
            )
        else:
            self.movement_canvas.coords(self.movement_indicator, cx, cy, cx, cy, cx, cy)
            self.movement_canvas.itemconfig(
                self.movement_indicator, fill="", outline=""
            )

    def update_gimbal_indicator(self):
        """Update the gimbal orientation indicator on canvas"""
        if not hasattr(self, "gimbal_canvas") or not self.gimbal_canvas.winfo_exists():
            return

        pitch_rad = np.radians(self.state.gimbal_pitch)
        yaw_rad = np.radians(self.state.gimbal_yaw)

        gx, gy_base = 60, 70
        length = 40
        end_x = gx + length * np.sin(yaw_rad) * 0.5
        end_y = gy_base - length * np.sin(pitch_rad)

        self.gimbal_canvas.coords(self.gimbal_line, gx, gy_base - 5, end_x, end_y)

        is_moving = (
            abs(self.state.gimbal_pitch_rate) > 1.0
            or abs(self.state.gimbal_yaw_rate) > 1.0
            or abs(self.state.gimbal_roll_rate) > 1.0
        )

        fill_color = "#E74C3C" if is_moving else "#0066CC"
        line_width = 4 if is_moving else 3
        self.gimbal_canvas.itemconfig(
            self.gimbal_line, fill=fill_color, width=line_width
        )

    def update_gui_loop(self):
        """Periodic GUI update loop. Calls process_keys and updates labels."""
        if not self.root.winfo_exists() or self.state.exit_flag:
            LOGGER.info("GUI update loop stopping.")
            return

        current_time = time.monotonic()

        self.process_keys()

        if current_time - self.last_telemetry_update_time >= 0.1:
            self.last_telemetry_update_time = current_time

            self.speed_label.config(text=f"{self.state.speed_multiplier:.1f}x")
            self.velocity_label.config(
                text=f"F:{self.state.velocity_forward:+.1f}, R:{self.state.velocity_right:+.1f}, D:{self.state.velocity_down:+.1f} m/s"
            )
            self.yaw_label.config(text=f"{self.state.yawspeed:+.1f} deg/s")
            self.gimbal_angle_label.config(
                text=f"P:{self.state.gimbal_pitch:+.1f}, R:{self.state.gimbal_roll:+.1f}, Y:{self.state.gimbal_yaw:+.1f}"
            )

        self._update_gui_loop_id = self.root.after(20, self.update_gui_loop)

    def increase_speed(self):
        """Increase the speed multiplier"""
        self.state.speed_multiplier = min(3.0, self.state.speed_multiplier + 0.1)
        self.speed_label.config(text=f"{self.state.speed_multiplier:.1f}x")
        self.update_status_message(
            f"Speed multiplier: {self.state.speed_multiplier:.1f}x"
        )
        self.event_bus.publish("speed_changed", self.state.speed_multiplier)

    def decrease_speed(self):
        """Decrease the speed multiplier"""
        self.state.speed_multiplier = max(0.1, self.state.speed_multiplier - 0.1)
        self.speed_label.config(text=f"{self.state.speed_multiplier:.1f}x")
        self.update_status_message(
            f"Speed multiplier: {self.state.speed_multiplier:.1f}x"
        )
        self.event_bus.publish("speed_changed", self.state.speed_multiplier)

    def reset_speed(self):
        """Reset the speed multiplier to default"""
        self.state.speed_multiplier = 1.0
        self.speed_label.config(text=f"{self.state.speed_multiplier:.1f}x")
        self.update_status_message("Speed multiplier reset to 1.0x")
        self.event_bus.publish("speed_changed", self.state.speed_multiplier)

    def exit_control(self):
        """Initiates the shutdown sequence from the GUI."""
        if self.state.exit_flag:
            return

        LOGGER.info("Exit requested via GUI (Close button or ESC).")
        self.update_status_message("Exit requested. Initiating landing...", "warning")

        self.conn_status.config(text="● DISCONNECTING", fg=self.status_warning_color)
        self.stop_button.config(text="EXITING...", state=tk.DISABLED)
        self.reset_speed_button.config(state=tk.DISABLED)
        self.center_gimbal_button.config(state=tk.DISABLED)

        if hasattr(self, "_update_gui_loop_id"):
            try:
                self.root.after_cancel(self._update_gui_loop_id)
            except Exception as e:
                LOGGER.error(f"Error cancelling GUI update loop: {e}")

        self.state.exit_flag = True
        self.event_bus.publish("exit_requested", "User requested exit via GUI")

        if self.root.winfo_exists():
            try:
                self.root.update_idletasks()
                self.root.after(500, self._delayed_destroy)
            except Exception as e:
                LOGGER.error(f"Error updating GUI before exit: {e}")
                self._delayed_destroy()

    def _delayed_destroy(self):
        """Safely destroys the Tkinter window."""
        if self.root.winfo_exists():

            LOGGER.info("Closing GUI window.")
            try:
                for widget in self.root.winfo_children():
                    if hasattr(widget, "destroy"):
                        widget.destroy()

                self.root.quit()
                self.root.destroy()
            except Exception as e:
                LOGGER.error(f"Error quitting GUI: {e}")


# =============================================================================
# APPLICATION COORDINATOR
# =============================================================================


class DroneApplicationCoordinator:
    """
    Main application coordinator that handles component lifecycle and communications
    """

    def __init__(self, config: DroneConfig):
        self.config = config
        self.state = ControlState()
        self.event_bus = EventBus()

        # Set initial gimbal position from config
        self.state.gimbal_pitch = config.initial_pitch
        self.state.gimbal_roll = 0.0
        self.state.gimbal_yaw = 0.0
        self.state.speed_multiplier = 1.0

        # Component references
        self.airsim_client = None
        self.drone_controller = None
        self.gui_thread = None

        # Subscribe to important events
        self.event_bus.subscribe("exit_requested", self._handle_exit)
        self.event_bus.subscribe("gimbal_update", self._handle_gimbal_update)

    def _handle_exit(self, data):
        """Handle exit request from any component"""
        LOGGER.info(f"Application exit requested: {data}")
        self.state.exit_flag = True

    def _handle_gimbal_update(self, pose_data):
        """Handle gimbal pose updates and forward to camera pose queue"""
        if self.airsim_client:
            try:
                camera_pose_queue = self.event_bus.get_queue("camera_pose_queue")
                if camera_pose_queue and not camera_pose_queue.full():
                    camera_pose_queue.put_nowait(pose_data)
            except Exception as e:
                LOGGER.error(f"Error handling gimbal update: {e}")

    def check_hardware_acceleration(self):
        """檢查並啟用可用的硬體加速選項"""
        LOGGER.info("檢查系統硬體加速功能...")
        acceleration_status = {
            "CUDA": False,
            "OpenCL": False,
            "IPP": False,
            "GPU_Info": "未檢測到",
        }

        # 檢查 OpenCV 版本
        LOGGER.info(f"OpenCV 版本: {cv2.__version__}")

        # 設置 OpenCV 優化
        if hasattr(cv2, "setUseOptimized"):
            cv2.setUseOptimized(True)
            is_optimized = cv2.useOptimized()
            LOGGER.info(f"OpenCV 優化已啟用: {is_optimized}")

        # 1. 檢查並啟用 CUDA
        try:
            if hasattr(cv2, "cuda") and hasattr(cv2.cuda, "getCudaEnabledDeviceCount"):
                device_count = cv2.cuda.getCudaEnabledDeviceCount()
                if device_count > 0:
                    try:
                        # 嘗試設置 CUDA 設備
                        cv2.cuda.setDevice(0)

                        # 測試 CUDA 功能
                        test_array = np.zeros((10, 10, 3), dtype=np.uint8)
                        gpu_mat = cv2.cuda.GpuMat()
                        gpu_mat.upload(test_array)

                        acceleration_status["CUDA"] = True
                        acceleration_status["GPU_Info"] = f"CUDA 設備 x{device_count}"
                        LOGGER.info(
                            f"✓ CUDA 加速已啟用 (檢測到 {device_count} 個 CUDA 設備)"
                        )
                    except Exception as e:
                        LOGGER.warning(f"! CUDA 啟用失敗: {e}")
                else:
                    LOGGER.info("✗ 未檢測到 CUDA 設備")
            else:
                LOGGER.info("✗ 此版本的 OpenCV 不支持 CUDA")
        except Exception as e:
            LOGGER.warning(f"! CUDA 檢測錯誤: {e}")

        # 2. 檢查並啟用 OpenCL
        if not acceleration_status["CUDA"]:  # 如果 CUDA 不可用，嘗試 OpenCL
            try:
                if hasattr(cv2, "ocl") and hasattr(cv2.ocl, "haveOpenCL"):
                    if cv2.ocl.haveOpenCL():
                        cv2.ocl.setUseOpenCL(True)
                        if cv2.ocl.useOpenCL():
                            acceleration_status["OpenCL"] = True
                            LOGGER.info("✓ OpenCL 加速已啟用")

                            # 獲取 OpenCL 設備信息
                            try:
                                ocl_info = cv2.ocl.Device.getDefault().name()
                                acceleration_status["GPU_Info"] = f"OpenCL: {ocl_info}"
                                LOGGER.info(f"  OpenCL 設備: {ocl_info}")
                            except Exception:
                                pass
                        else:
                            LOGGER.info("✗ OpenCL 可用但無法啟用")
                    else:
                        LOGGER.info("✗ OpenCL 不可用")
                else:
                    LOGGER.info("✗ 此版本的 OpenCV 不支持 OpenCL")
            except Exception as e:
                LOGGER.warning(f"! OpenCL 檢測錯誤: {e}")

        # 3. 檢查並啟用 IPP (Intel Integrated Performance Primitives)
        try:
            if hasattr(cv2, "ipp"):
                cv2.ipp.setUseIPP(True)
                if cv2.ipp.useIPP():
                    acceleration_status["IPP"] = True
                    LOGGER.info("✓ IPP 加速已啟用")
                else:
                    LOGGER.info("✗ IPP 可用但無法啟用")
            else:
                LOGGER.info("✗ 此版本的 OpenCV 不支持 IPP")
        except Exception as e:
            LOGGER.warning(f"! IPP 檢測錯誤: {e}")

        # 4. 嘗試檢測 GPU 型號信息
        try:
            import subprocess

            gpu_info = None

            # 嘗試檢測 NVIDIA GPU
            try:
                nvidia_info = (
                    subprocess.check_output(
                        "nvidia-smi --query-gpu=name --format=csv,noheader",
                        shell=True,
                        stderr=subprocess.DEVNULL,
                    )
                    .decode()
                    .strip()
                )
                if nvidia_info:
                    gpu_info = f"NVIDIA: {nvidia_info}"
                    LOGGER.info(f"檢測到 NVIDIA GPU: {nvidia_info}")
            except subprocess.CalledProcessError:
                pass

            # 如果沒有 NVIDIA GPU，嘗試檢測其他 GPU
            if not gpu_info:
                try:
                    if os.name == "posix":  # Linux/Mac
                        gpu_info_raw = (
                            subprocess.check_output(
                                "lspci | grep -i vga",
                                shell=True,
                                stderr=subprocess.DEVNULL,
                            )
                            .decode()
                            .strip()
                        )
                        if "nvidia" in gpu_info_raw.lower():
                            gpu_info = f"NVIDIA (驅動未加載): {gpu_info_raw}"
                        elif (
                            "amd" in gpu_info_raw.lower()
                            or "radeon" in gpu_info_raw.lower()
                        ):
                            gpu_info = f"AMD: {gpu_info_raw}"
                        elif "intel" in gpu_info_raw.lower():
                            gpu_info = f"Intel: {gpu_info_raw}"
                        else:
                            gpu_info = gpu_info_raw
                    if os.name == "nt":  # Windows
                        gpu_info_raw = (
                            subprocess.check_output(
                                "wmic path win32_VideoController get name",
                                shell=True,
                                stderr=subprocess.DEVNULL,
                            )
                            .decode()
                            .strip()
                        )
                        if gpu_info_raw:
                            lines = gpu_info_raw.split("\n")
                            if len(lines) > 1:
                                gpu_info = lines[1].strip()  # 第一行是標題
                except Exception:
                    pass

            if gpu_info and acceleration_status["GPU_Info"] == "未檢測到":
                acceleration_status["GPU_Info"] = gpu_info

        except Exception as e:
            LOGGER.warning(f"無法檢測 GPU 型號: {e}")

        # 5. 檢查 OpenMP 支持
        try:
            if hasattr(cv2, "setNumThreads"):
                # 獲取可用 CPU 核心數
                import multiprocessing

                num_cores = multiprocessing.cpu_count()

                # 設置 OpenCV 線程數
                cv2.setNumThreads(num_cores)
                LOGGER.info(f"OpenCV 線程數已設置為 {num_cores} (CPU 核心數)")
        except Exception as e:
            LOGGER.warning(f"設置 OpenCV 線程數失敗: {e}")

        # 6. 顯示最終加速狀態摘要
        active_acceleration = [
            name
            for name, status in acceleration_status.items()
            if status and name != "GPU_Info"
        ]

        if active_acceleration:
            LOGGER.info(f"已啟用硬體加速: {', '.join(active_acceleration)}")
            if acceleration_status["GPU_Info"] != "未檢測到":
                LOGGER.info(f"GPU 信息: {acceleration_status['GPU_Info']}")
        else:
            LOGGER.warning("沒有啟用任何硬體加速功能!")

        # 檢查是否有 GPU 但沒有啟用 CUDA/OpenCL
        if (
            not acceleration_status["CUDA"]
            and not acceleration_status["OpenCL"]
            and acceleration_status["GPU_Info"] != "未檢測到"
            and "nvidia" in acceleration_status["GPU_Info"].lower()
        ):
            LOGGER.warning(
                "檢測到 NVIDIA GPU 但 CUDA 未啟用! 可能需要安裝 opencv-contrib-python-gpu"
            )

        # 返回第一個啟用的加速類型，或 None
        for accel_type in ["CUDA", "OpenCL", "IPP"]:
            if acceleration_status[accel_type]:
                return accel_type
        return "None"

    async def initialize_components(self):
        """Initialize all system components"""
        # Initialize AirSim client for camera view
        LOGGER.info("檢查硬體加速...")
        accel_type = self.check_hardware_acceleration()
        LOGGER.info(f"硬體加速狀態: {accel_type}")

        LOGGER.info("Initializing AirSim connection...")
        self.airsim_client = SimulationDroneClient(
            self.config, self.state, self.event_bus
        )
        airsim_connected = self.airsim_client.connect()

        if airsim_connected:
            self.airsim_client.start_image_fetcher()
            self.airsim_client.start_display_thread()
            self.airsim_client.start_camera_pose_thread()
            LOGGER.info("AirSim client connected and threads started.")
        else:
            LOGGER.warning(
                "Failed to connect to AirSim. Proceeding without camera view."
            )
            self.airsim_client = None

        # Initialize drone controller
        LOGGER.info("Initializing Drone connection...")
        self.drone_controller = DroneController(self.config, self.event_bus)
        if not await self.drone_controller.connect_drone():
            LOGGER.error("Failed to connect to drone. Exiting.")
            self.state.exit_flag = True
            return False

        # Start GUI Thread
        LOGGER.info("Starting GUI thread...")
        success = self.start_gui()
        if not success:
            LOGGER.error("Failed to start GUI. Cannot control drone.")
            self.state.exit_flag = True
            if self.drone_controller:
                await self.drone_controller.land_drone()
            return False

        return True

    def start_gui(self):
        """Start the GUI in a separate thread"""
        gui_ready = threading.Event()
        gui_error = threading.Event()

        def gui_runner():
            try:
                root = tk.Tk()
                DroneControlGUI(root, self.config, self.state, self.event_bus)
                gui_ready.set()
                root.mainloop()
            except Exception as e:
                LOGGER.error(f"Error in GUI thread: {e}", exc_info=True)
                gui_error.set()
                self.state.exit_flag = True
            finally:
                LOGGER.info("GUI thread finished.")
                self.state.exit_flag = True

        self.gui_thread = threading.Thread(
            target=gui_runner, name="GUIThread", daemon=True
        )
        self.gui_thread.start()

        if gui_error.is_set():
            LOGGER.error("GUI thread failed to initialize immediately.")
            return False

        if not gui_ready.wait(timeout=5.0):
            LOGGER.error("GUI thread timed out during initialization.")
            self.state.exit_flag = True
            return False

        LOGGER.info("GUI thread started successfully.")
        return True

    async def pre_flight_sequence(self):
        """Perform pre-flight checks and takeoff"""
        if self.state.exit_flag:
            return False

        LOGGER.info("Performing drone pre-flight checks...")
        if not await self.drone_controller.check_drone_health():
            LOGGER.error("Drone health check failed. Exiting.")
            self.state.exit_flag = True
            return False

        LOGGER.info("Arming drone...")
        if not await self.drone_controller.arm_drone():
            LOGGER.error("Failed to arm drone. Exiting.")
            self.state.exit_flag = True
            return False

        LOGGER.info("Taking off...")
        if not await self.drone_controller.takeoff_drone():
            LOGGER.error("Failed to take off. Attempting to land.")
            self.state.exit_flag = True
            await self.drone_controller.land_drone()
            return False

        LOGGER.info("Drone airborne and ready for control.")
        return True

    async def run(self):
        """Main application execution flow"""
        try:
            # Initialize components
            if not await self.initialize_components():
                return 1

            # Pre-flight and takeoff
            if not await self.pre_flight_sequence():
                return 1

            # Start main control loop
            LOGGER.info("Starting main flight control loop...")
            await self.drone_controller.run_manual_control_loop(self.state)
            LOGGER.info("Main control loop finished.")

            return 0

        except asyncio.CancelledError:
            LOGGER.warning("Application execution cancelled.")
            self.state.exit_flag = True
            return 1
        except Exception as e:
            LOGGER.error(f"Unhandled error in application: {e}", exc_info=True)
            self.state.exit_flag = True
            return 1
        finally:
            await self.cleanup()

    async def cleanup(self):
        """Clean up all resources before exit"""
        LOGGER.info("--- Starting Main Cleanup Sequence ---")

        cleanup_tasks = []

        async def stop_airsim():
            if self.airsim_client:
                LOGGER.info("Stopping AirSim client threads...")
                with concurrent.futures.ThreadPoolExecutor() as executor:
                    try:
                        await asyncio.wait_for(
                            asyncio.get_event_loop().run_in_executor(
                                executor, self.airsim_client.stop_threads
                            ),
                            timeout=3.0,
                        )
                        LOGGER.info("AirSim client threads stopped.")
                    except asyncio.TimeoutError:
                        LOGGER.warning("Timeout stopping AirSim threads.")
                    except Exception as e:
                        LOGGER.error(f"Error stopping AirSim threads: {e}")

        async def stop_drone():
            if self.drone_controller:
                LOGGER.info("Ensuring drone is landed...")
                try:
                    await asyncio.wait_for(
                        self.drone_controller.land_drone(), timeout=30.0
                    )
                    LOGGER.info("Landing sequence complete.")
                except asyncio.TimeoutError:
                    LOGGER.warning("Drone landing timed out.")
                except Exception as e:
                    LOGGER.error(f"Error landing drone: {e}")

        cleanup_tasks.append(asyncio.create_task(stop_airsim()))
        cleanup_tasks.append(asyncio.create_task(stop_drone()))

        try:
            await asyncio.wait_for(
                asyncio.gather(*cleanup_tasks, return_exceptions=True), timeout=15.0
            )
        except asyncio.TimeoutError:
            LOGGER.warning("Cleanup tasks timed out.")

        if self.gui_thread and self.gui_thread.is_alive():
            LOGGER.info("Waiting for GUI thread to finish...")
            os._exit(0)
            try:
                self.state.exit_flag = True
                self.event_bus.publish("exit_requested", "Final cleanup")

                self.gui_thread.join(timeout=3.0)
                if self.gui_thread.is_alive():
                    LOGGER.warning("GUI thread did not exit cleanly.")
                else:
                    LOGGER.info("GUI thread exited.")
            except Exception as e:
                LOGGER.error(f"Error waiting for GUI thread: {e}")

        try:
            cv2.destroyAllWindows()
            for _ in range(3):
                cv2.waitKey(1)
                cv2.destroyAllWindows()
        except Exception:
            pass

        LOGGER.info("=== Main Cleanup Finished ===")


# =============================================================================
# COMMAND LINE PARSER AND ENTRY POINT
# =============================================================================


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="AirSim Drone Control System")

    # Connection parameters
    parser.add_argument(
        "--ip", type=str, default="172.19.160.1", help="IP address of AirSim server"
    )
    parser.add_argument(
        "--mavsdk-port",
        type=int,
        default=14540,
        help="UDP port for MAVSDK connection (e.g., 14540 for SITL)",
    )
    parser.add_argument(
        "--vehicle", type=str, default="PX4", help="Vehicle name in AirSim"
    )

    # Camera parameters
    parser.add_argument(
        "--camera", type=str, default="front_center", help="Camera name to use"
    )
    parser.add_argument(
        "--width",
        type=int,
        default=320,
        help="Request specific image width (0 for default)",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=240,
        help="Request specific image height (0 for default)",
    )

    # Display parameters
    parser.add_argument(
        "--no-resize",
        action="store_true",
        default=False,
        help="Disable resizing of AirSim image output",
    )
    parser.add_argument(
        "--framerate", type=float, default=60.0, help="Target framerate in Hz (10-240)"
    )
    parser.add_argument(
        "--output-width",
        type=int,
        default=640,
        help="Output display width when resizing",
    )
    parser.add_argument(
        "--output-height",
        type=int,
        default=480,
        help="Output display height when resizing",
    )

    # Processing options
    parser.add_argument(
        "--no-threading",
        action="store_true",
        default=False,
        help="Disable threaded image fetching",
    )
    parser.add_argument(
        "--thread-count",
        type=int,
        default=6,
        help="Number of image fetching threads (1-6)",
    )
    parser.add_argument(
        "--no-parallel",
        action="store_true",
        default=False,
        help="Disable parallel image processing",
    )

    # Logging control
    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")
    parser.add_argument("--quiet", action="store_true", help="Minimize logging output")

    return parser.parse_args()


async def main():
    """Main async function to orchestrate the system."""
    args = parse_arguments()
    exit_code = 0
    app = None

    original_sigint_handler = signal.getsignal(signal.SIGINT)
    original_sigterm_handler = signal.getsignal(signal.SIGTERM)

    def signal_handler(sig, frame):
        nonlocal exit_code
        LOGGER.info(f"Received signal {sig}, initiating shutdown...")
        if app:
            app.state.exit_flag = True
        exit_code = 130

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        config = DroneConfig()
        config.connection.ip_address = args.ip
        config.connection.vehicle_name = args.vehicle
        config.system_address = f"udp://:{args.mavsdk_port}"
        config.camera.camera_name = args.camera
        config.camera.width = args.width
        config.camera.height = args.height
        config.display.output_width = args.output_width
        config.display.output_height = args.output_height
        config.display.resize_output = not args.no_resize
        config.display.use_threading = not args.no_threading
        config.display.thread_count = args.thread_count
        config.processing.enable_parallel_processing = not args.no_parallel
        config.display.framerate_hz = args.framerate

        LOGGER.info("=== Drone Control System Starting ===")
        app = DroneApplicationCoordinator(config)

        exit_code = await app.run()

    except KeyboardInterrupt:
        LOGGER.info("Program terminated by user (KeyboardInterrupt).")
        if app:
            app.state.exit_flag = True
        exit_code = 130
    except asyncio.CancelledError:
        LOGGER.info("Asyncio task cancelled.")
        if app:
            app.state.exit_flag = True
        exit_code = 1
    except Exception as e:
        LOGGER.critical(f"Unhandled exception at top level: {e}", exc_info=True)
        if app:
            app.state.exit_flag = True
        exit_code = 1
    finally:
        if app:
            try:
                LOGGER.info("Running final cleanup...")
                cleanup_task = asyncio.create_task(app.cleanup())
                await asyncio.wait_for(cleanup_task, timeout=5.0)
                LOGGER.info("Cleanup completed successfully.")
            except asyncio.TimeoutError:
                LOGGER.warning("Cleanup timed out, forcing exit.")
            except Exception as e:
                LOGGER.error(f"Error during cleanup: {e}")

        signal.signal(signal.SIGINT, original_sigint_handler)
        signal.signal(signal.SIGTERM, original_sigterm_handler)

        try:
            cv2.destroyAllWindows()
            for _ in range(3):
                cv2.waitKey(1)
                cv2.destroyAllWindows()
        except Exception as e:
            LOGGER.warning(f"Error closing OpenCV windows: {e}")

        LOGGER.info(f"=== Program End (Exit Code: {exit_code}) ===")
        for t in threading.enumerate():
            LOGGER.info(f"Thread: {t.name}, daemon={t.daemon}, alive={t.is_alive()}")

        if any(
            t.is_alive() and not t.daemon
            for t in threading.enumerate()
            if t != threading.current_thread()
        ):
            LOGGER.warning("Non-daemon threads still alive. Forcing shutdown.")

        return exit_code


if __name__ == "__main__":
    final_exit_code = 0
    try:
        loop = asyncio.get_event_loop()
        final_exit_code = loop.run_until_complete(main())
        try:
            pending = asyncio.all_tasks(loop)
            if pending:
                LOGGER.warning(f"Cancelling {len(pending)} pending tasks...")
                for task in pending:
                    task.cancel()
                loop.run_until_complete(
                    asyncio.gather(*pending, return_exceptions=True)
                )
        except Exception as e:
            LOGGER.error(f"Error cancelling pending tasks: {e}")

        try:
            loop.run_until_complete(loop.shutdown_asyncgens())
            loop.close()
            LOGGER.info("Event loop closed successfully.")

        except Exception as e:
            LOGGER.error(f"Error closing event loop: {e}")

        cv2.destroyAllWindows()
        sys.exit(final_exit_code)
    except KeyboardInterrupt:
        LOGGER.info("Program terminated by user at entry point (KeyboardInterrupt).")
        final_exit_code = 130
    except Exception as e:
        LOGGER.critical(f"Fatal error at entry point: {e}", exc_info=True)
        final_exit_code = 1
    finally:
        os._exit(0)
