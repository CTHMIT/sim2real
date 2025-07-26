import cosysairsim as airsim  # type: ignore
import cv2
import numpy as np
import time
import sys
import signal
import threading
import asyncio
import concurrent.futures
from queue import Queue, Empty
from pathlib import Path
from abc import ABC, abstractmethod
from pydantic import BaseModel, Field
from typing import Optional, Any, List
import argparse
from utils.logger import LOGGER  # type: ignore

PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.append(str(PROJECT_ROOT))


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
    width: int = Field(default=0, description="Requested width (0 for default)")
    height: int = Field(default=0, description="Requested height (0 for default)")


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
    convert_to_rgb: bool = Field(default=True, description="Convert BGR to RGB")
    optimize_for_realtime: bool = Field(
        default=True, description="Optimize for realtime display"
    )
    use_threading: bool = Field(default=True, description="Use threaded image fetching")
    use_multiple_threads: bool = Field(
        default=False, description="Use multiple background threads"
    )
    thread_count: int = Field(
        default=2, ge=1, le=4, description="Number of background threads (1-4)"
    )
    queue_size: int = Field(default=1, description="Image queue size")
    waitKey_delay: int = Field(default=1, description="CV2 waitKey delay in ms (1-10)")
    resize_output: bool = Field(
        default=False, description="Resize output for performance"
    )
    output_width: int = Field(default=640, description="Output width if resizing")
    output_height: int = Field(default=480, description="Output height if resizing")

    @property
    def update_rate(self) -> float:
        """Convert framerate in Hz to update interval in seconds"""
        return 1.0 / self.framerate_hz


class ProcessingConfig(BaseModel):
    """Image processing configuration parameters"""

    enable_parallel_processing: bool = Field(
        default=False, description="Enable parallel image processing"
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


class AirSimConfig(BaseModel):
    """Overall configuration parameters"""

    connection: ConnectionConfig = Field(default_factory=ConnectionConfig)
    camera: CameraConfig = Field(default_factory=CameraConfig)
    display: DisplayConfig = Field(default_factory=DisplayConfig)
    processing: ProcessingConfig = Field(default_factory=ProcessingConfig)
    timeout: int = Field(default=10, description="Connection timeout (seconds)")


def timeout_handler(signum, frame):
    """Signal handler for operation timeouts"""
    raise TimeoutError("Operation timed out")


class ImageProcessingStrategy(ABC):
    """Base class for image processing strategies"""

    @abstractmethod
    def process_image(self, img_data, height, width):
        pass


class DefaultImageProcessingStrategy(ImageProcessingStrategy):
    """Default image processing implementation"""

    def __init__(self, convert_to_rgb=True):
        self.convert_to_rgb = convert_to_rgb

    def process_image(self, img_data, height, width):
        """Process raw image data into displayable format"""
        img_bgr = np.frombuffer(img_data, dtype=np.uint8).reshape(height, width, 3)
        if self.convert_to_rgb:
            return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        return img_bgr


class HighPerformanceProcessingStrategy(ImageProcessingStrategy):
    """High performance image processing with minimal operations"""

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
        """Process image with minimal operations for maximum speed"""
        try:
            # Direct reshape without unnecessary copies
            img = np.frombuffer(img_data, dtype=np.uint8).reshape(height, width, 3)

            # CPU processing
            if self.convert_to_rgb:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            if self.resize_output:
                img = cv2.resize(
                    img,
                    (self.output_width, self.output_height),
                    interpolation=cv2.INTER_NEAREST,
                )  # Fastest method

            return img

        except Exception as e:
            LOGGER.error(f"Error in image processing: {e}")
            # Return a black image of the expected size if processing fails
            if self.resize_output:
                return np.zeros(
                    (self.output_height, self.output_width, 3), dtype=np.uint8
                )
            else:
                return np.zeros((height, width, 3), dtype=np.uint8)


class AirSimClient:
    """AirSim client implementation"""

    def __init__(
        self,
        config: AirSimConfig,
        image_strategy: Optional[ImageProcessingStrategy] = None,
    ):
        self.config = config
        self.client: Optional[airsim.MultirotorClient] = None
        self.image_strategy: Optional[ImageProcessingStrategy] = None

        if image_strategy is None:
            if config.display.optimize_for_realtime:
                self.image_strategy = HighPerformanceProcessingStrategy(
                    convert_to_rgb=config.display.convert_to_rgb,
                    resize_output=config.display.resize_output,
                    output_width=config.display.output_width,
                    output_height=config.display.output_height,
                )
            else:
                self.image_strategy = DefaultImageProcessingStrategy(
                    config.display.convert_to_rgb
                )
        else:
            self.image_strategy = image_strategy

        self.last_update_time = 0
        self.running = False
        self.image_queue: Queue = Queue(
            maxsize=config.display.queue_size if config.display.queue_size > 0 else 1
        )
        self.threads: List[threading.Thread] = []
        self.thread_clients: List[Any] = []
        self.frame_times: List[float] = []
        self.processing_times: List[float] = []
        self.fetch_times: List[float] = []

        self.process_pool: Optional[concurrent.futures.ThreadPoolExecutor] = None

        if config.processing.enable_parallel_processing:
            self.process_pool = concurrent.futures.ThreadPoolExecutor(max_workers=8)
        else:
            self.process_pool = None

    def connect(self):
        """Connect to AirSim server"""
        LOGGER.info(
            f"Attempting to connect to AirSim at: {self.config.connection.ip_address}"
        )
        try:
            signal.alarm(self.config.timeout)

            self.client = airsim.MultirotorClient(ip=self.config.connection.ip_address)
            self.client.confirmConnection()

            signal.alarm(0)

            LOGGER.info(
                f"Connected to AirSim for vehicle: {self.config.connection.vehicle_name}"
            )

            vehicles = self.client.listVehicles()
            LOGGER.info(f"Available vehicles: {vehicles}")

            if self.config.connection.vehicle_name not in vehicles:
                LOGGER.warning(
                    f"Vehicle '{self.config.connection.vehicle_name}' not found in simulation"
                )

            try:
                camera_info = self.client.simGetCameraInfo(
                    self.config.camera.camera_name,
                    vehicle_name=self.config.connection.vehicle_name,
                )
                LOGGER.info(f"Camera info: {camera_info}")
            except Exception as e:
                LOGGER.error(f"Failed to get camera info: {e}")

            return True
        except Exception as e:
            LOGGER.error(f"Failed to connect to AirSim: {e}")
            signal.alarm(0)
            return False

    def get_image(self, client_instance=None):
        """Get a single image frame from the camera"""
        client_to_use = client_instance or self.client

        try:
            start_time = time.time()

            request = airsim.ImageRequest(
                camera_name=self.config.camera.camera_name,
                image_type=self.config.camera.image_type,
                pixels_as_float=self.config.camera.pixels_as_float,
                compress=self.config.camera.compress,
            )

            if self.config.camera.width > 0 and self.config.camera.height > 0:
                request.width = self.config.camera.width
                request.height = self.config.camera.height

            responses = client_to_use.simGetImages(
                [request], vehicle_name=self.config.connection.vehicle_name
            )

            if not responses or not responses[0].image_data_uint8:
                return None

            response = responses[0]

            fetch_time = time.time() - start_time
            self.fetch_times.append(fetch_time)
            if len(self.fetch_times) > 100:
                self.fetch_times.pop(0)

            return {
                "data": response.image_data_uint8,
                "width": response.width,
                "height": response.height,
                "timestamp": time.time(),
                "fetch_time": fetch_time,
            }
        except Exception as e:
            LOGGER.error(f"Failed to get image: {e}")
            return None

    def _setup_thread_event_loop(self):
        """Set up a new event loop for the thread"""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            return True
        except Exception as e:
            LOGGER.error(f"Failed to set up event loop in thread: {e}")
            return False

    def _thread_connect_client(self):
        """Create a separate client instance for the thread"""
        try:
            thread_client = airsim.MultirotorClient(
                ip=self.config.connection.ip_address
            )
            thread_client.confirmConnection()
            self.thread_clients.append(thread_client)
            return thread_client
        except Exception as e:
            LOGGER.error(f"Failed to connect thread client: {e}")
            return None

    def _fetch_images_thread(self, thread_id=0):
        """Background thread for fetching images"""
        thread_name = f"FetchThread-{thread_id}"
        LOGGER.info(f"Starting {thread_name}")

        if not self._setup_thread_event_loop():
            LOGGER.error(f"Could not set up event loop, terminating {thread_name}")
            return

        thread_client = self._thread_connect_client()
        if not thread_client:
            LOGGER.error(f"Could not connect client, terminating {thread_name}")
            return

        while self.running:
            if not self.image_queue.full():
                image_info = self.get_image(thread_client)
                if image_info:
                    try:
                        self.image_queue.put_nowait(image_info)
                    except Queue.Full:
                        try:
                            self.image_queue.get_nowait()
                            self.image_queue.put_nowait(image_info)
                        except Empty:
                            pass

            sleep_time = max(0.001, self.config.display.update_rate * 0.25)
            time.sleep(sleep_time)

    def start_image_threads(self):
        """Start the background image fetching threads"""
        if not self.config.display.use_threading:
            return

        self.running = True
        thread_count = 1

        if self.config.display.use_multiple_threads:
            thread_count = self.config.display.thread_count

        LOGGER.info(f"Starting {thread_count} image fetching thread(s)")

        for i in range(thread_count):
            thread = threading.Thread(target=self._fetch_images_thread, args=(i,))
            thread.daemon = True
            thread.start()
            self.threads.append(thread)

    def stop_image_threads(self):
        """Stop all background image fetching threads"""
        self.running = False

        for thread in self.threads:
            thread.join(timeout=1.0)

        self.threads = []
        self.thread_clients = []

        if self.process_pool:
            self.process_pool.shutdown(wait=False)

    def process_and_display(self):
        """Process and display image frames with rate limiting"""
        current_time = time.time()
        frame_start_time = current_time

        update_interval = self.config.display.update_rate

        if (
            update_interval > 0
            and (current_time - self.last_update_time) < update_interval
        ):
            time.sleep(0.0005)
            return True

        image_info = None
        if self.config.display.use_threading:
            try:
                image_info = self.image_queue.get_nowait()
            except Empty:
                pass
        else:
            image_info = self.get_image()

        if not image_info:
            sleep_time = 0.001 if self.config.display.optimize_for_realtime else 0.02
            time.sleep(sleep_time)
            return True

        process_start = time.time()

        if self.process_pool and self.config.processing.enable_parallel_processing:
            future = self.process_pool.submit(
                self.image_strategy.process_image,
                image_info["data"],
                image_info["height"],
                image_info["width"],
            )
            try:
                processed_img = future.result(timeout=update_interval)
            except concurrent.futures.TimeoutError:
                LOGGER.warning("Image processing timeout, skipping frame")
                return True
        else:
            processed_img = self.image_strategy.process_image(
                image_info["data"], image_info["height"], image_info["width"]
            )

        process_time = time.time() - process_start
        self.processing_times.append(process_time)
        if len(self.processing_times) > 100:
            self.processing_times.pop(0)

        cv2.imshow(self.config.display.window_name, processed_img)
        self.last_update_time = current_time

        frame_time = time.time() - frame_start_time
        self.frame_times.append(frame_time)
        if len(self.frame_times) > 100:
            self.frame_times.pop(0)

        if cv2.waitKey(self.config.display.waitKey_delay) & 0xFF == ord("q"):
            return False
        return True

    def get_performance_stats(self):
        """Get performance statistics"""
        stats = {}

        if self.frame_times:
            avg_frame_time = sum(self.frame_times) / len(self.frame_times)
            stats["avg_frame_time_ms"] = avg_frame_time * 1000
            stats["effective_fps"] = 1.0 / avg_frame_time if avg_frame_time > 0 else 0

        if self.processing_times:
            avg_process_time = sum(self.processing_times) / len(self.processing_times)
            stats["avg_process_time_ms"] = avg_process_time * 1000

        if self.fetch_times:
            avg_fetch_time = sum(self.fetch_times) / len(self.fetch_times)
            stats["avg_fetch_time_ms"] = avg_fetch_time * 1000

        stats["queue_size"] = self.image_queue.qsize()

        return stats


def create_config_from_args(args):
    """Create configuration from command line arguments"""
    config = AirSimConfig(
        connection=ConnectionConfig(ip_address=args.ip, vehicle_name=args.vehicle),
        camera=CameraConfig(
            camera_name=args.camera,
            image_type=airsim.ImageType.Scene,
            pixels_as_float=False,
            compress=False,
            width=args.width,
            height=args.height,
        ),
        display=DisplayConfig(
            window_name="AirSim Camera View",
            framerate_hz=args.framerate,
            convert_to_rgb=True,
            optimize_for_realtime=True,
            use_threading=not args.no_threading,
            use_multiple_threads=not args.no_threading and args.thread_count > 1,
            thread_count=max(1, min(4, args.thread_count)),
            queue_size=2,
            waitKey_delay=1,
            resize_output=args.resize,
            output_width=args.output_width,
            output_height=args.output_height,
        ),
        processing=ProcessingConfig(
            enable_parallel_processing=not args.no_parallel,
            skip_frames_if_busy=True,
            downscale_factor=max(0.1, min(1.0, args.downscale)),
        ),
        timeout=10,
    )

    return config


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="AirSim Camera Viewer with performance optimizations"
    )

    # Connection parameters
    parser.add_argument(
        "--ip", type=str, default="172.19.160.1", help="IP address of AirSim server"
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
        default=0,
        help="Request specific image width (0 for default)",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=0,
        help="Request specific image height (0 for default)",
    )

    # Display parameters
    parser.add_argument(
        "--framerate", type=float, default=60.0, help="Target framerate in Hz (10-240)"
    )
    parser.add_argument(
        "--output-width",
        type=int,
        default=1280,
        help="Output display width when resizing",
    )
    parser.add_argument(
        "--output-height",
        type=int,
        default=720,
        help="Output display height when resizing",
    )
    parser.add_argument(
        "--resize", action="store_true", help="Resize output for performance"
    )

    # Processing options
    parser.add_argument(
        "--no-threading", action="store_true", help="Disable threaded image fetching"
    )
    parser.add_argument(
        "--thread-count",
        type=int,
        default=2,
        help="Number of image fetching threads (1-4)",
    )
    parser.add_argument(
        "--no-parallel", action="store_true", help="Disable parallel image processing"
    )
    parser.add_argument(
        "--downscale",
        type=float,
        default=1.0,
        help="Downscale factor for processing (0.1-1.0)",
    )

    # Performance monitoring
    parser.add_argument(
        "--stats-interval",
        type=float,
        default=3.0,
        help="Interval in seconds for performance statistics (0 to disable)",
    )

    # Logging control
    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")
    parser.add_argument("--quiet", action="store_true", help="Minimize logging output")

    return parser.parse_args()


def main():
    """Main application entry point"""
    # Create high-performance configuration
    args = parse_arguments()

    config = AirSimConfig(
        connection=ConnectionConfig(ip_address=args.ip, vehicle_name=args.vehicle),
        camera=CameraConfig(
            camera_name="front_center",
            image_type=airsim.ImageType.Scene,
            pixels_as_float=False,
            compress=False,
            width=640,
            height=480,
        ),
        display=DisplayConfig(
            window_name="AirSim Camera View",
            framerate_hz=120.0,  # Higher framerate target
            convert_to_rgb=True,
            optimize_for_realtime=True,
            use_threading=True,
            use_multiple_threads=True,  # Use multiple image fetching threads
            thread_count=2,  # Number of threads for fetching
            queue_size=2,  # Small queue for lowest latency
            waitKey_delay=1,  # Minimum delay
            resize_output=True,  # Resize output for performance
            output_width=1280,  # Output resolution
            output_height=720,
        ),
        processing=ProcessingConfig(
            enable_parallel_processing=True,  # Use parallel processing
            skip_frames_if_busy=True,  # Skip frames if processing is busy
            downscale_factor=1.0,  # Full resolution processing
        ),
        timeout=10,
    )

    effective_framerate = 1.0 / config.display.update_rate
    LOGGER.info(
        f"Target framerate: {effective_framerate:.1f} Hz (update interval: {config.display.update_rate:.4f}s)"
    )
    LOGGER.info(
        f"Optimization settings: realtime={config.display.optimize_for_realtime}, "
        f"threading={config.display.use_threading}, "
        f"multi-threading={config.display.use_multiple_threads}"
    )

    # Set up main thread event loop if needed
    try:
        loop = asyncio.get_event_loop()
    except RuntimeError:
        # No event loop exists, create one
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        LOGGER.info("Created new event loop in main thread")

    # Create high performance image processing strategy
    image_strategy = HighPerformanceProcessingStrategy(
        convert_to_rgb=config.display.convert_to_rgb,
        resize_output=config.display.resize_output,
        output_width=config.display.output_width,
        output_height=config.display.output_height,
    )

    airsim_client = AirSimClient(config, image_strategy)

    signal.signal(signal.SIGALRM, timeout_handler)

    try:
        if not airsim_client.connect():
            return

        LOGGER.info(
            f"Attempting to get images from camera: {config.camera.camera_name}"
        )

        signal.alarm(config.timeout)

        LOGGER.info("Requesting first image (with timeout)...")
        try:
            image_info = airsim_client.get_image()
            signal.alarm(0)

            if not image_info:
                LOGGER.error("No valid image data received")
                return

            LOGGER.info(
                f"Image properties: width={image_info['width']}, height={image_info['height']}"
            )
            LOGGER.info(f"Image data size: {len(image_info['data'])}")
        except TimeoutError:
            LOGGER.error(
                "Image request timed out - camera may not be properly configured"
            )
            return
        except Exception as e:
            LOGGER.error(f"Error requesting image: {e}")
            return

        LOGGER.info("Successfully received first image, starting display loop")

        # Start the background image fetching threads
        airsim_client.start_image_threads()

        # Performance monitoring
        last_stats_time = time.time()
        frame_count = 0

        while True:
            if not airsim_client.process_and_display():
                break

            # Calculate and display performance stats periodically
            frame_count += 1
            current_time = time.time()
            elapsed = current_time - last_stats_time

            if elapsed >= 3.0:  # Show stats every 3 seconds
                # Get performance statistics
                stats = airsim_client.get_performance_stats()

                # Calculate actual FPS
                actual_fps = frame_count / elapsed

                LOGGER.info(
                    f"Performance: {actual_fps:.1f} FPS, "
                    f"frame time: {stats.get('avg_frame_time_ms', 0):.1f}ms, "
                    f"processing: {stats.get('avg_process_time_ms', 0):.1f}ms, "
                    f"fetch: {stats.get('avg_fetch_time_ms', 0):.1f}ms"
                )

                frame_count = 0
                last_stats_time = current_time

    except KeyboardInterrupt:
        LOGGER.info("Script interrupted by user")
    finally:
        # Clean up
        signal.alarm(0)
        airsim_client.stop_image_threads()
        cv2.destroyAllWindows()
        LOGGER.info("Script finished")


if __name__ == "__main__":
    main()
