import sys
from pathlib import Path
import asyncio
import tkinter as tk
import threading
from typing import Set
from pydantic import BaseModel, Field

from mavsdk import System  # type: ignore
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed  # type: ignore
from mavsdk.gimbal import GimbalMode, SendMode, ControlMode  # type: ignore
from .utils.logger import LOGGER  # type: ignore

PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.append(str(PROJECT_ROOT))


class DroneConfig(BaseModel):
    system_address: str = Field("udp://:14540", description="MAVSDK connection URL")
    default_takeoff_altitude: float = Field(
        5.0, description="Default altitude for takeoff in meters"
    )
    connection_timeout: float = Field(
        10.0, description="Timeout for drone connection attempt in seconds"
    )

    speed_increment: float = Field(0.5, description="Drone speed increment (m/s)")
    yaw_increment: float = Field(30.0, description="Drone yaw speed increment (deg/s)")
    gimbal_rate_increment: float = Field(
        5.0, description="Gimbal angular rate increment (deg/s)"
    )

    max_speed: float = Field(5.0, description="Maximum forward/right/down speed (m/s)")
    max_yaw_speed: float = Field(45.0, description="Maximum yaw speed (deg/s)")
    max_gimbal_rate: float = Field(
        30.0, description="Maximum gimbal angular rate (deg/s)"
    )

    acceleration_factor: float = Field(1.0, description="Control acceleration factor")
    decay_factor: float = Field(
        0.8, description="Control decay factor when keys are released"
    )
    zero_threshold: float = Field(0.05, description="Threshold to zero out values")

    target_gimbal_id: int = Field(0, description="Default Gimbal ID to control")
    gimbal_mode: GimbalMode = Field(
        GimbalMode.YAW_FOLLOW, description="Gimbal control mode"
    )
    gimbal_send_mode: SendMode = Field(
        SendMode.STREAM, description="Gimbal command send mode"
    )


class ControlState(BaseModel):
    velocity_forward: float = 0.0
    velocity_right: float = 0.0
    velocity_down: float = 0.0
    yawspeed: float = 0.0
    gimbal_pitch_rate: float = 0.0
    gimbal_yaw_rate: float = 0.0
    exit_flag: bool = False
    gimbal_available: bool = False
    speed_multiplier: float = 1.0
    pressed_keys: Set[str] = Field(default_factory=set)
    autonomous_mode: bool = False
    last_position: dict = Field(default_factory=dict)
    last_attitude: dict = Field(default_factory=dict)
    last_gimbal_position: dict = Field(default_factory=dict)
    current_velocity: dict = Field(default_factory=dict)
    target_position: dict = Field(default_factory=dict)


async def connect_drone(config: DroneConfig) -> System | None:
    drone = System()
    LOGGER.info(f"Connecting to drone at {config.system_address}...")
    try:
        await drone.connect(system_address=config.system_address)

        async for state in drone.core.connection_state():
            if state.is_connected:
                LOGGER.info("Drone connected!")
                return drone
        LOGGER.info("Connection attempt finished without success.")
        return None
    except asyncio.TimeoutError:
        LOGGER.info(f"Connection timed out after {config.connection_timeout} seconds.")
        return None
    except Exception as e:
        LOGGER.info(f"Connection error: {e}")
        return None


async def check_drone_health(drone: System) -> bool:
    try:
        LOGGER.info("Checking system health...")

        start_time = asyncio.get_event_loop().time()
        timeout = 30

        async for health in drone.telemetry.health():
            LOGGER.info(
                f"Health status: Gyro={health.is_gyrometer_calibration_ok}, Accel={health.is_accelerometer_calibration_ok}, "
                f"Mag={health.is_magnetometer_calibration_ok}, Armable={health.is_armable}"
            )

            if (
                health.is_gyrometer_calibration_ok
                and health.is_magnetometer_calibration_ok
            ):

                if health.is_armable:
                    LOGGER.info("Drone health check passed, ready to arm!")
                    return True
                else:
                    LOGGER.info(
                        "Core sensors calibrated but drone not armable. Attempting to continue..."
                    )
                    return True

            current_time = asyncio.get_event_loop().time()
            if current_time - start_time > timeout:
                LOGGER.info(f"Health check timed out after {timeout} seconds.")

                if health.is_gyrometer_calibration_ok:
                    LOGGER.info(
                        "Some sensors calibrated. Attempting to continue despite incomplete calibration..."
                    )
                    return True
                break

            await asyncio.sleep(1)

        LOGGER.info("Health check did not pass or stream ended.")
        return False
    except Exception as e:
        LOGGER.info(f"Error during health check: {e}")
        return False


async def arm_drone(drone: System) -> bool:
    try:
        LOGGER.info("Arming drone...")
        await drone.action.arm()
        LOGGER.info("Drone armed!")
        return True
    except Exception as e:
        LOGGER.info(f"Arming failed: {e}")
        return False


async def takeoff_drone(drone: System, config: DroneConfig) -> bool:
    altitude = config.default_takeoff_altitude
    LOGGER.info(f"Taking off to {altitude} meters...")
    try:
        await drone.action.set_takeoff_altitude(altitude)
        await drone.action.takeoff()

        async for position in drone.telemetry.position():
            relative_altitude = position.relative_altitude_m
            LOGGER.info(f"Current altitude: {relative_altitude:.2f} m")
            if relative_altitude >= altitude * 0.95:
                LOGGER.info("Target altitude reached!")
                return True

        LOGGER.info("Position stream ended before reaching target altitude.")
        return False
    except Exception as e:
        LOGGER.info(f"Takeoff failed: {e}")
        return False


async def land_drone(drone: System):
    LOGGER.info("Landing...")
    try:
        await drone.action.land()
        LOGGER.info("Drone landing command sent.")

        async for armed in drone.telemetry.armed():
            if not armed:
                LOGGER.info("Drone disarmed after landing.")
                break
            await asyncio.sleep(0.5)
    except Exception as e:
        LOGGER.info(f"Landing failed: {e}")


async def check_gimbal_devices(drone: System, config: DroneConfig, state: ControlState):
    LOGGER.info("Checking for available gimbal devices...")
    state.gimbal_available = False
    try:
        gimbal_list = None
        try:
            gimbal_list = await asyncio.wait_for(
                drone.gimbal.gimbal_list(), timeout=5.0
            )
        except asyncio.TimeoutError:
            LOGGER.warning("Timeout while fetching gimbal list.")
            return
        except Exception as e:
            LOGGER.warning(f"Could not fetch gimbal list: {e}")
            return e

        if gimbal_list and gimbal_list.gimbals:
            LOGGER.info(f"Found {len(gimbal_list.gimbals)} gimbal device(s)")
            found_target = False
            for gimbal in gimbal_list.gimbals:
                LOGGER.info(
                    f"  - ID: {gimbal.gimbal_id}, Model: {gimbal.model_name}, Vendor: {gimbal.vendor_name}"
                )
                if not found_target:
                    if config.target_gimbal_id != gimbal.gimbal_id:
                        LOGGER.info(
                            f"Updating target gimbal ID from {config.target_gimbal_id} to detected ID: {gimbal.gimbal_id}"
                        )
                        config.target_gimbal_id = gimbal.gimbal_id
                    state.gimbal_available = True
                    found_target = True
                    LOGGER.info(
                        f"Gimbal (ID: {config.target_gimbal_id}) marked as available for control."
                    )
            if not found_target:
                LOGGER.warning(
                    "Gimbal devices listed, but couldn't assign a target ID. Gimbal control disabled."
                )
        else:
            LOGGER.info("No gimbal devices found.")
    except Exception as e:
        LOGGER.info(f"Error checking gimbals: {e}")
        state.gimbal_available = False


async def control_gimbal_angular_rates(
    drone: System, config: DroneConfig, state: ControlState
):
    if not state.gimbal_available:
        if abs(state.gimbal_pitch_rate) > 0.01 or abs(state.gimbal_yaw_rate) > 0.01:
            LOGGER.debug("Gimbal not available, skipping control command.")
        return
    try:
        async for _ in drone.gimbal.set_angular_rates(
            gimbal_id=config.target_gimbal_id,
            roll_rate_deg_s=0.0,
            pitch_rate_deg_s=state.gimbal_pitch_rate,
            yaw_rate_deg_s=state.gimbal_yaw_rate,
            gimbal_mode=config.gimbal_mode,
            send_mode=config.gimbal_send_mode,
        ):
            break
    except Exception as e:
        if abs(state.gimbal_pitch_rate) > 0.01 or abs(state.gimbal_yaw_rate) > 0.01:
            LOGGER.info(f"Gimbal control error: {e}")


async def update_telemetry(drone: System, state: ControlState):
    """Update telemetry data in the state object"""
    try:
        # Position
        async for position in drone.telemetry.position():
            state.last_position = {
                "latitude_deg": position.latitude_deg,
                "longitude_deg": position.longitude_deg,
                "absolute_altitude_m": position.absolute_altitude_m,
                "relative_altitude_m": position.relative_altitude_m,
            }
            break

        # Attitude
        async for attitude in drone.telemetry.attitude_euler():
            state.last_attitude = {
                "roll_deg": attitude.roll_deg,
                "pitch_deg": attitude.pitch_deg,
                "yaw_deg": attitude.yaw_deg,
            }
            break

        # Velocity
        async for velocity in drone.telemetry.velocity_ned():
            state.current_velocity = {
                "north_m_s": velocity.north_m_s,
                "east_m_s": velocity.east_m_s,
                "down_m_s": velocity.down_m_s,
            }
            break

        # Gimbal position if available
        if state.gimbal_available:
            try:
                gimbal_attitude = await drone.gimbal.get_gimbal_attitude(gimbal_id=0)
                state.last_gimbal_position = {
                    "roll_deg": gimbal_attitude.roll_deg,
                    "pitch_deg": gimbal_attitude.pitch_deg,
                    "yaw_deg": gimbal_attitude.yaw_deg,
                }
            except Exception:
                pass

    except Exception as e:
        LOGGER.warning(f"Error updating telemetry: {e}")


class DroneControlGUI:
    def __init__(self, root: tk.Tk, config: DroneConfig, state: ControlState):
        self.root = root
        self.config = config
        self.state = state
        self.root.title("Drone Keyboard Control")
        self.pressed_keys: set[str] = set()
        self.root.geometry("600x550")

        # Main frame
        main_frame = tk.Frame(root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)

        # Title
        tk.Label(
            main_frame, text="Enhanced Drone Control System", font=("Arial", 16, "bold")
        ).pack(pady=10)

        # Control instructions
        controls_frame = tk.LabelFrame(
            main_frame, text="Control Reference", font=("Arial", 12)
        )
        controls_frame.pack(fill="x", pady=5)

        controls = [
            "Movement (Body Frame):",
            "  W/S: Forward / Backward",
            "  A/D: Left / Right",
            "  R/F: Up / Down",
            "  Q/E: Yaw Left / Yaw Right",
            "",
            "Gimbal Control:",
            "  I/K: Pitch Up / Pitch Down",
            "  J/L: Yaw Left / Yaw Right",
            "",
            "Speed Control:",
            "  Z/X: Speed Up / Speed Down",
            "  C: Reset Speed Multiplier",
            "",
            "System Control:",
            "  Esc: Stop Control & Land",
        ]

        for ctrl in controls:
            tk.Label(controls_frame, text=ctrl, anchor="w", justify=tk.LEFT).pack(
                fill="x", padx=20, pady=1
            )

        # Status section
        status_frame = tk.LabelFrame(
            main_frame, text="System Status", font=("Arial", 12)
        )
        status_frame.pack(fill="x", pady=10)

        # Create a frame for status information with two columns
        status_info_frame = tk.Frame(status_frame)
        status_info_frame.pack(fill="x", padx=10, pady=5)

        # Drone status
        self.status_label_drone = tk.Label(
            status_info_frame, text="Initializing...", font=("Arial", 10), anchor="w"
        )
        self.status_label_drone.grid(row=0, column=0, sticky="w", padx=5, pady=3)

        # Gimbal status
        self.status_label_gimbal = tk.Label(
            status_info_frame,
            text="Gimbal: Not detected",
            font=("Arial", 10),
            anchor="w",
        )
        self.status_label_gimbal.grid(row=1, column=0, sticky="w", padx=5, pady=3)

        # Speed multiplier
        self.speed_label = tk.Label(
            status_info_frame,
            text=f"Speed: {state.speed_multiplier:.1f}x",
            font=("Arial", 10),
            anchor="w",
        )
        self.speed_label.grid(row=0, column=1, sticky="w", padx=5, pady=3)

        # Position information
        self.position_label = tk.Label(
            status_info_frame, text="Position: N/A", font=("Arial", 10), anchor="w"
        )
        self.position_label.grid(
            row=2, column=0, columnspan=2, sticky="w", padx=5, pady=3
        )

        # Control buttons frame
        button_frame = tk.Frame(main_frame)
        button_frame.pack(fill="x", pady=10)

        # Stop button
        tk.Button(
            button_frame,
            text="Stop & Land (ESC)",
            command=self.exit_control,
            bg="#FF6B6B",
            fg="white",
            font=("Arial", 11, "bold"),
            padx=10,
            pady=5,
        ).pack(side=tk.LEFT, padx=10)

        # Reset speed button
        tk.Button(
            button_frame,
            text="Reset Speed (C)",
            command=self.reset_speed,
            bg="#FFD166",
            fg="black",
            font=("Arial", 11, "bold"),
            padx=10,
            pady=5,
        ).pack(side=tk.LEFT, padx=10)

        # Set up key bindings
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)

        # Start the UI update loop
        self.update_control_loop()

    def on_key_press(self, event):
        key = event.keysym.lower()
        self.state.pressed_keys.add(key)

        # Handle special keys
        if key == "escape":
            self.exit_control()
        elif key == "z":
            self.increase_speed()
        elif key == "x":
            self.decrease_speed()
        elif key == "c":
            self.reset_speed()

    def on_key_release(self, event):
        key = event.keysym.lower()
        if key in self.state.pressed_keys:
            self.state.pressed_keys.remove(key)

    def process_keys(self):
        """Process keyboard inputs and update control state"""
        # Process speed control keys separately since they only modify the multiplier
        if "z" in self.state.pressed_keys:
            self.increase_speed()
        elif "x" in self.state.pressed_keys:
            self.decrease_speed()

        # Process movement keys
        cfg = self.config
        state = self.state
        keys = state.pressed_keys
        speed_mult = state.speed_multiplier

        # --- Velocity Control ---
        # Forward/backward
        target_forward = 0.0
        if "w" in keys:
            target_forward = cfg.max_speed * speed_mult
        elif "s" in keys:
            target_forward = -cfg.max_speed * speed_mult

        # Right/left
        target_right = 0.0
        if "d" in keys:
            target_right = cfg.max_speed * speed_mult
        elif "a" in keys:
            target_right = -cfg.max_speed * speed_mult

        # Up/down (inverted because down is positive in MAVSDK)
        target_down = 0.0
        if "f" in keys:
            target_down = cfg.max_speed * speed_mult
        elif "r" in keys:
            target_down = -cfg.max_speed * speed_mult

        # Yaw control (influence reduced by half in max_yaw_speed config)
        target_yawspeed = 0.0
        if "e" in keys:
            target_yawspeed = cfg.max_yaw_speed * speed_mult
        elif "q" in keys:
            target_yawspeed = -cfg.max_yaw_speed * speed_mult

        # Apply smooth acceleration/deceleration
        state.velocity_forward += (
            (target_forward - state.velocity_forward)
            * (1.0 - cfg.decay_factor)
            * cfg.acceleration_factor
        )
        state.velocity_right += (
            (target_right - state.velocity_right)
            * (1.0 - cfg.decay_factor)
            * cfg.acceleration_factor
        )
        state.velocity_down += (
            (target_down - state.velocity_down)
            * (1.0 - cfg.decay_factor)
            * cfg.acceleration_factor
        )
        state.yawspeed += (
            (target_yawspeed - state.yawspeed)
            * (1.0 - cfg.decay_factor)
            * cfg.acceleration_factor
        )

        # Apply zero threshold (eliminates small values)
        if abs(state.velocity_forward) < cfg.zero_threshold and target_forward == 0.0:
            state.velocity_forward = 0.0
        if abs(state.velocity_right) < cfg.zero_threshold and target_right == 0.0:
            state.velocity_right = 0.0
        if abs(state.velocity_down) < cfg.zero_threshold and target_down == 0.0:
            state.velocity_down = 0.0
        if abs(state.yawspeed) < cfg.zero_threshold and target_yawspeed == 0.0:
            state.yawspeed = 0.0

        # --- Gimbal Control ---
        target_gimbal_pitch = 0.0
        if "i" in keys:
            target_gimbal_pitch = cfg.max_gimbal_rate
        elif "k" in keys:
            target_gimbal_pitch = -cfg.max_gimbal_rate

        target_gimbal_yaw = 0.0
        if "l" in keys:
            target_gimbal_yaw = cfg.max_gimbal_rate
        elif "j" in keys:
            target_gimbal_yaw = -cfg.max_gimbal_rate

        # Apply acceleration/decay to gimbal controls
        state.gimbal_pitch_rate += (
            (target_gimbal_pitch - state.gimbal_pitch_rate)
            * (1.0 - cfg.decay_factor)
            * cfg.acceleration_factor
        )
        state.gimbal_yaw_rate += (
            (target_gimbal_yaw - state.gimbal_yaw_rate)
            * (1.0 - cfg.decay_factor)
            * cfg.acceleration_factor
        )

        # Apply zero threshold
        if (
            abs(state.gimbal_pitch_rate) < cfg.zero_threshold
            and target_gimbal_pitch == 0.0
        ):
            state.gimbal_pitch_rate = 0.0
        if abs(state.gimbal_yaw_rate) < cfg.zero_threshold and target_gimbal_yaw == 0.0:
            state.gimbal_yaw_rate = 0.0

        # Clamp to max rates
        state.gimbal_pitch_rate = max(
            -cfg.max_gimbal_rate, min(cfg.max_gimbal_rate, state.gimbal_pitch_rate)
        )
        state.gimbal_yaw_rate = max(
            -cfg.max_gimbal_rate, min(cfg.max_gimbal_rate, state.gimbal_yaw_rate)
        )

    def update_control_loop(self):
        """Update the UI based on the current state"""
        # Process keyboard inputs
        self.process_keys()

        # Update drone status display
        drone_text = (
            f"Velocity (Fwd, Right, Down): "
            f"{self.state.velocity_forward:+.2f}, {self.state.velocity_right:+.2f}, {self.state.velocity_down:+.2f} m/s | "
            f"Yaw: {self.state.yawspeed:+.2f} deg/s"
        )
        self.status_label_drone.config(text=drone_text)

        # Update gimbal status display
        if self.state.gimbal_available:
            gimbal_text = (
                f"Gimbal Rate (Pitch, Yaw): "
                f"{self.state.gimbal_pitch_rate:+.2f}, {self.state.gimbal_yaw_rate:+.2f} deg/s"
            )
            if self.state.last_gimbal_position:
                pitch = self.state.last_gimbal_position.get("pitch_deg", 0)
                yaw = self.state.last_gimbal_position.get("yaw_deg", 0)
                gimbal_text += f" | Position: {pitch:.1f}°, {yaw:.1f}°"
        else:
            gimbal_text = "Gimbal: Not available"
        self.status_label_gimbal.config(text=gimbal_text)

        # Update speed multiplier display
        self.speed_label.config(text=f"Speed: {self.state.speed_multiplier:.1f}x")

        # Update position information if available
        if self.state.last_position:
            alt = self.state.last_position.get("relative_altitude_m", 0)
            pos_text = f"Altitude: {alt:.1f}m"

            if self.state.last_attitude:
                roll = self.state.last_attitude.get("roll_deg", 0)
                pitch = self.state.last_attitude.get("pitch_deg", 0)
                yaw = self.state.last_attitude.get("yaw_deg", 0)
                pos_text += f" | Attitude: R:{roll:.1f}°, P:{pitch:.1f}°, Y:{yaw:.1f}°"

            self.position_label.config(text=pos_text)

        # Schedule the next update if the window still exists
        if self.root.winfo_exists():
            self.root.after(50, self.update_control_loop)

    def increase_speed(self):
        """Increase the speed multiplier"""
        self.state.speed_multiplier = min(3.0, self.state.speed_multiplier + 0.1)
        LOGGER.debug(
            f"Speed multiplier increased to {self.state.speed_multiplier:.1f}x"
        )

    def decrease_speed(self):
        """Decrease the speed multiplier"""
        self.state.speed_multiplier = max(0.2, self.state.speed_multiplier - 0.1)
        LOGGER.debug(
            f"Speed multiplier decreased to {self.state.speed_multiplier:.1f}x"
        )

    def reset_speed(self):
        """Reset the speed multiplier to default"""
        self.state.speed_multiplier = 1.0
        LOGGER.debug("Speed multiplier reset to 1.0x")

    def exit_control(self):
        """Request to exit the control loop and land the drone"""
        LOGGER.info("Exit requested via GUI.")
        self.state.exit_flag = True
        if self.root.winfo_exists():
            self.root.destroy()


def start_gui_thread(config: DroneConfig, state: ControlState):
    """Start a separate thread for the GUI"""
    gui_thread_started = threading.Event()

    def gui_runner():
        try:
            root = tk.Tk()
            DroneControlGUI(root, config, state)
            gui_thread_started.set()
            root.mainloop()
        except Exception as e:
            LOGGER.error(f"Error in GUI thread: {e}", exc_info=True)
            state.exit_flag = True
        finally:
            LOGGER.info("GUI thread finished.")

    gui_thread = threading.Thread(target=gui_runner, daemon=True)
    gui_thread.start()

    if not gui_thread_started.wait(timeout=5.0):
        LOGGER.error("GUI thread did not start within timeout.")
        state.exit_flag = True
        return None
    LOGGER.info("GUI thread started successfully.")
    return gui_thread


async def run_manual_control_loop(
    drone: System, config: DroneConfig, state: ControlState
):
    """Run the main drone control loop"""
    LOGGER.info("Preparing for flight control...")
    gimbal_control_taken = False

    try:
        LOGGER.info("Sending initial zero velocity commands...")
        for _ in range(10):
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
            await asyncio.sleep(0.1)

        try:
            LOGGER.info("Starting Offboard mode...")
            await drone.offboard.start()
            LOGGER.info("Offboard mode started successfully.")
        except OffboardError as e:
            LOGGER.error(f"Failed to start Offboard mode: {e}")
            LOGGER.info("Offboard start failed. Attempting to land.")
            await drone.action.land()
            return

        if state.gimbal_available:
            LOGGER.info(
                f"Attempting to take control of gimbal ID: {config.target_gimbal_id}..."
            )
            try:
                async for _ in drone.gimbal.take_control(
                    gimbal_id=config.target_gimbal_id, control_mode=ControlMode.PRIMARY
                ):
                    break
                LOGGER.info("Gimbal control acquired successfully.")
                gimbal_control_taken = True
            except Exception as e:
                LOGGER.warning(
                    f"Failed to take gimbal control: {e}. Proceeding without gimbal control."
                )
                state.gimbal_available = False
        else:
            LOGGER.info("Gimbal not available, skipping gimbal control setup.")

        LOGGER.info("\n--- Flight Control Active ---")
        LOGGER.info("Press ESC in the GUI window to stop and land.")
        LOGGER.info("-----------------------------\n")

        last_print_time = asyncio.get_event_loop().time()
        last_telemetry_update = 0.0
        control_error_count = 0
        max_control_errors = 10

        while not state.exit_flag:
            current_time: float = asyncio.get_event_loop().time()

            # Update telemetry at 10Hz
            if current_time - last_telemetry_update >= 0.1:
                await update_telemetry(drone, state)
                last_telemetry_update = current_time

            try:
                # Send velocity command to drone
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(
                        state.velocity_forward,
                        state.velocity_right,
                        state.velocity_down,
                        state.yawspeed,
                    )
                )

                # Send gimbal commands if needed
                if state.gimbal_available and (
                    abs(state.gimbal_pitch_rate) > 0.01
                    or abs(state.gimbal_yaw_rate) > 0.01
                ):
                    await control_gimbal_angular_rates(drone, config, state)

                control_error_count = 0

                # Log status occasionally
                if current_time - last_print_time >= 1.0:
                    log_msg = (
                        f"Velocity (F,R,D,Y): {state.velocity_forward:+.1f}, {state.velocity_right:+.1f}, "
                        f"{state.velocity_down:+.1f}, {state.yawspeed:+.1f}"
                    )
                    if state.gimbal_available:
                        log_msg += f" | Gimbal (P,Y): {state.gimbal_pitch_rate:+.1f}, {state.gimbal_yaw_rate:+.1f}"
                    LOGGER.debug(log_msg)
                    last_print_time = current_time

                await asyncio.sleep(0.05)

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
                await asyncio.sleep(0.1)

    except Exception as e:
        LOGGER.error(f"Error during flight control: {e}", exc_info=True)
        state.exit_flag = True
    finally:
        LOGGER.info("Exiting control loop. Stopping systems...")

        try:
            await drone.offboard.stop()
            LOGGER.info("Offboard mode stopped.")
        except Exception as e:
            LOGGER.error(f"Error stopping offboard: {e}")

        if gimbal_control_taken:
            try:
                LOGGER.info(
                    f"Releasing control of gimbal ID: {config.target_gimbal_id}..."
                )
                async for _ in drone.gimbal.release_control(
                    gimbal_id=config.target_gimbal_id
                ):
                    break
                LOGGER.info("Gimbal control released.")
            except Exception as e:
                LOGGER.error(f"Error releasing gimbal control: {e}")
        elif state.gimbal_available:
            LOGGER.info("Gimbal control was not taken, skipping release.")

        LOGGER.info("Flight control systems stopped.")


async def main():
    """Main function to orchestrate the drone control system"""
    # Initialize configuration and state
    config = DroneConfig()
    state = ControlState()

    # Connect to the drone
    drone = await connect_drone(config)
    if drone is None:
        LOGGER.error("Failed to connect to drone. Exiting.")
        return

    # Check for gimbal devices
    await check_gimbal_devices(drone, config, state)

    # Perform health check
    if not await check_drone_health(drone):
        LOGGER.error("Drone health check failed. Cannot proceed to arm.")
        return

    # Arm the drone
    if not await arm_drone(drone):
        LOGGER.error("Failed to arm drone. Exiting.")
        return

    # Take off
    if not await takeoff_drone(drone, config):
        LOGGER.error("Failed to take off. Attempting to land.")
        await land_drone(drone)
        return

    # Start the GUI thread
    start_gui_thread(config, state)

    # Run the main control loop
    await run_manual_control_loop(drone, config, state)

    # Land the drone when finished
    await land_drone(drone)

    # Clean up and finish
    await asyncio.sleep(1)
    LOGGER.info("Program finished.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        LOGGER.info("Program terminated by user.")
    except Exception as e:
        LOGGER.error(f"Unhandled exception in main: {e}", exc_info=True)
