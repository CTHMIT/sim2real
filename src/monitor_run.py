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

from src.utils.logger import LOGGER  # type: ignore

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

    max_speed: float = Field(2.0, description="Maximum forward/right/down speed (m/s)")
    max_yaw_speed: float = Field(90.0, description="Maximum yaw speed (deg/s)")
    max_gimbal_rate: float = Field(
        30.0, description="Maximum gimbal angular rate (deg/s)"
    )

    acceleration_factor: float = Field(1.0, description="Control acceleration factor")
    decay_factor: float = Field(
        0.9, description="Control decay factor when keys are released"
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


async def check_gimbal_devices(drone: System, config: DroneConfig):
    LOGGER.info("Checking for available gimbal devices...")
    try:

        gimbal_list = None
        async for gimbals in drone.gimbal.gimbal_list():
            gimbal_list = gimbals
            break

        if gimbal_list and gimbal_list.gimbals:
            LOGGER.info(f"Found {len(gimbal_list.gimbals)} gimbal device(s)")
            for gimbal in gimbal_list.gimbals:
                LOGGER.info(
                    f"  - ID: {gimbal.gimbal_id}, Model: {gimbal.model_name}, Vendor: {gimbal.vendor_name}"
                )

            if config.target_gimbal_id != gimbal_list.gimbals[0].gimbal_id:
                LOGGER.info(
                    f"Updating target gimbal ID to: {gimbal_list.gimbals[0].gimbal_id}"
                )
                config.target_gimbal_id = gimbal_list.gimbals[0].gimbal_id
        else:
            LOGGER.info("No gimbal devices found.")
    except Exception as e:
        LOGGER.info(f"Error checking gimbals: {e}")


async def control_gimbal_angular_rates(
    drone: System, config: DroneConfig, state: ControlState
):
    try:

        try:
            await drone.gimbal.set_angular_rates(
                gimbal_id=config.target_gimbal_id,
                roll_rate_deg_s=0.0,
                pitch_rate_deg_s=state.gimbal_pitch_rate,
                yaw_rate_deg_s=state.gimbal_yaw_rate,
                gimbal_mode=config.gimbal_mode,
                send_mode=config.gimbal_send_mode,
            )
        except TypeError:

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


class DroneControlGUI:
    def __init__(self, root: tk.Tk, config: DroneConfig, state: ControlState):
        self.root = root
        self.config = config
        self.state = state
        self.root.title("Drone Keyboard Control")
        self.pressed_keys: Set[str] = set()
        self.root.geometry("500x450")

        tk.Label(root, text="Drone Keyboard Control", font=("Arial", 16)).pack(pady=10)

        controls_frame = tk.Frame(root)
        controls_frame.pack(pady=5)
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
            "Exit:",
            "  Esc: Stop Control & Land",
        ]
        for ctrl in controls:
            tk.Label(controls_frame, text=ctrl, anchor="w", justify=tk.LEFT).pack(
                fill="x", padx=20
            )

        self.status_label_drone = tk.Label(
            root, text="Drone Status: Initializing...", font=("Arial", 10)
        )
        self.status_label_drone.pack(pady=5)
        self.status_label_gimbal = tk.Label(
            root, text="Gimbal Status: Initializing...", font=("Arial", 10)
        )
        self.status_label_gimbal.pack(pady=5)

        tk.Button(root, text="Stop Control (ESC)", command=self.exit_control).pack(
            pady=10
        )

        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)

        self.update_control_loop()

    def on_key_press(self, event):
        key = event.keysym.lower()
        self.pressed_keys.add(key)
        if key == "escape":
            self.exit_control()

    def on_key_release(self, event):
        key = event.keysym.lower()
        self.pressed_keys.discard(key)

    def update_control_loop(self):
        self.process_keys()
        drone_text = (
            f"Drone Vel (Fwd, Right, Down): "
            f"{self.state.velocity_forward:+.1f}, {self.state.velocity_right:+.1f}, {self.state.velocity_down:+.1f} m/s | "
            f"Yaw Speed: {self.state.yawspeed:+.1f} deg/s"
        )
        gimbal_text = (
            f"Gimbal Rate (Pitch, Yaw): "
            f"{self.state.gimbal_pitch_rate:+.1f}, {self.state.gimbal_yaw_rate:+.1f} deg/s"
        )

        self.status_label_drone.config(text=drone_text)
        self.status_label_gimbal.config(text=gimbal_text)

        if self.root.winfo_exists():
            self.root.after(50, self.update_control_loop)

    def process_keys(self):
        cfg = self.config
        state = self.state
        keys = self.pressed_keys

        # Forward/Backward (W/S)
        if "w" in keys:
            state.velocity_forward = min(
                state.velocity_forward + cfg.speed_increment * cfg.acceleration_factor,
                cfg.max_speed,
            )
        elif "s" in keys:
            state.velocity_forward = max(
                state.velocity_forward - cfg.speed_increment * cfg.acceleration_factor,
                -cfg.max_speed,
            )
        else:
            state.velocity_forward *= cfg.decay_factor
            if abs(state.velocity_forward) < cfg.zero_threshold:
                state.velocity_forward = 0.0

        # Right/Left (A/D)
        if "d" in keys:
            state.velocity_right = min(
                state.velocity_right + cfg.speed_increment * cfg.acceleration_factor,
                cfg.max_speed,
            )
        elif "a" in keys:
            state.velocity_right = max(
                state.velocity_right - cfg.speed_increment * cfg.acceleration_factor,
                -cfg.max_speed,
            )
        else:
            state.velocity_right *= cfg.decay_factor
            if abs(state.velocity_right) < cfg.zero_threshold:
                state.velocity_right = 0.0

        # Down/Up (R/F)
        if "f" in keys:
            state.velocity_down = min(
                state.velocity_down + cfg.speed_increment * cfg.acceleration_factor,
                cfg.max_speed,
            )
        elif "r" in keys:
            state.velocity_down = max(
                state.velocity_down - cfg.speed_increment * cfg.acceleration_factor,
                -cfg.max_speed,
            )
        else:
            state.velocity_down *= cfg.decay_factor
            if abs(state.velocity_down) < cfg.zero_threshold:
                state.velocity_down = 0.0

        # Yaw Left/Right (Q/E)
        if "e" in keys:
            state.yawspeed = min(
                state.yawspeed + cfg.yaw_increment * cfg.acceleration_factor,
                cfg.max_yaw_speed,
            )
        elif "q" in keys:
            state.yawspeed = max(
                state.yawspeed - cfg.yaw_increment * cfg.acceleration_factor,
                -cfg.max_yaw_speed,
            )
        else:
            state.yawspeed *= cfg.decay_factor
            if abs(state.yawspeed) < cfg.zero_threshold:
                state.yawspeed = 0.0

        # Pitch Up/Down (I/K)
        if "i" in keys:
            state.gimbal_pitch_rate = min(
                state.gimbal_pitch_rate
                + cfg.gimbal_rate_increment * cfg.acceleration_factor,
                cfg.max_gimbal_rate,
            )
        elif "k" in keys:
            state.gimbal_pitch_rate = max(
                state.gimbal_pitch_rate
                - cfg.gimbal_rate_increment * cfg.acceleration_factor,
                -cfg.max_gimbal_rate,
            )
        else:
            state.gimbal_pitch_rate *= cfg.decay_factor
            if abs(state.gimbal_pitch_rate) < cfg.zero_threshold:
                state.gimbal_pitch_rate = 0.0

        # Yaw Left/Right (J/L)
        if "l" in keys:
            state.gimbal_yaw_rate = min(
                state.gimbal_yaw_rate
                + cfg.gimbal_rate_increment * cfg.acceleration_factor,
                cfg.max_gimbal_rate,
            )
        elif "j" in keys:
            state.gimbal_yaw_rate = max(
                state.gimbal_yaw_rate
                - cfg.gimbal_rate_increment * cfg.acceleration_factor,
                -cfg.max_gimbal_rate,
            )
        else:
            state.gimbal_yaw_rate *= cfg.decay_factor
            if abs(state.gimbal_yaw_rate) < cfg.zero_threshold:
                state.gimbal_yaw_rate = 0.0

    def exit_control(self):
        LOGGER.info("Exit requested via GUI.")
        self.state.exit_flag = True
        if self.root.winfo_exists():
            self.root.destroy()


def start_gui_thread(config: DroneConfig, state: ControlState):
    def gui_runner():
        try:
            root = tk.Tk()
            DroneControlGUI(root, config, state)
            root.mainloop()
        except Exception as e:
            LOGGER.info(f"Error in GUI thread: {e}")
            state.exit_flag = True

    gui_thread = threading.Thread(target=gui_runner, daemon=True)
    gui_thread.start()
    return gui_thread


async def run_manual_control_loop(
    drone: System, config: DroneConfig, state: ControlState
):
    LOGGER.info("Entering manual control mode...")

    try:

        for _ in range(5):
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
            await asyncio.sleep(0.1)

        try:
            await drone.offboard.start()
            LOGGER.info("Offboard mode started successfully.")
        except OffboardError as e:
            LOGGER.info(f"Offboard error: {e}. Attempting to fix...")

            try:

                if not await drone.telemetry.armed():
                    LOGGER.info("Rearming before offboard mode...")
                    await drone.action.arm()

                await asyncio.sleep(1)
                await drone.offboard.start()
                LOGGER.info("Offboard mode started on second attempt.")
            except Exception as retry_error:
                LOGGER.info(f"Failed to start offboard mode: {retry_error}")
                raise

        try:
            try:
                await drone.gimbal.take_control(
                    gimbal_id=config.target_gimbal_id, control_mode=ControlMode.PRIMARY
                )
                LOGGER.info("Gimbal control acquired.")
            except TypeError:
                LOGGER.info("Attempting alternative gimbal control method...")
                async for _ in drone.gimbal.take_control(
                    gimbal_id=config.target_gimbal_id, control_mode=ControlMode.PRIMARY
                ):
                    LOGGER.info("Gimbal control method executed through generator.")
                    break
        except Exception as e:
            LOGGER.info(
                f"Failed to take gimbal control: {e}. Proceeding without gimbal control."
            )

        LOGGER.info("\n--- Manual Control Active ---")
        LOGGER.info("Use keyboard controls for drone and gimbal movement.")
        LOGGER.info("Press ESC in the control window to stop and land.")
        LOGGER.info("-----------------------------\n")

        last_print_time = asyncio.get_event_loop().time()
        control_error_count = 0
        max_control_errors = 5

        while not state.exit_flag:
            try:
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(
                        state.velocity_forward,
                        state.velocity_right,
                        state.velocity_down,
                        state.yawspeed,
                    )
                )

                control_error_count = 0

                if (
                    abs(state.gimbal_pitch_rate) > 0.01
                    or abs(state.gimbal_yaw_rate) > 0.01
                ):
                    await control_gimbal_angular_rates(drone, config, state)

                current_time = asyncio.get_event_loop().time()
                if current_time - last_print_time >= 1.0:
                    LOGGER.info(
                        f"Drone Vel (F,R,D,Y): {state.velocity_forward:+.1f}, {state.velocity_right:+.1f}, {state.velocity_down:+.1f}, {state.yawspeed:+.1f} | "
                        f"Gimbal Rate (P,Y): {state.gimbal_pitch_rate:+.1f}, {state.gimbal_yaw_rate:+.1f}"
                    )
                    last_print_time = current_time

            except Exception as e:
                control_error_count += 1
                LOGGER.info(
                    f"Control command error ({control_error_count}/{max_control_errors}): {e}"
                )

                if control_error_count >= max_control_errors:
                    LOGGER.info("Too many control errors, stopping control loop")
                    break

            await asyncio.sleep(0.05)

    except Exception as e:
        LOGGER.info(f"Error during manual control: {e}")
    finally:
        LOGGER.info("Stopping control systems...")

        try:
            await drone.offboard.stop()
            LOGGER.info("Offboard mode stopped.")
        except Exception as e:
            LOGGER.info(f"Error stopping offboard: {e}")

        try:
            try:
                await drone.gimbal.release_control(gimbal_id=config.target_gimbal_id)
            except TypeError:

                async for _ in drone.gimbal.release_control(
                    gimbal_id=config.target_gimbal_id
                ):
                    break
            LOGGER.info("Gimbal control released.")
        except Exception as e:
            LOGGER.info(f"Error releasing gimbal: {e}")

        LOGGER.info("Manual control loop finished.")


async def main():
    config = DroneConfig()
    state = ControlState()

    drone = await connect_drone(config)
    if drone is None:
        LOGGER.info("Failed to connect to drone. Exiting.")
        return

    await check_gimbal_devices(drone, config)

    if not await check_drone_health(drone):
        LOGGER.info("Drone health check failed. Cannot proceed to arm.")
        return

    if not await arm_drone(drone):
        LOGGER.info("Failed to arm drone. Exiting.")
        return

    if not await takeoff_drone(drone, config):
        LOGGER.info("Failed to take off. Attempting to land.")
        await land_drone(drone)
        return

    start_gui_thread(config, state)
    await run_manual_control_loop(drone, config, state)
    await land_drone(drone)

    await asyncio.sleep(1)
    LOGGER.info("Program finished.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        LOGGER.info("Program terminated by user.")
    except Exception as e:
        LOGGER.info(f"Unhandled exception in main: {e}")
