#!/usr/bin/env python3
"""
MAVSDK Drone Control + Setpoint Joystick Display
"""
import asyncio
import numpy as np
from threading import Thread
import tkinter as tk
from tkinter import ttk
import signal

from mavsdk import System


class AsyncDroneController:
    def __init__(self):
        self.drone = System()
        self.is_connected = False
        self.is_armed = False
        self.is_in_air = False
        # 狀態資訊
        self.initial_position = None
        self.current_position = {"lat": 0.0, "lon": 0.0, "alt": 0.0}
        self.local_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.velocity = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
        self.attitude = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.distance_traveled = 0.0
        self.telemetry_tasks = []

        # setpoint 預設值，GUI/AI 可隨時更新
        self.setpoint = {"throttle": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}

    async def connect(self):
        try:
            print("Connecting to drone...")
            await self.drone.connect(system_address="udp://:14540")
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    print("Drone discovered!")
                    self.is_connected = True
                    break
            await self.start_telemetry()
            print("Telemetry started.")
        except Exception as e:
            print(f"Connection error: {e}")
            self.is_connected = False

    async def start_telemetry(self):
        self.telemetry_tasks = [
            asyncio.create_task(self._position_telemetry()),
            asyncio.create_task(self._velocity_ned_telemetry()),
            asyncio.create_task(self._attitude_telemetry()),
        ]

    async def _position_telemetry(self):
        try:
            async for position in self.drone.telemetry.position():
                self.current_position = {
                    "lat": position.latitude_deg,
                    "lon": position.longitude_deg,
                    "alt": position.relative_altitude_m,
                }
                if self.initial_position is None and position.relative_altitude_m > 0.1:
                    self.initial_position = self.current_position.copy()
        except Exception as e:
            print(f"Position telemetry error: {e}")

    async def _velocity_ned_telemetry(self):
        try:
            async for pos_vel in self.drone.telemetry.position_velocity_ned():
                self.local_position = {
                    "x": pos_vel.position.north_m,
                    "y": pos_vel.position.east_m,
                    "z": pos_vel.position.down_m,
                }
                self.velocity = {
                    "vx": pos_vel.velocity.north_m_s,
                    "vy": pos_vel.velocity.east_m_s,
                    "vz": pos_vel.velocity.down_m_s,
                }
                if self.initial_position:
                    dx = self.local_position["x"]
                    dy = self.local_position["y"]
                    dz = self.local_position["z"]
                    self.distance_traveled = np.sqrt(dx**2 + dy**2 + dz**2)
        except Exception as e:
            print(f"Velocity telemetry error: {e}")

    async def _attitude_telemetry(self):
        try:
            async for attitude in self.drone.telemetry.attitude_euler():
                self.attitude = {
                    "roll": attitude.roll_deg,
                    "pitch": attitude.pitch_deg,
                    "yaw": attitude.yaw_deg,
                }
        except Exception as e:
            print(f"Attitude telemetry error: {e}")

    async def arm(self):
        try:
            print("Arming...")
            await self.drone.action.arm()
            self.is_armed = True
            print("Armed successfully")
        except Exception as e:
            print(f"Arm failed: {e}")
            raise

    async def disarm(self):
        try:
            print("Disarming...")
            await self.drone.action.disarm()
            self.is_armed = False
            print("Disarmed successfully")
        except Exception as e:
            print(f"Disarm failed: {e}")

    async def takeoff(self, altitude=2.0):
        try:
            print(f"Taking off to {altitude}m...")
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()
            self.is_in_air = True
            print("Takeoff initiated")
        except Exception as e:
            print(f"Takeoff failed: {e}")
            raise

    async def land(self):
        try:
            print("Landing...")
            await self.drone.action.land()
            self.is_in_air = False
            print("Landing initiated")
        except Exception as e:
            print(f"Land failed: {e}")
            raise

    async def cleanup(self):
        for task in self.telemetry_tasks:
            task.cancel()
        await asyncio.gather(*self.telemetry_tasks, return_exceptions=True)

    def set_joystick_setpoint(self, throttle, yaw, pitch, roll):
        self.setpoint = {"throttle": throttle, "yaw": yaw, "pitch": pitch, "roll": roll}


class DroneGUI:
    def __init__(self):
        self.controller = AsyncDroneController()
        self.root = tk.Tk()
        self.root.title("MAVSDK Drone Control - Setpoint Joystick Display")
        self.root.geometry("500x700")
        self.loop = asyncio.new_event_loop()
        self.async_thread = Thread(target=self._run_event_loop, daemon=True)
        self.async_thread.start()
        self.setup_gui()
        self.update_display()

        self.shutdown_called = False  # 防止重複
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)
        signal.signal(signal.SIGINT, self.sigint_handler)

    def _run_event_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def run_async(self, coro):
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        return future

    def setup_gui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Status
        status_frame = ttk.LabelFrame(main_frame, text="Status", padding="10")
        status_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        self.status_label = ttk.Label(
            status_frame, text="Disconnected", font=("Arial", 14, "bold")
        )
        self.status_label.pack()

        # Position
        pos_frame = ttk.LabelFrame(main_frame, text="Position", padding="10")
        pos_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        ttk.Label(pos_frame, text="Initial:").grid(row=0, column=0, sticky=tk.W)
        self.initial_label = ttk.Label(pos_frame, text="Not set")
        self.initial_label.grid(row=0, column=1, sticky=tk.W, padx=10)
        ttk.Label(pos_frame, text="Current:").grid(row=1, column=0, sticky=tk.W)
        self.current_label = ttk.Label(pos_frame, text="0.0, 0.0, 0.0")
        self.current_label.grid(row=1, column=1, sticky=tk.W, padx=10)
        ttk.Label(pos_frame, text="Local (NED):").grid(row=2, column=0, sticky=tk.W)
        self.local_label = ttk.Label(pos_frame, text="0.0, 0.0, 0.0")
        self.local_label.grid(row=2, column=1, sticky=tk.W, padx=10)
        ttk.Label(pos_frame, text="Distance:").grid(row=3, column=0, sticky=tk.W)
        self.distance_label = ttk.Label(pos_frame, text="0.0 m")
        self.distance_label.grid(row=3, column=1, sticky=tk.W, padx=10)

        # Velocity
        vel_frame = ttk.LabelFrame(main_frame, text="Velocity", padding="10")
        vel_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        ttk.Label(vel_frame, text="3-axis (m/s):").grid(row=0, column=0, sticky=tk.W)
        self.velocity_label = ttk.Label(vel_frame, text="0.0, 0.0, 0.0")
        self.velocity_label.grid(row=0, column=1, sticky=tk.W, padx=10)
        ttk.Label(vel_frame, text="Ground speed:").grid(row=1, column=0, sticky=tk.W)
        self.speed_label = ttk.Label(vel_frame, text="0.0 m/s")
        self.speed_label.grid(row=1, column=1, sticky=tk.W, padx=10)

        # Joystick (Setpoint)
        joy_frame = ttk.LabelFrame(main_frame, text="Setpoint Joystick", padding="10")
        joy_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        self.joy_canvas = tk.Canvas(joy_frame, width=300, height=150, bg="lightgray")
        self.joy_canvas.pack()
        self.left_stick = self.joy_canvas.create_oval(65, 65, 85, 85, fill="blue")
        self.right_stick = self.joy_canvas.create_oval(215, 65, 235, 85, fill="red")
        self.joy_value_label = ttk.Label(
            joy_frame, text="Joystick Values: T:0.00 Y:0.00 P:0.00 R:0.00"
        )
        self.joy_value_label.pack(pady=5)

        # Controls
        control_frame = ttk.LabelFrame(main_frame, text="Controls", padding="10")
        control_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        self.connect_btn = ttk.Button(
            control_frame, text="Connect", command=self.connect_drone
        )
        self.connect_btn.grid(row=0, column=0, padx=5)
        self.arm_btn = ttk.Button(
            control_frame, text="Arm", command=self.arm_drone, state="disabled"
        )
        self.arm_btn.grid(row=0, column=1, padx=5)
        self.takeoff_btn = ttk.Button(
            control_frame, text="Takeoff", command=self.takeoff_drone, state="disabled"
        )
        self.takeoff_btn.grid(row=0, column=2, padx=5)
        self.land_btn = ttk.Button(
            control_frame, text="Land", command=self.land_drone, state="disabled"
        )
        self.land_btn.grid(row=0, column=3, padx=5)
        self.disarm_btn = ttk.Button(
            control_frame, text="Disarm", command=self.disarm_drone, state="disabled"
        )
        self.disarm_btn.grid(row=0, column=4, padx=5)

    def update_display(self):
        # 狀態
        status = "Connected" if self.controller.is_connected else "Disconnected"
        if self.controller.is_armed:
            status += " | Armed"
        if self.controller.is_in_air:
            status += " | Flying"
        self.status_label.config(
            text=status, foreground="green" if self.controller.is_connected else "red"
        )
        # 位置
        if self.controller.initial_position:
            self.initial_label.config(
                text=f"{self.controller.initial_position['lat']:.6f}, "
                f"{self.controller.initial_position['lon']:.6f}, "
                f"{self.controller.initial_position['alt']:.1f}"
            )
        self.current_label.config(
            text=f"{self.controller.current_position['lat']:.6f}, "
            f"{self.controller.current_position['lon']:.6f}, "
            f"{self.controller.current_position['alt']:.1f}"
        )
        self.local_label.config(
            text=f"{self.controller.local_position['x']:.1f}, "
            f"{self.controller.local_position['y']:.1f}, "
            f"{self.controller.local_position['z']:.1f}"
        )
        self.distance_label.config(text=f"{self.controller.distance_traveled:.1f} m")
        # 速度
        self.velocity_label.config(
            text=f"{self.controller.velocity['vx']:.1f}, "
            f"{self.controller.velocity['vy']:.1f}, "
            f"{self.controller.velocity['vz']:.1f}"
        )
        speed = np.sqrt(
            self.controller.velocity["vx"] ** 2 + self.controller.velocity["vy"] ** 2
        )
        self.speed_label.config(text=f"{speed:.1f} m/s")

        # Joystick (Setpoint)
        sp = self.controller.setpoint
        t = sp["throttle"]
        y = sp["yaw"]
        p = sp["pitch"]
        r = sp["roll"]
        left_x = 75 + y * 40
        left_y = 75 + t * 40
        right_x = 225 + r * 40
        right_y = 75 + p * 40
        self.joy_canvas.coords(
            self.left_stick, left_x - 10, left_y - 10, left_x + 10, left_y + 10
        )
        self.joy_canvas.coords(
            self.right_stick, right_x - 10, right_y - 10, right_x + 10, right_y + 10
        )
        self.joy_value_label.config(
            text=f"Joystick Values: T:{t:.2f} Y:{y:.2f} P:{p:.2f} R:{r:.2f}"
        )
        self.root.after(100, self.update_display)

    def connect_drone(self):
        self.connect_btn.config(state="disabled")

        def on_connected(future):
            try:
                future.result()
                self.root.after(0, self.enable_controls)
            except Exception as e:
                print(f"Connection failed: {e}")
                self.root.after(0, lambda: self.connect_btn.config(state="normal"))

        future = self.run_async(self.controller.connect())
        future.add_done_callback(on_connected)

    def enable_controls(self):
        self.arm_btn.config(state="normal")
        self.takeoff_btn.config(state="normal")
        self.land_btn.config(state="normal")
        self.disarm_btn.config(state="normal")

    def arm_drone(self):
        def on_complete(future):
            try:
                future.result()
            except Exception as e:
                print(f"Arm failed: {e}")

        future = self.run_async(self.controller.arm())
        future.add_done_callback(on_complete)

    def disarm_drone(self):
        def on_complete(future):
            try:
                future.result()
            except Exception as e:
                print(f"Disarm failed: {e}")

        future = self.run_async(self.controller.disarm())
        future.add_done_callback(on_complete)

    def takeoff_drone(self):
        def on_complete(future):
            try:
                future.result()
            except Exception as e:
                print(f"Takeoff failed: {e}")

        future = self.run_async(self.controller.takeoff(2.0))
        future.add_done_callback(on_complete)

    def land_drone(self):
        def on_complete(future):
            try:
                future.result()
            except Exception as e:
                print(f"Land failed: {e}")

        future = self.run_async(self.controller.land())
        future.add_done_callback(on_complete)

    def run(self):
        try:
            self.root.mainloop()
        finally:
            future = self.run_async(self.controller.cleanup())
            future.result(timeout=2.0)
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.async_thread.join(timeout=2.0)

    def shutdown(self):
        """正確清除資源並退出 GUI/event loop/thread"""
        if self.shutdown_called:
            return
        self.shutdown_called = True
        try:
            # 強制讓 mainloop 跳出
            if self.root:
                self.root.quit()
                self.root.update()
        except Exception:
            pass
        try:
            future = self.run_async(self.controller.cleanup())
            future.result(timeout=2.0)
        except Exception:
            pass
        try:
            self.loop.call_soon_threadsafe(self.loop.stop)
        except Exception:
            pass
        try:
            self.async_thread.join(timeout=2.0)
        except Exception:
            pass

    def sigint_handler(self, signum, frame):
        self.shutdown()


def main():
    gui = DroneGUI()
    try:
        gui.run()
    except KeyboardInterrupt:
        gui.shutdown()


if __name__ == "__main__":
    main()
