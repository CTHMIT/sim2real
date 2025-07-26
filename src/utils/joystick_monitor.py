import tkinter as tk
from tkinter import ttk, messagebox
import threading
import asyncio
import time
import math
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, List
import serial
import serial.tools.list_ports

try:
    from pymavlink import mavutil
except ImportError:
    print("請安裝 pymavlink")
    exit(1)
try:
    from mavsdk import System
except ImportError:
    print("請安裝 mavsdk")
    exit(1)
try:
    from evdev import InputDevice, ecodes

    JOY_OK = True
except ImportError:
    JOY_OK = False
try:
    import tkintermapview

    MAP_WIDGET_AVAILABLE = True
except ImportError:
    MAP_WIDGET_AVAILABLE = False


# ------------------------
# --- 狀態資料類 ---
# ------------------------
@dataclass
class TelemetryData:
    armed: str = "未知"
    flight_mode: str = "未知"
    gps_fix: str = "未知"
    num_sats: int = 0
    is_rtl: bool = False
    altitude_m: float = 0.0
    ang_vel: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    attitude: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    position: Optional[Tuple[float, float]] = None
    is_connected: bool = False
    battery_percent: float = 100.0
    home_position: Optional[Tuple[float, float]] = None
    last_heartbeat: float = 0.0
    px4_mode: int = 0
    gps_source: str = "SITL"  # 改為SITL作為預設
    mode_switch_error: str = ""


@dataclass
class JoystickState:
    axes: Dict[str, float]
    action: Optional[str] = None


@dataclass
class ControlSettings:
    use_joystick: bool = True
    joystick_connected: bool = False


telemetry_data = TelemetryData()
joystick_state = JoystickState(axes={})
control_settings = ControlSettings()

# ------------------------
# --- 飛行模式、地圖 ---
# ------------------------
FLIGHT_MODES = {
    "Manual": {
        "color": "#e74c3c",
        "description": "完全手動控制",
        "requires_gps": False,
        "requires_armed": False,
    },
    "Stabilized": {
        "color": "#3498db",
        "description": "穩定輔助模式",
        "requires_gps": False,
        "requires_armed": False,
    },
    "Acro": {
        "color": "#9b59b6",
        "description": "特技飛行模式",
        "requires_gps": False,
        "requires_armed": True,
    },
    "Altitude": {
        "color": "#27ae60",
        "description": "定高飛行模式",
        "requires_gps": False,
        "requires_armed": False,
    },
    "Position": {
        "color": "#2ecc71",
        "description": "GPS定位模式",
        "requires_gps": True,
        "requires_armed": False,
    },
    "Offboard": {
        "color": "#e67e22",
        "description": "外部控制模式",
        "requires_gps": True,
        "requires_armed": True,
    },
    "Hold": {
        "color": "#34495e",
        "description": "懸停保持模式",
        "requires_gps": True,
        "requires_armed": True,
    },
    "Mission": {
        "color": "#8e44ad",
        "description": "自動任務模式",
        "requires_gps": True,
        "requires_armed": True,
    },
    "Return": {
        "color": "#c0392b",
        "description": "自動返航模式",
        "requires_gps": True,
        "requires_armed": True,
    },
}
PX4_MODE_MAP = {
    "Manual": (1, 0),
    "Altitude": (2, 0),
    "Position": (3, 0),
    "Acro": (4, 0),
    "Offboard": (6, 0),
    "Stabilized": (7, 0),
    "Mission": (4, 4),
    "Hold": (4, 3),
    "Return": (4, 2),
}
PX4_MODE_REVERSE_MAP = {v: k for k, v in PX4_MODE_MAP.items()}
MAP_PROVIDERS_UI = {
    "OpenStreetMap": "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png",
    "ESRI Satellite": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
}
DEFAULT_ZOOM = 18


def encode_px4_mode(main_mode: int, sub_mode: int = 0) -> int:
    # PX4 custom_mode = (main_mode << 16) | (sub_mode << 8)
    return (main_mode << 16) | (sub_mode << 8)


def decode_px4_mode(custom_mode: int) -> str:
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 8) & 0xFF
    for mode_name, (main, sub) in PX4_MODE_MAP.items():
        if main == main_mode and sub == sub_mode:
            return mode_name
    return "Unknown"


# ------------------------
# --- GPS 模組 ---
# ------------------------
class GPSSimulator:
    def __init__(self, lat=None, lon=None):
        self.lat = lat
        self.lon = lon
        self.home_lat = lat
        self.home_lon = lon
        self.speed = 0.00001

    def update(self, vx, vy):
        if self.lat is not None and self.lon is not None:
            self.lat += vy * self.speed
            self.lon += vx * self.speed

    def get_position(self):
        if self.lat is not None and self.lon is not None:
            return (self.lat, self.lon)
        return None

    def reset_to_home(self):
        if self.home_lat is not None and self.home_lon is not None:
            self.lat, self.lon = self.home_lat, self.home_lon

    def set_position(self, lat, lon):
        """設定虛擬GPS位置"""
        self.lat, self.lon = lat, lon
        if self.home_lat is None or self.home_lon is None:
            self.home_lat, self.home_lon = lat, lon


class GPSManager:
    def __init__(self):
        self.real_gps_port = None
        self.real_gps_active = False
        self.virtual_gps = GPSSimulator()
        self.gps_ports = []
        self.scan_thread = None
        self.scanning = False
        self.last_sitl_position = None

        # 不設定初始位置，等待從SITL或其他源獲取
        telemetry_data.position = None

    def scan_gps_devices(self) -> List[str]:
        self.gps_ports = []
        ports = serial.tools.list_ports.comports()
        gps_keywords = ["GPS", "GNSS", "u-blox", "ublox", "CP210", "FTDI"]
        for port in ports:
            if any(
                keyword.lower() in (port.description or "").lower()
                for keyword in gps_keywords
            ):
                self.gps_ports.append(port.device)
        return self.gps_ports

    def connect_real_gps(self, port: str) -> bool:
        try:
            self.real_gps_port = serial.Serial(port, 9600, timeout=1)
            self.real_gps_active = True
            telemetry_data.gps_source = "真實GPS"
            self.scanning = True
            self.scan_thread = threading.Thread(target=self._read_gps_data, daemon=True)
            self.scan_thread.start()
            print(f"[GPS] 已連接真實GPS: {port}")
            return True
        except Exception as e:
            print(f"[GPS] 連接GPS失敗 {port}: {e}")
            return False

    def _read_gps_data(self):
        while self.scanning and self.real_gps_active:
            try:
                if self.real_gps_port and self.real_gps_port.in_waiting:
                    line = (
                        self.real_gps_port.readline()
                        .decode("ascii", errors="ignore")
                        .strip()
                    )
                    if line.startswith("$GPGGA") or line.startswith("$GNGGA"):
                        self._parse_gga(line)
                time.sleep(0.1)
            except Exception as e:
                print(f"[GPS] 讀取錯誤: {e}")
                time.sleep(1)

    def _parse_gga(self, gga_sentence: str):
        try:
            parts = gga_sentence.split(",")
            if len(parts) >= 15 and parts[2] and parts[4]:
                lat_raw = float(parts[2])
                lat_deg = int(lat_raw / 100)
                lat_min = lat_raw % 100
                lat = lat_deg + lat_min / 60
                if parts[3] == "S":
                    lat = -lat

                lon_raw = float(parts[4])
                lon_deg = int(lon_raw / 100)
                lon_min = lon_raw % 100
                lon = lon_deg + lon_min / 60
                if parts[5] == "W":
                    lon = -lon

                # 真實GPS優先級最高，直接更新位置
                telemetry_data.position = (lat, lon)
                telemetry_data.num_sats = int(parts[7]) if parts[7] else 0
                telemetry_data.gps_fix = "3D" if parts[6] == "1" else "無訊號"
                print(f"[GPS] 真實GPS位置: {lat:.6f}, {lon:.6f}")
        except Exception as e:
            print(f"[GPS] 解析GGA錯誤: {e}")

    def disconnect_real_gps(self):
        self.scanning = False
        self.real_gps_active = False
        if self.real_gps_port:
            self.real_gps_port.close()
            self.real_gps_port = None
        print("[GPS] 已斷開真實GPS")
        self._update_gps_source()

    def update_sitl_position(self, lat: float, lon: float):
        """更新SITL位置（來自MAVSDK遙測）"""
        if not self.real_gps_active:  # 只有在沒有真實GPS時才使用SITL
            self.last_sitl_position = (lat, lon)
            telemetry_data.position = (lat, lon)
            telemetry_data.gps_source = "SITL"

    def update_virtual_gps(self, vx: float, vy: float):
        """更新虛擬GPS（手動控制）"""
        if not self.real_gps_active and not self.last_sitl_position:
            # 如果虛擬GPS還沒有位置，嘗試從當前遙測資料獲取
            if self.virtual_gps.lat is None and telemetry_data.position:
                lat, lon = telemetry_data.position
                self.virtual_gps.set_position(lat, lon)
                print(f"[GPS] 虛擬GPS初始化位置: {lat:.6f}, {lon:.6f}")

            self.virtual_gps.update(vx, vy)
            new_pos = self.virtual_gps.get_position()
            if new_pos:
                telemetry_data.position = new_pos
                telemetry_data.gps_source = "虛擬"
                print(f"[GPS] 虛擬GPS位置: {new_pos[0]:.6f}, {new_pos[1]:.6f}")

    def _update_gps_source(self):
        """根據當前狀態更新GPS來源顯示"""
        if self.real_gps_active:
            telemetry_data.gps_source = "真實GPS"
        elif self.last_sitl_position:
            telemetry_data.gps_source = "SITL"
        else:
            telemetry_data.gps_source = "虛擬"

    def get_position(self):
        return telemetry_data.position


gps_manager = GPSManager()


class PyMAVLinkController:
    def __init__(self):
        self.mavlink_master = None
        self.last_mode_request_time = 0
        self.mode_request_cooldown = 1.0
        self.connection_attempts = [
            "udp:0.0.0.0:14550",
            "udp:127.0.0.1:14550",
            "udp:0.0.0.0:14540",
            "udp:127.0.0.1:14540",
        ]
        self.control_thread = None
        self.control_running = False
        self.last_manual_control_time = 0
        self.manual_control_interval = 0.02
        self.last_return_request_time = 0
        self.return_request_cooldown = 0.5
        self.drone: Optional[System] = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.use_mavsdk_takeoff = True

    def attach_async(self, drone: System, loop):
        self.drone = drone
        self.loop = loop

    def log(self, msg, level="INFO"):
        print(f"[PyMAVLink-{level}] {msg}")

    def setup_connection(self):
        for c in self.connection_attempts:
            try:
                if self.mavlink_master:
                    self.mavlink_master.close()
                self.mavlink_master = mavutil.mavlink_connection(c)
                self.mavlink_master.wait_heartbeat(timeout=5)
                self.start_control_thread()
                print(f"[PyMAVLink] 連接成功: {c}")
                return True
            except Exception as e:
                print(f"[PyMAVLink] 連接失敗 {c}: {e}")
                continue
        self.mavlink_master = None
        return False

    def start_control_thread(self):
        if not self.control_thread or not self.control_thread.is_alive():
            self.control_running = True
            self.control_thread = threading.Thread(
                target=self.control_loop, daemon=True
            )
            self.control_thread.start()

    def stop_control_thread(self):
        self.control_running = False
        if self.control_thread:
            self.control_thread.join(timeout=1)

    def control_loop(self):
        while self.control_running:
            try:
                current_time = time.time()
                if not self.mavlink_master:
                    time.sleep(1)
                    continue
                if (
                    current_time - self.last_manual_control_time
                    >= self.manual_control_interval
                ):
                    self.send_manual_control()
                    self.last_manual_control_time = current_time
                self.process_messages()
                time.sleep(0.001)
            except Exception:
                time.sleep(0.1)

    def send_manual_control(self):
        if not self.mavlink_master or telemetry_data.armed != "已解鎖":
            return
        roll = joystick_state.axes.get("roll", 0.0)
        pitch = -joystick_state.axes.get("pitch", 0.0)
        yaw = joystick_state.axes.get("yaw", 0.0)
        throttle_raw = joystick_state.axes.get("throttle", -1.0)
        throttle = (throttle_raw + 1.0) / 2.0
        x = int(pitch * 1000)
        y = int(roll * 1000)
        z = int(throttle * 1000)
        r = int(yaw * 1000)
        buttons = 0
        try:
            self.mavlink_master.mav.manual_control_send(
                self.mavlink_master.target_system, x, y, z, r, buttons
            )
        except Exception:
            pass

    def process_messages(self):
        try:
            msg = self.mavlink_master.recv_match(blocking=False)
            if not msg:
                return
            if msg.get_type() == "HEARTBEAT":
                telemetry_data.last_heartbeat = time.time()
                telemetry_data.px4_mode = msg.custom_mode
                mode_name = decode_px4_mode(msg.custom_mode)
                if mode_name != "Unknown":
                    telemetry_data.flight_mode = mode_name
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                telemetry_data.armed = "已解鎖" if armed else "未解鎖"
        except Exception:
            pass

    def check_mode_requirements(
        self, mode_name: str, telemetry_data
    ) -> Tuple[bool, str]:
        if mode_name not in FLIGHT_MODES:
            return False, "未知模式"
        mode_info = FLIGHT_MODES[mode_name]
        if not telemetry_data.is_connected:
            return False, "無人機未連接"
        if mode_info["requires_gps"]:
            if not telemetry_data.position or telemetry_data.num_sats < 6:
                return False, f"需要 GPS 鎖定 (當前: {telemetry_data.num_sats} 衛星)"
        if mode_info["requires_armed"]:
            if telemetry_data.armed != "已解鎖":
                return False, "需要先解鎖無人機"
        if mode_name == "Mission" and not telemetry_data.home_position:
            return False, "Mission模式需要設定起始點"
        return True, "OK"

    def arm(self):
        if not self.mavlink_master:
            return False
        try:
            self.mavlink_master.mav.command_long_send(
                self.mavlink_master.target_system,
                self.mavlink_master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                21196,
                0,
                0,
                0,
                0,
                0,
            )
            return True
        except Exception:
            return False

    def disarm(self):
        if not self.mavlink_master:
            return False
        try:
            self.mavlink_master.mav.command_long_send(
                self.mavlink_master.target_system,
                self.mavlink_master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            return True
        except Exception:
            return False

    async def _takeoff_via_mavsdk(self, altitude: float):
        await self.drone.action.set_takeoff_altitude(altitude)
        await self.drone.action.arm()
        await self.drone.action.takeoff()

    def takeoff(self, altitude=10.0):
        if self.use_mavsdk_takeoff and self.drone and self.loop:
            try:
                fut = asyncio.run_coroutine_threadsafe(
                    self._takeoff_via_mavsdk(altitude), self.loop
                )
                fut.result(timeout=15)
                return True
            except asyncio.TimeoutError:
                pass
        if not self.arm():
            return False
        lat, lon = telemetry_data.position or (0, 0)
        if (lat, lon) == (0, 0):
            return False
        try:
            self.mavlink_master.mav.command_long_send(
                self.mavlink_master.target_system,
                self.mavlink_master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                float("nan"),
                lat,
                lon,
                altitude,
            )
            return True
        except Exception:
            return False

    def land(self):
        if not self.mavlink_master:
            return False
        try:
            self.mavlink_master.mav.command_long_send(
                self.mavlink_master.target_system,
                self.mavlink_master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            return True
        except Exception:
            return False

    def set_flight_mode(self, mode_name: str) -> bool:
        current_time = time.time()
        if current_time - self.last_mode_request_time < self.mode_request_cooldown:
            telemetry_data.mode_switch_error = "請等待冷卻時間"
            return False
        if mode_name not in PX4_MODE_MAP:
            telemetry_data.mode_switch_error = "不支援的模式名稱"
            return False

        if not self.mavlink_master and not self.setup_connection():
            return False
        try:
            main_mode, sub_mode = PX4_MODE_MAP[mode_name]
            custom_mode = encode_px4_mode(main_mode, sub_mode)

            self.mavlink_master.mav.set_mode_send(
                self.mavlink_master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode,
            )
            self.last_mode_request_time = current_time
            t0 = time.time()
            while time.time() - t0 < 5.0:
                msg = self.mavlink_master.recv_match(
                    type="HEARTBEAT", blocking=True, timeout=1
                )
                if msg and msg.custom_mode == custom_mode:
                    telemetry_data.flight_mode = mode_name
                    telemetry_data.px4_mode = custom_mode
                    telemetry_data.mode_switch_error = ""
                    return True
                time.sleep(0.1)
            telemetry_data.mode_switch_error = "模式切換超時"
            return False
        except Exception as e:
            telemetry_data.mode_switch_error = f"模式切換失敗: {e}"
            return False

    def return_to_launch_immediate(self):
        """立即執行返航，使用 MAVLink 返航命令而非模式切換"""
        if not self.mavlink_master:
            return False, "MAVLink 未連接"

        current_time = time.time()
        if current_time - self.last_return_request_time < self.return_request_cooldown:
            return False, "返航請求太頻繁，請稍後再試"

        try:
            # 方法1：使用專門的返航命令（推薦）
            self.mavlink_master.mav.command_long_send(
                self.mavlink_master.target_system,
                self.mavlink_master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,  # 返航命令
                0,  # confirmation
                0,
                0,
                0,
                0,
                0,
                0,
                0,  # 參數（對返航命令大多數參數為0）
            )

            self.last_return_request_time = current_time
            print("[返航] 已發送立即返航命令 (MAV_CMD_NAV_RETURN_TO_LAUNCH)")

            # 可選：同時嘗試設定返航模式（不等待確認）
            try:
                main_mode, sub_mode = PX4_MODE_MAP["Return"]
                custom_mode = encode_px4_mode(main_mode, sub_mode)
                self.mavlink_master.mav.set_mode_send(
                    self.mavlink_master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    custom_mode,
                )
                print("[返航] 已發送返航模式切換命令")
            except Exception:
                pass

            return True, "返航命令已發送"

        except Exception as e:
            print(f"[返航] 發送返航命令失敗: {e}")
            return False, f"返航命令發送失敗: {e}"

    def return_to_launch_with_mode(self):
        """使用模式切換的返航方法（備用）"""
        if not self.mavlink_master:
            return False, "MAVLink 未連接"

        try:
            # 忽略冷卻時間限制，直接執行返航模式切換
            main_mode, sub_mode = PX4_MODE_MAP["Return"]
            custom_mode = encode_px4_mode(main_mode, sub_mode)

            self.mavlink_master.mav.set_mode_send(
                self.mavlink_master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode,
            )

            # 不等待確認，立即返回成功
            telemetry_data.flight_mode = "Return"
            telemetry_data.px4_mode = custom_mode
            telemetry_data.mode_switch_error = ""

            print("[返航] 返航模式切換命令已發送（不等待確認）")
            return True, "返航模式已設定"

        except Exception as e:
            print(f"[返航] 模式切換失敗: {e}")
            return False, f"返航模式切換失敗: {e}"

    def return_to_launch(self):
        """改進的返航方法 - 組合使用命令和模式切換"""
        success1, msg1 = self.return_to_launch_immediate()

        if success1:
            # 如果命令發送成功，再嘗試模式切換（不關心結果）
            try:
                self.return_to_launch_with_mode()
            except Exception:

                pass
            return True
        else:
            # 如果命令發送失敗，嘗試模式切換作為備用
            success2, msg2 = self.return_to_launch_with_mode()
            return success2

    def set_flight_mode_immediate(self, mode_name: str) -> Tuple[bool, str]:
        """立即設定飛行模式，不等待確認（用於緊急情況）"""
        if mode_name not in PX4_MODE_MAP:
            return False, "不支援的模式名稱"

        if not self.mavlink_master and not self.setup_connection():
            return False, "MAVLink 連接失敗"

        try:
            main_mode, sub_mode = PX4_MODE_MAP[mode_name]
            custom_mode = encode_px4_mode(main_mode, sub_mode)

            self.mavlink_master.mav.set_mode_send(
                self.mavlink_master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode,
            )

            # 立即更新狀態，不等待確認
            telemetry_data.flight_mode = mode_name
            telemetry_data.px4_mode = custom_mode
            telemetry_data.mode_switch_error = ""

            print(f"[模式] {mode_name} 模式命令已發送（立即模式）")
            return True, f"{mode_name} 模式已設定"

        except Exception as e:
            error_msg = f"模式切換失敗: {e}"
            telemetry_data.mode_switch_error = error_msg
            return False, error_msg


mavlink_controller = PyMAVLinkController()

# ------------------------
# --- 搖桿執行緒 ---
# ------------------------
DEVICE_PATH = "/dev/input/event0"
AXIS_MAP = {
    ecodes.ABS_Y: "pitch",
    ecodes.ABS_X: "roll",
    ecodes.ABS_Z: "throttle",
    ecodes.ABS_RX: "yaw",
}


def joystick_thread_func():
    global joystick_state, control_settings
    if not JOY_OK:
        return
    try:
        device = InputDevice(DEVICE_PATH)
        control_settings.joystick_connected = True
        for event in device.read_loop():
            if event.type == ecodes.EV_ABS and event.code in AXIS_MAP:
                axis_name = AXIS_MAP[event.code]
                info = device.absinfo(event.code)
                normalized = (2 * (event.value - info.min) / (info.max - info.min)) - 1
                joystick_state.axes[axis_name] = normalized
    except FileNotFoundError:
        control_settings.joystick_connected = False


# ------------------------
# --- MAVSDK 遙測 ---
# ------------------------
CONNECTION_TIMEOUT = 5.0
HEARTBEAT_INTERVAL = 1.0
app_running = True
main_loop = None


async def run_mavsdk():
    global main_loop
    main_loop = asyncio.get_event_loop()
    drone = System()
    while app_running:
        try:
            print("[MAVSDK] 嘗試連接...")
            await drone.connect(system_address="udp://:14540")
            async for state in drone.core.connection_state():
                if state.is_connected:
                    telemetry_data.is_connected = True
                    telemetry_data.last_heartbeat = time.time()
                    print("[MAVSDK] 連接成功")
                    mavlink_controller.setup_connection()
                    mavlink_controller.attach_async(drone, main_loop)
                    break
            asyncio.ensure_future(subscribe_telemetry(drone))
            asyncio.ensure_future(heartbeat_monitor(drone))
            while telemetry_data.is_connected and app_running:
                await asyncio.sleep(1)
            print("[MAVSDK] 連線中斷，自動重新連線中...")
        except Exception as e:
            print(f"[MAVSDK] 連線失敗：{e}")
            await asyncio.sleep(2)


async def heartbeat_monitor(drone):
    while app_running:
        current_time = time.time()
        if telemetry_data.is_connected:
            if current_time - telemetry_data.last_heartbeat > CONNECTION_TIMEOUT:
                telemetry_data.is_connected = False
                print("[MAVSDK] 心跳超時，連線中斷")
                if telemetry_data.armed == "已解鎖":
                    mavlink_controller.return_to_launch()
        await asyncio.sleep(HEARTBEAT_INTERVAL)


async def subscribe_telemetry(drone):
    await asyncio.gather(
        sub_gps_info(drone),
        sub_position(drone),
        sub_attitude_ang_vel(drone),
        sub_attitude_euler(drone),
        sub_battery(drone),
    )


async def sub_gps_info(drone):
    async for gps in drone.telemetry.gps_info():
        # 只有在沒有真實GPS時才更新GPS信息
        if not gps_manager.real_gps_active:
            telemetry_data.gps_fix = str(gps.fix_type).split(".")[-1]
            telemetry_data.num_sats = gps.num_satellites


async def sub_position(drone):
    async for pos in drone.telemetry.position():
        telemetry_data.altitude_m = pos.relative_altitude_m
        # 始終通過GPS管理器更新SITL位置
        gps_manager.update_sitl_position(pos.latitude_deg, pos.longitude_deg)


async def sub_attitude_ang_vel(drone):
    async for vel in drone.telemetry.attitude_angular_velocity_body():
        telemetry_data.ang_vel = (
            math.degrees(vel.roll_rad_s),
            math.degrees(vel.pitch_rad_s),
            math.degrees(vel.yaw_rad_s),
        )


async def sub_attitude_euler(drone):
    async for attitude in drone.telemetry.attitude_euler():
        telemetry_data.attitude = (
            attitude.roll_deg,
            attitude.pitch_deg,
            attitude.yaw_deg,
        )


async def sub_battery(drone):
    async for battery in drone.telemetry.battery():
        telemetry_data.battery_percent = battery.remaining_percent


# ------------------------
# --- 飛行動作 handler (優化版) ---
# ------------------------
def handle_flight_action(action: str) -> bool:
    if not mavlink_controller.mavlink_master:
        print("[錯誤] MAVLink 未連接")
        return False
    try:
        if action == "ARM":
            success = mavlink_controller.arm()
        elif action == "DISARM":
            success = mavlink_controller.disarm()
        elif action == "TAKEOFF":
            success = mavlink_controller.takeoff(10)
            if success:
                pos = telemetry_data.position
                telemetry_data.home_position = pos
                print(f"[起飛] 設定家位置: {pos}")
        elif action == "LAND":
            success = mavlink_controller.land()
        elif action == "Hold":
            success = mavlink_controller.set_flight_mode("Hold")
        elif action == "Return":
            success = mavlink_controller.return_to_launch()
        elif action in PX4_MODE_MAP:
            can_switch, reason = mavlink_controller.check_mode_requirements(
                action, telemetry_data
            )
            if not can_switch:
                print(f"[模式] 前置條件失敗: {reason}")
                return False
            success = mavlink_controller.set_flight_mode(action)
        else:
            print(f"[錯誤] 未知動作: {action}")
            return False
        return success
    except Exception as e:
        print(f"[錯誤] 執行動作失敗: {e}")
        return False


# 定義需要立即執行的高優先級動作
HIGH_PRIORITY_ACTIONS = {"ARM", "DISARM", "LAND", "Hold", "Return", "TAKEOFF"}


def handle_flight_action_immediate(action: str) -> Tuple[bool, str]:
    """立即執行飛行動作，返回(成功狀態, 錯誤訊息)"""
    if not mavlink_controller.mavlink_master:
        return False, "MAVLink 未連接"

    try:
        print(f"[立即執行] 動作: {action}")

        if action == "ARM":
            success = mavlink_controller.arm()
            msg = "解鎖成功" if success else "解鎖失敗"
        elif action == "DISARM":
            success = mavlink_controller.disarm()
            msg = "上鎖成功" if success else "上鎖失敗"
        elif action == "TAKEOFF":
            success = mavlink_controller.takeoff(10)
            if success:
                pos = telemetry_data.position
                telemetry_data.home_position = pos
                print(f"[起飛] 設定家位置: {pos}")
                msg = "起飛指令已發送"
            else:
                msg = "起飛失敗"
        elif action == "LAND":
            success = mavlink_controller.land()
            msg = "降落指令已發送" if success else "降落失敗"
        elif action == "Hold":
            success = mavlink_controller.set_flight_mode("Hold")
            msg = "懸停模式已設定" if success else "懸停模式設定失敗"
        elif action == "Return":
            success = mavlink_controller.return_to_launch()
            msg = "返航指令已發送" if success else "返航失敗"
        elif action in PX4_MODE_MAP:
            can_switch, reason = mavlink_controller.check_mode_requirements(
                action, telemetry_data
            )
            if not can_switch:
                return False, f"前置條件失敗: {reason}"
            success = mavlink_controller.set_flight_mode(action)
            msg = f"{action}模式已設定" if success else f"{action}模式設定失敗"
        else:
            return False, f"未知動作: {action}"

        return success, msg

    except Exception as e:
        error_msg = f"執行動作失敗: {e}"
        print(f"[錯誤] {error_msg}")
        return False, error_msg


# --- GUI 應用程式 ---
class GCSProEnhanced:
    def __init__(self, root):
        self.root = root
        self.root.title("GCS Enhanced - PyMAVLink 控制版本")
        self.root.geometry("1800x1000")
        self.root.configure(bg="#f0f0f0")

        # 地圖相關變數
        self.trail_points = []
        self.current_flight_mode = "Manual"
        self.map_widget: Optional[tkintermapview.TkinterMapView] = None
        self.drone_marker: Optional[tkintermapview.TkinterMapView.marker_class] = None
        self.home_marker: Optional[tkintermapview.TkinterMapView.marker_class] = None
        self.trail_path: Optional[tkintermapview.TkinterMapView.path_class] = None
        self.last_map_position = None  # 記錄上次地圖位置

        self.setup_styles()
        self.create_widgets()
        self.setup_keyboard_controls()

        self.root.after(100, self.initial_map_load)
        self.update_ui()

    def initial_map_load(self):
        """處理初始地圖載入，確保視窗已繪製"""
        if self.map_widget:
            if telemetry_data.position:
                lat, lon = telemetry_data.position
                self.map_widget.set_position(lat, lon)
                print(f"[地圖] 初始化位置: {lat:.6f}, {lon:.6f}")
            else:
                # 如果沒有位置資料，設定一個預設視圖（世界地圖中心）
                self.map_widget.set_position(0, 0)
                self.map_widget.set_zoom(2)  # 較小的縮放級別顯示世界地圖
                print("[地圖] 初始化為世界地圖視圖，等待GPS資料")

            if telemetry_data.position:
                self.map_widget.set_zoom(DEFAULT_ZOOM)
        else:
            self._create_fallback_map()

    def manual_reconnect(self):
        telemetry_data.is_connected = False
        print("[GUI] 手動觸發 MAVSDK 重新連線")

    def setup_styles(self):
        """設定GUI樣式"""
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(
            "Title.TLabel",
            font=("Arial", 16, "bold"),
            foreground="#000000",
            background="#f0f0f0",
        )
        style.configure(
            "Info.TLabel",
            font=("Arial", 11),
            foreground="#000000",
            background="#f0f0f0",
        )
        style.configure(
            "Status.TLabel",
            font=("Arial", 12, "bold"),
            foreground="#27ae60",
            background="#f0f0f0",
        )
        style.configure(
            "Warning.TLabel",
            font=("Arial", 12, "bold"),
            foreground="#e74c3c",
            background="#f0f0f0",
        )
        style.configure(
            "Error.TLabel",
            font=("Arial", 11, "bold"),
            foreground="#e74c3c",
            background="#f0f0f0",
        )
        style.configure(
            "Light.TFrame", background="#ffffff", relief="raised", borderwidth=1
        )
        style.configure("Control.TButton", font=("Arial", 10, "bold"))
        style.configure(
            "Mono.TLabel",
            font=("Courier", 11),
            foreground="#000000",
            background="#f0f0f0",
        )

    def create_widgets(self):
        """建立所有UI元件"""
        main_container = ttk.Frame(self.root, style="Light.TFrame")
        main_container.pack(fill="both", expand=True, padx=10, pady=10)

        self.create_status_bar(main_container)

        middle_frame = ttk.Frame(main_container, style="Light.TFrame")
        middle_frame.pack(fill="both", expand=True, pady=10)

        left_panel = ttk.Frame(middle_frame, style="Light.TFrame")
        left_panel.pack(side="left", fill="y", padx=5)

        self.create_emergency_panel(left_panel)
        self.create_gps_control_panel(left_panel)
        self.create_control_settings_panel(left_panel)
        self.create_control_panel(left_panel)
        self.create_flight_mode_selector(left_panel)
        self.create_joystick_display(left_panel)

        self.create_map_view_widget(middle_frame)

        right_panel = ttk.Frame(middle_frame, style="Light.TFrame")
        right_panel.pack(side="right", fill="y", padx=5)
        self.create_telemetry_panel(right_panel)

    def create_status_bar(self, parent):
        status_frame = ttk.Frame(parent, style="Light.TFrame", height=60)
        status_frame.pack(fill="x", pady=(0, 10))
        status_frame.pack_propagate(False)

        self.conn_status_var = tk.StringVar(value="未連線")
        self.conn_status_label = ttk.Label(
            status_frame, textvariable=self.conn_status_var, style="Warning.TLabel"
        )
        self.conn_status_label.pack(side="left", padx=20, pady=10)

        self.battery_var = tk.StringVar(value="電池: ---% ")
        ttk.Label(
            status_frame, textvariable=self.battery_var, style="Mono.TLabel"
        ).pack(side="left", padx=20, pady=10)

        # GPS來源顯示
        self.gps_source_var = tk.StringVar(value="GPS: SITL")
        ttk.Label(
            status_frame, textvariable=self.gps_source_var, style="Info.TLabel"
        ).pack(side="left", padx=20, pady=10)

        map_status = "TkinterMapView" if MAP_WIDGET_AVAILABLE else "模擬地圖"
        ttk.Label(status_frame, text=f"{map_status}", style="Info.TLabel").pack(
            side="left", padx=20, pady=10
        )

        # PyMAVLink 狀態指示
        ttk.Label(status_frame, text="PyMAVLink控制", style="Info.TLabel").pack(
            side="left", padx=20, pady=10
        )

        self.mode_status_var = tk.StringVar(value="模式: Manual")
        self.mode_status_label = ttk.Label(
            status_frame, textvariable=self.mode_status_var, style="Title.TLabel"
        )
        self.mode_status_label.pack(side="left", padx=20, pady=10)

        self.time_var = tk.StringVar(value="")
        ttk.Label(status_frame, textvariable=self.time_var, style="Info.TLabel").pack(
            side="right", padx=20, pady=10
        )

        tk.Button(
            status_frame,
            text="手動重新連線",
            command=self.manual_reconnect,
            bg="#e67e22",
            fg="white",
            font=("Arial", 10, "bold"),
        ).pack(side="left", padx=20)

        tk.Button(
            status_frame,
            text="關閉應用程式",
            command=self.root.quit,
            bg="#c0392b",
            fg="white",
            font=("Arial", 10, "bold"),
        ).pack(side="left", padx=20)

    def create_gps_control_panel(self, parent):
        """創建GPS控制面板"""
        gps_frame = ttk.LabelFrame(parent, text="GPS 控制", padding=10)
        gps_frame.pack(fill="x", pady=5)

        # GPS設備掃描
        scan_frame = ttk.Frame(gps_frame)
        scan_frame.pack(fill="x", pady=5)

        tk.Button(
            scan_frame,
            text="掃描GPS設備",
            command=self.scan_gps_devices,
            bg="#3498db",
            fg="white",
            font=("Arial", 10, "bold"),
        ).pack(side="left", padx=5)

        self.gps_status_var = tk.StringVar(value="未掃描")
        ttk.Label(
            scan_frame, textvariable=self.gps_status_var, style="Info.TLabel"
        ).pack(side="left", padx=10)

        # GPS設備選擇
        select_frame = ttk.Frame(gps_frame)
        select_frame.pack(fill="x", pady=5)

        ttk.Label(select_frame, text="選擇GPS:", style="Info.TLabel").pack(side="left")

        self.gps_device_var = tk.StringVar(value="SITL GPS")
        self.gps_device_combo = ttk.Combobox(
            select_frame,
            textvariable=self.gps_device_var,
            values=["SITL GPS", "虛擬GPS"],
            state="readonly",
            width=20,
        )
        self.gps_device_combo.pack(side="left", padx=5)
        self.gps_device_combo.bind("<<ComboboxSelected>>", self.on_gps_device_selected)

        # GPS連接按鈕
        connect_frame = ttk.Frame(gps_frame)
        connect_frame.pack(fill="x", pady=5)

        self.gps_connect_btn = tk.Button(
            connect_frame,
            text="連接GPS",
            command=self.connect_gps,
            bg="#27ae60",
            fg="white",
            font=("Arial", 10, "bold"),
        )
        self.gps_connect_btn.pack(side="left", padx=5)

        self.gps_disconnect_btn = tk.Button(
            connect_frame,
            text="斷開GPS",
            command=self.disconnect_gps,
            bg="#e74c3c",
            fg="white",
            font=("Arial", 10, "bold"),
        )
        self.gps_disconnect_btn.pack(side="left", padx=5)

    def create_control_settings_panel(self, parent):
        """創建控制設定面板"""
        settings_frame = ttk.LabelFrame(parent, text="控制設定", padding=10)
        settings_frame.pack(fill="x", pady=5)

        self.control_mode_var = tk.StringVar(
            value="搖桿" if control_settings.use_joystick else "鍵盤"
        )

        # 控制模式選擇
        ttk.Label(settings_frame, text="控制方式:", style="Info.TLabel").pack(
            anchor="w"
        )

        mode_frame = ttk.Frame(settings_frame)
        mode_frame.pack(fill="x", pady=5)

        ttk.Radiobutton(
            mode_frame,
            text="搖桿控制",
            variable=self.control_mode_var,
            value="搖桿",
            command=self.on_control_mode_changed,
        ).pack(side="left")
        ttk.Radiobutton(
            mode_frame,
            text="鍵盤控制",
            variable=self.control_mode_var,
            value="鍵盤",
            command=self.on_control_mode_changed,
        ).pack(side="left", padx=20)

        # 搖桿狀態顯示
        self.joystick_status_var = tk.StringVar(
            value=(
                "搖桿: 未連接"
                if not control_settings.joystick_connected
                else "搖桿: 已連接"
            )
        )
        status_label = ttk.Label(
            settings_frame, textvariable=self.joystick_status_var, style="Info.TLabel"
        )
        status_label.pack(anchor="w", pady=5)

        # 鍵盤控制說明
        help_frame = ttk.Frame(settings_frame)
        help_frame.pack(fill="x", pady=5)

        help_text = "鍵盤控制: W/S-俯仰, A/D-橫滾, Q/E-偏航, R/F-油門"
        ttk.Label(
            help_frame, text=help_text, style="Info.TLabel", wraplength=250
        ).pack()

    def create_control_panel(self, parent):
        control_frame = ttk.LabelFrame(
            parent, text="飛行控制 (高優先級動作立即執行)", padding=15
        )
        control_frame.pack(fill="x", pady=10)

        controls = [
            ("解鎖", "ARM", "#27ae60", True),
            ("起飛", "TAKEOFF", "#3498db", True),
            ("懸停", "Hold", "#f39c12", True),
            ("返航", "Return", "#e74c3c", True),
            ("降落", "LAND", "#9b59b6", True),
            ("上鎖", "DISARM", "#95a5a6", True),
        ]

        for i, (text, action, color, is_high_priority) in enumerate(controls):
            # 高優先級按鈕使用更粗的邊框和更大的字體
            if is_high_priority:
                btn = tk.Button(
                    control_frame,
                    text=f"{text}",
                    command=lambda a=action: self.execute_action(a),
                    bg=color,
                    fg="white",
                    font=("Arial", 12, "bold"),
                    width=12,
                    height=2,
                    relief="raised",
                    bd=3,
                    activebackground=self._darken_color(color),
                    cursor="hand2",
                )
            else:
                btn = tk.Button(
                    control_frame,
                    text=text,
                    command=lambda a=action: self.execute_action(a),
                    bg=color,
                    fg="white",
                    font=("Arial", 11, "bold"),
                    width=12,
                    height=2,
                    relief="raised",
                    bd=2,
                    cursor="hand2",
                )

            btn.grid(row=i // 2, column=i % 2, padx=5, pady=5)

            # 為高優先級按鈕添加滑鼠懸停效果
            if is_high_priority:
                btn.bind(
                    "<Enter>",
                    lambda e, b=btn, c=color: b.configure(bg=self._lighten_color(c)),
                )
                btn.bind("<Leave>", lambda e, b=btn, c=color: b.configure(bg=c))

    def _darken_color(self, hex_color):
        """使顏色變暗"""
        hex_color = hex_color.lstrip("#")
        rgb = tuple(int(hex_color[i : i + 2], 16) for i in (0, 2, 4))
        darkened = tuple(max(0, int(c * 0.8)) for c in rgb)
        return f"#{darkened[0]:02x}{darkened[1]:02x}{darkened[2]:02x}"

    def _lighten_color(self, hex_color):
        """使顏色變亮"""
        hex_color = hex_color.lstrip("#")
        rgb = tuple(int(hex_color[i : i + 2], 16) for i in (0, 2, 4))
        lightened = tuple(min(255, int(c * 1.2)) for c in rgb)
        return f"#{lightened[0]:02x}{lightened[1]:02x}{lightened[2]:02x}"

    def create_flight_mode_selector(self, parent):
        mode_frame = ttk.LabelFrame(parent, text="飛行模式選擇 (PyMAVLink)", padding=15)
        mode_frame.pack(fill="x", pady=10)

        ttk.Label(mode_frame, text="選擇飛行模式:", style="Info.TLabel").pack(
            anchor="w", pady=(0, 5)
        )

        self.flight_mode_var = tk.StringVar(value="Manual")
        self.mode_combobox = ttk.Combobox(
            mode_frame,
            textvariable=self.flight_mode_var,
            values=list(PX4_MODE_MAP.keys()),
            state="readonly",
            width=18,
            font=("Arial", 12),
        )
        self.mode_combobox.pack(fill="x", pady=(0, 10))
        self.mode_combobox.bind("<<ComboboxSelected>>", self.on_mode_selected)

        self.mode_desc_var = tk.StringVar(
            value=FLIGHT_MODES.get("Manual", {"description": "完全手動控制"})[
                "description"
            ]
        )
        desc_label = ttk.Label(
            mode_frame,
            textvariable=self.mode_desc_var,
            style="Info.TLabel",
            wraplength=200,
        )
        desc_label.pack(anchor="w", pady=(0, 10))

        # 錯誤訊息顯示
        self.mode_error_var = tk.StringVar(value="")
        self.mode_error_label = ttk.Label(
            mode_frame,
            textvariable=self.mode_error_var,
            style="Error.TLabel",
            wraplength=200,
        )
        self.mode_error_label.pack(anchor="w", pady=(0, 10))

        self.mode_indicator = tk.Canvas(
            mode_frame, width=200, height=30, highlightthickness=0
        )
        self.mode_indicator.pack(fill="x")
        self.update_mode_indicator()

    def create_joystick_display(self, parent):
        joystick_frame = ttk.LabelFrame(parent, text="控制狀態", padding=10)
        joystick_frame.pack(fill="x", pady=10)

        sticks_frame = ttk.Frame(joystick_frame)
        sticks_frame.pack(fill="x")

        left_frame = ttk.Frame(sticks_frame)
        left_frame.pack(side="left", padx=10)
        ttk.Label(left_frame, text="偏航/油門", style="Info.TLabel").pack()
        self.left_canvas = tk.Canvas(
            left_frame, width=150, height=150, bg="#e8e8e8", highlightthickness=1
        )
        self.left_canvas.pack()

        right_frame = ttk.Frame(sticks_frame)
        right_frame.pack(side="right", padx=10)
        ttk.Label(right_frame, text="橫滾/俯仰", style="Info.TLabel").pack()
        self.right_canvas = tk.Canvas(
            right_frame, width=150, height=150, bg="#e8e8e8", highlightthickness=1
        )
        self.right_canvas.pack()

        self._create_circular_joystick_background(self.left_canvas, 150)
        self._create_circular_joystick_background(self.right_canvas, 150)

        self.left_stick = self.left_canvas.create_oval(
            70, 70, 80, 80, fill="#3498db", outline="#2c3e50", width=2
        )
        self.right_stick = self.right_canvas.create_oval(
            70, 70, 80, 80, fill="#e74c3c", outline="#2c3e50", width=2
        )

    def create_map_view_widget(self, parent):
        """建立 tkintermapview 地圖元件"""
        map_frame = ttk.LabelFrame(parent, text="GPS定位地圖", padding=10)
        map_frame.pack(side="left", fill="both", expand=True, padx=10)

        toolbar_frame = ttk.Frame(map_frame)
        toolbar_frame.pack(fill="x", pady=(0, 5))

        ttk.Label(toolbar_frame, text="地圖:", style="Info.TLabel").pack(
            side="left", padx=5
        )
        self.map_provider_var = tk.StringVar(value="OpenStreetMap")
        provider_combo = ttk.Combobox(
            toolbar_frame,
            textvariable=self.map_provider_var,
            values=list(MAP_PROVIDERS_UI.keys()),
            width=15,
            state="readonly",
        )
        provider_combo.pack(side="left", padx=5)
        provider_combo.bind("<<ComboboxSelected>>", self.on_map_provider_changed)

        # 新增定位和刷新按鈕
        tk.Button(
            toolbar_frame,
            text="定位",
            command=self.center_on_drone,
            bg="#27ae60",
            fg="white",
            font=("Arial", 10, "bold"),
        ).pack(side="left", padx=(20, 5))
        tk.Button(
            toolbar_frame,
            text="刷新",
            command=self.refresh_map,
            bg="#3498db",
            fg="white",
            font=("Arial", 10, "bold"),
        ).pack(side="left", padx=5)
        tk.Button(
            toolbar_frame,
            text="清除軌跡",
            command=self.clear_trail,
            bg="#f39c12",
            fg="white",
            font=("Arial", 10, "bold"),
        ).pack(side="left", padx=5)

        map_container = ttk.Frame(map_frame)
        map_container.pack(fill="both", expand=True)

        if MAP_WIDGET_AVAILABLE:
            self.map_widget = tkintermapview.TkinterMapView(
                map_container, corner_radius=0
            )
            self.map_widget.pack(fill="both", expand=True)
            # 不在這裡設定初始位置，等資料準備好再設定
        else:
            self.map_canvas = tk.Canvas(
                map_container, bg="#f5f5dc", highlightthickness=1
            )
            self.map_canvas.pack(fill="both", expand=True)

        coord_frame = ttk.Frame(map_frame)
        coord_frame.pack(fill="x", pady=(5, 0))
        self.coord_var = tk.StringVar(value="座標: N/A")
        ttk.Label(coord_frame, textvariable=self.coord_var, style="Mono.TLabel").pack(
            side="left"
        )
        self.distance_var = tk.StringVar(value="距離家: N/A")
        ttk.Label(
            coord_frame, textvariable=self.distance_var, style="Mono.TLabel"
        ).pack(side="right")

    def create_telemetry_panel(self, parent):
        telemetry_frame = ttk.LabelFrame(parent, text="遙測資訊", padding=15)
        telemetry_frame.pack(fill="both", expand=True)

        self.telemetry_vars = {
            "armed": tk.StringVar(value="解鎖狀態: 未知"),
            "mode": tk.StringVar(value="飛行模式: 未知"),
            "gps": tk.StringVar(value="GPS: 未知"),
            "altitude": tk.StringVar(value="高度:   0.00 m"),
            "position": tk.StringVar(value="位置: 0.000000,   0.000000"),
            "attitude": tk.StringVar(value="姿態: R:  +0.0 P:  +0.0 Y:  +0.0"),
            "velocity": tk.StringVar(value="角速度: R:  +0.0 P:  +0.0 Y:  +0.0"),
            "satellites": tk.StringVar(value="衛星數:  0"),
            "home": tk.StringVar(value="家位置: 未設定"),
            "control": tk.StringVar(value="控制: 搖桿"),
            "mode_error": tk.StringVar(value=""),
        }

        for i, (key, var) in enumerate(self.telemetry_vars.items()):
            if key == "mode_error":
                style = "Error.TLabel"
            elif key in ["altitude", "position", "attitude", "velocity"]:
                style = "Mono.TLabel"
            else:
                style = "Info.TLabel"

            ttk.Label(telemetry_frame, textvariable=var, style=style).pack(
                anchor="w", pady=6
            )
            if i == 5:
                ttk.Separator(telemetry_frame, orient="horizontal").pack(
                    fill="x", pady=8
                )

    def _create_circular_joystick_background(self, canvas, size):
        """創建圓形搖桿背景，帶有桿量分割線"""
        center = size / 2
        canvas.delete("all")

        # 繪製同心圓和桿量分割線
        max_radius = size / 2 - 10
        percentages = [25, 50, 75, 100]
        colors = ["#f0f0f0", "#e0e0e0", "#d0d0d0", "#c0c0c0"]

        # 從外到內繪製圓圈
        for i, percentage in enumerate(reversed(percentages)):
            radius = max_radius * (percentage / 100)
            color = colors[len(percentages) - 1 - i]
            canvas.create_oval(
                center - radius,
                center - radius,
                center + radius,
                center + radius,
                fill=color,
                outline="#999999",
                width=1,
            )

            # 在適當位置標記百分比
            if percentage < 100:  # 不在最外圈標記100%
                text_radius = radius + 8
                canvas.create_text(
                    center + text_radius * 0.707,
                    center - text_radius * 0.707,
                    text=f"{percentage}%",
                    font=("Arial", 8),
                    fill="#666666",
                )

        # 繪製十字線
        canvas.create_line(
            center, 10, center, size - 10, fill="#999999", dash=(3, 3), width=1
        )
        canvas.create_line(
            10, center, size - 10, center, fill="#999999", dash=(3, 3), width=1
        )

        # 中心點
        canvas.create_oval(
            center - 2,
            center - 2,
            center + 2,
            center + 2,
            fill="#666666",
            outline="#666666",
        )

    def _create_fallback_map(self):
        """當tkintermapview不可用時，建立備用地圖"""
        if not hasattr(self, "map_canvas"):
            return

        canvas_width = self.map_canvas.winfo_width()
        canvas_height = self.map_canvas.winfo_height()
        if canvas_width <= 1:
            return

        self.map_canvas.delete("all")
        self.map_canvas.create_rectangle(
            0, 0, canvas_width, canvas_height, fill="#e8f5e8", outline=""
        )
        for i in range(0, canvas_width, 50):
            self.map_canvas.create_line(
                i, 0, i, canvas_height, fill="#cccccc", dash=(2, 2)
            )
        for j in range(0, canvas_height, 50):
            self.map_canvas.create_line(
                0, j, canvas_width, j, fill="#cccccc", dash=(2, 2)
            )

    def setup_keyboard_controls(self):
        """設定鍵盤控制"""
        self.root.focus_set()
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.keyboard_control = {
            "w": False,
            "s": False,
            "a": False,
            "d": False,
            "q": False,
            "e": False,
            "r": False,
            "f": False,
        }

    def on_key_press(self, event):
        if event.char.lower() in self.keyboard_control:
            self.keyboard_control[event.char.lower()] = True
            self.update_keyboard_control()

    def on_key_release(self, event):
        if event.char.lower() in self.keyboard_control:
            self.keyboard_control[event.char.lower()] = False
            self.update_keyboard_control()

    def update_keyboard_control(self):
        """更新鍵盤控制輸入"""
        if not control_settings.use_joystick:  # 只有在鍵盤模式下才生效
            pitch = (
                0.5
                if self.keyboard_control["w"]
                else -0.5 if self.keyboard_control["s"] else 0.0
            )
            roll = (
                -0.5
                if self.keyboard_control["a"]
                else 0.5 if self.keyboard_control["d"] else 0.0
            )
            yaw = (
                -0.5
                if self.keyboard_control["q"]
                else 0.5 if self.keyboard_control["e"] else 0.0
            )
            throttle = joystick_state.axes.get("throttle", 0.0)
            if self.keyboard_control["r"]:
                throttle = min(1.0, throttle + 0.1)
            if self.keyboard_control["f"]:
                throttle = max(-1.0, throttle - 0.1)

            if any(self.keyboard_control.values()):
                joystick_state.axes.update(
                    {"pitch": pitch, "roll": roll, "yaw": yaw, "throttle": throttle}
                )

    def scan_gps_devices(self):
        """掃描GPS設備"""
        self.gps_status_var.set("掃描中...")
        self.root.update()

        def scan_in_thread():
            ports = gps_manager.scan_gps_devices()
            devices = ["SITL GPS", "虛擬GPS"] + ports

            def update_gui():
                self.gps_device_combo["values"] = devices
                if len(ports) > 0:
                    self.gps_status_var.set(f"找到 {len(ports)} 個GPS設備")
                else:
                    self.gps_status_var.set("未找到外部GPS設備")

            self.root.after(0, update_gui)

        threading.Thread(target=scan_in_thread, daemon=True).start()

    def on_gps_device_selected(self, event):
        """GPS設備選擇變更"""
        selected = self.gps_device_var.get()
        print(f"[GUI] 選擇GPS設備: {selected}")

    def connect_gps(self):
        """連接GPS設備"""
        selected = self.gps_device_var.get()

        if selected == "SITL GPS":
            gps_manager.disconnect_real_gps()
            self.gps_status_var.set("使用SITL GPS")
            print("[GPS] 切換至SITL GPS")
        elif selected == "虛擬GPS":
            gps_manager.disconnect_real_gps()
            # 清除SITL位置，強制使用虛擬GPS
            gps_manager.last_sitl_position = None
            self.gps_status_var.set("使用虛擬GPS")
            print("[GPS] 切換至虛擬GPS")
        else:

            def connect_in_thread():
                success = gps_manager.connect_real_gps(selected)

                def update_gui():
                    if success:
                        self.gps_status_var.set(f"已連接: {selected}")
                    else:
                        self.gps_status_var.set(f"連接失敗: {selected}")

                self.root.after(0, update_gui)

            threading.Thread(target=connect_in_thread, daemon=True).start()

    def disconnect_gps(self):
        """斷開GPS連接"""
        gps_manager.disconnect_real_gps()
        gps_manager.last_sitl_position = None  # 清除SITL位置
        # 重置虛擬GPS，但不設定預設位置
        gps_manager.virtual_gps = GPSSimulator()
        self.gps_status_var.set("已斷開GPS - 等待新的GPS資料")
        print("[GPS] 已斷開所有GPS連接")

    def on_control_mode_changed(self):
        """控制模式變更"""
        mode = self.control_mode_var.get()
        control_settings.use_joystick = mode == "搖桿"
        print(f"[GUI] 切換控制模式: {mode}")

        # 清空當前控制輸入
        joystick_state.axes = {}

    def refresh_map(self):
        """刷新地圖"""
        if self.map_widget:
            current_pos = telemetry_data.position
            if current_pos:
                self.map_widget.set_position(current_pos[0], current_pos[1])
                print(
                    f"[地圖] 地圖已刷新至: {current_pos[0]:.6f}, {current_pos[1]:.6f}"
                )

    def on_mode_selected(self, event):
        """處理飛行模式選擇"""
        selected_mode = self.flight_mode_var.get()
        print(f"\n[GUI] 用戶選擇模式: {selected_mode}")

        can_switch, reason = mavlink_controller.check_mode_requirements(
            selected_mode, telemetry_data
        )

        if can_switch:
            self.current_flight_mode = selected_mode
            mode_desc = FLIGHT_MODES.get(
                selected_mode, {"description": f"{selected_mode} 模式"}
            )["description"]
            self.mode_desc_var.set(mode_desc)
            self.mode_error_var.set("")  # 清除錯誤訊息
            self.update_mode_indicator()
            print("[GUI] 開始執行模式切換...")
            self.execute_action(selected_mode)
        else:
            print(f"[GUI] 模式切換被阻止: {reason}")
            self.mode_error_var.set(f"無法切換: {reason}")
            messagebox.showwarning(
                "模式切換限制", f"無法切換至 {selected_mode}:\n{reason}"
            )
            self.flight_mode_var.set(self.current_flight_mode)

    def update_mode_indicator(self):
        mode = (
            telemetry_data.flight_mode
            if telemetry_data.flight_mode != "未知"
            else self.current_flight_mode
        )
        color = FLIGHT_MODES.get(mode, {"color": "#95a5a6"})["color"]
        self.mode_indicator.delete("all")
        self.mode_indicator.create_rectangle(
            0, 0, 200, 30, fill=color, outline="#000000", width=2
        )
        self.mode_indicator.create_text(
            100, 15, text=mode, fill="white", font=("Arial", 12, "bold")
        )

    def on_map_provider_changed(self, event):
        """處理地圖提供商變更"""
        if not self.map_widget:
            return
        provider_name = self.map_provider_var.get()
        if provider_name == "OpenStreetMap":
            self.map_widget.set_tile_server(
                "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png"
            )
        elif provider_name == "ESRI Satellite":
            self.map_widget.set_tile_server(
                "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
                max_zoom=19,
            )
        print(f"[地圖] 切換地圖提供商至: {provider_name}")

    def center_on_drone(self):
        """將地圖中心移至無人機位置"""
        if self.map_widget:
            current_pos = telemetry_data.position
            if current_pos:
                lat, lon = current_pos
                self.map_widget.set_position(lat, lon)
                print(f"[地圖] 定位至無人機位置: {lat:.6f}, {lon:.6f}")

    def clear_trail(self):
        """清除軌跡"""
        self.trail_points = []
        if self.trail_path:
            self.trail_path.delete()
            self.trail_path = None
        print("[地圖] 軌跡已清除")

    def execute_action(self, action):
        """執行飛行動作或模式切換 (優化版)"""
        print(f"\n[GUI] 執行動作: {action}")

        # 高優先級動作立即執行
        if action in HIGH_PRIORITY_ACTIONS:
            try:
                success, message = handle_flight_action_immediate(action)

                if success:
                    if action in PX4_MODE_MAP:
                        self.current_flight_mode = action
                        self.update_mode_indicator()
                        self.mode_error_var.set("")  # 清除錯誤訊息

                    # 立即顯示成功訊息
                    messagebox.showinfo("執行成功", f"{message}")
                    print(f"[GUI] ✓ {action} 立即執行成功")
                else:
                    # 立即顯示錯誤訊息
                    self.mode_error_var.set(message)
                    messagebox.showerror("執行失敗", f"{message}\n請查看系統狀態")
                    print(f"[GUI] ✗ {action} 立即執行失敗: {message}")

            except Exception as e:
                error_msg = f"執行動作時發生錯誤: {str(e)}"
                self.mode_error_var.set(error_msg)
                messagebox.showerror("錯誤", error_msg)
                print(f"[GUI] 執行異常: {e}")
                import traceback

                traceback.print_exc()
        else:
            # 其他動作異步執行
            self._execute_action_async(action)

    def _execute_action_async(self, action):
        """異步執行動作"""

        def execute_in_thread():
            try:
                success = handle_flight_action(action)

                def update_gui():
                    if success:
                        if action in PX4_MODE_MAP:
                            self.current_flight_mode = action
                            self.update_mode_indicator()
                            self.mode_error_var.set("")  # 清除錯誤訊息
                        messagebox.showinfo("成功", f"已執行 {action}")
                        print(f"[GUI] ✓ {action} 異步執行成功")
                    else:
                        error_msg = (
                            telemetry_data.mode_switch_error or f"{action} 執行失敗"
                        )
                        self.mode_error_var.set(error_msg)
                        messagebox.showerror("失敗", f"{error_msg}\n請查看控制台輸出")
                        print(f"[GUI] ✗ {action} 異步執行失敗")

                self.root.after(0, update_gui)

            except Exception as e:
                print(f"[GUI] 異步執行異常: {e}")
                import traceback

                traceback.print_exc()

                def show_error(e):
                    error_msg = f"執行動作時發生錯誤: {str(e)}"
                    self.mode_error_var.set(error_msg)
                    messagebox.showerror("錯誤", error_msg)

                self.root.after(0, show_error(e))

        threading.Thread(target=execute_in_thread, daemon=True).start()

    def update_ui(self):
        """更新UI顯示"""
        if not app_running:
            return

        self.time_var.set(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

        if telemetry_data.is_connected:
            self.conn_status_var.set("已連線")
            self.conn_status_label.configure(style="Status.TLabel")
        else:
            self.conn_status_var.set("未連線")
            self.conn_status_label.configure(style="Warning.TLabel")

        self.battery_var.set(f"電池: {telemetry_data.battery_percent:3.0f}%")
        self.mode_status_var.set(f"模式: {telemetry_data.flight_mode}")
        self.gps_source_var.set(f"GPS: {telemetry_data.gps_source}")

        # 更新搖桿連接狀態
        self.joystick_status_var.set(
            f"搖桿: {'已連接' if control_settings.joystick_connected else '未連接'}"
        )

        # 更新模式選擇器顯示
        if (
            telemetry_data.flight_mode != "未知"
            and telemetry_data.flight_mode != self.flight_mode_var.get()
        ):
            self.flight_mode_var.set(telemetry_data.flight_mode)
            self.update_mode_indicator()

        # 更新模式錯誤訊息
        if telemetry_data.mode_switch_error:
            self.mode_error_var.set(telemetry_data.mode_switch_error)

        self.update_joystick_display()
        self.update_telemetry_info()
        self.update_map_elements()

        self.root.after(50, self.update_ui)

    def update_joystick_display(self):
        """更新搖桿視覺顯示"""
        yaw, throttle = joystick_state.axes.get("yaw", 0.0), joystick_state.axes.get(
            "throttle", -1.0
        )
        lx, ly = 75 + yaw * 65, 75 - throttle * 65

        # 限制在圓形範圍內
        center = 75
        dx, dy = lx - center, ly - center
        distance = math.sqrt(dx * dx + dy * dy)
        if distance > 65:
            lx = center + dx * 65 / distance
            ly = center + dy * 65 / distance

        self.left_canvas.coords(self.left_stick, lx - 5, ly - 5, lx + 5, ly + 5)

        roll, pitch = joystick_state.axes.get("roll", 0.0), joystick_state.axes.get(
            "pitch", 0.0
        )
        rx, ry = 75 + roll * 65, 75 + pitch * 65

        # 限制在圓形範圍內
        dx, dy = rx - center, ry - center
        distance = math.sqrt(dx * dx + dy * dy)
        if distance > 65:
            rx = center + dx * 65 / distance
            ry = center + dy * 65 / distance

        self.right_canvas.coords(self.right_stick, rx - 5, ry - 5, rx + 5, ry + 5)

    def update_telemetry_info(self):
        """更新遙測資訊顯示"""
        self.telemetry_vars["armed"].set(f"解鎖狀態: {telemetry_data.armed}")
        self.telemetry_vars["mode"].set(f"飛行模式: {telemetry_data.flight_mode}")
        self.telemetry_vars["gps"].set(
            f"GPS: {telemetry_data.gps_source}({telemetry_data.gps_fix})"
        )
        self.telemetry_vars["satellites"].set(f"衛星數: {telemetry_data.num_sats:2d}")
        self.telemetry_vars["altitude"].set(f"高度: {telemetry_data.altitude_m:7.2f} m")
        self.telemetry_vars["control"].set(
            f"控制: {'搖桿' if control_settings.use_joystick else '鍵盤'}"
        )

        current_pos = telemetry_data.position
        if current_pos:
            lat, lon = current_pos
            self.telemetry_vars["position"].set(f"位置: {lat:9.6f}, {lon:10.6f}")
        else:
            self.telemetry_vars["position"].set("位置: 等待GPS資料...")

        r, p, y = telemetry_data.attitude
        self.telemetry_vars["attitude"].set(
            f"姿態: R:{r:+6.1f} P:{p:+6.1f} Y:{y:+6.1f}"
        )
        r, p, y = telemetry_data.ang_vel
        self.telemetry_vars["velocity"].set(
            f"角速度: R:{r:+6.1f} P:{p:+6.1f} Y:{y:+6.1f}"
        )

        if telemetry_data.home_position:
            hlat, hlon = telemetry_data.home_position
            self.telemetry_vars["home"].set(f"家位置: {hlat:9.6f}, {hlon:10.6f}")
        else:
            self.telemetry_vars["home"].set("家位置: 未設定")

        # 更新錯誤訊息
        if telemetry_data.mode_switch_error:
            self.telemetry_vars["mode_error"].set(
                f"錯誤: {telemetry_data.mode_switch_error}"
            )
        else:
            self.telemetry_vars["mode_error"].set("")

    def update_map_elements(self):
        """使用 tkintermapview 更新地圖元素"""
        if not self.map_widget:
            return

        current_pos = telemetry_data.position
        if not current_pos:
            # 更新座標顯示為等待GPS
            self.coord_var.set("座標: 等待GPS資料...")
            self.distance_var.set("距離家: N/A")
            return

        lat, lon = current_pos

        # 記錄軌跡 - 避免重複記錄相同位置
        if (
            len(self.trail_points) == 0
            or abs(self.trail_points[-1][0] - lat) > 1e-7
            or abs(self.trail_points[-1][1] - lon) > 1e-7
        ):
            self.trail_points.append((lat, lon))

        # 更新無人機標記
        if self.drone_marker is None:
            self.drone_marker = self.map_widget.set_marker(
                lat,
                lon,
                text="Drone",
                marker_color_circle="red",
                marker_color_outside="darkred",
            )
            print(f"[地圖] 創建無人機標記: {lat:.6f}, {lon:.6f}")
            # 第一次獲得位置時，設定適當的縮放級別和中心
            self.map_widget.set_position(lat, lon)
            self.map_widget.set_zoom(DEFAULT_ZOOM)
        else:
            self.drone_marker.set_position(lat, lon)

        # 設定地圖中心 - 只在位置變化較大時更新
        if (
            self.last_map_position is None
            or abs(self.last_map_position[0] - lat) > 1e-6
            or abs(self.last_map_position[1] - lon) > 1e-6
        ):
            self.map_widget.set_position(lat, lon)
            self.last_map_position = (lat, lon)

        # 更新家位置標記
        if telemetry_data.home_position:
            hlat, hlon = telemetry_data.home_position
            if self.home_marker is None:
                self.home_marker = self.map_widget.set_marker(
                    hlat,
                    hlon,
                    text="Start Position",
                    marker_color_circle="yellow",
                    marker_color_outside="orange",
                )
                print(f"[地圖] 創建家標記: {hlat:.6f}, {hlon:.6f}")
            else:
                self.home_marker.set_position(hlat, hlon)

        # 更新飛行軌跡
        if len(self.trail_points) > 1:
            if self.trail_path is None:
                self.trail_path = self.map_widget.set_path(
                    self.trail_points, color="#27ae60", width=3
                )
                print(f"[地圖] 創建軌跡路徑，包含 {len(self.trail_points)} 個點")
            else:
                self.trail_path.set_position_list(self.trail_points)

        # 更新座標顯示
        self.coord_var.set(f"座標: {lat:.6f}, {lon:.6f}")
        if telemetry_data.home_position:
            hlat, hlon = telemetry_data.home_position
            distance = self._calculate_distance(lat, lon, hlat, hlon)
            self.distance_var.set(f"距離家: {distance:.1f}m")

    def _calculate_distance(self, lat1, lon1, lat2, lon2):
        """計算兩點間距離（米）"""
        R = 6371000
        lat1_rad, lat2_rad = math.radians(lat1), math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        a = (
            math.sin(delta_lat / 2) ** 2
            + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def create_emergency_panel(self, parent):
        """創建緊急控制面板"""
        emergency_frame = ttk.LabelFrame(parent, text="緊急控制", padding=10)
        emergency_frame.pack(fill="x", pady=5)

        # 緊急返航按鈕 - 特別突出
        emergency_return_btn = tk.Button(
            emergency_frame,
            text="緊急返航",
            command=lambda: self.emergency_return(),
            bg="#ff0000",
            fg="white",
            font=("Arial", 14, "bold"),
            width=20,
            height=3,
            relief="raised",
            bd=4,
            cursor="hand2",
        )
        emergency_return_btn.pack(pady=5)

        # 添加滑鼠懸停效果
        emergency_return_btn.bind(
            "<Enter>", lambda e: emergency_return_btn.configure(bg="#cc0000")
        )
        emergency_return_btn.bind(
            "<Leave>", lambda e: emergency_return_btn.configure(bg="#ff0000")
        )

        # 緊急降落按鈕
        emergency_land_btn = tk.Button(
            emergency_frame,
            text="緊急降落",
            command=lambda: self.emergency_land(),
            bg="#ff6600",
            fg="white",
            font=("Arial", 12, "bold"),
            width=20,
            height=2,
            relief="raised",
            bd=3,
        )
        emergency_land_btn.pack(pady=3)

    def emergency_return(self):
        """緊急返航 - 繞過所有檢查"""
        print("[緊急] 執行緊急返航")
        try:
            success, msg = mavlink_controller.return_to_launch_immediate()
            if success:
                # 強制更新UI狀態
                self.current_flight_mode = "Return"
                self.flight_mode_var.set("Return")
                self.update_mode_indicator()
                telemetry_data.mode_switch_error = ""
                messagebox.showinfo("緊急返航", "緊急返航命令已發送！")
            else:
                messagebox.showerror("緊急返航失敗", f"無法執行緊急返航：{msg}")
        except Exception as e:
            messagebox.showerror("緊急返航錯誤", f"緊急返航執行時發生錯誤：{e}")

    def emergency_land(self):
        """緊急降落"""
        print("[緊急] 執行緊急降落")
        try:
            success = mavlink_controller.land()
            if success:
                messagebox.showinfo("緊急降落", "⬇️ 緊急降落命令已發送！")
            else:
                messagebox.showerror("緊急降落失敗", "無法執行緊急降落")
        except Exception as e:
            messagebox.showerror("緊急降落錯誤", f"緊急降落執行時發生錯誤：{e}")


def on_closing():
    global app_running, mavlink_controller, gps_manager
    app_running = False

    if mavlink_controller:
        mavlink_controller.stop_control_thread()

    if gps_manager:
        gps_manager.disconnect_real_gps()

    time.sleep(0.2)
    root.destroy()


def main():
    global root

    if not MAP_WIDGET_AVAILABLE:
        print("[警告] tkintermapview 未安裝，地圖功能將受限。")
        print("[提示] 請執行: pip install tkintermapview")

    threading.Thread(target=joystick_thread_func, daemon=True).start()
    threading.Thread(target=lambda: asyncio.run(run_mavsdk()), daemon=True).start()
    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
