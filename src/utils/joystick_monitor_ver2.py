"""
GCS Enhanced - PyMAVLink 控制版本

"""

import tkinter as tk
from tkinter import ttk, messagebox
import asyncio
import threading
import time
import math
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Tuple, Dict
from mavsdk import System
from evdev import InputDevice, ecodes
from pymavlink import mavutil

try:
    import tkintermapview

    MAP_WIDGET_AVAILABLE = True
    print("[地圖] tkintermapview 功能可用")
except ImportError:
    MAP_WIDGET_AVAILABLE = False
    print("[地圖] tkintermapview 功能不可用，請安裝: pip install tkintermapview")


# --- 資料類別定義 ---
@dataclass
class TelemetryData:
    """遙測資料結構"""

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
    px4_mode: int = 0  # 新增：儲存 PX4 custom_mode


@dataclass
class JoystickState:
    """搖桿狀態資料"""

    axes: Dict[str, float]
    action: Optional[str] = None


# --- 飛行模式定義 ---
FLIGHT_MODES = {
    "Manual": {"color": "#e74c3c", "description": "完全手動控制"},
    "Stabilized": {"color": "#3498db", "description": "穩定輔助模式"},
    "Acro": {"color": "#9b59b6", "description": "特技飛行模式"},
    "Altitude": {"color": "#27ae60", "description": "定高飛行模式"},
    "Position": {"color": "#2ecc71", "description": "GPS定位模式"},
    "Offboard": {"color": "#e67e22", "description": "外部控制模式"},
    "Hold": {"color": "#34495e", "description": "懸停保持模式"},
    "Mission": {"color": "#8e44ad", "description": "自動任務模式"},
    "Return": {"color": "#c0392b", "description": "自動返航模式"},
}

# --- 地圖服務提供商設定 ---
MAP_PROVIDERS_UI = {
    "OpenStreetMap": "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png",
    "ESRI Satellite": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
}

# --- 全域設定 ---
DEVICE_PATH = "/dev/input/event0"
AXIS_MAP = {
    ecodes.ABS_Y: "pitch",
    ecodes.ABS_X: "roll",
    ecodes.ABS_Z: "throttle",
    ecodes.ABS_RX: "yaw",
}
CONNECTION_TIMEOUT = 5.0
HEARTBEAT_INTERVAL = 1.0
DEFAULT_ZOOM = 18

# --- 全域狀態 ---
joystick_state = JoystickState(axes={})
telemetry_data = TelemetryData()
app_running = True
drone_system = None
connection_lost_time = None
main_loop = None
mavlink_controller = None


# --- 模擬GPS類別 ---
class GPSSimulator:
    """GPS模擬器，當沒有實際GPS時使用"""

    def __init__(self, start_lat=25.0330, start_lon=121.5654):
        self.lat = start_lat
        self.lon = start_lon
        self.home_lat = start_lat
        self.home_lon = start_lon
        self.speed = 0.00001
        telemetry_data.position = (self.lat, self.lon)

    def update(self, vx, vy):
        """根據速度更新位置"""
        self.lat += vy * self.speed
        self.lon += vx * self.speed
        telemetry_data.position = (self.lat, self.lon)

    def get_position(self):
        return (self.lat, self.lon)

    def reset_to_home(self):
        self.lat = self.home_lat
        self.lon = self.home_lon
        telemetry_data.position = (self.lat, self.lon)


gps_simulator = GPSSimulator()


# --- 搖桿執行緒 ---
def joystick_thread_func():
    """搖桿讀取執行緒"""
    global joystick_state
    try:
        device = InputDevice(DEVICE_PATH)
        print(f"[搖桿] 成功連接到 {device.name}")

        for event in device.read_loop():
            if not app_running:
                break

            if event.type == ecodes.EV_ABS:
                # 移除自動模式切換邏輯，只記錄搖桿動作
                if event.code == ecodes.ABS_RY:
                    # 記錄按鈕動作但不自動切換模式
                    pass

                elif event.code in AXIS_MAP:
                    axis_name = AXIS_MAP[event.code]
                    min_val, max_val = (
                        device.absinfo(event.code).min,
                        device.absinfo(event.code).max,
                    )
                    normalized = (2 * (event.value - min_val) / (max_val - min_val)) - 1
                    joystick_state.axes[axis_name] = normalized

    except FileNotFoundError:
        print(f"[搖桿] 找不到裝置 {DEVICE_PATH}，將使用鍵盤控制")
    except Exception as e:
        print(f"[搖桿] 錯誤: {e}")


# === PyMAVLink 控制器 ===
# PX4 飛行模式對應
PX4_MODE_MAP = {
    "Manual": 1,
    "Altitude": 2,
    "Position": 3,
    "Acro": 4,
    "Offboard": 6,
    "Stabilized": 7,
    "Mission": 10,
    "Hold": 14,
    "Return": 12,
}

# 反向映射
PX4_MODE_REVERSE_MAP = {v: k for k, v in PX4_MODE_MAP.items()}


def encode_px4_mode(main_mode: int, sub_mode: int = 0) -> int:
    """編碼 PX4 模式"""
    return (sub_mode << 8) | (main_mode << 16)


def decode_px4_mode(custom_mode: int) -> str:
    """解碼 PX4 模式"""
    main_mode = (custom_mode >> 16) & 0xFF
    return PX4_MODE_REVERSE_MAP.get(main_mode, "Unknown")


class PyMAVLinkController:
    """PyMAVLink 主控制器 - 負責所有飛行控制"""

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
        self.manual_control_interval = 0.02  # 50Hz

    def log(self, message, level="INFO"):
        """統一日誌輸出"""
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] [PyMAVLink-{level}] {message}")

    def setup_connection(self):
        """建立 MAVLink 連接"""
        for connection_string in self.connection_attempts:
            try:
                self.log(f"嘗試連接: {connection_string}")

                if self.mavlink_master:
                    self.mavlink_master.close()

                self.mavlink_master = mavutil.mavlink_connection(connection_string)
                self.mavlink_master.wait_heartbeat(timeout=5)

                self.log(f"✓ MAVLink 連接成功: {connection_string}")
                self.log(
                    f"系統ID: {self.mavlink_master.target_system}, 組件ID: {self.mavlink_master.target_component}"
                )

                # 開始控制執行緒
                self.start_control_thread()

                return True

            except Exception as e:
                self.log(f"✗ {connection_string} 連接失敗: {e}", "ERROR")
                continue

        self.log("所有連接嘗試都失敗", "ERROR")
        self.mavlink_master = None
        return False

    def start_control_thread(self):
        """啟動控制執行緒"""
        if not self.control_thread or not self.control_thread.is_alive():
            self.control_running = True
            self.control_thread = threading.Thread(
                target=self.control_loop, daemon=True
            )
            self.control_thread.start()
            self.log("控制執行緒已啟動")

    def stop_control_thread(self):
        """停止控制執行緒"""
        self.control_running = False
        if self.control_thread:
            self.control_thread.join(timeout=1)
            self.log("控制執行緒已停止")

    def control_loop(self):
        """主控制迴圈 - 發送搖桿控制命令"""
        while self.control_running and app_running:
            try:
                current_time = time.time()

                # 檢查連接
                if not self.mavlink_master:
                    time.sleep(1)
                    continue

                # 發送手動控制命令
                if (
                    current_time - self.last_manual_control_time
                    >= self.manual_control_interval
                ):
                    self.send_manual_control()
                    self.last_manual_control_time = current_time

                # 處理接收的消息
                self.process_messages()

                time.sleep(0.001)  # 短暫休眠避免CPU佔用過高

            except Exception as e:
                self.log(f"控制迴圈錯誤: {e}", "ERROR")
                time.sleep(0.1)

    def send_manual_control(self):
        """發送手動控制命令"""
        if not self.mavlink_master or telemetry_data.armed != "已解鎖":
            return

        # 獲取搖桿輸入
        roll = joystick_state.axes.get("roll", 0.0)
        pitch = -joystick_state.axes.get("pitch", 0.0)
        yaw = joystick_state.axes.get("yaw", 0.0)
        throttle_raw = joystick_state.axes.get("throttle", -1.0)
        throttle = (throttle_raw + 1.0) / 2.0

        # 轉換為 MAVLink 格式 (-1000 到 1000)
        x = int(pitch * 1000)  # pitch
        y = int(roll * 1000)  # roll
        z = int(throttle * 1000)  # throttle (0-1000)
        r = int(yaw * 1000)  # yaw

        # 按鈕狀態 (未使用)
        buttons = 0

        try:
            # 發送 MANUAL_CONTROL 消息
            self.mavlink_master.mav.manual_control_send(
                self.mavlink_master.target_system, x, y, z, r, buttons
            )
        except Exception as e:
            self.log(f"發送手動控制失敗: {e}", "ERROR")

    def process_messages(self):
        """處理接收的 MAVLink 消息"""
        try:
            msg = self.mavlink_master.recv_match(blocking=False)
            if not msg:
                return

            # 更新心跳時間
            if msg.get_type() == "HEARTBEAT":
                telemetry_data.last_heartbeat = time.time()
                telemetry_data.px4_mode = msg.custom_mode
                mode_name = decode_px4_mode(msg.custom_mode)
                if mode_name != "Unknown":
                    telemetry_data.flight_mode = mode_name

                # 檢查解鎖狀態
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                telemetry_data.armed = "已解鎖" if armed else "未解鎖"

        except Exception as e:
            self.log(f"處理消息錯誤: {e}", "ERROR")

    def arm(self):
        """解鎖無人機"""
        if not self.mavlink_master:
            return False

        try:
            self.log("發送解鎖指令")
            self.mavlink_master.mav.command_long_send(
                self.mavlink_master.target_system,
                self.mavlink_master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            return True
        except Exception as e:
            self.log(f"解鎖失敗: {e}", "ERROR")
            return False

    def disarm(self):
        """上鎖無人機"""
        if not self.mavlink_master:
            return False

        try:
            self.log("發送上鎖指令")
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
        except Exception as e:
            self.log(f"上鎖失敗: {e}", "ERROR")
            return False

    def takeoff(self, altitude=10.0):
        """起飛"""
        if not self.mavlink_master:
            return False

        try:
            self.log(f"發送起飛指令 (高度: {altitude}m)")
            self.mavlink_master.mav.command_long_send(
                self.mavlink_master.target_system,
                self.mavlink_master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                altitude,
            )
            return True
        except Exception as e:
            self.log(f"起飛失敗: {e}", "ERROR")
            return False

    def land(self):
        """降落"""
        if not self.mavlink_master:
            return False

        try:
            self.log("發送降落指令")
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
        except Exception as e:
            self.log(f"降落失敗: {e}", "ERROR")
            return False

    def return_to_launch(self):
        """返航"""
        if not self.mavlink_master:
            return False

        try:
            self.log("發送返航指令")
            # 使用模式切換到 Return
            return self.set_flight_mode("Return")
        except Exception as e:
            self.log(f"返航失敗: {e}", "ERROR")
            return False

    def set_flight_mode(self, mode_name: str) -> bool:
        """設定飛行模式"""
        current_time = time.time()

        # 防止頻繁切換
        if current_time - self.last_mode_request_time < self.mode_request_cooldown:
            self.log("請等待冷卻時間", "WARN")
            return False

        # 檢查模式是否存在
        if mode_name not in PX4_MODE_MAP:
            self.log(f"不支援的模式名稱: {mode_name}", "ERROR")
            return False

        # 檢查連接
        if not self.mavlink_master:
            if not self.setup_connection():
                return False

        try:
            # 編碼模式
            main_mode = PX4_MODE_MAP[mode_name]
            custom_mode = encode_px4_mode(main_mode)

            self.log(f"請求切換至模式: {mode_name} (custom_mode={custom_mode})")

            # 發送模式切換指令
            self.mavlink_master.mav.set_mode_send(
                self.mavlink_master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode,
            )

            self.last_mode_request_time = current_time

            # 確認切換成功
            start_time = time.time()
            while time.time() - start_time < 5.0:
                msg = self.mavlink_master.recv_match(
                    type="HEARTBEAT", blocking=True, timeout=1
                )
                if msg:
                    if msg.custom_mode == custom_mode:
                        self.log(f"✅ 成功切換至 {mode_name}")
                        telemetry_data.flight_mode = mode_name
                        telemetry_data.px4_mode = custom_mode
                        return True
                time.sleep(0.1)

            self.log("❌ 模式切換超時", "ERROR")
            return False

        except Exception as e:
            self.log(f"模式切換失敗: {e}", "ERROR")
            return False

    def check_mode_requirements(
        self, mode_name: str, telemetry_data
    ) -> Tuple[bool, str]:
        """檢查飛行模式切換的前置條件"""
        self.log(f"檢查 {mode_name} 模式要求...")

        mode_requirements = {
            "Manual": {"requires_gps": False, "requires_armed": False},
            "Stabilized": {"requires_gps": False, "requires_armed": False},
            "Acro": {"requires_gps": False, "requires_armed": True},
            "Altitude": {"requires_gps": False, "requires_armed": False},
            "Position": {"requires_gps": True, "requires_armed": False},
            "Offboard": {"requires_gps": True, "requires_armed": True},
            "Hold": {"requires_gps": True, "requires_armed": True},
            "Mission": {"requires_gps": True, "requires_armed": True},
            "Return": {"requires_gps": True, "requires_armed": True},
        }

        if mode_name not in mode_requirements:
            return False, f"未知模式: {mode_name}"

        req = mode_requirements[mode_name]

        if not telemetry_data.is_connected:
            return False, "無人機未連接"

        if req["requires_gps"]:
            if not telemetry_data.position or telemetry_data.num_sats < 6:
                return False, f"需要 GPS 鎖定 (當前: {telemetry_data.num_sats} 衛星)"

        if req["requires_armed"]:
            if telemetry_data.armed != "已解鎖":
                return False, "需要先解鎖無人機"

        self.log(f"✓ {mode_name} 前置條件通過")
        return True, "OK"


# 創建全域 PyMAVLink 控制器
mavlink_controller = PyMAVLinkController()


# --- MAVSDK 執行緒（僅用於遙測） ---
async def run_mavsdk():
    """MAVSDK 主要執行緒 - 僅用於遙測數據"""
    global drone_system, telemetry_data, connection_lost_time, main_loop

    main_loop = asyncio.get_event_loop()
    print(f"[MAVSDK] 主事件循環已保存: {main_loop}")

    drone = System()
    drone_system = drone

    try:
        await drone.connect(system_address="udp://:14540")
        print("[MAVSDK] 等待連接...")

        async for state in drone.core.connection_state():
            if state.is_connected:
                print("[MAVSDK] 已連接!")
                telemetry_data.is_connected = True
                telemetry_data.last_heartbeat = time.time()

                # 建立 PyMAVLink 連接
                mavlink_controller.setup_connection()
                break

        # 訂閱遙測數據
        asyncio.ensure_future(subscribe_telemetry(drone))
        asyncio.ensure_future(heartbeat_monitor(drone))

        # 保持運行
        while app_running:
            await asyncio.sleep(1)

    except Exception as e:
        print(f"[MAVSDK] 錯誤: {e}")
        telemetry_data.is_connected = False


async def heartbeat_monitor(drone):
    """心跳監控，檢測連線狀態"""
    global connection_lost_time

    while app_running:
        current_time = time.time()

        if telemetry_data.is_connected:
            if current_time - telemetry_data.last_heartbeat > CONNECTION_TIMEOUT:
                print("[MAVSDK] 連線中斷")
                telemetry_data.is_connected = False
                connection_lost_time = current_time

                if telemetry_data.armed == "已解鎖":
                    # 使用 PyMAVLink 返航
                    mavlink_controller.return_to_launch()

        await asyncio.sleep(HEARTBEAT_INTERVAL)


async def subscribe_telemetry(drone):
    """訂閱所有遙測資料"""
    tasks = [
        asyncio.ensure_future(sub_gps_info(drone)),
        asyncio.ensure_future(sub_position(drone)),
        asyncio.ensure_future(sub_attitude_ang_vel(drone)),
        asyncio.ensure_future(sub_attitude_euler(drone)),
        asyncio.ensure_future(sub_battery(drone)),
    ]
    await asyncio.gather(*tasks)


async def sub_gps_info(drone):
    async for gps in drone.telemetry.gps_info():
        telemetry_data.gps_fix = str(gps.fix_type).split(".")[-1]
        telemetry_data.num_sats = gps.num_satellites


async def sub_position(drone):
    async for pos in drone.telemetry.position():
        telemetry_data.altitude_m = pos.relative_altitude_m

        if telemetry_data.num_sats >= 4:
            telemetry_data.position = (pos.latitude_deg, pos.longitude_deg)
        else:
            vx = joystick_state.axes.get("roll", 0.0)
            vy = joystick_state.axes.get("pitch", 0.0)
            gps_simulator.update(vx, vy)


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
        telemetry_data.battery_percent = battery.remaining_percent * 100


# --- 飛行動作處理 ---
def handle_flight_action(action: str) -> bool:
    """處理飛行動作"""
    global mavlink_controller

    print(f"\n=== 執行飛行動作: {action} ===")

    if not mavlink_controller.mavlink_master:
        print("[錯誤] MAVLink 未連接")
        return False

    try:
        if action == "ARM":
            success = mavlink_controller.arm()
            if (
                success
                and telemetry_data.home_position is None
                and telemetry_data.position is not None
            ):
                telemetry_data.home_position = telemetry_data.position
            return success

        elif action == "DISARM":
            return mavlink_controller.disarm()

        elif action == "TAKEOFF":
            if telemetry_data.armed != "已解鎖":
                print("[動作] 需要先解鎖")
                return False
            return mavlink_controller.takeoff()

        elif action == "LAND":
            return mavlink_controller.land()

        elif action == "HOLD":
            # Hold 通過切換到 Hold 模式實現
            return mavlink_controller.set_flight_mode("Hold")

        elif action == "Return":
            return mavlink_controller.return_to_launch()

        # 飛行模式
        elif action in PX4_MODE_MAP:
            can_switch, reason = mavlink_controller.check_mode_requirements(
                action, telemetry_data
            )
            if not can_switch:
                print(f"[模式] 前置條件失敗: {reason}")
                return False
            return mavlink_controller.set_flight_mode(action)

        else:
            print(f"[錯誤] 未知動作: {action}")
            return False

    except Exception as e:
        print(f"[錯誤] 執行動作失敗: {e}")
        return False


# --- GUI 應用程式 ---
class GCSProEnhanced:
    def __init__(self, root):
        self.root = root
        self.root.title("GCS Enhanced - PyMAVLink 控制版")
        self.root.geometry("1600x1000")
        self.root.configure(bg="#f0f0f0")

        # 地圖相關變數
        self.trail_points = []
        self.current_flight_mode = "Manual"
        self.map_widget: Optional[tkintermapview.TkinterMapView] = None
        self.drone_marker: Optional[tkintermapview.TkinterMapView.marker_class] = None
        self.home_marker: Optional[tkintermapview.TkinterMapView.marker_class] = None
        self.trail_path: Optional[tkintermapview.TkinterMapView.path_class] = None

        self.setup_styles()
        self.create_widgets()
        self.setup_keyboard_controls()

        self.root.after(100, self.initial_map_load)
        self.update_ui()

    def initial_map_load(self):
        """處理初始地圖載入，確保視窗已繪製"""
        if self.map_widget:
            self.map_widget.set_zoom(DEFAULT_ZOOM)
        else:
            self._create_fallback_map()

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

        map_status = "TkinterMapView" if MAP_WIDGET_AVAILABLE else "模擬地圖"
        ttk.Label(status_frame, text=f"🗺️ {map_status}", style="Info.TLabel").pack(
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

    def create_control_panel(self, parent):
        control_frame = ttk.LabelFrame(parent, text="飛行控制", padding=15)
        control_frame.pack(fill="x", pady=10)

        controls = [
            ("解鎖", "ARM", "#27ae60"),
            ("起飛", "TAKEOFF", "#3498db"),
            ("懸停", "Hold", "#f39c12"),
            ("返航", "Return", "#e74c3c"),
            ("降落", "LAND", "#9b59b6"),
            ("上鎖", "DISARM", "#95a5a6"),
        ]

        for i, (text, action, color) in enumerate(controls):
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
            )
            btn.grid(row=i // 2, column=i % 2, padx=5, pady=5)

    def create_flight_mode_selector(self, parent):
        mode_frame = ttk.LabelFrame(parent, text="飛行模式選擇 (PyMAVLink)", padding=15)
        mode_frame.pack(fill="x", pady=10)

        ttk.Label(mode_frame, text="選擇飛行模式:", style="Info.TLabel").pack(
            anchor="w", pady=(0, 5)
        )

        self.flight_mode_var = tk.StringVar(value="Manual")
        # 使用 PX4_MODE_MAP 的模式列表
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

        self.mode_indicator = tk.Canvas(
            mode_frame, width=200, height=30, highlightthickness=0
        )
        self.mode_indicator.pack(fill="x")
        self.update_mode_indicator()

    def create_joystick_display(self, parent):
        joystick_frame = ttk.LabelFrame(parent, text="搖桿狀態", padding=10)
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

        self._create_joystick_background(self.left_canvas, 150)
        self._create_joystick_background(self.right_canvas, 150)

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

        tk.Button(
            toolbar_frame,
            text="定位",
            command=self.center_on_drone,
            bg="#27ae60",
            fg="white",
        ).pack(side="left", padx=(20, 5))
        tk.Button(
            toolbar_frame,
            text="清除軌跡",
            command=self.clear_trail,
            bg="#f39c12",
            fg="white",
        ).pack(side="left", padx=5)

        map_container = ttk.Frame(map_frame)
        map_container.pack(fill="both", expand=True)

        if MAP_WIDGET_AVAILABLE:
            self.map_widget = tkintermapview.TkinterMapView(
                map_container, corner_radius=0
            )
            self.map_widget.pack(fill="both", expand=True)
            self.map_widget.set_position(gps_simulator.lat, gps_simulator.lon)
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
        }

        for i, (key, var) in enumerate(self.telemetry_vars.items()):
            style = (
                "Mono.TLabel"
                if key in ["altitude", "position", "attitude", "velocity"]
                else "Info.TLabel"
            )
            ttk.Label(telemetry_frame, textvariable=var, style=style).pack(
                anchor="w", pady=8
            )
            if i == 4:
                ttk.Separator(telemetry_frame, orient="horizontal").pack(
                    fill="x", pady=8
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

    def _create_joystick_background(self, canvas, size):
        center = size / 2
        canvas.delete("all")
        for i in range(4):
            radius = (size / 2 - 10) * (1 - i / 5)
            color = f"#{200 + i * 10:02x}{200 + i * 10:02x}{200 + i * 10:02x}"
            canvas.create_oval(
                center - radius,
                center - radius,
                center + radius,
                center + radius,
                fill=color,
                outline="#999999",
            )
        canvas.create_line(center, 10, center, size - 10, fill="#999999", dash=(3, 3))
        canvas.create_line(10, center, size - 10, center, fill="#999999", dash=(3, 3))

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
            self.update_mode_indicator()
            print("[GUI] 開始執行模式切換...")
            self.execute_action(selected_mode)
        else:
            print(f"[GUI] 模式切換被阻止: {reason}")
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
        if self.map_widget and telemetry_data.position:
            lat, lon = telemetry_data.position
            self.map_widget.set_position(lat, lon)

    def clear_trail(self):
        """清除軌跡"""
        self.trail_points = []
        if self.trail_path:
            self.trail_path.delete()
            self.trail_path = None

    def execute_action(self, action):
        """執行飛行動作或模式切換"""
        print(f"\n[GUI] 執行動作: {action}")

        def execute_in_thread():
            try:
                success = handle_flight_action(action)

                def update_gui():
                    if success:
                        if action in PX4_MODE_MAP:
                            self.current_flight_mode = action
                            self.update_mode_indicator()
                        messagebox.showinfo("成功", f"已執行 {action}")
                        print(f"[GUI] ✓ {action} 執行成功")
                    else:
                        messagebox.showerror(
                            "失敗", f"{action} 執行失敗\n請查看控制台輸出"
                        )
                        print(f"[GUI] ✗ {action} 執行失敗")

                self.root.after(0, update_gui)

            except Exception as e:
                print(f"[GUI] 執行異常: {e}")
                import traceback

                traceback.print_exc()

                def show_error(e):
                    messagebox.showerror("錯誤", f"執行動作時發生錯誤:\n{str(e)}")

                self.root.after(0, show_error(e=e))

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

        # 更新模式選擇器顯示
        if (
            telemetry_data.flight_mode != "未知"
            and telemetry_data.flight_mode != self.flight_mode_var.get()
        ):
            self.flight_mode_var.set(telemetry_data.flight_mode)
            self.update_mode_indicator()

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
        self.left_canvas.coords(self.left_stick, lx - 5, ly - 5, lx + 5, ly + 5)

        roll, pitch = joystick_state.axes.get("roll", 0.0), joystick_state.axes.get(
            "pitch", 0.0
        )
        rx, ry = 75 + roll * 65, 75 + pitch * 65
        self.right_canvas.coords(self.right_stick, rx - 5, ry - 5, rx + 5, ry + 5)

    def update_telemetry_info(self):
        """更新遙測資訊顯示"""
        self.telemetry_vars["armed"].set(f"解鎖狀態: {telemetry_data.armed}")
        self.telemetry_vars["mode"].set(f"飛行模式: {telemetry_data.flight_mode}")
        self.telemetry_vars["gps"].set(
            f"GPS: {'真實' if telemetry_data.num_sats >= 4 else '模擬'}({telemetry_data.gps_fix})"
        )
        self.telemetry_vars["satellites"].set(f"衛星數: {telemetry_data.num_sats:2d}")
        self.telemetry_vars["altitude"].set(f"高度: {telemetry_data.altitude_m:7.2f} m")

        if telemetry_data.position:
            lat, lon = telemetry_data.position
            self.telemetry_vars["position"].set(f"位置: {lat:9.6f}, {lon:10.6f}")

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

    def update_map_elements(self):
        """使用 tkintermapview 更新地圖元素"""
        if not self.map_widget or not telemetry_data.position:
            return

        lat, lon = telemetry_data.position

        if self.drone_marker is None:
            self.drone_marker = self.map_widget.set_marker(lat, lon, text="Drone")
        else:
            self.drone_marker.set_position(lat, lon)

        if telemetry_data.home_position:
            hlat, hlon = telemetry_data.home_position
            if self.home_marker is None:
                self.home_marker = self.map_widget.set_marker(
                    hlat,
                    hlon,
                    text="Home",
                    marker_color_circle="yellow",
                    marker_color_outside="orange",
                )
            else:
                self.home_marker.set_position(hlat, hlon)

        self.trail_points.append((lat, lon))
        if len(self.trail_points) > 300:
            self.trail_points.pop(0)

        if len(self.trail_points) > 1:
            if self.trail_path is None:
                self.trail_path = self.map_widget.set_path(
                    self.trail_points, color="#27ae60", width=3
                )
            else:
                self.trail_path.set_position_list(self.trail_points)

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


def on_closing():
    global app_running, mavlink_controller
    app_running = False

    if mavlink_controller:
        mavlink_controller.stop_control_thread()

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
