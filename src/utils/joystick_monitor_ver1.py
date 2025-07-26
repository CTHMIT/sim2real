"""
GCS Pro Enhanced - 支援真實地圖的無人機地面控制站

功能特色：
1. 真實地圖支援 (OpenStreetMap, Google Maps, Bing Maps)
2. 搖桿控制支援 (TX16S 或其他標準搖桿)
3. 實時遙測資料顯示
4. GPS定位與動態地圖顯示
5. 斷線自動返航保護
6. 地圖瓦片緩存系統
7. QGC風格飛行模式選擇
8. 動態視窗調整支援

地圖服務提供商：
- OpenStreetMap (免費)
- Google Maps (需要API Key)
- Bing Maps (需要API Key)
- ESRI Satellite (免費)

飛行控制說明：
- 使用搖桿進行手動飛行控制
- 支援多種飛行模式切換
- 支援的動作: 解鎖/上鎖、起飛、降落、返航、懸停
- QGC風格的飛行模式選擇器
"""

import tkinter as tk
from tkinter import ttk
import asyncio
import threading
import time
import math
import os
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Tuple, Dict
from mavsdk import System
from evdev import InputDevice, ecodes

# 新增依賴庫
try:
    import requests
    from PIL import Image, ImageTk

    REAL_MAP_AVAILABLE = True
    print("[地圖] 真實地圖功能可用")
except ImportError as e:
    REAL_MAP_AVAILABLE = False
    print("[地圖] 真實地圖功能不可用，請安裝: pip install requests pillow")
    print(f"[地圖] 錯誤: {e}")


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
    position: Tuple[float, float] = (0.0, 0.0)
    is_connected: bool = False
    battery_percent: float = 100.0
    home_position: Optional[Tuple[float, float]] = None
    last_heartbeat: float = 0.0


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
    "Rattitude": {"color": "#f39c12", "description": "比例姿態模式"},
    "Altitude": {"color": "#27ae60", "description": "定高飛行模式"},
    "Offboard": {"color": "#e67e22", "description": "外部控制模式"},
    "Position": {"color": "#2ecc71", "description": "GPS定位模式"},
    "Hold": {"color": "#34495e", "description": "懸停保持模式"},
    "Mission": {"color": "#8e44ad", "description": "自動任務模式"},
    "Return": {"color": "#c0392b", "description": "自動返航模式"},
}

# --- 地圖服務提供商設定 ---
MAP_PROVIDERS = {
    "OpenStreetMap": {
        "url": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        "max_zoom": 19,
        "attribution": "© OpenStreetMap contributors",
    },
    "ESRI_Satellite": {
        "url": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        "max_zoom": 19,
        "attribution": "© ESRI",
    },
    "OpenTopoMap": {
        "url": "https://tile.opentopomap.org/{z}/{x}/{y}.png",
        "max_zoom": 17,
        "attribution": "© OpenTopoMap",
    },
    "CartoDB_Positron": {
        "url": "https://cartodb-basemaps-a.global.ssl.fastly.net/light_all/{z}/{x}/{y}.png",
        "max_zoom": 19,
        "attribution": "© CartoDB",
    },
}

# --- 全域設定 ---
DEVICE_PATH = "/dev/input/event0"
AXIS_MAP = {
    ecodes.ABS_Y: "pitch",
    ecodes.ABS_X: "roll",
    ecodes.ABS_Z: "throttle",
    ecodes.ABS_RX: "yaw",
}

# 連線超時設定
CONNECTION_TIMEOUT = 5.0
HEARTBEAT_INTERVAL = 1.0

# 地圖設定
DEFAULT_ZOOM = 18
TILE_SIZE = 256
CACHE_DIR = "map_cache"

# --- 全域狀態 ---
joystick_state = JoystickState(axes={})
telemetry_data = TelemetryData()
app_running = True
drone_system = None
connection_lost_time = None


# --- 真實地圖瓦片系統 ---
class RealMapTileSystem:
    """真實地圖瓦片系統"""

    def __init__(self):
        self.zoom = DEFAULT_ZOOM
        self.provider = "OpenStreetMap"
        self.cache_dir = CACHE_DIR
        self.tile_cache = {}  # 記憶體快取
        self.session = requests.Session() if REAL_MAP_AVAILABLE else None

        # 建立快取目錄
        if not os.path.exists(self.cache_dir):
            os.makedirs(self.cache_dir)

        # 設定請求標頭
        if self.session:
            self.session.headers.update(
                {"User-Agent": "GCS-Pro/1.0 (Drone Ground Control Station)"}
            )

    def deg2tile(self, lat_deg, lon_deg, zoom):
        """將經緯度轉換為瓦片座標"""
        lat_rad = math.radians(lat_deg)
        n = 2.0**zoom
        x = int((lon_deg + 180.0) / 360.0 * n)
        y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
        return (x, y)

    def tile2deg(self, x, y, zoom):
        """將瓦片座標轉換為經緯度"""
        n = 2.0**zoom
        lon_deg = x / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * y / n)))
        lat_deg = math.degrees(lat_rad)
        return (lat_deg, lon_deg)

    def get_tile_path(self, x, y, z, provider):
        """獲取瓦片快取路徑"""
        return os.path.join(self.cache_dir, provider, str(z), str(x), f"{y}.png")

    def download_tile(self, x, y, z, provider):
        """下載地圖瓦片"""
        if not REAL_MAP_AVAILABLE or not self.session:
            return None

        # 檢查記憶體快取
        cache_key = f"{provider}_{z}_{x}_{y}"
        if cache_key in self.tile_cache:
            return self.tile_cache[cache_key]

        # 檢查磁碟快取
        tile_path = self.get_tile_path(x, y, z, provider)
        if os.path.exists(tile_path):
            try:
                image = Image.open(tile_path)
                photo = ImageTk.PhotoImage(image)
                self.tile_cache[cache_key] = photo
                return photo
            except Exception as e:
                print(f"[地圖] 快取讀取失敗: {e}")

        # 下載新瓦片
        try:
            provider_info = MAP_PROVIDERS.get(provider, MAP_PROVIDERS["OpenStreetMap"])
            url = provider_info["url"].format(x=x, y=y, z=z)

            response = self.session.get(url, timeout=10)
            response.raise_for_status()

            # 儲存到磁碟快取
            os.makedirs(os.path.dirname(tile_path), exist_ok=True)
            with open(tile_path, "wb") as f:
                f.write(response.content)

            # 載入圖片
            image = Image.open(tile_path)
            photo = ImageTk.PhotoImage(image)

            # 儲存到記憶體快取
            self.tile_cache[cache_key] = photo

            return photo

        except Exception as e:
            print(f"[地圖] 下載瓦片失敗 {provider} {z}/{x}/{y}: {e}")
            return None

    def get_tile_bounds(self, x, y, z):
        """獲取瓦片的經緯度邊界"""
        nw_lat, nw_lon = self.tile2deg(x, y, z)
        se_lat, se_lon = self.tile2deg(x + 1, y + 1, z)
        return nw_lat, nw_lon, se_lat, se_lon

    def latlon_to_pixel(
        self, lat, lon, center_lat, center_lon, zoom, canvas_width, canvas_height
    ):
        """將經緯度轉換為畫布像素座標"""
        # 獲取中心點的瓦片座標
        center_tile_x, center_tile_y = self.deg2tile(center_lat, center_lon, zoom)

        # 獲取目標點的瓦片座標
        target_tile_x, target_tile_y = self.deg2tile(lat, lon, zoom)

        # 計算瓦片偏移
        tile_offset_x = target_tile_x - center_tile_x
        tile_offset_y = target_tile_y - center_tile_y

        # 轉換為像素偏移
        pixel_x = canvas_width // 2 + tile_offset_x * TILE_SIZE
        pixel_y = canvas_height // 2 + tile_offset_y * TILE_SIZE

        return pixel_x, pixel_y


# 建立地圖系統實例
real_map_system = RealMapTileSystem()


# --- 模擬GPS類別 ---
class GPSSimulator:
    """GPS模擬器，當沒有實際GPS時使用"""

    def __init__(self, start_lat=25.0330, start_lon=121.5654):
        self.lat = start_lat
        self.lon = start_lon
        self.home_lat = start_lat
        self.home_lon = start_lon
        self.speed = 0.00001

    def update(self, vx, vy):
        """根據速度更新位置"""
        self.lat += vy * self.speed
        self.lon += vx * self.speed

    def get_position(self):
        return (self.lat, self.lon)

    def reset_to_home(self):
        self.lat = self.home_lat
        self.lon = self.home_lon


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
                if event.code == ecodes.ABS_RY:
                    value = event.value
                    action = None
                    if value < 100:
                        action = "DISARMED_MODE"
                    elif 500 < value < 600:
                        action = "ALTITUDE_CONTROL"
                    elif 700 < value < 850:
                        action = "POSITION_CONTROL"
                    elif 1200 < value < 1300:
                        action = "RETURN_TO_LAUNCH"
                    elif 1400 < value < 1550:
                        action = "LAND"
                    elif value > 2000:
                        action = "ARM"

                    if action:
                        joystick_state.action = action

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


# --- MAVSDK 執行緒 ---
async def run_mavsdk():
    """MAVSDK 主要執行緒"""
    global drone_system, telemetry_data, connection_lost_time

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
                break

        asyncio.ensure_future(subscribe_telemetry(drone))
        asyncio.ensure_future(heartbeat_monitor(drone))
        await control_loop(drone)

    except Exception as e:
        print(f"[MAVSDK] 錯誤: {e}")
        telemetry_data.is_connected = False


async def control_loop(drone):
    """主控制迴圈"""
    arming_gesture_start_time = None
    disarming_gesture_start_time = None
    GESTURE_HOLD_DURATION = 1.5

    while app_running:
        if not telemetry_data.is_connected:
            await asyncio.sleep(0.1)
            continue

        roll = joystick_state.axes.get("roll", 0.0)
        pitch = -joystick_state.axes.get("pitch", 0.0)
        yaw = joystick_state.axes.get("yaw", 0.0)
        throttle_raw = joystick_state.axes.get("throttle", -1.0)
        throttle = (throttle_raw + 1.0) / 2.0

        is_arming_gesture = (
            (yaw < -0.8) and (pitch < -0.8) and (roll > 0.8) and (throttle < 0.1)
        )
        is_disarming_gesture = (
            (yaw > 0.8) and (pitch < -0.8) and (roll < -0.8) and (throttle < 0.1)
        )

        if is_arming_gesture:
            if arming_gesture_start_time is None:
                arming_gesture_start_time = time.time()
            elif time.time() - arming_gesture_start_time > GESTURE_HOLD_DURATION:
                try:
                    await drone.action.arm()
                    print("[MAVSDK] 解鎖成功")
                    if telemetry_data.home_position is None:
                        telemetry_data.home_position = telemetry_data.position
                except Exception as e:
                    print(f"[MAVSDK] 解鎖失敗: {e}")
                arming_gesture_start_time = None
        else:
            arming_gesture_start_time = None

        if is_disarming_gesture:
            if disarming_gesture_start_time is None:
                disarming_gesture_start_time = time.time()
            elif time.time() - disarming_gesture_start_time > GESTURE_HOLD_DURATION:
                try:
                    await drone.action.disarm()
                    print("[MAVSDK] 上鎖成功")
                except Exception as e:
                    print(f"[MAVSDK] 上鎖失敗: {e}")
                disarming_gesture_start_time = None
        else:
            disarming_gesture_start_time = None

        try:
            await drone.manual_control.set_manual_control_input(
                pitch, roll, throttle, yaw
            )
        except Exception:
            pass

        await asyncio.sleep(0.02)


async def heartbeat_monitor(drone):
    """心跳監控，檢測連線狀態"""
    global connection_lost_time

    while app_running:
        current_time = time.time()

        if telemetry_data.is_connected:
            if current_time - telemetry_data.last_heartbeat > CONNECTION_TIMEOUT:
                print("[MAVSDK] 連線中斷，啟動返航機制")
                telemetry_data.is_connected = False
                connection_lost_time = current_time

                if telemetry_data.armed == "已解鎖":
                    try:
                        await drone.action.return_to_launch()
                        print("[MAVSDK] 已發送返航指令")
                    except Exception as e:
                        print(f"[MAVSDK] 返航指令失敗: {e}")

        await asyncio.sleep(HEARTBEAT_INTERVAL)


async def subscribe_telemetry(drone):
    """訂閱所有遙測資料"""
    tasks = [
        asyncio.ensure_future(sub_armed(drone)),
        asyncio.ensure_future(sub_flight_mode(drone)),
        asyncio.ensure_future(sub_gps_info(drone)),
        asyncio.ensure_future(sub_position(drone)),
        asyncio.ensure_future(sub_attitude_ang_vel(drone)),
        asyncio.ensure_future(sub_attitude_euler(drone)),
        asyncio.ensure_future(sub_battery(drone)),
        asyncio.ensure_future(sub_health(drone)),
    ]
    await asyncio.gather(*tasks)


# 遙測訂閱函式
async def sub_armed(drone):
    async for armed in drone.telemetry.armed():
        telemetry_data.armed = "已解鎖" if armed else "未解鎖"


async def sub_flight_mode(drone):
    async for mode in drone.telemetry.flight_mode():
        mode_name = str(mode).split(".")[-1]
        telemetry_data.flight_mode = mode_name
        telemetry_data.is_rtl = mode_name == "RETURN_TO_LAUNCH"


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
            telemetry_data.position = gps_simulator.get_position()


async def sub_attitude_ang_vel(drone):
    async for vel in drone.telemetry.attitude_angular_velocity_body():
        telemetry_data.ang_vel = (
            math.degrees(vel.roll_rad_s),
            math.degrees(vel.pitch_rad_s),
            math.degrees(vel.yaw_rad_s),
        )


async def sub_attitude_euler(drone):
    """訂閱姿態角度資料"""
    async for attitude in drone.telemetry.attitude_euler():
        telemetry_data.attitude = (
            attitude.roll_deg,
            attitude.pitch_deg,
            attitude.yaw_deg,
        )


async def sub_battery(drone):
    async for battery in drone.telemetry.battery():
        telemetry_data.battery_percent = battery.remaining_percent * 100


async def sub_health(drone):
    async for health in drone.telemetry.health():
        telemetry_data.last_heartbeat = time.time()


# --- 飛行模式控制 ---
async def set_flight_mode(mode_name):
    """設定飛行模式"""
    global drone_system

    if not drone_system or not telemetry_data.is_connected:
        print("[MAVSDK] 無法切換模式: 未連線")
        return False

    try:
        if mode_name == "TAKEOFF":
            await drone_system.action.takeoff()
            print("[MAVSDK] 起飛指令已發送")
        elif mode_name == "LAND":
            await drone_system.action.land()
            print("[MAVSDK] 降落指令已發送")
        elif mode_name == "Return":
            await drone_system.action.return_to_launch()
            print("[MAVSDK] 返航指令已發送")
        elif mode_name == "Hold":
            await drone_system.action.hold()
            print("[MAVSDK] 懸停指令已發送")
        elif mode_name == "ARM":
            await drone_system.action.arm()
            print("[MAVSDK] 解鎖指令已發送")
        elif mode_name == "DISARM":
            await drone_system.action.disarm()
            print("[MAVSDK] 上鎖指令已發送")
        else:
            print(f"[MAVSDK] {mode_name} 模式需要透過 MAVLink 指令設定")
            return False
        return True
    except Exception as e:
        print(f"[MAVSDK] 動作執行失敗: {e}")
        return False


# --- GUI 應用程式 ---
class GCSProEnhanced:
    def __init__(self, root):
        self.root = root
        self.root.title("GCS Pro Enhanced - 真實地圖版")
        self.root.geometry("1600x1000")
        self.root.configure(bg="#f0f0f0")

        # 地圖相關變數
        self.map_center_lat = 25.0330
        self.map_center_lon = 121.5654
        self.trail_points = []
        self.current_flight_mode = "Manual"
        self.map_tiles = {}  # 儲存顯示的瓦片
        self.map_update_pending = False

        # 設定樣式
        self.setup_styles()

        # 建立UI元件
        self.create_widgets()

        # 綁定鍵盤控制
        self.setup_keyboard_controls()

        # 綁定視窗調整事件
        self.root.bind("<Configure>", self.on_window_resize)

        # 開始更新UI
        self.update_ui()

    def setup_styles(self):
        """設定GUI樣式 - 黑色字體"""
        style = ttk.Style()
        style.theme_use("clam")

        # 黑色字體主題
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
        # 主容器
        main_container = ttk.Frame(self.root, style="Light.TFrame")
        main_container.pack(fill="both", expand=True, padx=10, pady=10)

        # 頂部狀態列
        self.create_status_bar(main_container)

        # 中間區域
        middle_frame = ttk.Frame(main_container, style="Light.TFrame")
        middle_frame.pack(fill="both", expand=True, pady=10)

        # 左側控制面板
        left_panel = ttk.Frame(middle_frame, style="Light.TFrame")
        left_panel.pack(side="left", fill="y", padx=5)

        self.create_control_panel(left_panel)
        self.create_flight_mode_selector(left_panel)
        self.create_joystick_display(left_panel)

        # 中間地圖區域
        self.create_real_map_area(middle_frame)

        # 右側資訊面板
        right_panel = ttk.Frame(middle_frame, style="Light.TFrame")
        right_panel.pack(side="right", fill="y", padx=5)

        self.create_telemetry_panel(right_panel)

    def create_status_bar(self, parent):
        """建立頂部狀態列"""
        status_frame = ttk.Frame(parent, style="Light.TFrame", height=60)
        status_frame.pack(fill="x", pady=(0, 10))
        status_frame.pack_propagate(False)

        # 連線狀態
        self.conn_status_var = tk.StringVar(value="未連線")
        self.conn_status_label = ttk.Label(
            status_frame, textvariable=self.conn_status_var, style="Warning.TLabel"
        )
        self.conn_status_label.pack(side="left", padx=20, pady=10)

        # 電池狀態
        self.battery_var = tk.StringVar(value="電池: ---% ")
        ttk.Label(
            status_frame, textvariable=self.battery_var, style="Mono.TLabel"
        ).pack(side="left", padx=20, pady=10)

        # 地圖狀態
        map_status = "真實地圖" if REAL_MAP_AVAILABLE else "🗺️ 模擬地圖"
        self.map_status_var = tk.StringVar(value=map_status)
        ttk.Label(
            status_frame, textvariable=self.map_status_var, style="Info.TLabel"
        ).pack(side="left", padx=20, pady=10)

        # 當前飛行模式狀態
        self.mode_status_var = tk.StringVar(value="模式: Manual")
        self.mode_status_label = ttk.Label(
            status_frame, textvariable=self.mode_status_var, style="Title.TLabel"
        )
        self.mode_status_label.pack(side="left", padx=20, pady=10)

        # 時間顯示
        self.time_var = tk.StringVar(value="")
        ttk.Label(status_frame, textvariable=self.time_var, style="Info.TLabel").pack(
            side="right", padx=20, pady=10
        )

    def create_control_panel(self, parent):
        """建立控制面板"""
        control_frame = ttk.LabelFrame(parent, text="飛行控制", padding=15)
        control_frame.pack(fill="x", pady=10)

        # 基本控制按鈕
        controls = [
            ("解鎖", "ARM", "#27ae60"),
            ("起飛", "TAKEOFF", "#3498db"),
            ("懸停", "HOLD", "#f39c12"),
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
                activebackground=color,
                activeforeground="white",
            )
            btn.grid(row=i // 2, column=i % 2, padx=5, pady=5)

    def create_flight_mode_selector(self, parent):
        """建立QGC風格的飛行模式選擇器"""
        mode_frame = ttk.LabelFrame(parent, text="飛行模式選擇", padding=15)
        mode_frame.pack(fill="x", pady=10)

        # 飛行模式下拉選單
        ttk.Label(mode_frame, text="選擇飛行模式:", style="Info.TLabel").pack(
            anchor="w", pady=(0, 5)
        )

        self.flight_mode_var = tk.StringVar(value="Manual")
        self.mode_combobox = ttk.Combobox(
            mode_frame,
            textvariable=self.flight_mode_var,
            values=list(FLIGHT_MODES.keys()),
            state="readonly",
            width=18,
            font=("Arial", 12),
        )
        self.mode_combobox.pack(fill="x", pady=(0, 10))
        self.mode_combobox.bind("<<ComboboxSelected>>", self.on_mode_selected)

        # 模式描述顯示
        self.mode_desc_var = tk.StringVar(value=FLIGHT_MODES["Manual"]["description"])
        desc_label = ttk.Label(
            mode_frame,
            textvariable=self.mode_desc_var,
            style="Info.TLabel",
            wraplength=200,
        )
        desc_label.pack(anchor="w", pady=(0, 10))

        # 模式顏色指示器
        self.mode_indicator = tk.Canvas(
            mode_frame, width=200, height=30, highlightthickness=0
        )
        self.mode_indicator.pack(fill="x")
        self.update_mode_indicator()

        # 快捷模式按鈕
        ttk.Separator(mode_frame, orient="horizontal").pack(fill="x", pady=10)
        ttk.Label(mode_frame, text="快捷模式:", style="Info.TLabel").pack(anchor="w")

        quick_modes_frame = ttk.Frame(mode_frame)
        quick_modes_frame.pack(fill="x", pady=5)

        quick_modes = ["Manual", "Stabilized", "Position", "Hold", "Return"]
        for i, mode in enumerate(quick_modes):
            color = FLIGHT_MODES[mode]["color"]
            btn = tk.Button(
                quick_modes_frame,
                text=mode,
                command=lambda m=mode: self.set_quick_mode(m),
                bg=color,
                fg="white",
                font=("Arial", 9, "bold"),
                width=8,
                height=1,
                relief="raised",
                bd=1,
            )
            btn.grid(row=i // 3, column=i % 3, padx=2, pady=2)

    def create_joystick_display(self, parent):
        """建立搖桿顯示區"""
        joystick_frame = ttk.LabelFrame(parent, text="搖桿狀態", padding=10)
        joystick_frame.pack(fill="x", pady=10)

        # 搖桿顯示容器
        sticks_frame = ttk.Frame(joystick_frame)
        sticks_frame.pack(fill="x")

        # 左搖桿 (Yaw/Throttle)
        left_frame = ttk.Frame(sticks_frame)
        left_frame.pack(side="left", padx=10)
        ttk.Label(left_frame, text="偏航/油門", style="Info.TLabel").pack()
        self.left_canvas = tk.Canvas(
            left_frame,
            width=150,
            height=150,
            bg="#e8e8e8",
            highlightthickness=1,
            highlightbackground="#cccccc",
        )
        self.left_canvas.pack()

        # 右搖桿 (Roll/Pitch)
        right_frame = ttk.Frame(sticks_frame)
        right_frame.pack(side="right", padx=10)
        ttk.Label(right_frame, text="橫滾/俯仰", style="Info.TLabel").pack()
        self.right_canvas = tk.Canvas(
            right_frame,
            width=150,
            height=150,
            bg="#e8e8e8",
            highlightthickness=1,
            highlightbackground="#cccccc",
        )
        self.right_canvas.pack()

        # 初始化搖桿圖形
        self._create_joystick_background(self.left_canvas, 150)
        self._create_joystick_background(self.right_canvas, 150)

        self.left_stick = self.left_canvas.create_oval(
            70, 70, 80, 80, fill="#3498db", outline="#2c3e50", width=2
        )
        self.right_stick = self.right_canvas.create_oval(
            70, 70, 80, 80, fill="#e74c3c", outline="#2c3e50", width=2
        )

        # 鍵盤控制提示
        ttk.Separator(joystick_frame, orient="horizontal").pack(fill="x", pady=5)
        ttk.Label(
            joystick_frame,
            text="鍵盤控制 (WASD):",
            font=("Arial", 10, "bold"),
            style="Info.TLabel",
        ).pack()
        controls_text = "W/S: 前/後  A/D: 左/右\nQ/E: 左轉/右轉  R/F: 上/下"
        ttk.Label(joystick_frame, text=controls_text, style="Info.TLabel").pack()

    def create_real_map_area(self, parent):
        """建立真實地圖顯示區域"""
        map_frame = ttk.LabelFrame(parent, text="GPS定位地圖 (真實地圖)", padding=10)
        map_frame.pack(side="left", fill="both", expand=True, padx=10)

        # 地圖控制工具列
        toolbar_frame = ttk.Frame(map_frame)
        toolbar_frame.pack(fill="x", pady=(0, 5))

        # 地圖提供商選擇
        ttk.Label(toolbar_frame, text="地圖提供商:", style="Info.TLabel").pack(
            side="left", padx=5
        )
        self.map_provider_var = tk.StringVar(value="OpenStreetMap")
        provider_combo = ttk.Combobox(
            toolbar_frame,
            textvariable=self.map_provider_var,
            values=list(MAP_PROVIDERS.keys()),
            width=15,
            state="readonly",
        )
        provider_combo.pack(side="left", padx=5)
        provider_combo.bind("<<ComboboxSelected>>", self.on_map_provider_changed)

        # 縮放控制
        ttk.Label(toolbar_frame, text="縮放:", style="Info.TLabel").pack(
            side="left", padx=(20, 5)
        )

        # 縮放等級顯示
        self.zoom_var = tk.StringVar(value=f"Z{real_map_system.zoom}")
        ttk.Label(
            toolbar_frame, textvariable=self.zoom_var, style="Mono.TLabel", width=4
        ).pack(side="left", padx=5)

        zoom_in_btn = tk.Button(
            toolbar_frame,
            text="＋",
            command=self.zoom_in,
            width=3,
            bg="#3498db",
            fg="white",
            font=("Arial", 12, "bold"),
        )
        zoom_in_btn.pack(side="left", padx=2)
        zoom_out_btn = tk.Button(
            toolbar_frame,
            text="－",
            command=self.zoom_out,
            width=3,
            bg="#e74c3c",
            fg="white",
            font=("Arial", 12, "bold"),
        )
        zoom_out_btn.pack(side="left", padx=2)

        # 地圖功能按鈕
        center_btn = tk.Button(
            toolbar_frame,
            text="定位",
            command=self.center_on_drone,
            bg="#27ae60",
            fg="white",
            font=("Arial", 10, "bold"),
        )
        center_btn.pack(side="left", padx=(20, 5))

        clear_btn = tk.Button(
            toolbar_frame,
            text="清除軌跡",
            command=self.clear_trail,
            bg="#f39c12",
            fg="white",
            font=("Arial", 10, "bold"),
        )
        clear_btn.pack(side="left", padx=5)

        # 重新載入地圖按鈕
        reload_btn = tk.Button(
            toolbar_frame,
            text="重載",
            command=self.reload_map,
            bg="#9b59b6",
            fg="white",
            font=("Arial", 10, "bold"),
        )
        reload_btn.pack(side="left", padx=5)

        # 地圖畫布容器
        map_container = ttk.Frame(map_frame)
        map_container.pack(fill="both", expand=True)

        # 地圖畫布
        self.map_canvas = tk.Canvas(
            map_container,
            bg="#f5f5dc",
            highlightthickness=1,
            highlightbackground="#cccccc",
        )
        self.map_canvas.pack(fill="both", expand=True)

        # 無人機圖標
        self.drone_icon = None
        self.home_marker = None
        self.trail_line = None

        # 座標顯示
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

        # 地圖載入狀態
        self.map_loading_var = tk.StringVar(value="")
        ttk.Label(
            coord_frame, textvariable=self.map_loading_var, style="Info.TLabel"
        ).pack()

    def create_telemetry_panel(self, parent):
        """建立遙測資訊面板"""
        telemetry_frame = ttk.LabelFrame(parent, text="遙測資訊", padding=15)
        telemetry_frame.pack(fill="both", expand=True)

        # 建立資訊標籤
        self.telemetry_vars = {
            "armed": tk.StringVar(value="解鎖狀態: 未知"),
            "mode": tk.StringVar(value="飛行模式: 未知"),
            "gps": tk.StringVar(value="GPS: 未知"),
            "altitude": tk.StringVar(value="高度:    0.00 m"),
            "position": tk.StringVar(value="位置:  0.000000,   0.000000"),
            "attitude": tk.StringVar(value="姿態: R:  +0.0 P:  +0.0 Y:  +0.0"),
            "velocity": tk.StringVar(value="角速度: R:  +0.0 P:  +0.0 Y:  +0.0"),
            "satellites": tk.StringVar(value="衛星數:  0"),
            "home": tk.StringVar(value="家位置: 未設定"),
        }

        mono_labels = ["altitude", "position", "attitude", "velocity"]

        for i, (key, var) in enumerate(self.telemetry_vars.items()):
            style = "Mono.TLabel" if key in mono_labels else "Info.TLabel"
            label = ttk.Label(telemetry_frame, textvariable=var, style=style)
            label.pack(anchor="w", pady=8)

            if i == 4:
                ttk.Separator(telemetry_frame, orient="horizontal").pack(
                    fill="x", pady=8
                )

    def load_map_tiles_async(self):
        """非同步載入地圖瓦片"""
        if not REAL_MAP_AVAILABLE:
            self._create_fallback_map()
            return

        def load_tiles():
            try:
                canvas_width = self.map_canvas.winfo_width()
                canvas_height = self.map_canvas.winfo_height()

                if canvas_width <= 1 or canvas_height <= 1:
                    return

                zoom = real_map_system.zoom
                provider = self.map_provider_var.get()

                # 計算中心瓦片
                center_tile_x, center_tile_y = real_map_system.deg2tile(
                    self.map_center_lat, self.map_center_lon, zoom
                )

                # 計算需要載入的瓦片範圍
                tiles_x = (canvas_width // TILE_SIZE) + 2
                tiles_y = (canvas_height // TILE_SIZE) + 2

                start_x = center_tile_x - tiles_x // 2
                start_y = center_tile_y - tiles_y // 2

                # 更新載入狀態
                self.root.after(0, lambda: self.map_loading_var.set("📥 載入地圖..."))

                # 載入瓦片
                loaded_tiles = []
                for x in range(start_x, start_x + tiles_x):
                    for y in range(start_y, start_y + tiles_y):
                        if x >= 0 and y >= 0:  # 確保瓦片座標有效
                            tile_image = real_map_system.download_tile(
                                x, y, zoom, provider
                            )
                            if tile_image:
                                # 計算瓦片在畫布上的位置
                                tile_canvas_x = (
                                    canvas_width // 2 + (x - center_tile_x) * TILE_SIZE
                                )
                                tile_canvas_y = (
                                    canvas_height // 2 + (y - center_tile_y) * TILE_SIZE
                                )
                                loaded_tiles.append(
                                    (tile_image, tile_canvas_x, tile_canvas_y)
                                )

                # 在主執行緒中更新畫布
                self.root.after(0, lambda: self._update_map_canvas(loaded_tiles))

            except Exception as e:
                print(f"[地圖] 載入瓦片失敗: {e}")
                self.root.after(0, lambda: self.map_loading_var.set("❌ 地圖載入失敗"))

        # 在後台執行緒中載入瓦片
        threading.Thread(target=load_tiles, daemon=True).start()

    def _update_map_canvas(self, tiles):
        """更新地圖畫布"""
        try:
            # 清除舊瓦片
            self.map_canvas.delete("tile")

            # 顯示新瓦片
            for tile_image, x, y in tiles:
                self.map_canvas.create_image(
                    x, y, image=tile_image, anchor="nw", tags="tile"
                )

            # 確保無人機和標記在最上層
            self.map_canvas.tag_raise("drone")
            self.map_canvas.tag_raise("home")
            self.map_canvas.tag_raise("trail")

            self.map_loading_var.set("地圖已載入")

            # 2秒後清除載入狀態
            self.root.after(2000, lambda: self.map_loading_var.set(""))

        except Exception as e:
            print(f"[地圖] 更新畫布失敗: {e}")
            self.map_loading_var.set("顯示失敗")

    def _create_fallback_map(self):
        """建立備用地圖（當真實地圖不可用時）"""
        canvas_width = self.map_canvas.winfo_width()
        canvas_height = self.map_canvas.winfo_height()

        if canvas_width <= 1 or canvas_height <= 1:
            self.root.after(100, self._create_fallback_map)
            return

        self.map_canvas.delete("tile")

        # 建立簡單的網格地圖
        self.map_canvas.create_rectangle(
            0, 0, canvas_width, canvas_height, fill="#e8f5e8", outline="", tags="tile"
        )

        # 添加網格
        for i in range(0, canvas_width, 50):
            self.map_canvas.create_line(
                i, 0, i, canvas_height, fill="#cccccc", dash=(2, 2), tags="tile"
            )
        for j in range(0, canvas_height, 50):
            self.map_canvas.create_line(
                0, j, canvas_width, j, fill="#cccccc", dash=(2, 2), tags="tile"
            )

        # 添加指北針
        self._add_compass(canvas_width, canvas_height)

        self.map_loading_var.set("使用模擬地圖")

    def _add_compass(self, width, height):
        """添加指北針"""
        compass_x = width - 50
        compass_y = 50

        # 指北針背景
        self.map_canvas.create_oval(
            compass_x - 25,
            compass_y - 25,
            compass_x + 25,
            compass_y + 25,
            fill="#ffffff",
            outline="#000000",
            width=2,
            tags="compass",
        )

        # 北方指針
        self.map_canvas.create_polygon(
            compass_x,
            compass_y - 20,
            compass_x - 8,
            compass_y + 5,
            compass_x + 8,
            compass_y + 5,
            fill="#e74c3c",
            tags="compass",
        )

        # N標記
        self.map_canvas.create_text(
            compass_x,
            compass_y - 30,
            text="N",
            fill="#000000",
            font=("Arial", 14, "bold"),
            tags="compass",
        )

    def _create_joystick_background(self, canvas, size):
        """繪製搖桿背景"""
        center = size / 2

        # 清除舊內容
        canvas.delete("all")

        # 背景圓圈
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

        # 十字準線
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
        """處理按鍵按下事件"""
        key = event.char.lower()
        if key in self.keyboard_control:
            self.keyboard_control[key] = True
            self.update_keyboard_control()

    def on_key_release(self, event):
        """處理按鍵釋放事件"""
        key = event.char.lower()
        if key in self.keyboard_control:
            self.keyboard_control[key] = False
            self.update_keyboard_control()

    def update_keyboard_control(self):
        """更新鍵盤控制到搖桿狀態"""
        pitch = 0.0
        if self.keyboard_control["w"]:
            pitch = 0.5
        if self.keyboard_control["s"]:
            pitch = -0.5

        roll = 0.0
        if self.keyboard_control["a"]:
            roll = -0.5
        if self.keyboard_control["d"]:
            roll = 0.5

        yaw = 0.0
        if self.keyboard_control["q"]:
            yaw = -0.5
        if self.keyboard_control["e"]:
            yaw = 0.5

        throttle = joystick_state.axes.get("throttle", 0.0)
        if self.keyboard_control["r"]:
            throttle = min(1.0, throttle + 0.1)
        if self.keyboard_control["f"]:
            throttle = max(-1.0, throttle - 0.1)

        if any(self.keyboard_control.values()):
            joystick_state.axes["pitch"] = pitch
            joystick_state.axes["roll"] = roll
            joystick_state.axes["yaw"] = yaw
            joystick_state.axes["throttle"] = throttle

    def on_window_resize(self, event):
        """處理視窗大小調整"""
        if event.widget == self.root and not self.map_update_pending:
            self.map_update_pending = True
            # 延遲重新載入地圖，避免頻繁更新
            self.root.after(500, self._delayed_map_update)

    def _delayed_map_update(self):
        """延遲地圖更新"""
        self.map_update_pending = False
        self.load_map_tiles_async()

    def on_mode_selected(self, event):
        """處理飛行模式選擇"""
        selected_mode = self.flight_mode_var.get()
        self.current_flight_mode = selected_mode
        self.mode_desc_var.set(FLIGHT_MODES[selected_mode]["description"])
        self.update_mode_indicator()

        # 執行模式切換
        def async_set_mode():
            asyncio.run(set_flight_mode(selected_mode))

        thread = threading.Thread(target=async_set_mode, daemon=True)
        thread.start()

        print(f"[GUI] 切換飛行模式至: {selected_mode}")

    def set_quick_mode(self, mode):
        """設定快捷模式"""
        self.flight_mode_var.set(mode)
        self.on_mode_selected(None)

    def update_mode_indicator(self):
        """更新模式顏色指示器"""
        mode = self.current_flight_mode
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
        real_map_system.provider = self.map_provider_var.get()
        self.load_map_tiles_async()
        print(f"[地圖] 切換地圖提供商至: {real_map_system.provider}")

    def zoom_in(self):
        """放大地圖"""
        max_zoom = MAP_PROVIDERS.get(real_map_system.provider, {"max_zoom": 19})[
            "max_zoom"
        ]
        if real_map_system.zoom < max_zoom:
            real_map_system.zoom += 1
            self.zoom_var.set(f"Z{real_map_system.zoom}")
            self.load_map_tiles_async()

    def zoom_out(self):
        """縮小地圖"""
        if real_map_system.zoom > 1:
            real_map_system.zoom -= 1
            self.zoom_var.set(f"Z{real_map_system.zoom}")
            self.load_map_tiles_async()

    def center_on_drone(self):
        """將地圖中心移至無人機位置"""
        if telemetry_data.position:
            self.map_center_lat, self.map_center_lon = telemetry_data.position
            self.load_map_tiles_async()

    def clear_trail(self):
        """清除軌跡"""
        self.trail_points = []
        self.map_canvas.delete("trail")

    def reload_map(self):
        """重新載入地圖"""
        # 清除瓦片快取
        real_map_system.tile_cache.clear()
        self.load_map_tiles_async()
        print("[地圖] 重新載入地圖")

    def update_ui(self):
        """更新UI顯示"""
        if not app_running:
            return

        # 更新時間
        self.time_var.set(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

        # 更新連線狀態
        if telemetry_data.is_connected:
            self.conn_status_var.set("已連線")
            self.conn_status_label.configure(style="Status.TLabel")
        else:
            self.conn_status_var.set("未連線")
            self.conn_status_label.configure(style="Warning.TLabel")

        # 更新電池
        self.battery_var.set(f"電池: {telemetry_data.battery_percent:3.0f}%")

        # 更新飛行模式狀態
        current_mode = (
            telemetry_data.flight_mode
            if telemetry_data.flight_mode != "未知"
            else self.current_flight_mode
        )
        self.mode_status_var.set(f"模式: {current_mode}")

        # 更新縮放等級顯示
        self.zoom_var.set(f"Z{real_map_system.zoom}")

        # 更新搖桿顯示
        self.update_joystick_display()

        # 更新遙測資訊
        self.update_telemetry_info()

        # 更新地圖
        self.update_enhanced_map()

        # 繼續更新
        self.root.after(30, self.update_ui)

    def update_joystick_display(self):
        """更新搖桿視覺顯示"""
        # 左搖桿 (Yaw/Throttle)
        yaw = joystick_state.axes.get("yaw", 0.0)
        throttle = joystick_state.axes.get("throttle", -1.0)
        lx = 75 + yaw * 65
        ly = 75 - throttle * 65
        self.left_canvas.coords(self.left_stick, lx - 5, ly - 5, lx + 5, ly + 5)

        # 右搖桿 (Roll/Pitch)
        roll = joystick_state.axes.get("roll", 0.0)
        pitch = joystick_state.axes.get("pitch", 0.0)
        rx = 75 + roll * 65
        ry = 75 + pitch * 65
        self.right_canvas.coords(self.right_stick, rx - 5, ry - 5, rx + 5, ry + 5)

    def update_telemetry_info(self):
        """更新遙測資訊顯示"""
        self.telemetry_vars["armed"].set(f"解鎖狀態: {telemetry_data.armed}")
        self.telemetry_vars["mode"].set(f"飛行模式: {telemetry_data.flight_mode}")

        if telemetry_data.num_sats >= 4:
            gps_status = f"🛰️ GPS: {telemetry_data.gps_fix} (真實)"
        else:
            gps_status = "🛰️ GPS: 模擬定位"
        self.telemetry_vars["gps"].set(gps_status)

        self.telemetry_vars["satellites"].set(f"衛星數: {telemetry_data.num_sats:2d}")
        self.telemetry_vars["altitude"].set(f"高度: {telemetry_data.altitude_m:7.2f} m")

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
            home_lat, home_lon = telemetry_data.home_position
            self.telemetry_vars["home"].set(
                f"家位置: {home_lat:9.6f}, {home_lon:10.6f}"
            )

    def update_enhanced_map(self):
        """更新增強型地圖顯示"""
        if not telemetry_data.position:
            return

        lat, lon = telemetry_data.position
        canvas_width = self.map_canvas.winfo_width()
        canvas_height = self.map_canvas.winfo_height()

        if canvas_width <= 1 or canvas_height <= 1:
            return

        # 檢查是否需要重新載入地圖（位置變化較大時）
        lat_diff = abs(lat - self.map_center_lat)
        lon_diff = abs(lon - self.map_center_lon)

        if lat_diff > 0.001 or lon_diff > 0.001:  # 位置變化超過閾值
            self.map_center_lat = lat
            self.map_center_lon = lon
            self.load_map_tiles_async()

        # 無人機位置（相對於當前地圖中心）
        if REAL_MAP_AVAILABLE:
            drone_x, drone_y = real_map_system.latlon_to_pixel(
                lat,
                lon,
                self.map_center_lat,
                self.map_center_lon,
                real_map_system.zoom,
                canvas_width,
                canvas_height,
            )
        else:
            # 備用計算方式
            drone_x = canvas_width // 2
            drone_y = canvas_height // 2

        # 更新或創建無人機圖標
        if self.drone_icon is None:
            self.drone_icon = self.map_canvas.create_polygon(
                drone_x,
                drone_y - 12,
                drone_x - 8,
                drone_y + 8,
                drone_x,
                drone_y + 12,
                drone_x + 8,
                drone_y + 8,
                fill="#e74c3c",
                outline="#ffffff",
                width=3,
                tags="drone",
            )
        else:
            self.map_canvas.coords(
                self.drone_icon,
                drone_x,
                drone_y - 12,
                drone_x - 8,
                drone_y + 8,
                drone_x,
                drone_y + 12,
                drone_x + 8,
                drone_y + 8,
            )

        # 更新家位置標記
        if telemetry_data.home_position:
            home_lat, home_lon = telemetry_data.home_position

            if REAL_MAP_AVAILABLE:
                home_x, home_y = real_map_system.latlon_to_pixel(
                    home_lat,
                    home_lon,
                    self.map_center_lat,
                    self.map_center_lon,
                    real_map_system.zoom,
                    canvas_width,
                    canvas_height,
                )
            else:
                # 簡化計算
                home_x = canvas_width // 2 + (home_lon - lon) * 10000
                home_y = canvas_height // 2 - (home_lat - lat) * 10000

            if 0 <= home_x <= canvas_width and 0 <= home_y <= canvas_height:
                if self.home_marker is None:
                    self.home_marker = self.map_canvas.create_oval(
                        home_x - 8,
                        home_y - 8,
                        home_x + 8,
                        home_y + 8,
                        fill="#ffeb3b",
                        outline="#f57f17",
                        width=3,
                        tags="home",
                    )
                else:
                    self.map_canvas.coords(
                        self.home_marker, home_x - 8, home_y - 8, home_x + 8, home_y + 8
                    )
                    self.map_canvas.itemconfig(self.home_marker, state="normal")
            else:
                if self.home_marker:
                    self.map_canvas.itemconfig(self.home_marker, state="hidden")

        # 更新軌跡
        self.trail_points.append((lat, lon))
        if len(self.trail_points) > 300:
            self.trail_points.pop(0)

        # 重繪軌跡線
        self.map_canvas.delete("trail")
        if len(self.trail_points) > 1:
            trail_coords = []
            for trail_lat, trail_lon in self.trail_points:
                if REAL_MAP_AVAILABLE:
                    trail_x, trail_y = real_map_system.latlon_to_pixel(
                        trail_lat,
                        trail_lon,
                        self.map_center_lat,
                        self.map_center_lon,
                        real_map_system.zoom,
                        canvas_width,
                        canvas_height,
                    )
                else:
                    trail_x = canvas_width // 2 + (trail_lon - lon) * 10000
                    trail_y = canvas_height // 2 - (trail_lat - lat) * 10000

                if 0 <= trail_x <= canvas_width and 0 <= trail_y <= canvas_height:
                    trail_coords.extend([trail_x, trail_y])

            if len(trail_coords) >= 4:
                self.map_canvas.create_line(
                    trail_coords, fill="#27ae60", width=3, smooth=True, tags="trail"
                )

        # 更新座標顯示
        self.coord_var.set(f"座標: {lat:.6f}, {lon:.6f}")

        # 計算並顯示距離
        if telemetry_data.home_position:
            home_lat, home_lon = telemetry_data.home_position
            distance = self._calculate_distance(lat, lon, home_lat, home_lon)
            self.distance_var.set(f"距離家: {distance:.1f}m")

    def _calculate_distance(self, lat1, lon1, lat2, lon2):
        """計算兩點間距離（米）"""
        R = 6371000
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)

        a = math.sin(delta_lat / 2) * math.sin(delta_lat / 2) + math.cos(
            lat1_rad
        ) * math.cos(lat2_rad) * math.sin(delta_lon / 2) * math.sin(delta_lon / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    def execute_action(self, action):
        """執行飛行動作"""

        def async_execute():
            asyncio.run(set_flight_mode(action))

        thread = threading.Thread(target=async_execute, daemon=True)
        thread.start()


# --- 主程式入口 ---
def on_closing():
    """關閉程式時的處理"""
    global app_running
    app_running = False
    time.sleep(0.2)
    root.destroy()


def main():
    """主程式"""
    global root

    print("[啟動] GCS Pro Enhanced - 真實地圖版")
    print(f"[系統] 真實地圖功能: {'可用' if REAL_MAP_AVAILABLE else '不可用'}")

    if not REAL_MAP_AVAILABLE:
        print("[提示] 要使用真實地圖功能，請執行:")
        print("       pip install requests pillow")

    # 啟動搖桿執行緒
    joystick_thread = threading.Thread(target=joystick_thread_func, daemon=True)
    joystick_thread.start()

    # 啟動MAVSDK執行緒
    mavsdk_thread = threading.Thread(
        target=lambda: asyncio.run(run_mavsdk()), daemon=True
    )
    mavsdk_thread.start()

    # 建立GUI
    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", on_closing)

    # 啟動主迴圈
    root.mainloop()


if __name__ == "__main__":
    main()
