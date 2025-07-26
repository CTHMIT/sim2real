import tkinter as tk
from tkinter import ttk
import asyncio
import threading
import time
from mavsdk import System
from evdev import InputDevice, ecodes
from typing import Dict

# --- 全域共享狀態 ---
# 這兩個字典會在不同執行緒之間安全地傳遞資料
joystick_state: Dict = {"axes": {}, "buttons": {}}
telemetry_data: Dict = {"armed": "未知", "flight_mode": "未知"}
app_running: bool = True  # 用於優雅地關閉程式

# --- 搖桿與 MAVSDK 設定 ---
# 搖桿裝置檔案路徑
DEVICE_PATH = "/dev/input/event0"

# 軸的事件代碼對應 (請填入您搖桿的代碼)
# 假設為美國手 (Mode 2)
AXIS_MAP = {
    ecodes.ABS_Y: "pitch",  # 左搖桿上下 (通常是 Pitch 或 Throttle)
    ecodes.ABS_X: "yaw",  # 左搖桿左右 (通常是 Yaw 或 Roll)
    ecodes.ABS_Z: "throttle",  # 右搖桿上下
    ecodes.ABS_RX: "roll",  # 右搖桿左右
}
# 有些搖桿可能用 ABS_Z, ABS_RY 等，請務必確認

# 按鈕的事件代碼對應 (請填入您搖桿的代碼)
BUTTON_MAP = {
    ecodes.BTN_SOUTH: "arm",  # A 按鈕
    ecodes.BTN_EAST: "disarm",  # B 按鈕
    ecodes.BTN_NORTH: "takeoff",  # X 按鈕
    ecodes.BTN_WEST: "land",  # Y 按鈕
}


# --- 執行緒 1: 搖桿讀取 ---
def joystick_thread_func():
    """在背景持續讀取 evdev 事件並更新 joystick_state"""
    try:
        device = InputDevice(DEVICE_PATH)
        print(f"搖桿執行緒: 成功連接到 {device.name}")

        for event in device.read_loop():
            if not app_running:
                break

            if event.type == ecodes.EV_ABS:  # 軸移動
                if event.code in AXIS_MAP:
                    axis_name = AXIS_MAP[event.code]
                    # 將搖桿的原始值 (通常是 0-255 或 0-1023) 標準化到 -1.0 到 1.0
                    min_val = device.absinfo(event.code).min
                    max_val = device.absinfo(event.code).max
                    value_normalized = (
                        2 * (event.value - min_val) / (max_val - min_val)
                    ) - 1
                    joystick_state["axes"][axis_name] = value_normalized

            elif event.type == ecodes.EV_KEY:  # 按鈕按壓
                if event.code in BUTTON_MAP:
                    button_name = BUTTON_MAP[event.code]
                    joystick_state["buttons"][button_name] = (
                        event.value == 1
                    )  # 1 for press, 0 for release

    except Exception as e:
        print(f"搖桿執行緒錯誤: {e}")


# --- 執行緒 2: MAVSDK 通訊 ---
async def run_mavsdk():
    """連接無人機，接收遙測，並根據搖桿手勢和軸位置發送指令"""
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("MAVSDK: 等待連接...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("MAVSDK: 已連接!")
            break

    asyncio.ensure_future(subscribe_telemetry(drone))

    # --- 新增：手勢計時器相關變數 ---
    arming_gesture_start_time = None
    disarming_gesture_start_time = None
    GESTURE_HOLD_DURATION = 1.5  # 需要維持手勢 1.5 秒

    # 主控制迴圈
    while app_running:
        # --- 讀取所有需要的軸狀態 ---
        roll = joystick_state["axes"].get("roll", 0.0)
        pitch = -joystick_state["axes"].get("pitch", 0.0)  # MAVSDK 俯仰向前為正
        yaw = joystick_state["axes"].get("yaw", 0.0)
        throttle_raw = joystick_state["axes"].get("throttle", -1.0)
        throttle = (throttle_raw + 1.0) / 2.0

        # --- 新增：手勢檢測邏輯 ---
        # 為了讓手勢更容易觸發，我們用 0.8 作為閾值而不是 1.0
        is_arming_gesture = (
            (yaw < -0.8) and (pitch < -0.8) and (roll > 0.8) and (throttle < 0.1)
        )
        is_disarming_gesture = (
            (yaw > 0.8) and (pitch < -0.8) and (roll < -0.8) and (throttle < 0.1)
        )

        # --- 處理解鎖手勢 ---
        if is_arming_gesture:
            if arming_gesture_start_time is None:
                # 第一次偵測到手勢，開始計時
                arming_gesture_start_time = time.time()
                print("偵測到解鎖手勢，請維持...")
            elif time.time() - arming_gesture_start_time > GESTURE_HOLD_DURATION:
                print("MAVSDK: 手勢有效，嘗試解鎖...")
                try:
                    await drone.action.arm()
                    print(">>> 解鎖成功！")
                    # <<< 在這裡加入起飛指令 >>>
                    print(">>> 現在嘗試自動起飛至 2.5 公尺...")
                    await drone.action.set_takeoff_altitude(2.5)
                    await drone.action.takeoff()

                except Exception as e:
                    print(f"解鎖或起飛失敗: {e}")
                arming_gesture_start_time = None
        else:
            # 如果手勢中斷，重置計時器
            arming_gesture_start_time = None

        # --- 處理上鎖手勢 ---
        if is_disarming_gesture:
            if disarming_gesture_start_time is None:
                disarming_gesture_start_time = time.time()
                print("偵測到上鎖手勢，請維持...")
            elif time.time() - disarming_gesture_start_time > GESTURE_HOLD_DURATION:
                print("MAVSDK: 手勢有效，嘗試上鎖...")
                try:
                    await drone.action.disarm()
                except Exception as e:
                    print(f"上鎖失敗: {e}")
                disarming_gesture_start_time = None
        else:
            disarming_gesture_start_time = None

        # --- 持續發送搖桿軸的飛行控制指令 ---
        try:
            await drone.manual_control.set_manual_control_input(
                pitch, roll, throttle, yaw
            )
        except Exception:
            pass  # 忽略在非手動模式下的錯誤

        await asyncio.sleep(0.02)  # 以約 50Hz 的頻率發送


async def subscribe_telemetry(drone):
    """監聽遙測並更新 telemetry_data"""
    async for armed in drone.telemetry.armed():
        telemetry_data["armed"] = "已解鎖" if armed else "未解鎖"
    async for mode in drone.telemetry.flight_mode():
        telemetry_data["flight_mode"] = f"飛行模式: {mode}"


# --- 主執行緒: Tkinter GUI ---
class GCSApp:
    def __init__(self, root):
        self.root = root
        self.root.title("最終版 GCS (evdev + mavsdk)")
        # ... (GUI 元件的建立與之前相同) ...
        main_frame = ttk.Frame(root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        joystick_frame = ttk.LabelFrame(
            main_frame, text="搖桿狀態 (TX16S)", padding="10"
        )
        joystick_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E))
        self.left_canvas = tk.Canvas(joystick_frame, width=150, height=150, bg="white")
        self.left_canvas.grid(row=0, column=0, padx=5)
        self.left_stick = self.left_canvas.create_oval(
            75 - 15, 75 - 15, 75 + 15, 75 + 15, fill="blue"
        )
        self.right_canvas = tk.Canvas(joystick_frame, width=150, height=150, bg="white")
        self.right_canvas.grid(row=0, column=1, padx=5)
        self.right_stick = self.right_canvas.create_oval(
            75 - 15, 75 - 15, 75 + 15, 75 + 15, fill="red"
        )
        telemetry_frame = ttk.LabelFrame(main_frame, text="遙測資訊", padding="10")
        telemetry_frame.grid(
            row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10
        )
        self.armed_label = ttk.Label(
            telemetry_frame, text="狀態: -", font=("Arial", 12)
        )
        self.armed_label.pack(anchor="w")
        self.mode_label = ttk.Label(
            telemetry_frame, text="飛行模式: -", font=("Arial", 12)
        )
        self.mode_label.pack(anchor="w")
        self.update_ui()

    def update_ui(self):
        # 從共享狀態中讀取資料並更新 GUI
        # 更新虛擬搖桿
        roll = joystick_state["axes"].get("roll", 0.0)
        pitch = joystick_state["axes"].get("pitch", 0.0)
        yaw = joystick_state["axes"].get("yaw", 0.0)
        throttle = joystick_state["axes"].get("throttle", 0.0)

        # 左搖桿
        lx, ly = 75 + roll * 60, 75 - throttle * 60
        self.left_canvas.coords(self.left_stick, lx - 15, ly - 15, lx + 15, ly + 15)

        # 右搖桿
        rx, ry = 75 + yaw * 60, 75 + pitch * 60
        self.right_canvas.coords(self.right_stick, rx - 15, ry - 15, rx + 15, ry + 15)

        # 更新遙測文字
        self.armed_label.config(text=f"狀態: {telemetry_data['armed']}")
        self.mode_label.config(text=telemetry_data["flight_mode"])

        if app_running:
            self.root.after(20, self.update_ui)


def on_closing():
    """處理關閉視窗事件"""
    global app_running
    print("關閉程式...")
    app_running = False
    time.sleep(0.1)  # 給執行緒一點時間來退出迴圈
    root.destroy()


if __name__ == "__main__":
    # 建立並啟動搖桿執行緒
    joystick_thread = threading.Thread(target=joystick_thread_func, daemon=True)
    joystick_thread.start()

    # 建立並啟動 MAVSDK 執行緒
    mavsdk_thread = threading.Thread(
        target=lambda: asyncio.run(run_mavsdk()), daemon=True
    )
    mavsdk_thread.start()

    # 在主執行緒中啟動 Tkinter GUI
    root = tk.Tk()
    app = GCSApp(root)
    root.protocol("WM_DELETE_WINDOW", on_closing)  # 處理關閉視窗
    root.mainloop()
