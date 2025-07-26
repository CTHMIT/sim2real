from pymavlink import mavutil
import time

# PX4 飛行模式對應 custom_mode ID
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


def encode_px4_mode(main_mode: int, sub_mode: int = 0) -> int:
    return (sub_mode << 8) | (main_mode << 16)


# === 使用者指定目標模式 ===
target_mode_name = "Altitude"

# === 檢查模式是否存在 ===
if target_mode_name not in PX4_MODE_MAP:
    print(f"[錯誤] 不支援的模式名稱: {target_mode_name}")
    exit(1)

main_mode = PX4_MODE_MAP[target_mode_name]
custom_mode = encode_px4_mode(main_mode)

# === 建立連線 ===
print("[MAVLINK] 建立連線中...")
master = mavutil.mavlink_connection("udp:0.0.0.0:14550")
master.wait_heartbeat()
print(
    f"[MAVLINK] 已連線 PX4 (sys={master.target_system}, comp={master.target_component})"
)

# === 顯示當前模式 ===
msg = master.recv_match(type="HEARTBEAT", blocking=True)
print(f"[目前模式] custom_mode={msg.custom_mode} (type={msg.type})")

# === 發送模式切換指令 ===
master.mav.set_mode_send(
    target_system=master.target_system,
    base_mode=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    custom_mode=custom_mode,
)
print(f"[MAVLINK] 請求切換至模式: {target_mode_name} (custom_mode={custom_mode})")

# === 確認是否切換成功 ===
for i in range(5):
    hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    if hb:
        print(f"[確認中] 第{i+1}次: custom_mode={hb.custom_mode}")
        if hb.custom_mode == custom_mode:
            print(f"[✅ 成功] 模式已切換為 {target_mode_name}")
            break
    else:
        print(f"[警告] 第{i+1}次未收到 heartbeat")
    time.sleep(1)
else:
    print("[❌ 失敗] 模式切換未成功")
