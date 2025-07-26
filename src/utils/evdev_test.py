import sys
from evdev import InputDevice, categorize, ecodes

# 預設的裝置檔案路徑。如果您的搖桿不是 event0，請修改此處。
# 您可以透過 `ls -l /dev/input/` 來查看所有事件裝置。
DEVICE_PATH = "/dev/input/event0"


def main():
    """
    主函式，連接到裝置並讀取事件。
    """
    try:
        # 連接到指定的輸入裝置
        device = InputDevice(DEVICE_PATH)
        print(f"✅ 成功連接到裝置: {device.name}")
        print(f"   物理路徑: {device.phys}")
        print(
            f"   裝置資訊: VID={device.info.vendor:04x}, PID={device.info.product:04x}, Version={device.info.version:04x}"
        )
        print("\n--- 請移動搖桿或按下按鈕... (按下 Ctrl+C 結束) ---\n")

        # 進入事件讀取迴圈
        for event in device.read_loop():
            # 我們主要關心兩種事件：
            # 1. EV_ABS: 絕對軸事件 (搖桿的軸、推桿)
            # 2. EV_KEY: 按鍵事件 (搖桿上的按鈕、開關)

            if event.type == ecodes.EV_ABS:
                # categorize 函式可以讓輸出結果更容易閱讀
                abs_event = categorize(event)
                # 透過 ecodes 字典找出事件代碼的名稱 (例如 'ABS_X')
                code_name = ecodes.bytype[abs_event.event.type][abs_event.event.code]
                print(
                    f"軸事件   -> 名稱: {code_name},  \t代碼: {abs_event.event.code}, \t數值: {abs_event.event.value}"
                )

            elif event.type == ecodes.EV_KEY:
                # 我們只在按鈕 "按下" (value=1) 的時候顯示資訊，避免 "放開" (value=0) 的訊息洗版
                if event.value == 1:
                    print(f"按鈕事件 -> 代碼: {event.code} (按下)")

    except FileNotFoundError:
        print(f"❌ 錯誤: 找不到裝置檔案 '{DEVICE_PATH}'。")
        print("   請確認您的搖桿已透過 usbipd 正確附加到 WSL2。")
    except PermissionError:
        print(f"❌ 錯誤: 權限不足，無法讀取 '{DEVICE_PATH}'。")
        print(
            "   請執行 `sudo usermod -aG input $USER` 並徹底重啟 WSL2 終端機 (`wsl --shutdown`)。"
        )
    except Exception as e:
        print(f"發生未知錯誤: {e}")
    finally:
        print("\n--- 測試結束 ---")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # 讓使用者可以用 Ctrl+C 優雅地退出程式
        print("\n使用者中斷程式。")
        sys.exit(0)
