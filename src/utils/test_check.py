#!/usr/bin/env python3
"""
PX4 參數檢查腳本
檢查影響解鎖的關鍵參數
"""

from pymavlink import mavutil
import time
import asyncio


def get_param(master, param_name):
    """獲取單個參數值"""
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode("utf-8").ljust(16, b"\0"),
        -1,
    )

    start_time = time.time()
    while time.time() - start_time < 2:
        msg = master.recv_match(type="PARAM_VALUE", blocking=False)
        if msg and msg.param_id.strip("\0") == param_name:
            return msg.param_value
    return None


def main():
    print("PX4 參數檢查工具")
    print("=" * 50)

    # 建立連接
    print("\n正在連接 MAVLink...")
    master = mavutil.mavlink_connection("udp:0.0.0.0:14550")
    master.wait_heartbeat()
    print(f"✓ 已連接到系統 {master.target_system}")

    # 關鍵參數列表
    critical_params = {
        "COM_ARM_WO_GPS": {
            "desc": "允許無GPS解鎖",
            "good": 1,
            "note": "設為1允許在沒有GPS的情況下解鎖",
        },
        "COM_ARM_EKF_AB": {
            "desc": "EKF加速度偏差閾值",
            "good": 0.005,
            "note": "較大的值更寬鬆，建議0.005-0.01",
        },
        "COM_ARM_EKF_GB": {
            "desc": "EKF陀螺儀偏差閾值",
            "good": 0.0011,
            "note": "較大的值更寬鬆，建議0.0011-0.002",
        },
        "COM_ARM_EKF_HGT": {
            "desc": "EKF高度誤差閾值",
            "good": 1.0,
            "note": "較大的值更寬鬆，建議1.0-2.0米",
        },
        "COM_ARM_EKF_VEL": {
            "desc": "EKF速度誤差閾值",
            "good": 0.5,
            "note": "較大的值更寬鬆，建議0.5-1.0 m/s",
        },
        "COM_ARM_EKF_POS": {
            "desc": "EKF位置誤差閾值",
            "good": 2.5,
            "note": "較大的值更寬鬆，建議2.5-5.0米",
        },
        "COM_ARM_EKF_YAW": {
            "desc": "EKF偏航誤差閾值",
            "good": 0.5,
            "note": "較大的值更寬鬆，建議0.5-1.0弧度",
        },
        "COM_ARM_IMU_ACC": {
            "desc": "IMU加速度一致性檢查",
            "good": 0.7,
            "note": "較大的值更寬鬆，建議0.7-1.0 m/s²",
        },
        "COM_ARM_IMU_GYR": {
            "desc": "IMU陀螺儀一致性檢查",
            "good": 0.25,
            "note": "較大的值更寬鬆，建議0.25-0.35 rad/s",
        },
        "COM_ARM_MAG_ANG": {
            "desc": "磁力計最大角度差",
            "good": 45,
            "note": "較大的值更寬鬆，建議30-60度",
        },
        "COM_ARM_MAG_STR": {
            "desc": "磁力計強度檢查",
            "good": 1,
            "note": "設為0禁用磁力計強度檢查",
        },
        "COM_PREARM_MODE": {
            "desc": "預解鎖模式",
            "good": 0,
            "note": "0=禁用, 1=安全開關, 2=始終",
        },
        "CBRK_SUPPLY_CHK": {
            "desc": "電源檢查斷路器",
            "good": 894281,
            "note": "設為894281禁用電源檢查",
        },
        "CBRK_USB_CHK": {
            "desc": "USB檢查斷路器",
            "good": 197848,
            "note": "設為197848允許USB連接時解鎖",
        },
        "CBRK_AIRSPD_CHK": {
            "desc": "空速檢查斷路器",
            "good": 162128,
            "note": "設為162128禁用空速檢查",
        },
    }

    print("\n檢查關鍵參數...")
    print("-" * 80)
    print(f"{'參數名':<20} {'當前值':<12} {'建議值':<12} {'說明':<40}")
    print("-" * 80)

    issues = []

    for param_name, info in critical_params.items():
        current_value = get_param(master, param_name)

        if current_value is not None:
            # 判斷是否有問題
            is_good = True
            if param_name.startswith("CBRK_"):
                # 斷路器參數需要精確匹配
                is_good = current_value == info["good"]
            elif param_name == "COM_ARM_WO_GPS":
                # 布爾參數
                is_good = current_value >= info["good"]
            else:
                # 閾值參數，當前值應該大於等於建議值
                is_good = current_value >= info["good"]

            status = "✓" if is_good else "✗"

            print(
                f"{status} {param_name:<18} {current_value:<12.4f} {info['good']:<12} {info['desc']}"
            )

            if not is_good:
                issues.append((param_name, current_value, info))
        else:
            print(f"? {param_name:<18} {'N/A':<12} {info['good']:<12} {info['desc']}")

    # 總結和建議
    print("\n\n診斷總結:")
    print("=" * 50)

    if not issues:
        print("✓ 所有參數看起來都正常！")
        print("\n如果仍然無法解鎖，請檢查：")
        print("1. 等待系統完全初始化（10-30秒）")
        print("2. 檢查 QGroundControl 的消息選項卡")
        print("3. 確保傳感器已校準")
        print("4. 嘗試重啟飛控")
    else:
        print(f"✗ 發現 {len(issues)} 個潛在問題：\n")

        for param_name, current_value, info in issues:
            print(f"問題: {param_name}")
            print(f"  當前值: {current_value}")
            print(f"  建議值: {info['good']}")
            print(f"  說明: {info['note']}\n")

        print("\n修改參數的方法：")
        print("1. 使用 QGroundControl -> 參數")
        print("2. 或使用 MAVLink 命令設置參數")

        # 詢問是否要自動修復
        print("\n是否要查看修復命令？(僅顯示，不會自動執行)")
        for param_name, current_value, info in issues:
            print(f"\n# 修復 {param_name}")
            print("# 使用 mavlink 設置參數：")
            print("master.mav.param_set_send(")
            print("    master.target_system,")
            print("    master.target_component,")
            print(f"    b'{param_name}'.ljust(16),")
            print(f"    {info['good']},")
            print("    mavutil.mavlink.MAV_PARAM_TYPE_REAL32")
            print(")")

    print("\n完成檢查")
    master.close()


if __name__ == "__main__":
    asyncio.run(main())
