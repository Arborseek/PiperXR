"""诊断 PICO 手柄数据是否正常流入。

用法：
    python scripts/diagnose_xr.py

会以 20Hz 持续打印左右手柄的位姿(xyz+quat)、grip、trigger、以及 A/B/X/Y 按键状态。
- 位姿非零且随手移动而变化 → 追踪正常。
- grip/trigger 按下时数值变 1 → 按键正常。
- 全是 0 或不变 → PC 服务没收到 PICO 数据（检查头显/客户端/网络）。
"""

import time

import numpy as np

from xrobotoolkit_teleop.common.xr_client import XrClient


def main():
    client = XrClient()
    print("XRoboToolkit SDK initialized.")
    print("移动手柄 / 捏 grip / 扣 trigger，观察下面数值是否变化。Ctrl+C 退出。\n")
    try:
        while True:
            for hand in ("right", "left"):
                try:
                    pose = client.get_pose_by_name(f"{hand}_controller")
                    grip = client.get_key_value_by_name(f"{hand}_grip")
                    trig = client.get_key_value_by_name(f"{hand}_trigger")
                    xyz = np.array(pose[:3])
                    quat = np.array([pose[6], pose[3], pose[4], pose[5]])  # w,x,y,z
                    print(f"[{hand}] xyz={xyz.round(3)} q={quat.round(3)} "
                          f"grip={grip:.2f} trigger={trig:.2f}")
                except Exception as e:
                    print(f"[{hand}] 读取失败: {e}")
            print("-" * 40)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n退出诊断。")


if __name__ == "__main__":
    main()
