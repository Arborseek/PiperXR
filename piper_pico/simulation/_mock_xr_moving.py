"""会自己动的 mock xrobotoolkit_sdk，用于 --mock 模式：无 PICO 时也能看到仿真臂跟随。

get_pose_by_name 返回一个随时间做圆周/上下运动的位姿；
grip 恒为 1（始终激活），trigger 做缓慢开合，便于观察夹爪。
"""

import math
import time

import numpy as np

_T0 = time.time()


def _t():
    return time.time() - _T0


def _pose(hand: str):
    t = _t()
    # 右手在 +y 侧、左手在 -y 侧，各自做小幅圆周 + 上下运动
    sign = 1.0 if hand == "right" else -1.0
    x = 0.35 + 0.08 * math.sin(0.6 * t)
    y = sign * 0.18 + 0.06 * math.cos(0.6 * t)
    z = 0.30 + 0.06 * math.sin(0.9 * t)
    # 朝向：基本朝下抓取（绕 x 180°），叠加小幅扰动
    qw = math.cos(0.5 * math.sin(0.3 * t))
    qx = math.sin(0.5 * math.sin(0.3 * t))
    return np.array([x, y, z, qx, 0.0, 0.0, qw], dtype=float)


def init():
    print("[mock-xr-moving] 注入会自己动的假手柄数据（--mock 模式）")


def get_left_controller_pose():
    return _pose("left")


def get_right_controller_pose():
    return _pose("right")


def get_headset_pose():
    return _pose("right")


def get_left_trigger():
    return 0.5 + 0.5 * math.sin(0.7 * _t())


def get_right_trigger():
    return 0.5 + 0.5 * math.sin(0.7 * _t() + 1.0)


def get_left_grip():
    return 1.0


def get_right_grip():
    return 1.0


def get_left_menu_button():
    return False


def get_right_menu_button():
    return False


def get_A_button():
    return False


def get_B_button():
    return False


def get_X_button():
    return False


def get_Y_button():
    return False


def get_left_axis_click():
    return False


def get_right_axis_click():
    return False


def get_motion_tracker_data():
    return {}


def num_motion_data_available():
    return 0
