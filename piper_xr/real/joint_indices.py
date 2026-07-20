"""placo state.q 中各臂 joint1..joint6 的切片（与 URDF 布局一致）。

单臂 piper_description.urdf:        joint1..6 @ q[7:13]
双臂 piper_dual_description.urdf:   left @ q[7:13],  right @ q[15:21]
"""

from typing import Tuple


def q6_slice_for_link(link_name: str) -> slice:
    """由末端 link 名推断该臂 6 关节在 placo state.q 中的 slice。"""
    if link_name.startswith("right_"):
        return slice(15, 21)
    # left_* 或单臂 link6
    return slice(7, 13)


def joint1_name_for_link(link_name: str) -> str:
    if link_name.startswith("right_"):
        return "right_joint1"
    if link_name.startswith("left_"):
        return "left_joint1"
    return "joint1"
