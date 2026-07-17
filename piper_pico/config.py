"""PiPER 遥操作配置。

PiPER 机械臂：6 自由度腕部（joint1..joint6，末端法兰 link6）+ 平行夹爪。
MJCF 中 "Gripper" 执行器驱动 joint7（prismatic, 0~0.035），joint8 通过 equality
约束耦合（joint8 = -joint7），故只需控制 joint7：open_pos=0（张开），close_pos=0.035（闭合）。
"""

from copy import deepcopy
from typing import Any, Dict

# 默认单臂（右手）配置
PIPER_TELEOP_CONFIG: Dict[str, Dict[str, Any]] = {
    "right_hand": {
        "link_name": "link6",
        "pose_source": "right_controller",
        "control_trigger": "right_grip",
        "vis_target": "piper_target",
        "control_mode": "pose",
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "right_trigger",
            "joint_names": ["joint7"],
            "open_pos": [0.0],
            "close_pos": [0.035],
        },
    },
}


def build_piper_config(
    control_mode: str = "pose",
    hand: str = "right",
) -> Dict[str, Dict[str, Any]]:
    """返回一份可修改的 PiPER 遥操作配置副本。

    Args:
        control_mode: "pose"（完整 6 自由度位姿）或 "position"（仅位置）。
        hand: "right" 或 "left"，选择使用哪只手控制器。
    """
    config = deepcopy(PIPER_TELEOP_CONFIG)
    key = f"{hand}_hand"
    if key not in config:
        # left_hand 由 right_hand 镜像得到
        src = config["right_hand"]
        config[key] = deepcopy(src)
        config[key]["pose_source"] = f"{hand}_controller"
        config[key]["control_trigger"] = f"{hand}_grip"
        config[key]["gripper_config"]["gripper_trigger"] = f"{hand}_trigger"
        config.pop("right_hand", None)
    config[key]["control_mode"] = control_mode
    return config
