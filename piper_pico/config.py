"""PiPER 遥操作配置。

PiPER 机械臂：6 自由度腕部（joint1..joint6，末端法兰 link6）+ 平行夹爪。
MJCF 中 "Gripper" 执行器驱动 joint7（prismatic, 0~0.035），joint8 通过 equality
约束耦合（joint8 = -joint7），故只需控制 joint7：open_pos=0（张开），close_pos=0.035（闭合）。
"""

from copy import deepcopy
from typing import Any, Dict

import numpy as np

# PiPER 专用 头显->世界 旋转：在 XRoboToolkit 默认 R_HEADSET_TO_WORLD 基础上翻转 Y 行，
# 使“手往右移 -> 末端往右移”（默认会把左右映射反）。sim 与 real 共用，保证 sim2real 一致。
R_HEADSET_TO_WORLD_PIPER = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, 1, 0],
    ]
)

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
    """返回一份可修改的 PiPER 遥操作配置副本（单臂）。

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


# 双臂配置：两臂并排安装于桌面，关节/链接名带 right_/left_ 前缀（与 piper_dual.xml 一致）
PIPER_DUAL_TELEOP_CONFIG: Dict[str, Dict[str, Any]] = {
    "right_hand": {
        "link_name": "right_link6",
        "pose_source": "right_controller",
        "control_trigger": "right_grip",
        "vis_target": "right_target",
        "control_mode": "pose",
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "right_trigger",
            "joint_names": ["right_joint7"],
            "open_pos": [0.0],
            "close_pos": [0.035],
        },
    },
    "left_hand": {
        "link_name": "left_link6",
        "pose_source": "left_controller",
        "control_trigger": "left_grip",
        "vis_target": "left_target",
        "control_mode": "pose",
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "left_trigger",
            "joint_names": ["left_joint7"],
            "open_pos": [0.0],
            "close_pos": [0.035],
        },
    },
}


def build_dual_piper_config(control_mode: str = "pose") -> Dict[str, Dict[str, Any]]:
    """返回双臂遥操作配置副本。"""
    config = deepcopy(PIPER_DUAL_TELEOP_CONFIG)
    for key in config:
        config[key]["control_mode"] = control_mode
    return config


# 真机配置：夹爪用归一化 [0..1]（0=张、1=合），由 PiperArmProxy 换算到 0.001mm。
PIPER_REAL_TELEOP_CONFIG: Dict[str, Dict[str, Any]] = {
    "right_hand": {
        "link_name": "link6",
        "pose_source": "right_controller",
        "control_trigger": "right_grip",
        "vis_target": "",  # 真机无 mocap 可视化
        "control_mode": "pose",
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "right_trigger",
            "joint_names": ["gripper"],
            "open_pos": [0.0],
            "close_pos": [1.0],
        },
    },
}


def build_real_piper_config(control_mode: str = "pose", hand: str = "right") -> Dict[str, Dict[str, Any]]:
    """单臂真机配置。"""
    config = deepcopy(PIPER_REAL_TELEOP_CONFIG)
    key = f"{hand}_hand"
    if key not in config:
        src = config["right_hand"]
        config[key] = deepcopy(src)
        config[key]["pose_source"] = f"{hand}_controller"
        config[key]["control_trigger"] = f"{hand}_grip"
        config[key]["gripper_config"]["gripper_trigger"] = f"{hand}_trigger"
        config.pop("right_hand", None)
    config[key]["control_mode"] = control_mode
    return config


def build_real_dual_piper_config(control_mode: str = "pose") -> Dict[str, Dict[str, Any]]:
    """双臂真机配置（right_/left_ 前缀，与 piper_dual_description.urdf 一致）。"""
    base = {
        "right_hand": {
            "link_name": "right_link6",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "vis_target": "",
            "control_mode": control_mode,
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": "right_trigger",
                "joint_names": ["gripper"],
                "open_pos": [0.0],
                "close_pos": [1.0],
            },
        },
        "left_hand": {
            "link_name": "left_link6",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "vis_target": "",
            "control_mode": control_mode,
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": "left_trigger",
                "joint_names": ["gripper"],
                "open_pos": [0.0],
                "close_pos": [1.0],
            },
        },
    }
    return base
