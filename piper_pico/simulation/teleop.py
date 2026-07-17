"""PICO 4 Ultra -> MuJoCo 仿真中松灵 PiPER 机械臂的遥操作入口。

支持单臂（桌面右侧 PiPER）与双臂（桌面双 PiPER + 抓取物）两种模式。

数据流：
    PICO 头显 (XRoboToolkit-PICO-Client)
        -> XRoboToolkit-PC-Service (PC 端服务, 端口 60061)
        -> xrobotoolkit_sdk (Python 绑定, XrClient)
        -> MujocoTeleopController (placo 逆运动学 + MuJoCo 仿真)
        -> PiPER 机械臂在 MuJoCo 中实时跟随手部运动

运行前请确认：
  1. 已启动 XRoboToolkit-PC-Service，且 PICO 端客户端已连接；
  2. 已在 conda 环境 pico_teleop 中执行过 `bash scripts/setup_env.sh`。
"""

import tyro

from piper_pico.config import build_dual_piper_config, build_piper_config
from piper_pico.paths import (
    PIPER_DUAL_SCENE_XML,
    PIPER_DUAL_URDF,
    PIPER_SCENE_XML,
    PIPER_URDF,
)
from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)


def run(
    dual: bool = False,
    xml_path: str = "",
    robot_urdf_path: str = "",
    scale_factor: float = 1.5,
    control_mode: str = "pose",
    hand: str = "right",
    visualize_placo: bool = False,
):
    """PiPER 遥操作，支持单臂/双臂。

    Args:
        dual: True 使用双臂场景（桌面双 PiPER + 抓取物），False 使用单臂场景。
        xml_path: MuJoCo 场景文件。留空则按 dual 选择默认场景。
        robot_urdf_path: URDF（供 placo IK）。留空则按 dual 选择默认 URDF。
        scale_factor: 手部位移到机械臂末端位移的缩放系数。
        control_mode: "pose" 为完整位姿控制（6 自由度），"position" 为仅位置控制。
        hand: 单臂模式下使用哪只手控制器，"right" 或 "left"。
        visualize_placo: 是否在浏览器中可视化 placo 的 IK 求解结果。
    """
    if dual:
        xml_path = xml_path or PIPER_DUAL_SCENE_XML
        robot_urdf_path = robot_urdf_path or PIPER_DUAL_URDF
        config = build_dual_piper_config(control_mode=control_mode)
    else:
        xml_path = xml_path or PIPER_SCENE_XML
        robot_urdf_path = robot_urdf_path or PIPER_URDF
        config = build_piper_config(control_mode=control_mode, hand=hand)

    controller = MujocoTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
    )

    # 关节正则化（软约束），让 IK 倾向于回到零位，避免关节漂移。
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    controller.run()


def main():
    """tyro CLI 入口（支持 --help 与命令行覆盖）。"""
    tyro.cli(run)


if __name__ == "__main__":
    main()
