"""PICO 4 Ultra -> PiPER 遥操作入口（仿真 / 真机统一）。

- sim: MuJoCo 仿真（单臂 scene.xml / 双臂 scene_dual.xml），用 MujocoTeleopController。
- real: 真机 PiPER（piper_sdk over CAN），用 RealPiperTeleopController。

两者共享 BaseTeleopController 的 IK + 手部→末端映射 + 夹爪逻辑，关节目标数据一致，
便于 sim2real。--log 可把同一份 schema 的数据写到 CSV（sim 与 real 互通）。

运行前：
  - sim: 已 `bash scripts/setup_env.sh`；可选启动 XRoboToolkit-PC-Service + PICO 客户端。
  - real: `pip install piper_sdk python-can`、`bash can_activate.sh can0 1000000`，
          启动 PC 服务 + PICO 客户端，机械臂回到 home。
"""

import os
import time

import tyro

from piper_pico.config import (
    R_HEADSET_TO_WORLD_PIPER,
    build_dual_piper_config,
    build_piper_config,
    build_real_dual_piper_config,
    build_real_piper_config,
)
from piper_pico.paths import (
    PIPER_DUAL_SCENE_XML,
    PIPER_DUAL_URDF,
    PIPER_SCENE_XML,
    PIPER_URDF,
)
from piper_pico.real.piper_arm_proxy import DEFAULT_GRIPPER_CLOSE_UMM, PiperArmProxy


def run(
    backend: str = "sim",
    dual: bool = False,
    xml_path: str = "",
    robot_urdf_path: str = "",
    scale_factor: float = 1.5,
    control_mode: str = "pose",
    hand: str = "right",
    visualize_placo: bool = False,
    can_right: str = "can0",
    can_left: str = "can1",
    gripper_close_mm: float = 50.0,
    log: bool = False,
    log_dir: str = "logs",
    mock: bool = False,
):
    """PiPER 遥操作，仿真/真机统一入口。

    Args:
        backend: "sim" 或 "real"。
        dual: True 双臂，False 单臂。
        xml_path: MuJoCo 场景（仅 sim，留空按 dual 选默认）。
        robot_urdf_path: URDF（供 placo IK，留空按 dual 选默认）。
        scale_factor: 手部位移到末端位移的缩放系数。
        control_mode: "pose" 6 自由度位姿，"position" 仅位置。
        hand: 单臂模式下使用哪只手控制器。
        visualize_placo: 浏览器可视化 placo IK（仅 sim）。
        can_right: 右臂 CAN 通道（仅 real）。
        can_left: 左臂 CAN 通道（仅 real，双臂时使用）。
        gripper_close_mm: 真机夹爪合拢行程 mm（仅 real）。
        log: 是否写统一 schema 的遥操作日志。
        log_dir: 日志目录。
        mock: True 时注入会自己动的假手柄数据（无需 PICO，用于验证 sim+IK 流程）。
    """
    if mock:
        import sys
        from piper_pico.simulation import _mock_xr_moving

        sys.modules["xrobotoolkit_sdk"] = _mock_xr_moving  # 必须在导入控制器前占据

    log_path = ""
    if log:
        os.makedirs(log_dir, exist_ok=True)
        tag = f"{backend}_{'dual' if dual else 'single'}"
        log_path = os.path.join(log_dir, f"teleop_{tag}_{int(time.time())}.csv")

    if backend == "real":
        _run_real(dual, robot_urdf_path, scale_factor, control_mode, hand,
                  can_right, can_left, gripper_close_mm, log_path)
    else:
        _run_sim(dual, xml_path, robot_urdf_path, scale_factor, control_mode,
                 hand, visualize_placo, log_path)


def _run_sim(dual, xml_path, robot_urdf_path, scale_factor, control_mode,
             hand, visualize_placo, log_path):
    from piper_pico.simulation.piper_mujoco_controller import LoggingMujocoTeleopController

    if dual:
        xml_path = xml_path or PIPER_DUAL_SCENE_XML
        robot_urdf_path = robot_urdf_path or PIPER_DUAL_URDF
        config = build_dual_piper_config(control_mode=control_mode)
    else:
        xml_path = xml_path or PIPER_SCENE_XML
        robot_urdf_path = robot_urdf_path or PIPER_URDF
        config = build_piper_config(control_mode=control_mode, hand=hand)

    controller = LoggingMujocoTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        log_path=log_path or None,
        R_headset_world=R_HEADSET_TO_WORLD_PIPER,
    )
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({j: 0.0 for j in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)
    controller.run()


def _run_real(dual, robot_urdf_path, scale_factor, control_mode, hand,
              can_right, can_left, gripper_close_mm, log_path):
    from piper_pico.real.real_piper_teleop_controller import RealPiperTeleopController

    close_umm = int(gripper_close_mm * 1000)
    if dual:
        robot_urdf_path = robot_urdf_path or PIPER_DUAL_URDF
        config = build_real_dual_piper_config(control_mode=control_mode)
        proxies = {
            "right_hand": PiperArmProxy(can_name=can_right, gripper_close_umm=close_umm),
            "left_hand": PiperArmProxy(can_name=can_left, gripper_close_umm=close_umm),
        }
    else:
        robot_urdf_path = robot_urdf_path or PIPER_URDF
        config = build_real_piper_config(control_mode=control_mode, hand=hand)
        key = next(iter(config))
        proxies = {key: PiperArmProxy(can_name=can_right, gripper_close_umm=close_umm)}

    controller = RealPiperTeleopController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        arm_proxies=proxies,
        scale_factor=scale_factor,
        control_mode=control_mode,
        log_path=log_path or None,
        R_headset_world=R_HEADSET_TO_WORLD_PIPER,
    )
    controller.run()


def main():
    """tyro CLI 入口（支持 --help 与命令行覆盖）。"""
    tyro.cli(run)


if __name__ == "__main__":
    main()
