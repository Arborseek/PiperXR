"""无头流水线验证：在不启动 viewer / 不连接头显的情况下验证 PiPER 适配是否正确。

`validate_pipeline` 构造 MujocoTeleopController 并跑若干步 IK + 仿真，
返回最终状态供测试断言。需在调用前注入 mock 版 `xrobotoolkit_sdk`
（见 tests/conftest.py 或 tests/validate_piper_pipeline.py）。
"""

from dataclasses import dataclass

import mujoco
import numpy as np

from piper_xr.config import build_dual_piper_config, build_piper_config
from piper_xr.paths import (
    PIPER_DUAL_SCENE_XML,
    PIPER_DUAL_URDF,
    PIPER_SCENE_XML,
    PIPER_URDF,
)
from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)


@dataclass
class ValidationResult:
    qpos: np.ndarray
    ctrl: np.ndarray
    ee_xyz: np.ndarray
    controller: MujocoTeleopController


def validate_pipeline(
    steps: int = 50,
    dual: bool = False,
    control_mode: str = "pose",
) -> ValidationResult:
    """构造控制器并运行 `steps` 步 IK + 仿真，返回最终状态。"""
    if dual:
        xml_path = PIPER_DUAL_SCENE_XML
        robot_urdf_path = PIPER_DUAL_URDF
        config = build_dual_piper_config(control_mode=control_mode)
        ee_name = "right_link6"
    else:
        xml_path = PIPER_SCENE_XML
        robot_urdf_path = PIPER_URDF
        config = build_piper_config(control_mode=control_mode, hand="right")
        ee_name = "link6"

    controller = MujocoTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=1.5,
        visualize_placo=False,
    )

    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({j: 0.0 for j in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    for _ in range(steps):
        controller._update_robot_state()
        controller._update_ik()
        controller._update_gripper_target()
        controller._update_mocap_target()
        controller._send_command()
        mujoco.mj_step(controller.mj_model, controller.mj_data)

    ee_id = mujoco.mj_name2id(controller.mj_model, mujoco.mjtObj.mjOBJ_BODY, ee_name)
    return ValidationResult(
        qpos=controller.mj_data.qpos.copy(),
        ctrl=controller.mj_data.ctrl.copy(),
        ee_xyz=controller.mj_data.xpos[ee_id].copy(),
        controller=controller,
    )
