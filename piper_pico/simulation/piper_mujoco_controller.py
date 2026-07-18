"""MujocoTeleopController 的 sim 侧日志子类：与真机写同一份 schema，便于 sim2real。"""

from typing import Optional

import numpy as np

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController

from piper_pico.common.teleop_logger import TeleopFrame, TeleopLogger


class LoggingMujocoTeleopController(MujocoTeleopController):
    def __init__(self, *args, log_path: Optional[str] = None, R_headset_world=None, **kwargs):
        self._logger_path = log_path
        self._logger: Optional[TeleopLogger] = None
        if R_headset_world is not None:
            kwargs["R_headset_world"] = R_headset_world
        super().__init__(*args, **kwargs)
        if self._logger_path:
            self._logger = TeleopLogger(self._logger_path, "sim")

    def _send_command(self):
        super()._send_command()
        if self._logger is None:
            return
        import mujoco
        cmd_j6 = np.array(self.placo_robot.state.q[7:13], dtype=float)
        for name, config in self.manipulator_config.items():
            link = config["link_name"]
            prefix = "right_" if link.startswith("right_") else ("left_" if link.startswith("left_") else "")
            ee_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, link)
            ee_xyz = self.mj_data.xpos[ee_id].copy()
            ee_quat = self.mj_data.xquat[ee_id].copy()  # [w,x,y,z]
            q6 = np.array([
                self.mj_data.qpos[mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}joint{i}")]
                for i in range(1, 7)
            ], dtype=float)
            gripper = 0.0
            for v in self.gripper_pos_target.get(name, {}).values():
                gripper = float(v) / 0.035  # 归一化到 [0..1]
            self._logger.log(TeleopFrame(
                backend="sim", hand=name, joint_pos=q6, gripper=gripper,
                ee_xyz=ee_xyz, ee_quat=ee_quat, cmd_joint=cmd_j6, cmd_gripper=gripper,
            ))
