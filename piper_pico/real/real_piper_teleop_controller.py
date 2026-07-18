"""真机 PiPER 遥操作控制器。

继承 BaseTeleopController，复用其 IK + 手部→末端映射 + 夹爪逻辑（与仿真同源），
仅实现真机侧 I/O：从 piper_sdk 读关节、写关节/夹爪。仿真与真机走同一份基类逻辑，
关节目标数据完全一致，便于 sim2real。

单臂：一个 PiperArmProxy(can0)；双臂：right=can0、left=can1（可配置）。
"""

import time
from typing import Dict, Optional

import meshcat.transformations as tf
import numpy as np

from xrobotoolkit_teleop.common.base_teleop_controller import BaseTeleopController
from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD

from piper_pico.common.hand_ee_mapping import PiperHandEEMixin
from piper_pico.common.teleop_logger import TeleopFrame, TeleopLogger
from piper_pico.real.piper_arm_proxy import PiperArmProxy


class RealPiperTeleopController(PiperHandEEMixin, BaseTeleopController):
    def __init__(
        self,
        robot_urdf_path: str,
        manipulator_config: Dict,
        arm_proxies: Dict[str, PiperArmProxy],
        scale_factor: float = 1.5,
        control_mode: str = "pose",
        dt: float = 0.02,
        q_init: Optional[np.ndarray] = None,
        log_path: Optional[str] = None,
        R_headset_world: Optional[np.ndarray] = None,
    ):
        self.arm_proxies = arm_proxies  # {hand_name: PiperArmProxy}
        self.logger = TeleopLogger(log_path, "real") if log_path else None
        super().__init__(
            robot_urdf_path=robot_urdf_path,
            manipulator_config=manipulator_config,
            floating_base=False,
            R_headset_world=R_headset_world if R_headset_world is not None else R_HEADSET_TO_WORLD,
            scale_factor=scale_factor,
            q_init=q_init,
            dt=dt,
        )
        self._hand_ee_map = {
            name: np.array(cfg.get("R_hand_to_ee", np.eye(3)), dtype=float)
            for name, cfg in self.manipulator_config.items()
        }
        self._orient_mode = {
            name: cfg.get("orientation_mode", "absolute")
            for name, cfg in self.manipulator_config.items()
        }
        self._R_headset_world = np.array(self.R_headset_world, dtype=float)

    # ---- 基类抽象实现 ----
    def _robot_setup(self):
        for proxy in self.arm_proxies.values():
            proxy.connect()

    def _update_robot_state(self):
        for name, config in self.manipulator_config.items():
            proxy = self.arm_proxies.get(name)
            if proxy is None:
                continue
            q6 = proxy.get_joint_angles_rad()
            self.placo_robot.state.q[7:13] = q6
            self.placo_robot.update_kinematics()

    def _get_link_pose(self, link_name):
        T = self.placo_robot.get_T_world_frame(link_name)
        xyz = T[:3, 3].copy()
        quat = tf.quaternion_from_matrix(T)  # [w,x,y,z]
        return xyz, quat

    def _send_command(self):
        q = self.placo_robot.state.q
        cmd_j6 = np.array(q[7:13], dtype=float)  # joint1-6 rad（IK 解）
        for name, config in self.manipulator_config.items():
            proxy = self.arm_proxies.get(name)
            if proxy is None:
                continue
            proxy.send_joint_angles_rad(cmd_j6)
            # 夹爪目标（real 配置 close_pos=1.0 归一化）
            for joint_name, val in self.gripper_pos_target.get(name, {}).items():
                proxy.send_gripper_normalized(float(val))
        self._log_frame(cmd_j6)

    def _log_frame(self, cmd_j6):
        if self.logger is None:
            return
        for name, config in self.manipulator_config.items():
            proxy = self.arm_proxies.get(name)
            if proxy is None:
                continue
            ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
            gripper = 0.0
            for v in self.gripper_pos_target.get(name, {}).values():
                gripper = float(v)
            self.logger.log(TeleopFrame(
                backend="real", hand=name,
                joint_pos=proxy.get_joint_angles_rad(),
                gripper=proxy.get_gripper_normalized(),
                ee_xyz=ee_xyz, ee_quat=ee_quat,
                cmd_joint=cmd_j6, cmd_gripper=gripper,
            ))

    def run(self):
        print("Real PiPER teleop running. Ctrl+C to stop.")
        try:
            while not self._stop_event.is_set():
                self._update_robot_state()
                self._update_ik()
                self._update_gripper_target()
                self._send_command()
                time.sleep(self.dt)
        except KeyboardInterrupt:
            print("\nReal teleop stopped.")
            self._stop_event.set()
        finally:
            if self.logger:
                self.logger.close()
