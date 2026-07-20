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

from piper_xr.common.pose_mapping import CorrectedPoseMixin
from piper_xr.common.teleop_logger import TeleopFrame, TeleopLogger
from piper_xr.real.joint_indices import q6_slice_for_link
from piper_xr.real.piper_arm_proxy import PiperArmProxy

# 单帧最大关节增量（rad）。50Hz 下 0.08 rad/帧 ≈ 4 rad/s，抑制 IK 跳变。
DEFAULT_MAX_DQ = 0.08


class RealPiperTeleopController(CorrectedPoseMixin, BaseTeleopController):
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
        max_dq: float = DEFAULT_MAX_DQ,
    ):
        self.arm_proxies = arm_proxies  # {hand_name: PiperArmProxy}
        self.max_dq = float(max_dq)
        self._last_cmd_q: Dict[str, np.ndarray] = {}
        self._q_slices: Dict[str, slice] = {}
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
        for name, config in self.manipulator_config.items():
            self._q_slices[name] = q6_slice_for_link(config["link_name"])

    def _clip_q6_to_limits(self, sl: slice, q6: np.ndarray) -> np.ndarray:
        q6 = np.asarray(q6, dtype=float).copy()
        model = self.placo_robot.model
        for local_i, qi in enumerate(range(sl.start, sl.stop)):
            lo = model.lowerPositionLimit[qi]
            hi = model.upperPositionLimit[qi]
            if np.isfinite(lo):
                q6[local_i] = max(q6[local_i], lo)
            if np.isfinite(hi):
                q6[local_i] = min(q6[local_i], hi)
        return q6

    def _limit_q6_step(self, hand: str, q6: np.ndarray) -> np.ndarray:
        q6 = self._clip_q6_to_limits(self._q_slices[hand], q6)
        prev = self._last_cmd_q.get(hand)
        if prev is None:
            self._last_cmd_q[hand] = q6.copy()
            return q6
        dq = np.clip(q6 - prev, -self.max_dq, self.max_dq)
        q6 = prev + dq
        self._last_cmd_q[hand] = q6.copy()
        return q6

    # ---- 基类抽象实现 ----
    def _robot_setup(self):
        for proxy in self.arm_proxies.values():
            proxy.connect()

    def _update_robot_state(self):
        q = self.placo_robot.state.q
        for name, config in self.manipulator_config.items():
            proxy = self.arm_proxies.get(name)
            if proxy is None:
                continue
            sl = self._q_slices[name]
            q6 = proxy.get_joint_angles_rad()
            q[sl] = q6
            if name not in self._last_cmd_q:
                self._last_cmd_q[name] = q6.copy()
        self.placo_robot.update_kinematics()

    def _get_link_pose(self, link_name):
        T = self.placo_robot.get_T_world_frame(link_name)
        xyz = T[:3, 3].copy()
        quat = tf.quaternion_from_matrix(T)  # [w,x,y,z]
        return xyz, quat

    def _send_command(self):
        q = self.placo_robot.state.q
        for name, config in self.manipulator_config.items():
            proxy = self.arm_proxies.get(name)
            if proxy is None:
                continue
            sl = self._q_slices[name]
            cmd_j6 = self._limit_q6_step(name, np.array(q[sl], dtype=float))
            proxy.send_joint_angles_rad(cmd_j6)
            for joint_name, val in self.gripper_pos_target.get(name, {}).items():
                proxy.send_gripper_normalized(float(val))
            self._log_frame(name, cmd_j6)

    def _log_frame(self, hand: str, cmd_j6: np.ndarray):
        if self.logger is None:
            return
        config = self.manipulator_config[hand]
        proxy = self.arm_proxies.get(hand)
        if proxy is None:
            return
        ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
        gripper = 0.0
        for v in self.gripper_pos_target.get(hand, {}).values():
            gripper = float(v)
        self.logger.log(TeleopFrame(
            backend="real", hand=hand,
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
