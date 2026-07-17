"""对 piper_sdk C_PiperInterface 的薄封装，统一单位为「弧度 / 归一化夹爪」。

- 关节：piper_sdk 用 0.001 度的 int；这里对外用弧度 np.ndarray(6)。
- 夹爪：piper_sdk 用 0.001 mm 的 int；这里对外用归一化 [0..1]（0=张、1=合），
  与仿真侧 joint7 的归一化一致，便于 sim2real 同数据。

真机运行需 `pip install piper_sdk python-can` 并已 `bash can_activate.sh can0 1000000`。
"""

import time
from typing import Optional

import numpy as np

# 默认夹爪行程（0.001mm）：约 50mm。不同末端执行器可改 --gripper-close-mm。
DEFAULT_GRIPPER_CLOSE_UMM = 50_000


class PiperArmProxy:
    """单臂 CAN 代理。双臂时各建一个实例（不同 can 通道）。"""

    def __init__(
        self,
        can_name: str = "can0",
        can_auto_init: bool = True,
        move_spd_rate: int = 50,
        gripper_close_umm: int = DEFAULT_GRIPPER_CLOSE_UMM,
        piper: Optional[object] = None,
    ):
        self.can_name = can_name
        self.move_spd_rate = move_spd_rate
        self.gripper_close_umm = int(gripper_close_umm)
        self._piper = piper  # 可注入（测试用）；为 None 时延迟到 connect() 创建

    def connect(self):
        """连接 CAN 口并使能机械臂。"""
        if self._piper is None:
            from piper_sdk import C_PiperInterface

            self._piper = C_PiperInterface(can_name=self.can_name, can_auto_init=True)
        self._piper.ConnectPort()
        self._piper.EnableArm(7)
        # 进入「CAN 指令 / 关节」模式；每次运动前需发送一次。
        self._set_joint_mode()
        time.sleep(0.3)

    def _set_joint_mode(self):
        try:
            self._piper.ModeCtrl(ctrl_mode=0x01, move_mode=0x01,
                                 move_spd_rate_ctrl=self.move_spd_rate, is_mit_mode=0x00)
        except AttributeError:
            # 旧版方法名兜底
            self._piper.MotionCtrl_2(0x01, 0x01, self.move_spd_rate, 0x00)

    # ---- 读状态 ----
    def get_joint_angles_rad(self) -> np.ndarray:
        js = self._piper.GetArmJointMsgs().joint_state
        deg = np.array([js.joint_1, js.joint_2, js.joint_3,
                        js.joint_4, js.joint_5, js.joint_6], dtype=float) / 1000.0
        return np.deg2rad(deg)

    def get_gripper_normalized(self) -> float:
        g = self._piper.GetArmGripperMsgs().gripper_state
        umm = float(getattr(g, "gripper_angle", 0) or 0)
        return float(np.clip(umm / self.gripper_close_umm, 0.0, 1.0))

    # ---- 写指令 ----
    def send_joint_angles_rad(self, q: np.ndarray):
        v = (np.rad2deg(np.asarray(q, dtype=float)) * 1000.0).astype(int)
        self._piper.JointCtrl(int(v[0]), int(v[1]), int(v[2]),
                               int(v[3]), int(v[4]), int(v[5]))

    def send_gripper_normalized(self, norm: float):
        umm = int(np.clip(norm, 0.0, 1.0) * self.gripper_close_umm)
        self._piper.GripperCtrl(umm, 0, 0, 0)

    def step(self):
        """真机由 SDK 内部线程收发，这里仅限速。"""
        time.sleep(0.0)
