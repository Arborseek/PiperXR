"""测试用 mock piper_sdk，模拟 C_PiperInterface 的关节/夹爪收发。"""

import numpy as np


class _JointState:
    def __init__(self):
        self.joint_1 = self.joint_2 = self.joint_3 = 0
        self.joint_4 = self.joint_5 = self.joint_6 = 0


class _JointMsgs:
    def __init__(self):
        self.joint_state = _JointState()


class _GripperState:
    def __init__(self):
        self.gripper_angle = 0


class _GripperMsgs:
    def __init__(self):
        self.gripper_state = _GripperState()


class C_PiperInterface:
    def __init__(self, can_name="can0", can_auto_init=True, **kwargs):
        self.can_name = can_name
        self._joints = _JointMsgs()
        self._gripper = _GripperMsgs()
        self._last_joint = None
        self._last_gripper = None
        self.connected = False
        self.enabled = False

    def ConnectPort(self):
        self.connected = True

    def EnableArm(self, motor_num):
        self.enabled = True

    def ModeCtrl(self, ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=50, is_mit_mode=0x00):
        self._mode = (ctrl_mode, move_mode)

    def GetArmJointMsgs(self):
        return self._joints

    def GetArmGripperMsgs(self):
        return self._gripper

    def JointCtrl(self, j1, j2, j3, j4, j5, j6):
        self._last_joint = np.array([j1, j2, j3, j4, j5, j6], dtype=float)
        # 反馈跟随指令（0.001 度）
        self._joints.joint_state.joint_1 = j1
        self.joint_2 = self._joints.joint_state.joint_2 = j2
        self._joints.joint_state.joint_3 = j3
        self._joints.joint_state.joint_4 = j4
        self._joints.joint_state.joint_5 = j5
        self._joints.joint_state.joint_6 = j6

    def GripperCtrl(self, gripper_angle, gripper_effort, gripper_code, set_zero):
        self._last_gripper = gripper_angle
        self._gripper.gripper_state.gripper_angle = gripper_angle
