"""手柄位姿 -> 末端增量 的映射修正（sim/real 共享）。

R_HEADSET_TO_WORLD_PIPER 现为 **det=+1 的正常旋转**（见 config.py，已用干净录制数据
trans_x/y/z、yaw/pitch/roll 离线回放校准，左右/前后/上下/偏航全部正确）。位置与朝向
共用同一矩阵，映射一致、右手系自洽。

本 Mixin 覆写：
  1) _process_xr_pose：位置 (Mr·Ryaw)·xyz、朝向 (Mr·Ryaw)·R_ctrl·(Mr·Ryaw)ᵀ。
     - Mr = R_headset_world 的正常旋转版本；_as_proper_rotation 是安全网：若 R 意外为
       det=-1 的反射矩阵（会翻转旋转手性、令偏航/滚转反向），把翻过的那一行还原成
       正常旋转；正常旋转下它是空操作。
     - Ryaw = 朝向自对齐（见 3）。
  2) _update_ik（朝下基准）：
     激活（按下 grip）瞬间，把姿态基准从“夹爪当前朝向（约前向）”改为“朝下”——
     用最短弧把夹爪接近方向（link6 本体 +Z）旋到世界 -Z，保留朝向（yaw）。
     于是手自然前伸时夹爪即朝下，随时可抓桌面物体；小幅翻手腕再微调接近角。
  3) 朝向自对齐 Ryaw：手柄位姿在“头显世界系”里给出，其水平朝向由头显开机/划定边界
     时决定，与操作者面朝的方向无关。若操作者转身 ~90°，其身体“前/右”就落到不同的
     头显水平轴上，导致整套水平旋转（例如“向右平移”变成机械臂“前后走”）。为此在按下
     grip 的瞬间读头显朝向，算一个绕竖直轴的 Ryaw，把操作者当前正前方对齐到机械臂
     正前方，之后握持期间保持不变。这样站位/朝向都不影响操作。
     若拿不到头显位姿（如离线回放未录该数据），Ryaw=单位阵，退回原映射。

仍是相对增量（激活时锚定基准位姿，只跟随手腕变化量），稳定性与原方案一致。
"""

import meshcat.transformations as tf
import numpy as np

from xrobotoolkit_teleop.utils.geometry import quat_diff_as_angle_axis

# 抓取时夹爪接近方向的目标（世界系，-Z 即竖直朝下）
GRASP_APPROACH_WORLD = np.array([0.0, 0.0, -1.0])


def _shortest_arc_matrix(a: np.ndarray, b: np.ndarray, eps: float = 1e-8) -> np.ndarray:
    """返回把单位向量 a 旋到单位向量 b 的最短弧旋转矩阵（3x3）。"""
    a = a / (np.linalg.norm(a) + eps)
    b = b / (np.linalg.norm(b) + eps)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if c < -1 + eps:
        # 反向：绕任意垂直轴转 180°
        axis = np.cross(a, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < eps:
            axis = np.cross(a, np.array([0.0, 1.0, 0.0]))
        axis /= np.linalg.norm(axis)
        return tf.rotation_matrix(np.pi, axis)[:3, :3]
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def _as_proper_rotation(M: np.ndarray) -> np.ndarray:
    """把可能是反射(det=-1)的正交矩阵还原成正常旋转(det=+1)。

    正常情况下 R_HEADSET_TO_WORLD_PIPER 已是 det=+1，此函数为空操作。仅当矩阵意外为
    反射(det<0)时，翻回第 1 行(世界 Y)符号得到对应的正常旋转，保证朝向共轭不翻手性
    （左右转向不反）。
    """
    M = np.asarray(M, dtype=float)
    if np.linalg.det(M) < 0:
        Mr = M.copy()
        Mr[1, :] = -Mr[1, :]
        return Mr
    return M


class CorrectedPoseMixin:
    """修正手柄朝向映射的 Mixin，需排在 BaseTeleopController 之前（MRO）。"""

    # 是否启用“朝向自对齐”（按 grip 时把操作者正前方对齐到机械臂正前方）
    enable_yaw_align = True

    def _yaw_align_matrix(self, src_name) -> np.ndarray:
        """返回该臂当前生效的朝向自对齐旋转（3x3），未捕获时为单位阵。"""
        return getattr(self, "_yaw_align", {}).get(src_name, np.eye(3))

    def _capture_yaw_align(self, src_name):
        """按下 grip 的瞬间捕获朝向自对齐：把操作者当前正前方旋到机械臂正前方。

        纯绕头显世界系竖直轴的 yaw，握持期间保持不变。拿不到头显位姿则不启用。
        """
        if not self.enable_yaw_align:
            return
        Mr = _as_proper_rotation(np.asarray(self.R_headset_world, dtype=float))
        # 由 Mr 反推：头显系中“映射到机械臂 +X(前)/+Z(上)”的方向
        up_h = Mr.T @ np.array([0.0, 0.0, 1.0])
        fwd_target_h = Mr.T @ np.array([1.0, 0.0, 0.0])
        try:
            hmd = self.xr_client.get_pose_by_name("headset")
            q = np.array([hmd[6], hmd[3], hmd[4], hmd[5]])  # [w,x,y,z]
        except Exception:
            return  # 无头显数据（如离线回放）：不启用，退回原映射
        R_hmd = tf.quaternion_matrix(q)[:3, :3]
        heading = R_hmd @ np.array([0.0, 0.0, -1.0])  # OpenXR 头显前向 = 本体 -Z
        heading = heading - np.dot(heading, up_h) * up_h  # 投影到水平面
        n = np.linalg.norm(heading)
        if n < 1e-4:  # 头显几乎朝正上/下，无法定水平朝向
            return
        heading /= n
        if not hasattr(self, "_yaw_align"):
            self._yaw_align = {}
        self._yaw_align[src_name] = _shortest_arc_matrix(heading, fwd_target_h)

    def _topdown_anchor_quat(self, ee_quat) -> np.ndarray:
        """把末端当前朝向转成“接近方向朝下”的基准朝向（保留 yaw）。

        Args:
            ee_quat: 末端当前朝向四元数 [w, x, y, z]。
        Returns:
            朝下基准朝向四元数 [w, x, y, z]。
        """
        R_ee = tf.quaternion_matrix(np.asarray(ee_quat, dtype=float))[:3, :3]
        approach = R_ee[:, 2]  # link6 本体 +Z = 夹爪接近方向
        R_align = _shortest_arc_matrix(approach, GRASP_APPROACH_WORLD)
        T = np.eye(4)
        T[:3, :3] = R_align @ R_ee
        return tf.quaternion_from_matrix(T)

    def _update_ik(self):
        # 记录本帧前哪些臂还未锚定（用于检测“刚激活”的跳变）
        was_idle = {name: self.ref_ee_xyz[name] is None for name in self.manipulator_config}

        # 在锚定前，为“刚要激活”的臂捕获朝向自对齐（此帧 _process_xr_pose 会用到）
        for name, config in self.manipulator_config.items():
            if was_idle[name] and self.xr_client.get_key_value_by_name(config["control_trigger"]) > 0.9:
                self._capture_yaw_align(name)

        super()._update_ik()

        # 对刚激活的臂：把姿态基准改为“朝下”，并把本帧任务目标重指到朝下基准
        for name, config in self.manipulator_config.items():
            just_engaged = was_idle[name] and self.ref_ee_xyz[name] is not None
            if not just_engaged:
                continue
            if self.effector_control_mode.get(name) == "position":
                continue  # 仅位置控制无姿态基准
            anchor_quat = self._topdown_anchor_quat(self.ref_ee_quat[name])
            self.ref_ee_quat[name] = anchor_quat
            T = tf.quaternion_matrix(anchor_quat)
            T[:3, 3] = self.ref_ee_xyz[name]
            self.effector_task[name].T_world_frame = T

    def _process_xr_pose(self, xr_pose, src_name):
        controller_xyz = np.array([xr_pose[0], xr_pose[1], xr_pose[2]])
        # xr_pose 四元数为 [x, y, z, w]，tf 用 [w, x, y, z]
        controller_quat = np.array([xr_pose[6], xr_pose[3], xr_pose[4], xr_pose[5]])

        M = np.asarray(self.R_headset_world, dtype=float)

        # 位置与朝向共用同一变换 A = Mr · Ryaw：
        #   Mr   = R_headset_world 的正常旋转版本（det=+1，安全网还原可能的反射）；
        #   Ryaw = 朝向自对齐（把操作者正前方对齐到机械臂正前方，与站位无关）。
        Mr = _as_proper_rotation(M)
        A = Mr @ self._yaw_align_matrix(src_name)
        controller_xyz = A @ controller_xyz

        # 朝向：相似变换 A · R_ctrl · Aᵀ 把 headset 系旋转搬到世界系。
        R_ctrl = tf.quaternion_matrix(controller_quat)[:3, :3]
        R_world = A @ R_ctrl @ A.T
        T = np.eye(4)
        T[:3, :3] = R_world
        controller_quat = tf.quaternion_from_matrix(T)

        if self.ref_controller_xyz[src_name] is None:
            self.ref_controller_xyz[src_name] = controller_xyz
            self.ref_controller_quat[src_name] = controller_quat
            delta_xyz = np.zeros(3)
            delta_rot = np.zeros(3)
        else:
            delta_xyz = (controller_xyz - self.ref_controller_xyz[src_name]) * self.scale_factor
            delta_rot = quat_diff_as_angle_axis(self.ref_controller_quat[src_name], controller_quat)

        return delta_xyz, delta_rot
