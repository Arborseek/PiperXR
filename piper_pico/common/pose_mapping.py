"""手柄位姿 -> 末端增量 的映射修正（sim/real 共享）。

第三方 BaseTeleopController._process_xr_pose 用
    R_quat = quaternion_from_matrix(R_headset_world)
把 headset 系旋转搬到世界系。但 PiPER 用的 R_HEADSET_TO_WORLD_PIPER 为了修左右，
是一个 **行列式 = -1 的反射矩阵**（镜像）。四元数只能表示 det=+1 的正常旋转，
因此 quaternion_from_matrix(反射) 得到的是错乱的朝向，导致“手腕翻转”和夹爪翻转
对应不上（不符合操作习惯）。

本 Mixin 覆写：
  1) _process_xr_pose（朝向轴向修正）：
     - 位置：仍用 R @ xyz（镜像，保留现有已调好的手感/稳定性）；
     - 朝向：改用矩阵共轭 M · R_ctrl · Mᵀ，det = (-1)(1)(-1) = +1，
       得到与位置镜像一致的合法旋转，手腕翻转会 1:1 正确映射到夹爪翻转。
  2) _update_ik（朝下基准）：
     激活（按下 grip）瞬间，把姿态基准从“夹爪当前朝向（约前向）”改为“朝下”——
     用最短弧把夹爪接近方向（link6 本体 +Z）旋到世界 -Z，保留朝向（yaw）。
     于是手自然前伸时夹爪即朝下，随时可抓桌面物体；小幅翻手腕再微调接近角。

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


class CorrectedPoseMixin:
    """修正手柄朝向映射的 Mixin，需排在 BaseTeleopController 之前（MRO）。"""

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

        # 位置：直接用 M 变换（M 为反射时即左右镜像，沿用现有手感）
        controller_xyz = M @ controller_xyz

        # 朝向：在矩阵空间做相似变换 M · R_ctrl · Mᵀ。
        # M 正交，Mᵀ = M⁻¹；即便 det(M) = -1，结果 det = +1 仍是合法旋转，
        # 且与位置镜像自洽，避免 quaternion_from_matrix(反射) 的错乱。
        R_ctrl = tf.quaternion_matrix(controller_quat)[:3, :3]
        R_world = M @ R_ctrl @ M.T
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
