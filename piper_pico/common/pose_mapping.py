"""手柄位姿 -> 末端增量 的映射修正（sim/real 共享）。

第三方 BaseTeleopController._process_xr_pose 用
    R_quat = quaternion_from_matrix(R_headset_world)
把 headset 系旋转搬到世界系。但 PiPER 用的 R_HEADSET_TO_WORLD_PIPER 为了修左右，
是一个 **行列式 = -1 的反射矩阵**（镜像）。四元数只能表示 det=+1 的正常旋转，
因此 quaternion_from_matrix(反射) 得到的是错乱的朝向，导致“手腕翻转”和夹爪翻转
对应不上（不符合操作习惯）。

本 Mixin 覆写 _process_xr_pose：
  - 位置：仍用 R @ xyz（镜像，保留现有已调好的手感/稳定性）；
  - 朝向：改用矩阵共轭 M · R_ctrl · Mᵀ，det = (-1)(1)(-1) = +1，
    得到与位置镜像一致的合法旋转，手腕翻转会 1:1 正确映射到夹爪翻转。

仍是相对增量（激活时锚定当前末端位姿，只跟随变化量），稳定性与原方案一致。
"""

import meshcat.transformations as tf
import numpy as np

from xrobotoolkit_teleop.utils.geometry import quat_diff_as_angle_axis


class CorrectedPoseMixin:
    """修正手柄朝向映射的 Mixin，需排在 BaseTeleopController 之前（MRO）。"""

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
