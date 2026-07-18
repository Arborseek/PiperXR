"""手柄坐标系 -> 末端坐标系的旋转对齐 mixin。

基类 BaseTeleopController 的 _update_ik 用 apply_delta_pose 把控制器旋转 delta 在
**世界系**左乘到末端（target = D_world @ ref_ee），导致末端绕世界轴转，而不是绕自己的
小臂轴转——人手做腕部旋前/旋后时，末端的小臂朝向跟人手对不上。

本 mixin 改用带固定对齐旋转 R_hand_to_ee 的映射：
    target_R = ref_ee_R @ R_hand_to_ee @ D_world @ R_hand_to_ee^-1
即先把控制器的世界系 delta 通过 R_hand_to_ee 重表达到末端对齐的坐标系，再局部应用到末端。
R_hand_to_ee = I 时退化为“末端局部系应用 delta”（末端绕自己当前轴转），已比世界系左乘自然；
若仍有轴向偏置，可在 config 里给每只手设 R_hand_to_ee 微调。

位置（大臂）仍走基类的 delta 逻辑，不变——只改旋转（小臂）映射。
"""

import numpy as np
import meshcat.transformations as tf

from xrobotoolkit_teleop.utils.geometry import apply_delta_pose


def _axis_angle_to_quat(delta_rot: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(delta_rot))
    if angle < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = delta_rot / angle
    return tf.quaternion_about_axis(angle, axis)


def _quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    return tf.quaternion_matrix(q)[:3, :3]


def _rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = R
    return tf.quaternion_from_matrix(M)


def compute_ee_target_rotation(
    ref_ee_quat: np.ndarray,
    delta_rot_world: np.ndarray,
    R_hand_to_ee: np.ndarray,
) -> np.ndarray:
    """返回末端目标姿态四元数 [w,x,y,z]。

    Args:
        ref_ee_quat: 该末端激活时记录的参考姿态 [w,x,y,z]。
        delta_rot_world: 控制器世界系旋转 delta（angle-axis 3 向量，来自基类 _process_xr_pose）。
        R_hand_to_ee: 固定 3x3 旋转，把手柄坐标系对齐到末端坐标系。
    """
    ref_R = _quat_to_rotmat(ref_ee_quat)
    D_q = _axis_angle_to_quat(delta_rot_world)
    D_R = _quat_to_rotmat(D_q)
    target_R = ref_R @ R_hand_to_ee @ D_R @ R_hand_to_ee.T
    return _rotmat_to_quat(target_R)


class PiperHandEEMixin:
    """为 PiPER 控制器提供“小臂对齐”的 _update_ik 覆盖。

    需要在 __init__ 里设置 self._hand_ee_map = {src_name: 3x3 旋转}（来自 config 的 R_hand_to_ee）。
    """

    _hand_ee_map: dict

    def _update_ik(self):
        self._update_robot_state()
        self.placo_robot.update_kinematics()

        for src_name, config in self.manipulator_config.items():
            xr_grip_val = self.xr_client.get_key_value_by_name(config["control_trigger"])
            self.active[src_name] = xr_grip_val > 0.9

            if self.active[src_name]:
                if self.ref_ee_xyz[src_name] is None:
                    print(f"{src_name} is activated.")
                    self.ref_ee_xyz[src_name], self.ref_ee_quat[src_name] = self._get_link_pose(
                        config["link_name"]
                    )

                xr_pose = self.xr_client.get_pose_by_name(config["pose_source"])
                delta_xyz, delta_rot = self._process_xr_pose(xr_pose, src_name)

                if self.effector_control_mode[src_name] == "position":
                    target_xyz = self.ref_ee_xyz[src_name] + delta_xyz
                    self.effector_task[src_name].target_world = target_xyz
                else:
                    target_xyz, _ = apply_delta_pose(
                        self.ref_ee_xyz[src_name],
                        self.ref_ee_quat[src_name],
                        delta_xyz,
                        np.zeros(3),  # 旋转由下方单独处理
                    )
                    R_hand_ee = self._hand_ee_map.get(src_name, np.eye(3))
                    target_quat = compute_ee_target_rotation(
                        self.ref_ee_quat[src_name], delta_rot, R_hand_ee
                    )
                    target_pose = tf.quaternion_matrix(target_quat)
                    target_pose[:3, 3] = target_xyz
                    self.effector_task[src_name].T_world_frame = target_pose
            else:
                if self.ref_ee_xyz[src_name] is not None:
                    print(f"{src_name} is deactivated.")
                    self.ref_ee_xyz[src_name] = None
                    self.ref_controller_quat[src_name] = None

        self._update_motion_tracker_tasks()

        try:
            self.solver.solve(True)
        except RuntimeError as e:
            print(f"IK solver failed: {e}")
