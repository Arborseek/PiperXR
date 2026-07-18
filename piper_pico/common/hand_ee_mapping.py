"""手柄坐标系 -> 末端坐标系的姿态映射。

提供两种朝向映射模式（位置始终用 delta 相对映射）：

- "delta"（相对）：target = ref_ee @ R_hand_to_ee @ D @ R_hand_to_ee^-1
  末端从激活位姿出发，按控制器旋转 delta 转动。激活时无跳变，但做"向下抓取"这类
  需要末端持续朝下的动作时，手腕要一直保持倾斜，累且易漂移。

- "absolute"（绝对，PiPER 默认）：target = R_hand_to_ee @ R_headset_world @ R_controller
  末端 1:1 镜像手腕朝向。手腕下翻 -> 末端下指 -> 直接抓桌面物体，无需保持倾斜。
  激活瞬间末端会 snap 到手腕朝向（操作者本就握成期望姿态，snap 即期望）。
  R_hand_to_ee 用于把手柄轴对齐到末端(link6)轴：默认 I，若静止时夹爪朝向不对，
  改成绕某轴的旋转微调即可。

位置（大臂）仍走基类 delta：target_xyz = ref_ee_xyz + scale * delta_xyz，不变。
"""

import numpy as np
import meshcat.transformations as tf

from xrobotoolkit_teleop.utils.geometry import apply_delta_pose


def _axis_angle_to_quat(delta_rot: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(delta_rot))
    if angle < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return tf.quaternion_about_axis(angle, delta_rot / angle)


def _quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    return tf.quaternion_matrix(q)[:3, :3]


def _rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = R
    return tf.quaternion_from_matrix(M)


def _controller_world_rotmat(xr_pose, R_headset_world: np.ndarray) -> np.ndarray:
    """从 xr_pose 取控制器姿态，经 R_headset_world 转到世界系，返回 3x3。"""
    q_local = np.array([xr_pose[6], xr_pose[3], xr_pose[4], xr_pose[5]], dtype=float)
    R_local = _quat_to_rotmat(q_local)
    return np.asarray(R_headset_world) @ R_local


def compute_ee_target_rotation_delta(
    ref_ee_quat: np.ndarray,
    delta_rot_world: np.ndarray,
    R_hand_to_ee: np.ndarray,
) -> np.ndarray:
    ref_R = _quat_to_rotmat(ref_ee_quat)
    D_R = _quat_to_rotmat(_axis_angle_to_quat(delta_rot_world))
    target_R = ref_R @ R_hand_to_ee @ D_R @ R_hand_to_ee.T
    return _rotmat_to_quat(target_R)


def compute_ee_target_rotation_absolute(
    xr_pose,
    R_headset_world: np.ndarray,
    R_hand_to_ee: np.ndarray,
) -> np.ndarray:
    R_ctrl_world = _controller_world_rotmat(xr_pose, R_headset_world)
    target_R = R_hand_to_ee @ R_ctrl_world
    return _rotmat_to_quat(target_R)


class PiperHandEEMixin:
    """为 PiPER 控制器提供"小臂对齐"的 _update_ik 覆盖。

    需在 __init__ 里设置：
      self._hand_ee_map = {src_name: 3x3}      # 来自 config 的 R_hand_to_ee
      self._orient_mode = {src_name: "absolute"|"delta"}
      self._R_headset_world  # 3x3
    """

    _hand_ee_map: dict
    _orient_mode: dict
    _R_headset_world: np.ndarray

    # 末端目标低通平滑系数（0~1，越小越平滑/越滞后）。滤掉 PICO 控制器姿态噪声，
    # 避免绝对映射下手不动末端也抖。0.5 兼顾响应与稳定。
    _smooth_alpha: float = 0.5
    _smooth_deadzone: float = 1e-3  # 位置/角度变化小于此值视为噪声，保持上一帧目标

    def _store_prev(self, src_name: str, xyz, quat) -> None:
        if not hasattr(self, "_prev_target"):
            self._prev_target = {}
        prev = self._prev_target.get(src_name)
        prev_q = quat if quat is not None else (prev[1] if prev is not None else None)
        self._prev_target[src_name] = (np.asarray(xyz).copy(), prev_q)

    def _smooth_xyz(self, src_name: str, target_xyz: np.ndarray) -> np.ndarray:
        prev = getattr(self, "_prev_target", {}).get(src_name)
        if prev is None:
            return np.asarray(target_xyz).copy()
        prev_xyz = prev[0]
        diff = np.linalg.norm(target_xyz - prev_xyz)
        if diff < self._smooth_deadzone:
            return prev_xyz.copy()
        return prev_xyz + self._smooth_alpha * (target_xyz - prev_xyz)

    def _smooth_quat(self, src_name: str, target_quat: np.ndarray) -> np.ndarray:
        prev = getattr(self, "_prev_target", {}).get(src_name)
        if prev is None:
            return target_quat.copy()
        prev_q = prev[1]
        # 四元数双覆盖：取较短弧
        q = target_quat
        if np.dot(prev_q, q) < 0:
            q = -q
        ang = np.arccos(np.clip(abs(np.dot(prev_q, q)), -1.0, 1.0))
        if ang < self._smooth_deadzone:
            return prev_q.copy()
        slerped = tf.quaternion_slerp(prev_q, q, self._smooth_alpha)
        return slerped

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
                R_hand_ee = self._hand_ee_map.get(src_name, np.eye(3))
                mode = self._orient_mode.get(src_name, "absolute")

                if self.effector_task and src_name in self.effector_task and self.effector_control_mode[src_name] == "position":
                    target_xyz = self._smooth_xyz(src_name, self.ref_ee_xyz[src_name] + delta_xyz)
                    self._store_prev(src_name, target_xyz, None)
                    self.effector_task[src_name].target_world = target_xyz
                else:
                    target_xyz, _ = apply_delta_pose(
                        self.ref_ee_xyz[src_name], self.ref_ee_quat[src_name], delta_xyz, np.zeros(3)
                    )
                    if mode == "absolute":
                        target_quat = compute_ee_target_rotation_absolute(
                            xr_pose, self._R_headset_world, R_hand_ee
                        )
                    else:
                        target_quat = compute_ee_target_rotation_delta(
                            self.ref_ee_quat[src_name], delta_rot, R_hand_ee
                        )
                    target_xyz = self._smooth_xyz(src_name, target_xyz)
                    target_quat = self._smooth_quat(src_name, target_quat)
                    self._store_prev(src_name, target_xyz, target_quat)
                    target_pose = tf.quaternion_matrix(target_quat)
                    target_pose[:3, 3] = target_xyz
                    self.effector_task[src_name].T_world_frame = target_pose
            else:
                if self.ref_ee_xyz[src_name] is not None:
                    print(f"{src_name} is deactivated.")
                    self.ref_ee_xyz[src_name] = None
                    self.ref_controller_xyz[src_name] = None
                    if hasattr(self, "_prev_target"):
                        self._prev_target.pop(src_name, None)

        self._update_motion_tracker_tasks()

        try:
            self.solver.solve(True)
        except RuntimeError as e:
            print(f"IK solver failed: {e}")
