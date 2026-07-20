"""真机 RealPiperTeleopController 的无头测试（mock piper_sdk + mock xrobotoolkit_sdk）。

验证：关节 rad <-> 0.001度 换算、夹爪归一化 <-> 0.001mm 换算、与仿真同源的 IK 流程。
"""

import numpy as np

from piper_xr.config import build_real_dual_piper_config, build_real_piper_config
from piper_xr.paths import PIPER_DUAL_URDF, PIPER_URDF
from piper_xr.real.joint_indices import q6_slice_for_link
from piper_xr.real.piper_arm_proxy import PiperArmProxy
from piper_xr.real.real_piper_teleop_controller import RealPiperTeleopController

HOME_Q6 = np.array([0.0, 1.57, -1.3485, 0.0, 0.0, 0.0])
HOME_Q8 = np.array([0.0, 1.57, -1.3485, 0.0, 0.0, 0.0, 0.0, 0.0])


def _make_controller():
    config = build_real_piper_config(control_mode="pose", hand="right")
    hand = next(iter(config))
    proxy = PiperArmProxy(can_name="can0", gripper_close_umm=50_000)
    ctrl = RealPiperTeleopController(
        robot_urdf_path=PIPER_URDF,
        manipulator_config=config,
        arm_proxies={hand: proxy},
        scale_factor=1.5,
        control_mode="pose",
        q_init=HOME_Q8,
    )
    return ctrl, proxy, hand


def test_real_controller_connects_and_ticks():
    ctrl, proxy, hand = _make_controller()
    assert proxy._piper.connected
    assert proxy._piper.enabled

    for _ in range(30):
        ctrl._update_robot_state()
        ctrl._update_ik()
        ctrl._update_gripper_target()
        ctrl._send_command()

    # 未按 grip（mock trigger=0）→ 不激活，IK 在 frame task+正则化下收敛到 home 附近
    last = proxy._piper._last_joint
    assert last is not None
    expected_001deg = np.rad2deg(HOME_Q6) * 1000.0
    # 容差 0.03 rad ≈ 1700（0.001度），IK 受可操作度任务轻微扰动
    assert np.allclose(last, expected_001deg, atol=2000.0), f"{last} vs {expected_001deg}"


def test_real_gripper_normalized_mapping():
    ctrl, proxy, hand = _make_controller()
    # 直接测代理换算：归一化 1.0 -> 50000 (0.001mm)
    proxy.send_gripper_normalized(1.0)
    assert proxy._piper._last_gripper == 50_000
    proxy.send_gripper_normalized(0.0)
    assert proxy._piper._last_gripper == 0
    # 反馈归一化
    proxy._piper._gripper.gripper_state.gripper_angle = 25_000
    assert abs(proxy.get_gripper_normalized() - 0.5) < 1e-6


def test_real_joint_roundtrip_units():
    ctrl, proxy, hand = _make_controller()
    q = np.array([0.0, 0.5, -0.3, 0.1, -0.2, 0.05])
    proxy.send_joint_angles_rad(q)
    expected = np.rad2deg(q) * 1000.0
    assert np.allclose(proxy._piper._last_joint, expected, atol=1.0)
    # 反馈读回应一致（rad）
    back = proxy.get_joint_angles_rad()
    assert np.allclose(back, q, atol=1e-3)


def test_q6_slice_for_dual_arm():
    assert q6_slice_for_link("link6") == slice(7, 13)
    assert q6_slice_for_link("left_link6") == slice(7, 13)
    assert q6_slice_for_link("right_link6") == slice(15, 21)


def test_dual_real_controller_sends_distinct_joint_cmds():
    config = build_real_dual_piper_config(control_mode="pose")
    right = PiperArmProxy(can_name="can0", gripper_close_umm=50_000)
    left = PiperArmProxy(can_name="can1", gripper_close_umm=50_000)
    ctrl = RealPiperTeleopController(
        robot_urdf_path=PIPER_DUAL_URDF,
        manipulator_config=config,
        arm_proxies={"right_hand": right, "left_hand": left},
        scale_factor=1.5,
        max_dq=10.0,  # 测试时不限步长
    )
    # 人为设置 placo 解：两臂关节角不同（均在 URDF 限位内，joint3≤0）
    ctrl.placo_robot.state.q[15:21] = np.array([0.1, 0.2, -0.3, 0.4, 0.5, 0.6])
    ctrl.placo_robot.state.q[7:13] = np.array([-0.1, 0.5, -0.2, -0.4, -0.5, -0.6])
    ctrl._last_cmd_q["right_hand"] = ctrl.placo_robot.state.q[15:21].copy()
    ctrl._last_cmd_q["left_hand"] = ctrl.placo_robot.state.q[7:13].copy()
    ctrl._send_command()
    assert not np.allclose(right._piper._last_joint, left._piper._last_joint)
    assert np.allclose(right._piper._last_joint, np.rad2deg([0.1, 0.2, -0.3, 0.4, 0.5, 0.6]) * 1000.0, atol=1.0)
    assert np.allclose(left._piper._last_joint, np.rad2deg([-0.1, 0.5, -0.2, -0.4, -0.5, -0.6]) * 1000.0, atol=1.0)


def test_max_dq_limits_joint_jump():
    ctrl, proxy, hand = _make_controller()
    hand = next(iter(ctrl.manipulator_config))
    ctrl._last_cmd_q[hand] = np.zeros(6)
    target = np.array([1.0, 0.5, -0.5, 0.0, 0.0, 0.0])
    cmd = ctrl._limit_q6_step(hand, target)
    expected = np.array([0.08, 0.08, -0.08, 0.0, 0.0, 0.0])
    assert np.allclose(cmd, expected, atol=1e-9)
