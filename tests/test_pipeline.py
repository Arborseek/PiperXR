"""PiPER 遥操作流水线的 pytest 用例（无头，使用 mock SDK）。"""

import numpy as np

from piper_xr.simulation.validate import validate_pipeline


def test_pipeline_runs_without_error():
    result = validate_pipeline(steps=50)
    assert np.all(np.isfinite(result.qpos))
    assert np.all(np.isfinite(result.ctrl))


def test_arm_stays_near_home_pose():
    result = validate_pipeline(steps=50)
    assert np.allclose(
        result.qpos[:6], [0, 1.57, -1.3485, 0, 0, 0], atol=1e-1
    ), f"arm drifted from home pose: {result.qpos[:6]}"


def test_end_effector_pose_reasonable():
    result = validate_pipeline(steps=50)
    # link6 在 home 位姿下应位于基座前上方约 (0.36, 0, 0.38)
    assert result.ee_xyz.shape == (3,)
    assert np.all(np.isfinite(result.ee_xyz))
    assert 0.2 < result.ee_xyz[0] < 0.5
    assert 0.2 < result.ee_xyz[2] < 0.5


def test_dual_pipeline_runs_without_error():
    result = validate_pipeline(steps=50, dual=True)
    assert np.all(np.isfinite(result.qpos))
    assert np.all(np.isfinite(result.ctrl))


def test_dual_arms_stay_near_home():
    result = validate_pipeline(steps=50, dual=True)
    # 双臂 qpos 前 8 为右臂、后 8 为左臂（MJCF 顺序）
    home = np.array([0, 1.57, -1.3485, 0, 0, 0, 0, 0])
    assert np.allclose(result.qpos[:8], home, atol=1e-1), f"right arm: {result.qpos[:8]}"
    assert np.allclose(result.qpos[8:16], home, atol=1e-1), f"left arm: {result.qpos[8:16]}"
