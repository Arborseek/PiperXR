"""无头验证脚本（不依赖 pytest，可直接运行）：

    conda activate pico_teleop
    python tests/validate_piper_pipeline.py

注入 mock 版 xrobotoolkit_sdk 后，构造 PiPER 控制器并跑 50 步 IK + 仿真，
断言关节保持在 home 位姿附近、状态无 NaN。
"""

import os
import sys

# 注入 mock SDK，使脚本在无 PC 服务 / 无头显时也能验证流水线
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _mock_xrobotoolkit_sdk  # noqa: F401,E402
sys.modules["xrobotoolkit_sdk"] = _mock_xrobotoolkit_sdk  # noqa: E402

import numpy as np  # noqa: E402

from piper_xr.simulation.validate import validate_pipeline  # noqa: E402


def main():
    print("== Constructing MujocoTeleopController for PiPER (mock SDK) ==")
    result = validate_pipeline(steps=50)
    print("final qpos:", np.round(result.qpos, 4))
    print("final ctrl:", np.round(result.ctrl, 4))
    print("link6 world pos:", np.round(result.ee_xyz, 4))

    assert np.all(np.isfinite(result.qpos)), "qpos contains non-finite values"
    assert np.all(np.isfinite(result.ctrl)), "ctrl contains non-finite values"
    assert np.allclose(result.qpos[:6], [0, 1.57, -1.3485, 0, 0, 0], atol=1e-1), (
        f"arm drifted from home pose: {result.qpos[:6]}"
    )
    print("\n[OK] PiPER teleop pipeline validated headlessly.")


if __name__ == "__main__":
    main()
