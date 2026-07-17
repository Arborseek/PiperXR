"""pytest 全局夹具：在导入 xrobotoolkit_teleop 之前注入 mock 版 xrobotoolkit_sdk，
使测试在无 PC 服务 / 无头显的环境下也能运行。
"""

import os
import sys

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _TESTS_DIR)

import _mock_xrobotoolkit_sdk  # noqa: F401,E402
import _mock_piper_sdk  # noqa: F401,E402

# 必须在 xrobotoolkit_teleop.common.xr_client 被导入前占据 xrobotoolkit_sdk 名字
sys.modules.setdefault("xrobotoolkit_sdk", _mock_xrobotoolkit_sdk)
# 真机后端延迟导入 piper_sdk，这里提前注入 mock，便于无 CAN 环境测试
sys.modules.setdefault("piper_sdk", _mock_piper_sdk)
