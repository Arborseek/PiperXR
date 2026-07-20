"""真机 PiPER 遥操作入口（兼容性脚本）。

等价于：
    python -m piper_xr --backend real
    python -m piper_xr --backend real --dual
"""

from piper_xr.simulation.teleop import main

if __name__ == "__main__":
    main()
