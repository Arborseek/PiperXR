"""兼容性入口：保留 `python scripts/simulation/teleop_piper_mujoco.py` 的运行方式。

实际逻辑位于 `piper_xr.simulation.teleop`。也可用：
    python -m piper_xr
    piper-xr   （pip install -e . 后安装的控制台命令）
"""

from piper_xr.simulation.teleop import main

if __name__ == "__main__":
    main()
