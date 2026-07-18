"""录制 / 回放 PICO 手柄原始输入，用于离线验证与调映射。

思路：遥操作流水线通过 `import xrobotoolkit_sdk as xrt` 读取手柄。这里提供两个
可替换 `sys.modules["xrobotoolkit_sdk"]` 的对象：

- RecordingSdk：**透传**真实 SDK，同时把每次 getter 调用的返回值按时间戳
  逐条写入 JSONL 文件（录制时需连着 PICO + PC 服务）。
- ReplaySdk：从 JSONL 读回，按经过时间回放每个 getter 的取值（无需任何硬件）。
  用于把“同一段真实手部动作”反复喂给不同版本的映射代码做对比。

只记录/回放 getter 的返回值，按函数名各自成一条时间序列，回放时对每次调用
取“时间戳 ≤ 当前经过时间”中最新的一条（阶梯保持），因此对调用顺序不敏感。
"""

import atexit
import json
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# 不记录的控制类函数（无返回值语义）
_SKIP_RECORD = {"init", "close"}


def _to_jsonable(v: Any) -> Any:
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer)):
        return v.item()
    if isinstance(v, (list, tuple)):
        return [_to_jsonable(x) for x in v]
    return v


class RecordingSdk:
    """包裹真实 xrobotoolkit_sdk，透传调用并记录每次 getter 返回值。"""

    def __init__(self, real_module, out_path: str):
        self._real = real_module
        self._out_path = out_path
        self._f = open(out_path, "w")
        self._t0: Optional[float] = None
        self._n = 0
        atexit.register(self._finalize)  # 防止运行循环未调用 close() 时丢数据

    def _record(self, fn: str, value: Any):
        if self._t0 is None:
            self._t0 = time.monotonic()
        t = time.monotonic() - self._t0
        self._f.write(json.dumps({"t": round(t, 4), "fn": fn, "v": _to_jsonable(value)}) + "\n")
        self._f.flush()
        self._n += 1

    def _finalize(self):
        if self._f and not self._f.closed:
            self._f.flush()
            self._f.close()
            print(f"[record] 已写入 {self._n} 条记录 -> {self._out_path}")

    def __getattr__(self, name: str):
        real_attr = getattr(self._real, name)
        if not callable(real_attr) or name in _SKIP_RECORD:
            return real_attr

        def wrapper(*args, **kwargs):
            value = real_attr(*args, **kwargs)
            try:
                self._record(name, value)
            except Exception:  # 记录失败不应影响遥操作
                pass
            return value

        return wrapper

    # 显式包装 init/close 以便管理文件
    def init(self, *a, **k):
        print(f"[record] 透传真实 SDK 并录制手柄输入 -> {self._out_path}")
        return self._real.init(*a, **k)

    def close(self, *a, **k):
        self._finalize()
        return self._real.close(*a, **k)


class ReplaySdk:
    """从 JSONL 回放手柄输入，按经过时间返回各 getter 的取值。"""

    def __init__(self, path: str):
        self._path = path
        self._series: Dict[str, List[Tuple[float, Any]]] = {}
        self._duration = 0.0
        self._load()
        self._t0: Optional[float] = None
        # 供离线分析：非 None 时用它当“当前时间”，否则用 wall-clock
        self._now_override: Optional[float] = None
        self._ended = False

    def _load(self):
        with open(self._path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                rec = json.loads(line)
                self._series.setdefault(rec["fn"], []).append((rec["t"], rec["v"]))
                self._duration = max(self._duration, rec["t"])
        for fn in self._series:
            self._series[fn].sort(key=lambda x: x[0])

    @property
    def duration(self) -> float:
        return self._duration

    def set_time(self, t: Optional[float]):
        """离线分析用：固定当前回放时间（秒）。传 None 恢复 wall-clock。"""
        self._now_override = t

    def _now(self) -> float:
        if self._now_override is not None:
            return self._now_override
        if self._t0 is None:
            self._t0 = time.monotonic()
        return time.monotonic() - self._t0

    def _lookup(self, fn: str, default: Any = 0.0) -> Any:
        seq = self._series.get(fn)
        if not seq:
            return default
        t = self._now()
        if t > self._duration and not self._ended:
            self._ended = True
            print(f"[replay] 回放结束（时长 {self._duration:.1f}s），保持末帧。Ctrl+C 退出。")
        # 取时间戳 <= t 的最新一条；t 早于首条则取首条
        lo, hi, idx = 0, len(seq) - 1, 0
        if t <= seq[0][0]:
            idx = 0
        elif t >= seq[-1][0]:
            idx = len(seq) - 1
        else:
            while lo <= hi:
                mid = (lo + hi) // 2
                if seq[mid][0] <= t:
                    idx = mid
                    lo = mid + 1
                else:
                    hi = mid - 1
        return seq[idx][1]

    def init(self, *a, **k):
        print(f"[replay] 从 {self._path} 回放手柄输入（时长 {self._duration:.1f}s，无需硬件）")

    def close(self, *a, **k):
        pass

    def __getattr__(self, name: str):
        # 位姿返回 np.array；其余按记录返回（float/bool/int）
        def wrapper(*args, **kwargs):
            val = self._lookup(name, default=0.0)
            if isinstance(val, list):
                return np.array(val, dtype=float)
            return val

        return wrapper
