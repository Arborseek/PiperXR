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
import os
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

    def __init__(self, real_module, out_path: str, note: str = ""):
        self._real = real_module
        self._out_path = out_path
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        self._f = open(out_path, "w")
        # 首行写元信息（意图标注），使录制文件自解释
        meta = {"meta": {"note": note, "created": time.strftime("%Y-%m-%d %H:%M:%S")}}
        self._f.write(json.dumps(meta, ensure_ascii=False) + "\n")
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
        self._note = ""
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
                if "meta" in rec:
                    self._note = rec["meta"].get("note", "")
                    continue
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
        note = f"，标注：{self._note}" if self._note else ""
        print(f"[replay] 从 {self._path} 回放手柄输入（时长 {self._duration:.1f}s，无需硬件{note}）")

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


# ------------------------------------------------------------------
# 离线分析：从录制文件自动识别“主要动作”（平移/旋转、主导轴、方向）
# ------------------------------------------------------------------
# 中性轴名：不预设“左右/上下/前后”的人类语义（那依赖 headset 约定，需用 note 校准）
_AXIS_NAME = ["headsetX", "headsetY", "headsetZ"]


def summarize(path: str, hand: str = "right") -> dict:
    """读取录制文件，输出该段手部动作在 **headset 原始系** 的客观特征。

    不做任何我们自己的坐标变换，纯看手柄本身怎么动，用于：
      - 自动判断这段主要是平移还是旋转、主导哪个轴、往哪个方向；
      - 与录制标注(note)/文件名对照，确认贴标签没错、方向对不对。

    Returns:
        dict: {note, duration, n_active, translation:{axis,delta_xyz}, rotation:{axis,net_angle_axis_deg}}
    """
    import json as _json

    import meshcat.transformations as tf

    note = ""
    poses: List[Tuple[float, list]] = []
    grips: List[Tuple[float, float]] = []
    pose_fn = f"get_{hand}_controller_pose"
    grip_fn = f"get_{hand}_grip"
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = _json.loads(line)
            if "meta" in rec:
                note = rec["meta"].get("note", "")
                continue
            if rec["fn"] == pose_fn:
                poses.append((rec["t"], rec["v"]))
            elif rec["fn"] == grip_fn:
                grips.append((rec["t"], float(rec["v"])))

    def grip_at(t):
        g = 0.0
        for gt, gv in grips:
            if gt <= t:
                g = gv
            else:
                break
        return g

    active = [(t, np.array(v, dtype=float)) for t, v in poses if grip_at(t) > 0.9]
    result: Dict[str, Any] = {"note": note, "duration": poses[-1][0] if poses else 0.0,
                              "n_active": len(active)}
    if len(active) < 2:
        result["warning"] = "激活帧太少，请全程握住 grip 再做动作"
        return result

    xyz = np.array([p[1][:3] for p in active])
    quats = [np.array([p[1][6], p[1][3], p[1][4], p[1][5]]) for p in active]  # [w,x,y,z]

    # 平移：各轴峰峰值 + 净位移（末-首）
    delta_xyz = xyz[-1] - xyz[0]
    span_xyz = xyz.max(axis=0) - xyz.min(axis=0)
    t_axis = int(np.argmax(span_xyz))

    # 旋转：相对首帧的净角轴（headset 原始系）
    q0 = quats[0]
    net_aa = np.zeros(3)
    max_ang = 0.0
    for q in quats:
        dq = tf.quaternion_multiply(q, tf.quaternion_inverse(q0))
        if dq[0] < 0:
            dq = -dq
        ang = 2 * np.arccos(np.clip(dq[0], -1, 1))
        if ang > max_ang:
            max_ang = ang
            s = np.sin(ang / 2)
            net_aa = (dq[1:] / s * ang) if s > 1e-6 else np.zeros(3)
    r_axis = int(np.argmax(np.abs(net_aa)))

    result["translation"] = {"dominant_axis": _AXIS_NAME[t_axis],
                             "delta_xyz_m": np.round(delta_xyz, 3).tolist(),
                             "span_xyz_m": np.round(span_xyz, 3).tolist()}
    result["rotation"] = {"dominant_axis": _AXIS_NAME[r_axis],
                          "net_angle_axis_deg": np.round(net_aa * 180 / np.pi, 1).tolist(),
                          "max_angle_deg": round(max_ang * 180 / np.pi, 1)}
    # 判定主导类型：平移位移(m) 与 旋转角(rad) 的相对量级
    result["dominant"] = "旋转" if max_ang > 0.35 and max_ang > span_xyz.max() * 3 else "平移"
    return result

