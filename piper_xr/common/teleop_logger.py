"""sim/real 统一 schema 的遥操作日志写入器。

每帧写一行 CSV，列固定、与后端无关，使仿真与真机的数据集可直接互换：
timestamp, backend, hand, j1..j6(rad), gripper(0..1), ee_xyz(3), ee_quat(4),
cmd_j1..cmd_j6(rad), cmd_gripper(0..1)
"""

import csv
import os
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class TeleopFrame:
    backend: str            # "sim" / "real"
    hand: str               # "right_hand" / "left_hand"
    joint_pos: np.ndarray   # (6,) rad
    gripper: float          # 0..1
    ee_xyz: np.ndarray      # (3,)
    ee_quat: np.ndarray      # (4,) [w,x,y,z]
    cmd_joint: np.ndarray    # (6,) rad
    cmd_gripper: float       # 0..1
    timestamp: float = 0.0


class TeleopLogger:
    def __init__(self, log_path: str, backend: str):
        self.backend = backend
        os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)
        self._file = open(log_path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self._header())
        self._t0 = time.time()

    @staticmethod
    def _header():
        return (["timestamp", "backend", "hand"]
                + [f"j{i}" for i in range(1, 7)]
                + ["gripper", "ee_x", "ee_y", "ee_z", "qw", "qx", "qy", "qz"]
                + [f"cmd_j{i}" for i in range(1, 7)]
                + ["cmd_gripper"])

    def log(self, frame: TeleopFrame):
        row = ([f"{frame.timestamp or (time.time() - self._t0):.4f}", self.backend, frame.hand]
               + [f"{x:.6f}" for x in np.asarray(frame.joint_pos, dtype=float).reshape(-1)[:6]]
               + [f"{float(frame.gripper):.6f}"]
               + [f"{x:.6f}" for x in np.asarray(frame.ee_xyz, dtype=float).reshape(-1)[:3]]
               + [f"{x:.6f}" for x in np.asarray(frame.ee_quat, dtype=float).reshape(-1)[:4]]
               + [f"{x:.6f}" for x in np.asarray(frame.cmd_joint, dtype=float).reshape(-1)[:6]]
               + [f"{float(frame.cmd_gripper):.6f}"])
        self._writer.writerow(row)
        self._file.flush()

    def close(self):
        if self._file and not self._file.closed:
            self._file.close()
