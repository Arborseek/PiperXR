# PiperXR

**Open-source XR teleoperation for AgileX PiPER**

[中文文档](README_zh.md)

Teleoperate the AgileX PiPER 6-DOF arm with an XR controller (PICO 4 Ultra + [XRoboToolkit](https://github.com/XR-Robotics)). One unified pipeline for **MuJoCo simulation** and **real-robot sim2real**.

Built on Ubuntu 22.04 with one-command setup, placo inverse kinematics, pose-mapping calibration, and controller input record/replay for offline iteration.

## Data flow

```
PICO 4 Ultra (XRoboToolkit-PICO-Client)
        │  Wi-Fi LAN
        ▼
XRoboToolkit-PC-Service          # PC service, port 60061
        │  C++ SDK (libPXREARobotSDK.so)
        ▼
xrobotoolkit_sdk (Python)        # XrClient: controller pose / triggers
        ▼
TeleopController + placo IK      # sim: MuJoCo  |  real: piper_sdk over CAN
        ▼
PiPER arm follows your hand motion
```

## Project layout

```
piper-xr/
├── pyproject.toml              # metadata, deps, CLI entry piper-xr
├── requirements.txt
├── Makefile
├── README.md                   # English (default)
├── README_zh.md                # 中文文档
├── LICENSE
├── piper_xr/                   # installable Python package
│   ├── __main__.py             # python -m piper_xr
│   ├── config.py               # teleop config (single / dual arm)
│   ├── paths.py
│   ├── common/                 # shared sim/real utilities
│   │   ├── pose_mapping.py     # corrected XR → end-effector mapping
│   │   ├── xr_record.py        # record / replay controller input
│   │   └── teleop_logger.py
│   ├── simulation/
│   │   ├── teleop.py           # main entry + tyro CLI
│   │   └── validate.py
│   └── real/
│       ├── real_piper_teleop_controller.py
│       └── piper_arm_proxy.py
├── scripts/
│   ├── setup_env.sh
│   └── simulation/teleop_piper_mujoco.py
├── assets/piper/
├── tests/
└── third_party/                # cloned by setup_env.sh (gitignored)
```

## Requirements

- **OS**: Ubuntu 22.04
- **Conda**: Miniconda or Anaconda
- **Build tools**: `git cmake build-essential`
- **PICO**: PICO 4 Ultra + XRoboToolkit-PICO-Client APK (developer mode)
- **PC service**: `XRoboToolkit-PC-Service` `.deb` package (port 60061)

## Quick start

### 1. Set up the environment

```bash
git clone <this-repo> piper-xr
cd piper-xr
bash scripts/setup_env.sh
```

The script will:

1. Create/reuse conda env `pico_teleop` (Python 3.10)
2. Sparse-clone `mujoco_menagerie` (`agilex_piper` only)
3. Clone `XRoboToolkit-Teleop-Sample-Python`
4. Copy PiPER MJCF assets into `assets/piper/`
5. Download and strip PiPER URDF for placo IK
6. Build `PXREARobotSDK` and `xrobotoolkit_sdk` Python bindings
7. Install `xrobotoolkit_teleop` and dependencies
8. Install this project (`piper_xr`) in editable mode

> Note: the upstream `setup_conda.sh --install` may replace CPython with GraalPy on some conda mirrors, breaking native extensions. Our script avoids this by installing `pybind11` via pip and reusing the system `libstdc++`.

### 2. Run teleoperation

```bash
# 1) Start the PC service
XRoboToolkit-PC-Service

# 2) Open XRoboToolkit on the PICO headset, same Wi-Fi, connect to PC

# 3) Run teleop (pick one)
conda activate pico_teleop
python -m piper_xr          # module entry
piper-xr                    # console command
make teleop                 # via Makefile
```

Hold the **grip** button to activate arm control; use the **trigger** for the gripper. In simulation, the MuJoCo viewer shows the arm following your hand.

### 3. Common options

```bash
# Simulation
piper-xr --dual                          # dual-arm
piper-xr --control-mode position         # position-only (default: full 6-DOF pose)
piper-xr --scale-factor 2.0
piper-xr --hand left
piper-xr --visualize-placo
piper-xr --mock                          # fake moving controller (no headset)

# Real robot
piper-xr --backend real                  # single arm on can0
piper-xr --backend real --dual           # dual arm on can0 / can1

# Record / replay (offline mapping iteration)
piper-xr --record logs/motion.jsonl --note "yaw: turn left then right"
piper-xr --replay logs/motion.jsonl      # no hardware needed

piper-xr --help
```

## Testing

```bash
conda activate pico_teleop

make test       # pytest with mock SDK (no headset)
make validate   # headless pipeline check
```

Validation covers: MuJoCo scene load, URDF in placo, joint name mapping, IK + gripper pipeline, end-effector near home pose.

## Configuration

Teleop config lives in `piper_xr/config.py`:

| Field | Value | Description |
|-------|-------|-------------|
| `link_name` | `link6` | End-effector flange (6-DOF wrist) |
| `pose_source` | `right_controller` | XR controller pose source |
| `control_trigger` | `right_grip` | Hold grip to activate control |
| `vis_target` | `piper_target` | Mocap target body in scene |
| `joint_names` | `["joint7"]` | Gripper actuator (0–0.035 m); joint8 coupled via equality |

Joint names are `joint1..joint6` (arm) + `joint7`/`joint8` (gripper). placo↔MuJoCo mapping is **by name**, so URDF and MJCF joint names must match.

| Field | Default | Description |
|-------|---------|-------------|
| `R_headset_world` | `R_HEADSET_TO_WORLD_PIPER` | Fixed rotation from headset frame to robot world frame |
| `enable_yaw_align` | `True` | Align operator forward to robot forward on grip press (`pose_mapping.py`) |

## Coordinate transform & mapping

End-effector 6-DoF relative teleoperation: one transform chain for **position** (shoulder/elbow) and **orientation** (wrist). Implemented in `piper_xr/common/pose_mapping.py`.

### Headset → world rotation

Calibrated with recorded clips (`trans_x/y/z`, `yaw/pitch/roll`):

$$
\mathbf{R}_{hw} =
\begin{pmatrix}
0 & 0 & -1 \\
-1 & 0 & 0 \\
0 & 1 & 0
\end{pmatrix}
$$

$$
\det(\mathbf{R}_{hw}) = +1
$$

Axis mapping (headset → robot world):

- $X_{\mathrm{robot}} = -Z_{\mathrm{headset}}$ (forward)
- $Y_{\mathrm{robot}} = -X_{\mathrm{headset}}$ (left)
- $Z_{\mathrm{robot}} = Y_{\mathrm{headset}}$ (up)

### Controller pose in robot world frame

Position and orientation share the same transform $A$:

$$
\mathbf{p}_c^w = A \mathbf{p}_c^h
$$

$$
\mathbf{R}_c^w = A \mathbf{R}_c^h A^{\mathsf{T}}
$$

### Relative deltas (while grip held)

$$
\Delta\mathbf{p} = s\left(\mathbf{p}_c^w - \mathbf{p}_c^{\mathrm{ref}}\right)
$$

$$
\Delta\mathbf{R} = \text{quatDiff}\!\left(\mathbf{R}_c^{\mathrm{ref}},\, \mathbf{R}_c^w\right)
$$

(`quatDiff` = `quat_diff_as_angle_axis` in code)

### End-effector target

Applied in world frame (`apply_delta_pose`):

$$
\mathbf{p}_e^{\mathrm{tgt}} = \mathbf{p}_e^{\mathrm{ref}} + \Delta\mathbf{p}
$$

$$
\mathbf{R}_e^{\mathrm{tgt}} = \Delta\mathbf{R}_{q}\, \mathbf{R}_e^{\mathrm{ref}}
$$

where $\Delta\mathbf{R}_{q}$ is the unit quaternion for angle-axis $\Delta\mathbf{R}$.

### On activation (grip press)

Two one-shot calibrations:

**1. Yaw self-align** — headset world frame has a fixed horizontal heading unrelated to where the operator stands. $\mathbf{R}_{\mathrm{yaw}}$ is the shortest rotation about the vertical axis that maps the operator's current forward (HMD $-Z$, projected to horizontal) to robot forward in headset frame. Held constant **until grip is released** (re-captured on next activation).

**2. Top-down anchor** — replace $\mathbf{R}_{e}^{\mathrm{ref}}$ with a downward grasp orientation: shortest arc rotates link6 body $+Z$ (gripper approach) to world $-Z$, preserving yaw. Natural reach → gripper points down for tabletop grasping.

### Operation correspondence

| Human motion | End-effector response | Mechanism |
|--------------|----------------------|-----------|
| Hand translation (shoulder/elbow) | EE moves from reference position | $\Delta\mathbf{p}$ |
| Wrist pitch / roll / yaw | EE rotates from reference orientation | $\Delta\mathbf{R}$ |
| Hold grip | Activate control, capture refs + yaw align + top-down anchor | `control_trigger` |
| Trigger | Gripper open/close | `gripper_config` |

### Properties

- **Relative control**: only deltas from the activation reference are applied → stable, no drift from absolute pose noise.
- **Shared $A$** for position and orientation → consistent chirality (yaw/roll directions match intuition).
- **$\mathbf{R}_{\mathrm{yaw}}$** decouples operator heading from headset boundary orientation → body forward/right map to robot forward/right regardless of stance.

Implementation: `piper_xr/common/pose_mapping.py` (`CorrectedPoseMixin`).

## Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| `No module named 'xrobotoolkit_sdk'` | Run `setup_env.sh` and activate `pico_teleop` |
| GraalPy / native extension import error | Rebuild env with `setup_env.sh` (avoid GraalPy) |
| `init()` then core dump | PC service not running on port 60061 — start it first |
| Connection failure / lag | PC and PICO not on same LAN, or firewall blocking 60061; prefer 5 GHz Wi-Fi |
| Model load failure | Check `assets/piper/` was generated by `setup_env.sh` |
| pytest `No module named 'lark'` | ROS on `PYTHONPATH` — use `env -u PYTHONPATH pytest` |

## References

- [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) — PiPER MJCF (`agilex_piper`)
- [XRoboToolkit-Teleop-Sample-Python](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python)
- [XRoboToolkit-PC-Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)
- [piper_ros](https://github.com/agilexrobotics/piper_ros) — PiPER URDF source

## License

**Non-commercial use only** (PiperXR Non-Commercial License) — see [LICENSE](LICENSE). Contact the maintainers for commercial licensing. PiPER models and XRoboToolkit third-party assets follow their respective licenses.
