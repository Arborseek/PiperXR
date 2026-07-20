# PiperXR

**Open-source XR teleoperation for AgileX PiPER**

[English](README.md)

用 XR 手柄（PICO 4 Ultra + [XRoboToolkit](https://github.com/XR-Robotics)）实时遥操作松灵 PiPER 六轴机械臂。**MuJoCo 仿真** 与 **真机 sim2real** 共用同一套控制流水线。

本项目在 Ubuntu 22.04 上提供一键环境搭建、placo 逆运动学、位姿映射校准、手柄输入录制/回放，便于离线迭代遥操作逻辑。

## 数据流

```
PICO 4 Ultra (XRoboToolkit-PICO-Client)
        │  Wi-Fi 局域网
        ▼
XRoboToolkit-PC-Service          # PC 端服务，端口 60061
        │  C++ SDK (libPXREARobotSDK.so)
        ▼
xrobotoolkit_sdk (Python 绑定)    # XrClient 读取手部位姿 / 扳机
        ▼
TeleopController + placo IK      # 仿真: MuJoCo  |  真机: piper_sdk over CAN
        ▼
PiPER 机械臂实时跟随手部运动
```

## 目录结构

```
piper-xr/
├── pyproject.toml              # 项目元数据、依赖、控制台入口 piper-xr
├── requirements.txt
├── Makefile
├── README.md                   # English (default)
├── README_zh.md                # 中文文档
├── LICENSE
├── piper_xr/                   # 可安装的 Python 包
│   ├── __main__.py             # python -m piper_xr 入口
│   ├── config.py               # 遥操作配置（单臂 / 双臂）
│   ├── paths.py
│   ├── common/                 # sim/real 共享工具
│   │   ├── pose_mapping.py     # XR → 末端位姿映射修正
│   │   ├── xr_record.py        # 手柄输入录制 / 回放
│   │   └── teleop_logger.py
│   ├── simulation/
│   │   ├── teleop.py           # 遥操作主逻辑 + tyro CLI
│   │   └── validate.py
│   └── real/
│       ├── real_piper_teleop_controller.py
│       └── piper_arm_proxy.py
├── scripts/
│   ├── setup_env.sh
│   └── simulation/teleop_piper_mujoco.py
├── assets/piper/
├── tests/
└── third_party/                # 由 setup_env.sh 自动克隆（已 gitignore）
```

## 前置条件

- **操作系统**：Ubuntu 22.04
- **Conda**：Miniconda 或 Anaconda
- **编译工具**：`git cmake build-essential`
- **PICO 端**：PICO 4 Ultra 头显 + XRoboToolkit-PICO-Client APK，开启开发者模式
- **PC 端服务**：需安装 `XRoboToolkit-PC-Service` 的 `.deb` 包（端口 60061）

## 快速开始

### 1. 一键搭建环境

```bash
git clone <this-repo> piper-xr
cd piper-xr
bash scripts/setup_env.sh
```

该脚本会自动完成：

1. 创建/复用 conda 环境 `pico_teleop`（Python 3.10）
2. 稀疏克隆 `mujoco_menagerie`（仅 `agilex_piper`）
3. 克隆 `XRoboToolkit-Teleop-Sample-Python`
4. 复制 PiPER MJCF 模型与网格到 `assets/piper/`
5. 下载并精简 PiPER URDF（供 placo 逆运动学）
6. 编译 `PXREARobotSDK`、构建 `xrobotoolkit_sdk` Python 绑定
7. 安装 `xrobotoolkit_teleop` 及全部依赖
8. 以 editable 方式安装本项目 `piper_xr`

> 说明：官方 `setup_conda.sh --install` 在部分 conda 镜像下会把 CPython 替换为 GraalPy，导致原生扩展不可用。本脚本用 pip 安装 `pybind11`、复用系统 `libstdc++`，完全绕开该问题。

### 2. 运行遥操作

```bash
# 1) 启动 PC 端服务
XRoboToolkit-PC-Service

# 2) 在 PICO 头显中打开 XRoboToolkit 应用，连同一 Wi-Fi，连接 PC

# 3) 运行遥操作（任选其一）
conda activate pico_teleop
python -m piper_xr          # 模块入口
piper-xr                    # 控制台命令
make teleop                 # Makefile 入口
```

握住 **grip** 激活手臂控制，**trigger** 控制夹爪开合。仿真模式下 MuJoCo viewer 中 PiPER 将实时跟随手部运动。

### 3. 常用选项

```bash
# 仿真
piper-xr --dual                          # 双臂
piper-xr --control-mode position         # 仅位置控制（默认 pose 完整 6 自由度）
piper-xr --scale-factor 2.0
piper-xr --hand left
piper-xr --visualize-placo
piper-xr --mock                          # 假手柄数据（无需头显）

# 真机
piper-xr --backend real                  # 单臂 can0
piper-xr --backend real --dual           # 双臂 can0 / can1

# 录制 / 回放（离线调映射）
piper-xr --record logs/motion.jsonl --note "yaw:先左转再右转"
piper-xr --replay logs/motion.jsonl      # 无需硬件

piper-xr --help
```

## 测试与验证

```bash
conda activate pico_teleop

make test       # pytest（mock SDK，无需头显）
make validate   # 无头流水线验证
```

验证内容：MuJoCo 场景加载、URDF 在 placo 中解析、placo↔MuJoCo 关节映射、IK + 夹爪流水线、末端保持在 home 位姿附近。

## 配置说明

遥操作配置定义在 `piper_xr/config.py`：

| 字段 | 值 | 说明 |
|------|----|----|
| `link_name` | `link6` | 末端法兰（6 自由度腕部） |
| `pose_source` | `right_controller` | XR 手柄位姿来源 |
| `control_trigger` | `right_grip` | 握住 grip 激活控制 |
| `vis_target` | `piper_target` | 场景中的 mocap 目标体 |
| `joint_names` | `["joint7"]` | 夹爪执行器（0~0.035 m），joint8 由 equality 耦合 |

关节名为 `joint1..joint6`（臂）+ `joint7`/`joint8`（夹爪）。placo↔MuJoCo 按**名字**映射，URDF 与 MJCF 关节名必须一致。

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `R_headset_world` | `R_HEADSET_TO_WORLD_PIPER` | 头显坐标系 → 机器人世界系的固定旋转 |
| `enable_yaw_align` | `True` | 按 grip 时朝向自对齐（`pose_mapping.py`） |

## 坐标变换与关节映射

末端 6-DoF **相对遥操作**：一套变换链同时负责**位置**（大臂/肩肘）和**朝向**（小臂/手腕）。实现在 `piper_xr/common/pose_mapping.py`。

### 记号

GitHub 公式采用简化符号（下标 `c` = 控制器，`e` = 末端；上标 `h` = 头显系，`w` = 世界系）：

- $\mathbf{p}_c, \mathbf{R}_c$ — 控制器在**头显系**下的位姿（PICO 原始数据）
- $\mathbf{p}_c^w, \mathbf{R}_c^w$ — 经变换 $A$ 后的控制器位姿（机器人世界系）
- $\mathbf{p}_c^{\mathrm{ref}}, \mathbf{R}_c^{\mathrm{ref}}$ — 激活瞬间的控制器参考位姿（世界系，$A$ 变换后存储）
- $\mathbf{p}_e^{\mathrm{ref}}, \mathbf{R}_e^{\mathrm{ref}}$ — 激活瞬间末端参考位姿（朝向可能被朝下基准替换）
- $\mathbf{R}_{hw}$ — 头显系 → 世界系固定旋转（`R_HEADSET_TO_WORLD_PIPER`）
- $\mathbf{M}_r$ — $\mathbf{R}_{hw}$ 的正常旋转版本（$\det=+1$）
- $\mathbf{R}_{\mathrm{yaw}}$ — 激活时捕获的朝向自对齐
- $A = \mathbf{M}_r \mathbf{R}_{\mathrm{yaw}}$ — 组合变换矩阵
- $s$ — 位移缩放系数（`scale_factor`，默认 1.5）

### 头显 → 世界旋转

由录制片段（`trans_x/y/z`、`yaw/pitch/roll`）离线回放校准：

$$
\mathbf{R}_{hw} =
\begin{bmatrix} 0 & 0 & -1 \\ -1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix},
\quad \det(\mathbf{R}_{hw}) = +1
$$

轴对应（头显 → 机器人世界系）：

- 机械臂 $+X$（前）$\leftarrow -$头显 $Z$
- 机械臂 $+Y$（左）$\leftarrow -$头显 $X$
- 机械臂 $+Z$（上）$\leftarrow$ 头显 $Y$

### 控制器位姿搬到机器人世界系

位置与朝向共用同一变换 $A$：

$$
\mathbf{p}_c^w = A \mathbf{p}_c^h
$$

$$
\mathbf{R}_c^w = A \mathbf{R}_c^h A^{\mathsf{T}}
$$

### 相对增量（握 grip 期间）

$$
\Delta\mathbf{p} = s\left(\mathbf{p}_c^w - \mathbf{p}_c^{\mathrm{ref}}\right)
$$

$$
\Delta\mathbf{R} = \text{quatDiff}\!\left(\mathbf{R}_c^{\mathrm{ref}},\, \mathbf{R}_c^w\right)
$$

（`quatDiff` 对应代码中的 `quat_diff_as_angle_axis`）

### 末端目标位姿

在世界系下叠加（`apply_delta_pose`）：

$$
\mathbf{p}_e^{\mathrm{tgt}} = \mathbf{p}_e^{\mathrm{ref}} + \Delta\mathbf{p}
$$

$$
\mathbf{R}_e^{\mathrm{tgt}} = \Delta\mathbf{R}_{q}\, \mathbf{R}_e^{\mathrm{ref}}
$$

其中 $\Delta\mathbf{R}_{q}$ 为角轴 $\Delta\mathbf{R}$ 对应的单位四元数。

### 激活瞬间（按 grip）

两项一次性校准：

**1. 朝向自对齐** — 头显世界系的水平朝向由头显开机/边界决定，与操作者面朝无关。$\mathbf{R}_{\mathrm{yaw}}$ 为绕竖直轴的最短旋转，把操作者当前正前方（头显 $-Z$ 投影到水平面）对齐到机械臂正前方（头显系表示）。**松开 grip 前保持不变**（下次激活重新捕获）。

**2. 朝下基准** — 把 $\mathbf{R}_e^{\mathrm{ref}}$ 替换为朝下抓取姿态：最短弧把 link6 本体 $+Z$（夹爪接近方向）旋到世界 $-Z$，保留 yaw。手自然前伸时夹爪即朝下，便于抓桌面物体。

### 人手臂操作对应

| 人手动作 | 末端响应 | 实现 |
|----------|---------|------|
| 手平移（肩肘） | 末端从参考位置按缩放平移 | $\Delta\mathbf{p}$ |
| 手腕俯仰 / 滚转 / 偏航 | 末端从参考朝向旋转 | $\Delta\mathbf{R}$ |
| 握 grip | 激活控制，捕获参考 + 朝向自对齐 + 朝下基准 | `control_trigger` |
| 扣 trigger | 夹爪开合 | `gripper_config` |

### 性质

- **相对控制**：只跟随激活参考位姿的变化量 → 稳定，不受绝对位姿噪声漂移。
- 位置与朝向共用 $A$ → 旋转手性一致（偏航/滚转方向符合直觉）。
- $\mathbf{R}_{\mathrm{yaw}}$ 解耦操作者朝向与头显边界 → 无论站哪、朝哪，身体前后左右都对上机械臂前后左右。

实现代码：`piper_xr/common/pose_mapping.py`（`CorrectedPoseMixin`）。

## 故障排查

| 现象 | 原因 / 处理 |
|------|------------|
| `No module named 'xrobotoolkit_sdk'` | 未运行 `setup_env.sh`，或未激活 `pico_teleop` 环境 |
| `GraalPy` 字样 / 原生扩展导入失败 | conda 把 CPython 替换为 GraalPy，用 `setup_env.sh` 重建环境 |
| `init()` 后进程 core dump | PC 端服务未启动（60061 端口无服务），启动服务即可 |
| 连接失败 / 卡顿 | PC 与 PICO 不在同一局域网，或防火墙阻止 60061；优先用 5 GHz Wi-Fi |
| 模型加载失败 | 检查 `assets/piper/` 是否由 `setup_env.sh` 生成 |
| pytest 报 `No module named 'lark'` | `PYTHONPATH` 混入 ROS，用 `env -u PYTHONPATH pytest` |

## 参考资源

- [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) — PiPER 的 MJCF 模型（`agilex_piper`）
- [XRoboToolkit-Teleop-Sample-Python](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python) — 官方遥操作示例
- [XRoboToolkit-PC-Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) — PC 端服务
- [piper_ros](https://github.com/agilexrobotics/piper_ros) — PiPER URDF 来源

## 许可证

**仅供学习交流使用，禁止商业用途**（PiperXR Non-Commercial License），详见 [LICENSE](LICENSE)。如需商用授权请联系仓库维护者。PiPER 模型与 XRoboToolkit 等第三方资源遵循各自原始许可证。
