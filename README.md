# piper-pico

基于 **PICO 4 Ultra 头显** 手势追踪，实时遥操作 **MuJoCo 仿真** 中的 **松灵（AgileX）PiPER 六轴机械臂**。

本项目在 Ubuntu 22.04 上搭建完整的遥操作开发环境，复用 PICO 官方的 [XRoboToolkit](https://github.com/XR-Robotics) 通信框架，将 PiPER 的 MuJoCo 模型接入官方示例的 `MujocoTeleopController` 控制流水线。

## 数据流

```
PICO 4 Ultra (XRoboToolkit-PICO-Client)
        │  Wi-Fi 局域网
        ▼
XRoboToolkit-PC-Service          # PC 端服务，监听 127.0.0.1:60061
        │  C++ SDK (libPXREARobotSDK.so)
        ▼
xrobotoolkit_sdk (Python 绑定)    # XrClient 读取手部位姿 / 扳机
        ▼
MujocoTeleopController            # placo 逆运动学 + MuJoCo 仿真
        ▼
PiPER 机械臂在 MuJoCo 中实时跟随手部运动
```

## 目录结构

```
piper-pico/
├── pyproject.toml              # 项目元数据、依赖、控制台入口 piper-teleop
├── requirements.txt            # 已验证的依赖版本锁
├── Makefile                    # setup / teleop / validate / test / clean
├── README.md
├── LICENSE
├── piper_pico/                 # 可安装的 Python 包
│   ├── __init__.py             # 版本
│   ├── __main__.py             # python -m piper_pico 入口
│   ├── paths.py                # 资源路径常量
│   ├── config.py               # PiPER 遥操作配置（可镜像左手）
│   └── simulation/
│       ├── __init__.py
│       ├── teleop.py           # 遥操作主逻辑 + tyro CLI
│       └── validate.py         # 无头流水线验证函数
├── scripts/
│   ├── setup_env.sh            # 一键搭建环境
│   ├── strip_urdf.py           # URDF 精简工具（去除网格供 placo IK）
│   └── simulation/
│       └── teleop_piper_mujoco.py  # 兼容性入口（薄包装）
├── assets/piper/
│   ├── scene.xml               # MuJoCo 场景（含 piper_target mocap 体）
│   └── piper_description.urdf  # 精简 URDF（关节名与 MJCF 一致）
├── tests/
│   ├── conftest.py             # 自动注入 mock SDK
│   ├── test_pipeline.py        # pytest 用例
│   ├── validate_piper_pipeline.py  # 独立验证脚本（无需 pytest）
│   └── _mock_xrobotoolkit_sdk.py   # 离线 mock SDK
└── third_party/               # 由 setup_env.sh 自动克隆（已 gitignore）
    ├── mujoco_menagerie/agilex_piper/
    ├── XRoboToolkit-Teleop-Sample-Python/
    ├── XRoboToolkit-PC-Service/
    └── XRoboToolkit-PC-Service-Pybind/
```

## 前置条件

- **操作系统**：Ubuntu 22.04
- **Conda**：Miniconda 或 Anaconda
- **编译工具**：`git cmake build-essential`（`sudo apt install git cmake build-essential`）
- **PICO 端**：PICO 4 Ultra 头显 + XRoboToolkit-PICO-Client APK，开启开发者模式
- **PC 端服务**：需安装 `XRoboToolkit-PC-Service` 的 `.deb` 包（运行遥操作时提供 60061 端口服务）

## 快速开始

### 1. 一键搭建环境

```bash
git clone <this-repo> piper-pico
cd piper-pico
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
8. 以 editable 方式安装本项目 `piper_pico`

> 说明：官方 `setup_conda.sh --install` 在部分 conda 镜像下会把 CPython 替换为 GraalPy，导致原生扩展不可用。本脚本用 pip 安装 `pybind11`、复用系统 `libstdc++`，完全绕开该问题；`git clone` 失败时自动回退到 codeload tarball。

### 2. 运行遥操作

```bash
# 1) 启动 PC 端服务（系统菜单或命令行）
XRoboToolkit-PC-Service

# 2) 在 PICO 头显中打开 XRoboToolkit 应用，连同一 Wi-Fi，连接 PC

# 3) 运行遥操作（任选其一）
conda activate pico_teleop
python scripts/simulation/teleop_piper_mujoco.py   # 兼容入口
python -m piper_pico                                # 模块入口
piper-teleop                                        # 控制台命令
```

握住右手手柄激活手臂控制，右手扳机控制夹爪开合。MuJoCo viewer 窗口中 PiPER 将实时跟随你的手部运动。

### 3. 常用选项

```bash
piper-teleop --control-mode position     # 仅位置控制（默认 pose 完整 6 自由度）
piper-teleop --scale-factor 2.0          # 放大手部位移
piper-teleop --hand left                  # 使用左手控制器
piper-teleop --visualize-placo            # 浏览器可视化 IK 求解
piper-teleop --help                       # 查看全部参数
```

## 测试与验证

```bash
conda activate pico_teleop

# pytest 用例（mock SDK，无需头显）
make test
# 或：env -u PYTHONPATH python -m pytest tests

# 独立验证脚本（无需 pytest）
make validate
# 或：python tests/validate_piper_pipeline.py
```

验证内容：MuJoCo 场景加载、URDF 在 placo 中解析、placo↔MuJoCo 关节映射、IK + 夹爪 + 控制流水线、末端保持在 home 位姿附近。

## 配置说明

PiPER 配置定义在 `piper_pico/config.py`：

| 字段 | 值 | 说明 |
|------|----|----|
| `link_name` | `link6` | 末端法兰（6 自由度腕部） |
| `pose_source` | `right_controller` | XR 手柄位姿来源 |
| `control_trigger` | `right_grip` | 握住手柄激活控制 |
| `vis_target` | `piper_target` | 场景中的 mocap 目标体 |
| `joint_names` | `["joint7"]` | 夹爪执行器驱动 joint7（0~0.035），joint8 由 equality 耦合 |

PiPER 模型关节名为 `joint1..joint6`（6 自由度）+ 夹爪 `joint7`/`joint8`。`MujocoTeleopController` 按**名字**做 placo↔MuJoCo 关节映射，因此 URDF 关节名必须与 MJCF 一致——本项目从 `agilexrobotics/piper_ros` 取 URDF 并去除网格（避免 `package://` 解析失败），关节名天然匹配。

## 故障排查

| 现象 | 原因 / 处理 |
|------|------------|
| `No module named 'xrobotoolkit_sdk'` | 未运行 `setup_env.sh`，或未激活 `pico_teleop` 环境 |
| `GraalPy` 字样 / 原生扩展导入失败 | conda 把 CPython 替换为 GraalPy，用 `setup_env.sh` 重建环境 |
| `init()` 后进程退出 core dump | PC 端服务未启动（60061 端口无服务），属正常；启动服务即可 |
| 连接失败 / 卡顿 | PC 与 PICO 不在同一局域网，或防火墙阻止 60061；优先用 5GHz Wi-Fi |
| 模型加载失败 | 检查 `assets/piper/piper.xml` 与 `assets/piper/assets/` 是否由 `setup_env.sh` 生成 |
| pytest 报 `No module named 'lark'` | `PYTHONPATH` 混入 ROS，用 `env -u PYTHONPATH python -m pytest tests` |

## 进阶扩展

- **双臂**：在 `config.py` 中添加 `left_hand` 配置，或扩展 `build_piper_config` 支持双臂 dict。
- **ROS2 集成**：参考社区 `agilex_arm_mujoco` 项目将 PiPER 仿真接入 ROS2。
- **精确控制**：`piper-physics`（Rust）提供基于 MuJoCo 的重力补偿与逆动力学。
- **全身遥操作**：NVIDIA GR00T 项目的 PICO 数据流架构可作参考。

## 参考资源

- [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) — PiPER 的 MJCF 模型（`agilex_piper`）
- [XRoboToolkit-Teleop-Sample-Python](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python) — 官方遥操作示例
- [XRoboToolkit-PC-Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) — PC 端服务
- [piper_ros](https://github.com/agilexrobotics/piper_ros) — PiPER URDF 来源

## 许可证

MIT，详见 [LICENSE](LICENSE)。PiPER 模型与 XRoboToolkit 等第三方资源遵循各自原始许可证。
