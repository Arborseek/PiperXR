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

| 字段 | 默认值 | 说明 |
|------|--------|----|
| `R_headset_world` | `R_HEADSET_TO_WORLD_PIPER` | 头显坐标系→世界坐标系的固定旋转，纠正 PICO 默认映射的左右镜像 |
| `R_hand_to_ee` | `np.eye(3)` | 手柄坐标系→末端(link6)坐标系的固定对齐旋转，用于小臂朝向微调 |

## 坐标变换与关节映射

本节给出 PICO 手部位姿到 PiPER 末端位姿的完整数学推导。本项目只做**末端 6-DoF 相对遥操作**（不重定向手指/手掌），核心是两个刚体姿态的坐标系变换：一个负责**位置/方向**（大臂），一个负责**末端朝向**（小臂）。

### 记号

- $R_{wc}\in SO(3)$：控制器在世界系下的姿态（PICO 给的四元数转旋转矩阵）
- $R_{wc}^{ref}$：激活瞬间记录的控制器参考姿态
- $R_{we}^{ref}$：激活瞬间末端(link6)的参考姿态（home）
- $R_{h\to w}$：头显系→世界系的固定旋转（`R_headset_world`）
- $R_{h\to e}$：手柄系→末端系的固定对齐旋转（`R_hand_to_ee`）
- $\Delta p$：控制器位置 delta（经 $R_{h\to w}$ 旋转到世界系，再乘缩放 $s$）
- $D = R_{wc}\,(R_{wc}^{ref})^{-1}$：控制器从参考到当前的旋转 delta（世界系）

### 位置映射（大臂）

纯平移相对控制，无旋转数学：

$$p_{we}^{target} = p_{we}^{ref} + s\cdot\Delta p,\qquad \Delta p = R_{h\to w}\big(p_{wc} - p_{wc}^{ref}\big)$$

### 朝向映射（小臂）

XRoboToolkit 基类 `apply_delta_pose` 把旋转 delta 在**世界系左乘**到末端：

$$R_{we}^{target} = D\,R_{we}^{ref}$$

左乘 = 在世界系应用 delta。问题在于 $D$ 是绕"控制器当时所在世界轴"转的，而末端自己的小臂轴在世界里指向另一个方向，于是末端绕的不是自己的小臂轴——人手做腕部旋前/旋后时，小臂朝向对不上人手。

本项目（`piper_pico/common/hand_ee_mapping.py` 的 `PiperHandEEMixin`）改成**带固定对齐的局部系应用**。推导分两步：

**1. 用 $R_{h\to e}$ 把世界系 delta 重表达到末端语义的坐标系。**
相似变换（共轭）保旋转角、只换基底轴：

$$D_{ee} = R_{h\to e}\,D\,R_{h\to e}^{-1}$$

**2. 在末端局部系应用（右乘到参考姿态）。**

$$\boxed{\;R_{we}^{target} = R_{we}^{ref}\,R_{h\to e}\,D\,R_{h\to e}^{-1}\;}$$

这就是代码里 `target_R = ref_ee_R @ R_hand_to_ee @ D @ R_hand_to_ee.T` 那一行（$R_{h\to e}^{-1}=R_{h\to e}^{\top}$，因旋转矩阵正交）。四元数版本等价，工程上用矩阵算再转回四元数，共轭比四元数乘法直观。

### 性质

- $R_{h\to e}=I$ 时退化为 $R_{we}^{target}=R_{we}^{ref}D$，即"末端局部系应用世界 delta"，已比基类的世界系左乘自然，且不引入额外偏置。
- $D=I$（手没动）时 $R_{we}^{target}=R_{we}^{ref}$，末端停在 home，符合相对控制语义。
- 旋转角守恒：$\angle(D_{ee})=\angle(D)$（共轭保角），人手转多少末端转多少，不放大不缩小。

### $R_{h\to e}$ 的标定

$R_{h\to e}$ 是手柄坐标系到 link6 坐标系的固定对齐。理论值是两坐标系在"自然握持"姿态下的相对旋转：

$$R_{h\to e} = (R_{we}^{ref})^{-1}\,R_{h\to w}\,R_{wc}^{neutral}$$

但 $R_{wc}^{neutral}$ 因人而异，故实践中默认 $I$，再用一个绕单轴 $90^\circ$ 的旋转微调。实测时若某方向反了（如"手腕内翻时末端外翻"），即可反推 $R_{h\to e}$ 应绕哪个轴转 $90^\circ$，无需盲调。

### 头显→世界旋转 $R_{h\to w}$

XRoboToolkit 默认 $R_{h\to w}^{default}$ 会把 PICO 的左右映射反掉，本项目用自定义

$$R_{h\to w}^{PiPER}=\begin{bmatrix}0&0&-1\\1&0&0\\0&1&0\end{bmatrix}$$

相对默认矩阵翻转第二行（$Y$ 轴映射），使"手往右移 → 末端往右移"。sim 与 real 共用同一份，保证 sim2real 一致。

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
