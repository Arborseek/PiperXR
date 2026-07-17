"""项目路径工具：所有资源路径以本模块为基准推导，避免硬编码。"""

import os

# piper_pico/ -> 项目根目录
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ASSETS_DIR = os.path.join(PROJECT_ROOT, "assets")
PIPER_ASSETS_DIR = os.path.join(ASSETS_DIR, "piper")

# MuJoCo 场景（含 PiPER 模型与 piper_target mocap 体）
PIPER_SCENE_XML = os.path.join(PIPER_ASSETS_DIR, "scene.xml")
# PiPER 的 MJCF 模型（由 setup_env.sh 从 mujoco_menagerie 复制而来）
PIPER_MJCF_XML = os.path.join(PIPER_ASSETS_DIR, "piper.xml")
# 供 placo 逆运动学的精简 URDF（关节名与 MJCF 一致）
PIPER_URDF = os.path.join(PIPER_ASSETS_DIR, "piper_description.urdf")

THIRD_PARTY_DIR = os.path.join(PROJECT_ROOT, "third_party")
