#!/usr/bin/env bash
# ============================================================================
# 一键搭建 PICO 4 Ultra + MuJoCo + 松灵 PiPER 遥操作开发环境
#
# 本脚本完成：
#   1. 创建/复用 conda 环境 pico_teleop (Python 3.10)
#   2. 克隆 mujoco_menagerie（稀疏，仅 agilex_piper）到 third_party/
#   3. 克隆 XRoboToolkit-Teleop-Sample-Python 到 third_party/
#   4. 将 PiPER 的 MJCF 模型与网格复制到 assets/piper/（供 scene.xml 引用）
#   5. 下载并精简 PiPER URDF（供 placo 逆运动学，去除网格避免 package:// 解析）
#   6. 在 conda 环境中安装 xrobotoolkit_teleop 及其依赖（含 xrobotoolkit_sdk）
#
# 用法：
#   bash scripts/setup_env.sh
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_NAME="${PICO_TELEOP_ENV:-pico_teleop}"
THIRD_PARTY="${PROJECT_ROOT}/third_party"

# ---------- 颜色输出 ----------
info()  { echo -e "\033[0;34m[INFO]\033[0m  $*"; }
ok()    { echo -e "\033[0;32m[OK]\033[0m    $*"; }
warn()  { echo -e "\033[0;33m[WARN]\033[0m  $*"; }
err()   { echo -e "\033[0;31m[ERR]\033[0m   $*"; }

# ---------- 0. 检查 conda ----------
if [[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]]; then
  CONDA_SH="$HOME/miniconda3/etc/profile.d/conda.sh"
elif [[ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]]; then
  CONDA_SH="$HOME/anaconda3/etc/profile.d/conda.sh"
else
  err "未找到 conda 初始化脚本，请先安装 Miniconda/Anaconda。"; exit 1
fi
# shellcheck disable=SC1090
source "$CONDA_SH"

# ---------- 1. conda 环境 ----------
if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  ok "conda 环境 '$ENV_NAME' 已存在"
else
  info "创建 conda 环境 '$ENV_NAME' (Python 3.10) ..."
  conda create -n "$ENV_NAME" python=3.10 -y
fi
conda activate "$ENV_NAME"
ok "已激活 conda 环境 '$ENV_NAME' ($(python --version 2>&1))"

mkdir -p "$THIRD_PARTY"

# 匿名克隆公开仓库（禁用可能失效的凭证助手）
clone_public() { git -c credential.helper= clone "$@"; }

# ---------- 2. mujoco_menagerie (稀疏: agilex_piper) ----------
MENAGERIE_DIR="${THIRD_PARTY}/mujoco_menagerie"
if [[ -d "$MENAGERIE_DIR/.git" ]]; then
  ok "mujoco_menagerie 已存在，跳过克隆"
else
  info "稀疏克隆 mujoco_menagerie (仅 agilex_piper) ..."
  cd "$THIRD_PARTY"
  clone_public --filter=blob:none --no-checkout --depth=1 https://github.com/google-deepmind/mujoco_menagerie.git
  cd mujoco_menagerie
  git sparse-checkout init --cone
  git sparse-checkout set agilex_piper
  git -c credential.helper= checkout main
  ok "mujoco_menagerie 克隆完成"
fi

# ---------- 3. XRoboToolkit-Teleop-Sample-Python ----------
SAMPLE_DIR="${THIRD_PARTY}/XRoboToolkit-Teleop-Sample-Python"
if [[ -d "$SAMPLE_DIR/.git" ]]; then
  ok "XRoboToolkit-Teleop-Sample-Python 已存在，跳过克隆"
else
  info "克隆 XRoboToolkit-Teleop-Sample-Python ..."
  cd "$THIRD_PARTY"
  clone_public --depth=1 https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python.git
  ok "示例项目克隆完成"
fi

# ---------- 4. 复制 PiPER MJCF 模型与网格到 assets/piper/ ----------
PIPER_ASSETS="${PROJECT_ROOT}/assets/piper"
mkdir -p "$PIPER_ASSETS"
cp -f "${MENAGERIE_DIR}/agilex_piper/piper.xml" "$PIPER_ASSETS/piper.xml"
rm -rf "$PIPER_ASSETS/assets"
cp -r "${MENAGERIE_DIR}/agilex_piper/assets" "$PIPER_ASSETS/assets"
ok "已复制 PiPER MJCF 模型与网格到 assets/piper/"

# ---------- 5. 下载并精简 PiPER URDF ----------
URDF_OUT="${PIPER_ASSETS}/piper_description.urdf"
if [[ -s "$URDF_OUT" ]]; then
  ok "PiPER URDF 已存在，跳过下载"
else
  info "从 agilexrobotics/piper_ros 下载 piper_description.urdf ..."
  TMP_URDF="$(mktemp -d)/piper_description.urdf"
  if curl -fsSL -m 60 \
      "https://api.github.com/repos/agilexrobotics/piper_ros/contents/src/piper_description/urdf/piper_description.urdf?ref=noetic" \
      | python -c "import sys,json,base64; d=json.load(sys.stdin); sys.stdout.buffer.write(base64.b64decode(d['content']))" \
      > "$TMP_URDF"; then
    :
  else
    err "下载 URDF 失败，请检查网络或手动放置 piper_description.urdf"; exit 1
  fi
  python "${SCRIPT_DIR}/strip_urdf.py" "$TMP_URDF" "$URDF_OUT"
  ok "PiPER URDF 已精简并写入 $URDF_OUT"
fi

# ---------- 5.5 生成双臂 MJCF / URDF ----------
info "生成双臂模型 piper_dual.xml / piper_dual_description.urdf ..."
python "${SCRIPT_DIR}/generate_dual_arm.py"
ok "双臂模型已生成（桌面双 PiPER，right_/left_ 前缀）"

# ---------- 6. 安装 xrobotoolkit_teleop 及依赖 ----------
# 说明：官方 setup_conda.sh --install 内部会 `conda install -c conda-forge libstdcxx-ng / pybind11`，
# 在部分 conda 镜像配置下会把 CPython 替换为 GraalPy（导致原生扩展不可用）。
# 因此这里采用等价的手动流程：用 pip 装 pybind11、从源码编译 PXREARobotSDK、构建 Python 绑定，
# 完全绕开 conda 求解器。Ubuntu 22.04 自带的 libstdc++ 已满足 SDK 编译需求。
info "在 conda 环境 '$ENV_NAME' 中安装 xrobotoolkit_teleop（手动流程，GraalPy 安全）..."
warn "该步骤会编译 XRoboToolkit-PC-Service 的 C++ SDK，需要 cmake/g++ 且耗时较长。"

# 6.1 pybind11（pip，避免 conda 求解器）
pip install -q uv pybind11

# 获取仓库的通用函数：先试 git clone，失败则回退到 codeload tarball
fetch_repo() {
  local org_repo="$1" branch="${2:-main}" dest="$3"
  if [[ -d "$dest/.git" ]] || [[ -d "$dest" ]]; then
    ok "$dest 已存在，跳过"; return 0
  fi
  info "获取 $org_repo (branch=$branch) -> $dest"
  if git -c credential.helper= clone --depth=1 -b "$branch" "https://github.com/$org_repo.git" "$dest" 2>/dev/null; then
    return 0
  fi
  warn "git clone 失败，回退到 codeload tarball ..."
  local tmp_tar="$(mktemp -d)/repo.tar.gz"
  curl -fsSL -m 180 -o "$tmp_tar" "https://codeload.github.com/$org_repo/tar.gz/refs/heads/$branch" || {
    err "下载 $org_repo 失败，请检查网络"; return 1
  }
  local tmp_extract="$(mktemp -d)"
  tar xzf "$tmp_tar" -C "$tmp_extract"
  mv "$tmp_extract"/* "$dest"
  rm -rf "$tmp_extract" "$tmp_tar"
}

# 6.2 编译 PXREARobotSDK（C++ .so）
PCS_DIR="${THIRD_PARTY}/XRoboToolkit-PC-Service"
fetch_repo "XR-Robotics/XRoboToolkit-PC-Service" "main" "$PCS_DIR"
SDK_SRC="${PCS_DIR}/RoboticsService/PXREARobotSDK"
if [[ ! -f "${SDK_SRC}/build/libPXREARobotSDK.so" ]]; then
  info "编译 PXREARobotSDK ..."
  ( cd "$SDK_SRC" && bash build.sh ) || { err "PXREARobotSDK 编译失败"; exit 1; }
else
  ok "PXREARobotSDK 已编译，跳过"
fi
SO_PATH="${SDK_SRC}/build/libPXREARobotSDK.so"
[[ -f "$SO_PATH" ]] || SO_PATH="${PCS_DIR}/RoboticsService/SDK/linux/64/libPXREARobotSDK.so"
[[ -f "$SO_PATH" ]] || { err "未找到 libPXREARobotSDK.so"; exit 1; }

# 6.3 构建 xrobotoolkit_sdk（Python 绑定）
PYBIND_DIR="${THIRD_PARTY}/XRoboToolkit-PC-Service-Pybind"
fetch_repo "XR-Robotics/XRoboToolkit-PC-Service-Pybind" "main" "$PYBIND_DIR"
mkdir -p "${PYBIND_DIR}/lib"
cp -f "$SO_PATH" "${PYBIND_DIR}/lib/libPXREARobotSDK.so"
# include/ 已随仓库提供（PXREARobotSDK.h + nlohmann/），如缺则从 PC-Service 补
[[ -f "${PYBIND_DIR}/include/PXREARobotSDK.h" ]] || cp -f "${SDK_SRC}/PXREARobotSDK.h" "${PYBIND_DIR}/include/"
[[ -d "${PYBIND_DIR}/include/nlohmann" ]] || cp -r "${SDK_SRC}/nlohmann" "${PYBIND_DIR}/include/nlohmann"
info "构建并安装 xrobotoolkit_sdk ..."
( cd "$PYBIND_DIR" && pip uninstall -y xrobotoolkit_sdk >/dev/null 2>&1; python setup.py install ) || {
  err "xrobotoolkit_sdk 构建失败"; exit 1
}
ok "xrobotoolkit_sdk 安装完成"

# 6.4 安装 xrobotoolkit_teleop（含全部依赖：mujoco/placo/meshcat/tyro/torch 等）
info "安装 xrobotoolkit_teleop（editable，含依赖）..."
( cd "$SAMPLE_DIR" && uv pip install -e . ) || { err "xrobotoolkit_teleop 安装失败"; exit 1; }
ok "xrobotoolkit_teleop 安装完成"

# 6.5 安装本项目 piper_xr（editable，无依赖——依赖已由上一步装好）
info "安装本项目 piper_xr（editable）..."
( cd "$PROJECT_ROOT" && uv pip install -e . --no-deps ) || { err "piper_xr 安装失败"; exit 1; }
ok "piper_xr 安装完成"

cd "$PROJECT_ROOT"
echo
ok "========== 环境搭建完成 =========="
echo "  1. 启动 PC 端服务：  XRoboToolkit-PC-Service （或系统菜单启动）"
echo "  2. 在 PICO 头显中打开 XRoboToolkit 应用并连接 PC"
echo "  3. 运行遥操作（任选其一）："
echo "       conda activate $ENV_NAME"
echo "       python scripts/simulation/teleop_piper_mujoco.py"
echo "       # 或：python -m piper_xr"
echo "       # 或：piper-xr"
echo "  4. 无头验证（无需头显）：python tests/validate_piper_pipeline.py"
echo "==================================="
