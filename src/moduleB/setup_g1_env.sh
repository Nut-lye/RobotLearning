#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODULE_DIR="$ROOT_DIR/src/moduleB"
THIRD_PARTY_DIR="$ROOT_DIR/third_party"
SDK_DIR="$THIRD_PARTY_DIR/unitree_sdk2_python"
SDK2_SRC_DIR="$THIRD_PARTY_DIR/unitree_sdk2"
MUJOCO_REPO_DIR="$THIRD_PARTY_DIR/unitree_mujoco"

ensure_git_repo() {
  local label="$1"
  local repo_url="$2"
  local repo_dir="$3"
  shift 3

  local needs_download=0
  local missing_paths=()

  if [ ! -d "$repo_dir/.git" ]; then
    needs_download=1
    missing_paths+=(".git")
  fi

  for relative_path in "$@"; do
    if [ ! -e "$repo_dir/$relative_path" ]; then
      needs_download=1
      missing_paths+=("$relative_path")
    fi
  done

  if [ "$needs_download" -eq 0 ]; then
    echo "  [OK] $label 已存在，尝试更新"
    git -C "$repo_dir" pull --ff-only
    return
  fi

  echo "  [缺失] $label 不完整：${missing_paths[*]}"

  if [ -e "$repo_dir" ]; then
    local backup_dir="${repo_dir}.bak.$(date +%Y%m%d%H%M%S)"
    echo "  备份不完整目录：$backup_dir"
    mv "$repo_dir" "$backup_dir"
  fi

  echo "  下载 $label -> $repo_dir"
  git clone "$repo_url" "$repo_dir"
}

echo "[1/4] 检查 Python 环境"
python3 --version
python3 -m pip --version >/dev/null

echo "[2/4] 安装模块 B Python 依赖"
python3 -m pip install --user -r "$MODULE_DIR/requirements.txt"

echo "[3/4] 检查并补齐 third_party 依赖"
mkdir -p "$THIRD_PARTY_DIR"
ensure_git_repo \
  "unitree_sdk2_python" \
  "https://github.com/unitreerobotics/unitree_sdk2_python.git" \
  "$SDK_DIR" \
  "setup.py" \
  "unitree_sdk2py"

ensure_git_repo \
  "unitree_sdk2" \
  "https://github.com/unitreerobotics/unitree_sdk2.git" \
  "$SDK2_SRC_DIR" \
  "CMakeLists.txt" \
  "include/unitree/robot/g1"

ensure_git_repo \
  "unitree_mujoco" \
  "https://github.com/unitreerobotics/unitree_mujoco.git" \
  "$MUJOCO_REPO_DIR" \
  "simulate/CMakeLists.txt" \
  "unitree_robots/g1/scene_29dof.xml"

echo "[4/4] 安装 Unitree Python SDK"
python3 -m pip install --user "$SDK_DIR"

echo
echo "模块 B 环境准备完成。"
echo "本地演练：python3 src/moduleB/g1_action_interface.py --dry-run --actions \"站立,鼓掌,释放手臂\""
echo "连接 G1： python3 src/moduleB/g1_action_interface.py --iface <网卡名> --actions \"start,clap,release arm\""
