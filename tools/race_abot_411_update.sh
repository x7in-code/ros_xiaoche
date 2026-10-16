#!/usr/bin/env bash
# =====================================================================
# race_abot_411_update.sh —— 在车上一条命令完成：拉代码 -> 修换行 -> 加权限 -> 编译
#
# 用法（在车上执行）：
#   bash ~/catkin_ws/src/race_abot_411/tools/race_abot_411_update.sh
#
# 会做的事：
#   1. git pull --ff-only            拉取队友最新代码
#   2. 检测并修正 CRLF 换行          防止 Windows 提交导致的 bad interpreter
#   3. chmod +x 脚本                 否则 rosrun 会报权限不足
#   4. catkin_make 并 source        让 ROS 重新注册这个包
#
# 若报 \r 错误： sed -i 's/\r$//' race_abot_411_update.sh
# =====================================================================
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$(dirname "$PKG_DIR")"
WS_DIR="$(dirname "$SRC_DIR")"

echo "======================================================"
echo " 包目录  : $PKG_DIR"
echo " src目录 : $SRC_DIR"
echo " 工作区  : $WS_DIR"
echo "======================================================"

if [ ! -f "$PKG_DIR/package.xml" ]; then
  echo "[!] 没找到 package.xml，脚本可能不在包内。请确认路径。"
  exit 1
fi

echo
echo "[1/4] git pull ..."
if [ -d "$PKG_DIR/.git" ]; then
  git -C "$PKG_DIR" pull --ff-only || echo "  [警告] git pull 失败（网络/冲突/未跟踪文件？），继续用当前代码"
else
  echo "  [跳过] 本目录不是 git 仓库"
fi

echo
echo "[2/4] 检查换行符 ..."
BAD=$(grep -rlI $'\r' "$PKG_DIR" --exclude-dir=.git 2>/dev/null || true)
if [ -n "$BAD" ]; then
  echo "$BAD" | while IFS= read -r f; do
    sed -i 's/\r$//' "$f"
    echo "  已修正 CRLF: ${f#$PKG_DIR/}"
  done
else
  echo "  全部 LF，无需修正"
fi

echo
echo "[3/4] 加可执行权限 ..."
chmod +x "$PKG_DIR"/scripts/*.py 2>/dev/null || true
chmod +x "$PKG_DIR"/tools/*.sh 2>/dev/null || true
ls -l "$PKG_DIR"/scripts/*.py | awk '{print "  "$1"  "$NF}'

echo
echo "[4/4] 编译 ..."
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
cd "$WS_DIR"
if ! catkin_make --pkg race_abot_411; then
  echo "  [回退] 单包编译失败，尝试全量 catkin_make"
  catkin_make
fi
# shellcheck disable=SC1091
source devel/setup.bash

echo
echo "======================================================"
echo " 完成。当前终端已 source 好，可直接："
echo "   roslaunch race_abot_411 race_abot_411_debug.launch"
echo " 自检："
echo "   rosrun race_abot_411 race_abot_411_diagnostics.py \\"
echo "     _params_yaml:=\$(rospack find race_abot_411)/config/race_abot_411_params.yaml \\"
echo "     _route_yaml:=\$(rospack find race_abot_411)/config/race_abot_411_route_a.yaml"
echo "======================================================"
