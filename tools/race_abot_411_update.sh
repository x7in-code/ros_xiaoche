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
# 只编译本包 + abot_driver：
#   车上 abot 包有并行编译竞争缺陷（abot_base.cpp 未声明依赖 riki_msgs 的消息生成），
#   裸 catkin_make -j 会随机失败；我们不需要它，绕开即可。
if ! catkin_make --pkg race_abot_411 abot_driver; then
  echo "  [回退] 单包编译失败，尝试串行全量 catkin_make -j1"
  catkin_make -j1
fi
# shellcheck disable=SC1091
source devel/setup.bash

echo
echo "  注意：catkin_make --pkg 会把「编译白名单」写进 CMake 缓存并且长期生效。"
echo "        以后想全量编译，先执行： catkin_make -DCATKIN_WHITELIST_PACKAGES=\"\""
if grep -qE '^CATKIN_WHITELIST_PACKAGES' "$WS_DIR/build/CMakeCache.txt" 2>/dev/null; then
  echo "        当前白名单：$(grep -E '^CATKIN_WHITELIST_PACKAGES' "$WS_DIR/build/CMakeCache.txt" | cut -d= -f2-)"
fi

echo
echo "======================================================"
echo " 完成。建议下一步："
echo "   换车体检 : bash $PKG_DIR/tools/race_abot_411_start.sh check"
echo "   一键启动 : bash $PKG_DIR/tools/race_abot_411_start.sh"
echo "   （不要用裸 roslaunch —— 用 start.sh 会连同 master/驱动/相机一起拉好，"
echo "     并生成 ~/race_abot_env.sh 供其他终端 source）"
echo "======================================================"
