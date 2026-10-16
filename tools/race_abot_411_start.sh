#!/usr/bin/env bash
# =====================================================================
# race_abot_411_start.sh —— 车上「一键启动 / 体检」（换车友好）
#
#   bash race_abot_411_start.sh            # 启动全部（前台跑主系统）
#   bash race_abot_411_start.sh check      # 只体检，不启动（换车第一步就跑它）
#   bash race_abot_411_start.sh stop       # 停掉全部
#
# 可覆盖的环境变量：
#   MODEL=/路径/xx.pt     指定权重（默认自动查找 aaabest.pt）
#   CAM_DEV=/dev/video2   指定相机设备（默认自动试）
#   USE_CAMERA=0          相机已在别处启动，跳过相机
#   MASTER=http://172.21.x.x:11311   连别人的 master（默认本机 127.0.0.1）
#   WS=~/catkin_ws        指定工作区（默认自动探测）
#
# 换车小贴士：不同车的 /dev/videoN、模型路径、工作区路径都可能不同，
#   本脚本会自己找；另外每个新终端跑一次  source ~/race_abot_env.sh  即可对齐环境。
# =====================================================================
set -u

MODE="${1:-start}"
LOGDIR=/tmp/race_abot_logs
ENVFILE="$HOME/race_abot_env.sh"
PORT=11311

# ---------------------------------------------------------------------
# 0) 自动探测工作区 / 环境
# ---------------------------------------------------------------------
if [ -z "${WS:-}" ]; then
  if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
    WS="$HOME/catkin_ws"
  else
    # 从本脚本位置反推：<WS>/src/race_abot_411/tools/xxx.sh
    _here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    _guess="$(cd "$_here/../../.." 2>/dev/null && pwd || echo "")"
    WS="${_guess:-$HOME/catkin_ws}"
  fi
fi

if [ -n "${MASTER:-}" ] && [ "$MASTER" != "http://127.0.0.1:$PORT" ]; then
  export ROS_MASTER_URI="$MASTER"
  export ROS_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
else
  export ROS_MASTER_URI="http://127.0.0.1:$PORT"
  export ROS_IP=127.0.0.1
fi

# ---------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------
kill_ours() {
  pkill -9 -f "roslaunch race_abot_411"      2>/dev/null
  pkill -9 -f "race_abot_411_node.py"        2>/dev/null
  pkill -9 -f "race_abot_411_sign_detect.py" 2>/dev/null
  pkill -9 -f "uvc_camera_node"              2>/dev/null
  pkill -9 -f "abot_driver"                  2>/dev/null
}

stop_all() {
  echo "[stop] 停掉我们的节点/驱动/相机 ..."
  kill_ours
  echo "[stop] 停掉 roscore ..."
  pkill -9 -f "rosmaster --core" 2>/dev/null
  pkill -9 -f "roscore"          2>/dev/null
  sleep 1
  echo "[stop] 完成。"
  exit 0
}

find_model() {
  local cands=(
    "$HOME/catkin_ws/src/traffic_detector/models/aaabest.pt"
    "$HOME/catkin_ws/src/traffic_detector/models/best.pt"
    "$HOME/catkin_ws/src/yolov8_ros/models/aaabest.pt"
    "$HOME/catkin_ws/src/yolov8_ros/models/best.pt"
    "/home/abot/yolov8/aaabest.pt"
    "/home/abot/yolov8/best.pt"
  )
  local c
  for c in "${cands[@]}"; do
    [ -f "$c" ] && { echo "$c"; return 0; }
  done
  # 兜底：在工作区里搜（aaabest 优先）
  local hit
  hit="$(ls -1 "$WS"/src/*/models/aaabest.pt 2>/dev/null | head -1)"
  [ -n "$hit" ] && { echo "$hit"; return 0; }
  hit="$(ls -1 "$WS"/src/*/models/best.pt 2>/dev/null | head -1)"
  [ -n "$hit" ] && { echo "$hit"; return 0; }
  echo ""
  return 1
}

list_cam_devs() {
  ls -1 /dev/video* 2>/dev/null
}

write_envfile() {
  cat > "$ENVFILE" <<EOF
# 由 race_abot_411_start.sh 生成 —— 其他终端执行 source ~/race_abot_env.sh
export ROS_MASTER_URI=$ROS_MASTER_URI
export ROS_IP=$ROS_IP
source /opt/ros/noetic/setup.bash
[ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash"
EOF
}

# =====================================================================
# check 模式：只体检
# =====================================================================
do_check() {
  echo "=============================================="
  echo " race_abot_411 换车体检（只读，不启动任何东西）"
  echo "=============================================="

  echo
  echo "[1] 系统与 ROS"
  echo "  架构      : $(uname -m)"
  echo "  系统      : $( (lsb_release -ds 2>/dev/null) || echo '未知')"
  if [ -f /opt/ros/noetic/setup.bash ]; then
    echo "  ROS       : Noetic ✓ (/opt/ros/noetic)"
  else
    echo "  ROS       : !! 没找到 /opt/ros/noetic —— 这车不是 Noetic？"
    ls -d /opt/ros/* 2>/dev/null | sed 's/^/              /'
  fi
  echo "  python3   : $(python3 -V 2>&1)"
  echo "  ROS_MASTER_URI = $ROS_MASTER_URI"
  echo "  ROS_IP         = $ROS_IP"
  echo "  本机 IP        : $(hostname -I 2>/dev/null)"
  if pgrep -f rosmaster >/dev/null 2>&1; then
    echo "  master    : 正在运行 ✓"
  else
    echo "  master    : 没在跑（正常，启动时会自动拉起）"
  fi

  echo
  echo "[2] 工作区"
  echo "  WS        = $WS"
  if [ -f "$WS/devel/setup.bash" ]; then
    echo "  devel/    : 存在 ✓（已编译过）"
  else
    echo "  devel/    : !! 不存在，需要 catkin_make"
  fi
  echo "  本包目录  : $(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  if [ -f "$WS/build/CMakeCache.txt" ]; then
    local wl
    wl="$(grep -E '^CATKIN_WHITELIST_PACKAGES' "$WS/build/CMakeCache.txt" 2>/dev/null | cut -d= -f2-)"
    if [ -n "$wl" ]; then
      echo "  !! 编译白名单生效中: '$wl'"
      echo "     只编译这些包，其他包不会被编译。恢复全量："
      echo "     catkin_make -DCATKIN_WHITELIST_PACKAGES=\"\""
    else
      echo "  编译白名单: 无（全量编译）✓"
    fi
  fi
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash 2>/dev/null || true
  # shellcheck disable=SC1091
  [ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash" 2>/dev/null
  local p
  for p in race_abot_411 abot_driver abot traffic_detector uvc_camera image_view; do
    if rospack find "$p" >/dev/null 2>&1; then
      echo "  包 $p : $(rospack find "$p")"
    else
      echo "  包 $p : !! 找不到"
    fi
  done

  echo
  echo "[3] 相机设备"
  if [ -n "$(list_cam_devs)" ]; then
    list_cam_devs | sed 's/^/  /'
  else
    echo "  !! /dev/video* 一个都没有 —— 相机没插好？"
  fi

  echo
  echo "[4] 模型权重（自动查找的顺序）"
  local m
  m="$(find_model)"
  if [ -n "$m" ]; then
    echo "  找到: $m  ($(du -h "$m" 2>/dev/null | cut -f1))"
  else
    echo "  !! 没找到任何 .pt —— 手动指定：MODEL=/路径/xx.pt ..."
  fi
  echo "  全盘 .pt（最多列 10 个）:"
  find "$HOME" /home -maxdepth 6 -name "*.pt" 2>/dev/null | head -10 | sed 's/^/    /'

  echo
  echo "[5] Python 依赖"
  python3 - <<'PY' 2>/dev/null || echo "  !! python3 检查失败"
import importlib
for m in ("rospy", "cv2", "cv_bridge", "numpy", "ultralytics", "yaml"):
    try:
        importlib.import_module(m)
        print("  %-12s OK" % m)
    except Exception as e:
        print("  %-12s !! %s" % (m, e))
PY

  echo
  echo "=============================================="
  echo " 体检完。红色(!!)项先解决，然后："
  echo "   bash $(basename "${BASH_SOURCE[0]}")"
  echo "=============================================="
  exit 0
}

[ "$MODE" = "check" ] && do_check
[ "$MODE" = "stop" ] && stop_all

# =====================================================================
# start 模式
# =====================================================================
echo "=============================================="
echo " race_abot_411 一键启动"
echo "=============================================="

echo
echo "[0/5] 加载 ROS 环境"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash 2>/dev/null || { echo "  !! 找不到 /opt/ros/noetic/setup.bash"; exit 1; }
if [ -f "$WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/devel/setup.bash"
else
  echo "  !! 找不到 $WS/devel/setup.bash，先编译：cd $WS && catkin_make --pkg race_abot_411 abot_driver"
  exit 1
fi
mkdir -p "$LOGDIR"
write_envfile
echo "  工作区 : $WS"
echo "  master : $ROS_MASTER_URI   ROS_IP=$ROS_IP"
echo "  其他终端对齐环境： source $ENVFILE"

echo
echo "[1/5] 清掉上一次的残留"
kill_ours
sleep 1

echo
echo "[2/5] 启动 roscore（后台）"
if pgrep -f rosmaster >/dev/null 2>&1; then
  echo "  已有 rosmaster 在跑，复用"
else
  nohup roscore > "$LOGDIR/roscore.log" 2>&1 &
  sleep 5
  if pgrep -f rosmaster >/dev/null 2>&1; then
    echo "  roscore 起来了 ✓  (日志 $LOGDIR/roscore.log)"
  else
    echo "  !! roscore 启动失败，请看 $LOGDIR/roscore.log"
    exit 1
  fi
fi
rosnode list >/dev/null 2>&1 && echo "  master 通信正常 ✓" || { echo "  !! 连不上 master"; exit 1; }

echo
echo "[3/5] 启动底盘驱动（后台）"
nohup roslaunch abot_driver abot_driver.launch > "$LOGDIR/driver.log" 2>&1 &
sleep 6
if rostopic info /akm_vel 2>/dev/null | grep -q "abot_driver"; then
  echo "  /akm_vel 有发布者 ✓（底盘串口通了）"
else
  echo "  !! /akm_vel 还没有发布者 —— 查串口 /dev/abotbase 与 $LOGDIR/driver.log"
fi

echo
echo "[4/5] 启动相机（后台）"
if [ "${USE_CAMERA:-1}" = "1" ]; then
  if [ -n "${CAM_DEV:-}" ]; then
    TRY_DEVS="$CAM_DEV"
  else
    TRY_DEVS="$(list_cam_devs)"
  fi
  if [ -z "$TRY_DEVS" ]; then
    echo "  !! /dev/video* 不存在，跳过相机"
  else
    CAM_OK=0
    for dev in $TRY_DEVS; do
      pkill -9 -f uvc_camera_node 2>/dev/null
      sleep 1
      echo "  试 $dev ..."
      nohup rosrun uvc_camera uvc_camera_node \
          _device:="$dev" _width:=640 _height:=480 _fps:=15 \
          image_raw:=/camera/rgb/image_raw camera_info:=/camera/rgb/camera_info \
          > "$LOGDIR/camera.log" 2>&1 &
      sleep 5
      if timeout 6 env PYTHONUNBUFFERED=1 rostopic hz /camera/rgb/image_raw 2>/dev/null \
           | grep -q "average rate"; then
        echo "  $dev 出图正常 ✓  (日志 $LOGDIR/camera.log)"
        CAM_OK=1
        break
      else
        echo "  $dev 没有画面，换一个试（详见 $LOGDIR/camera.log）"
      fi
    done
    [ "$CAM_OK" = "0" ] && echo "  !! 所有设备都没画面 —— 相机被别的程序占着？跑 sudo fuser -k /dev/video0 试试"
  fi
else
  echo "  (USE_CAMERA=0，跳过相机)"
fi

echo
echo "[5/5] 启动 race_abot_411（前台，Ctrl-C 停止本系统）"
MODEL="${MODEL:-$(find_model)}"
if [ -n "$MODEL" ] && [ -f "$MODEL" ]; then
  echo "  模型: $MODEL"
  MODEL_ARGS=(model_path:="$MODEL")
else
  echo "  模型: (未指定，交给节点自动查找 —— 日志里看『模型路径』和『模型类别』)"
  MODEL_ARGS=()
fi
echo "  相机话题: 自动挑选（也可 camera_sign:=/camera/xxx/image_raw 固定）"
echo "----------------------------------------------"
echo " 另开终端验证（先 source $ENVFILE）："
echo "   rostopic echo /race_abot_411/detections      # 拿标志对着相机看输出"
echo "   grep -nE '模型|类别|自检|ERROR' $LOGDIR/race.log | tail -20"
echo "----------------------------------------------"

roslaunch race_abot_411 race_abot_411_a.launch \
  "${MODEL_ARGS[@]}" \
  start_camera:=false \
  2>&1 | tee "$LOGDIR/race.log"
