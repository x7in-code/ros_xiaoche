#!/usr/bin/env bash
# =====================================================================
# race_abot_411_record.sh —— 录制调试数据（每跑一轮都录，出问题回放分析）
#
# 用法：
#   bash race_abot_411_record.sh lap1
#   DUR=120 bash race_abot_411_record.sh lap1        # 只录 120 秒
#   bash race_abot_411_record.sh lap1 /mnt/usb
#
# 若报 \r 错误： sed -i 's/\r$//' race_abot_411_record.sh
# =====================================================================
NAME="${1:-lap}"
OUTDIR="${2:-$HOME/bags}"
DUR="${DUR:-0}"

mkdir -p "$OUTDIR"

CANDIDATES=(
  /camera/rgb/image_raw
  /camera/rgb/image_raw/compressed
  /camera/right/image_raw
  /camera/back/image_raw
  /race_abot_411/detections
  /akm_vel
  /odom
  /battery
  /sonar
  /scan
)

TOPICS=()
for t in "${CANDIDATES[@]}"; do
  if rostopic list 2>/dev/null | grep -qx "$t"; then
    TOPICS+=("$t")
  fi
done

if [ ${#TOPICS[@]} -eq 0 ]; then
  echo "[!] 没找到候选话题，当前话题："
  rostopic list 2>/dev/null | head -40
  exit 1
fi

echo "录制话题："; printf '  %s\n' "${TOPICS[@]}"
echo "输出： $OUTDIR/$NAME.bag"

ARGS=(-O "$OUTDIR/$NAME" "${TOPICS[@]}")
[ "$DUR" != "0" ] && ARGS+=(--duration="$DUR")
exec rosbag record "${ARGS[@]}"
