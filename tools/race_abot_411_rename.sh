#!/usr/bin/env bash
# =====================================================================
# race_abot_411_rename.sh —— 把整包的 race_abot_411 前缀换成你们队名（共用小车防冲突）
#
# 用法：
#   bash race_abot_411_rename.sh                # 先干跑，只列出会改什么（不实际改）
#   bash race_abot_411_rename.sh --apply race_xiaoche   # 真正执行
#
# 做了什么：
#   1. 把所有文件名 / 目录名里的 race_abot_411 换成新前缀
#   2. 把所有文本文件内容里的 race_abot_411 换成新前缀
#   3. 最后把包目录名一起改掉
#
# 若报 \r 错误： sed -i 's/\r$//' race_abot_411_rename.sh
# =====================================================================
set -euo pipefail

OLD="race_abot_411"
APPLY=0
NEW=""

if [ "${1:-}" = "--apply" ]; then
  APPLY=1
  NEW="${2:-}"
else
  NEW="${1:-}"
fi

if [ "$APPLY" = "1" ] && [ -z "$NEW" ]; then
  echo "用法: bash race_abot_411_rename.sh --apply <新前缀>"; exit 1
fi
if [ -n "$NEW" ] && [ "$NEW" = "$OLD" ]; then
  echo "新前缀和旧的一样，无需改。"; exit 0
fi

# 定位包根目录（本脚本在 tools/ 下）
PKG_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARENT="$(dirname "$PKG_ROOT")"
echo "包目录: $PKG_ROOT"
[ -n "$NEW" ] && echo "新前缀: $NEW" || echo "（干跑模式，不会修改任何文件）"

# ---- 1) 文件名 ----
echo
echo "[1] 需要改名的文件/目录："
FILES=$(find "$PKG_ROOT" -depth -name "*${OLD}*" 2>/dev/null || true)
if [ -z "$FILES" ]; then echo "  （无）"; else echo "$FILES" | sed "s|$PARENT/||; s/^/  /"; fi

# ---- 2) 内容 ----
echo
echo "[2] 内容里含 ${OLD} 的文件："
GREP_FILES=$(grep -rl "$OLD" "$PKG_ROOT" 2>/dev/null || true)
if [ -z "$GREP_FILES" ]; then echo "  （无）"; else echo "$GREP_FILES" | sed "s|$PARENT/||; s/^/  /"; fi

if [ "$APPLY" != "1" ]; then
  echo
  echo "这是干跑。确认无误后执行："
  echo "  bash race_abot_411_rename.sh --apply ${NEW:-race_你的队名}"
  exit 0
fi

# ---- 3) 真正执行 ----
echo
echo "[3] 替换内容..."
while IFS= read -r f; do
  [ -z "$f" ] && continue
  sed -i "s/${OLD}/${NEW}/g" "$f"
done <<< "$GREP_FILES"

echo "[4] 重命名文件/目录..."
while IFS= read -r p; do
  [ -z "$p" ] && continue
  newpath="$(dirname "$p")/$(basename "$p" | sed "s/${OLD}/${NEW}/g")"
  mv "$p" "$newpath"
done <<< "$FILES"

echo "[5] 重命名包目录..."
NEW_ROOT="$(dirname "$PKG_ROOT")/$(basename "$PKG_ROOT" | sed "s/${OLD}/${NEW}/g")"
if [ "$NEW_ROOT" = "$PKG_ROOT" ]; then
  echo "  （目录名无需更改）"
elif [ -d "$PKG_ROOT" ]; then
  mv "$PKG_ROOT" "$NEW_ROOT"
else
  echo "  （目录已在第 4 步随文件名一起改过，跳过）"
fi

echo
echo "完成。新包目录: $NEW_ROOT"
echo "别忘了：重命名后删掉 build/ 与 devel/ 里旧的缓存再 catkin_make。"
