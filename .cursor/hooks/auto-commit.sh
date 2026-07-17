#!/usr/bin/env bash
# Cursor `stop` hook: agent 每轮改完停下时自动提交一个版本。
# 无改动则静默退出；提交失败也不阻塞 agent（fail open）。

set -u

root="$(git rev-parse --show-toplevel 2>/dev/null)" || exit 0
[ -n "$root" ] || exit 0
cd "$root"

# 没有任何改动（暂存区/工作区/未跟踪均干净）则直接退出。
if git diff --quiet --cached \
   && git diff --quiet HEAD -- \
   && [ -z "$(git ls-files --others --exclude-standard)" ]; then
  exit 0
fi

# 汇总变更路径（最多展示 5 个，其余用 +N more）。
mapfile -t changed < <(git status --porcelain | awk '{print $2}' | sort -u)
n=${#changed[@]}
preview=$(printf '%s, ' "${changed[@]:0:5}" | sed 's/,$//')
[ "$n" -gt 5 ] && preview="${preview}, +$((n-5)) more"

ts=$(date +%Y-%m-%d_%H:%M:%S)
git add -A
git commit -q -m "auto(${ts}): ${preview}" >/dev/null 2>&1 || true
exit 0
