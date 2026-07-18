#!/usr/bin/env bash
# Cursor `stop` hook: agent 每轮改完停下时自动提交一个版本。
# 无改动则静默退出；提交失败也不阻塞 agent（fail open）。
#
# 提交信息格式（conventional commit 风格）：
#   <type>(<scope>): <一句话摘要>  +<ins>/-<del>
#
#   <body: 完整 diffstat>
#
# type 由改动路径推断（docs/ -> docs, tests/ -> test, *.md -> docs, ...）。
# scope 取改动最多的顶层目录。摘要列出主要文件（最多 4 个）。

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

git add -A

# --- 推断 commit type ---
mapfile -t changed < <(git diff --cached --name-only | sort -u)
n=${#changed[@]}

guess_type() {
  local files=("$@")
  local has_py=0 has_md=0 has_test=0 has_doc=0 has_cfg=0 has_other=0
  for f in "${files[@]}"; do
    case "$f" in
      tests/*|*/test_*|*_test.py|conftest.py) has_test=1 ;;
      *.md|docs/*|README*|LICENSE*) has_md=1 ;;
      pyproject.toml|requirements.txt|Makefile|.gitignore|*.cfg|*.toml|*.sh|.cursor/*) has_cfg=1 ;;
      *.py) has_py=1 ;;
      *) has_other=1 ;;
    esac
  done
  # 优先级：纯测试 -> test；纯文档 -> docs；纯配置 -> chore；含 py -> feat/fix 不区分则 feat
  if [ "$has_py" = 0 ] && [ "$has_test" = 1 ]; then echo "test"; return; fi
  if [ "$has_py" = 0 ] && [ "$has_md" = 1 ] && [ "$has_other" = 0 ] && [ "$has_cfg" = 0 ]; then echo "docs"; return; fi
  if [ "$has_py" = 0 ] && [ "$has_cfg" = 1 ] && [ "$has_other" = 0 ]; then echo "chore"; return; fi
  if [ "$has_py" = 1 ]; then echo "feat"; return; fi
  echo "chore"
}

# --- 推断 scope（改动最多的顶层目录 / 文件名）---
scope=$(printf '%s\n' "${changed[@]}" \
  | awk -F/ '{print $1}' \
  | sort | uniq -c | sort -rn | awk 'NR==1{print $2}')
# 顶层就是文件名时，scope 用不带后缀的文件名
case "$scope" in
  *.md|*.toml|*.txt|*.sh|Makefile) scope="${scope%.*}";;
esac
[ -z "$scope" ] && scope="root"

ctype=$(guess_type "${changed[@]}")

# --- 增删行数统计 ---
read ins del < <(git diff --cached --numstat \
  | awk '{i+=$1; d+=$2} END {printf "%d %d", i+0, d+0}')
stat="+${ins}/-${del}"

# --- 摘要：主要文件（最多 4 个，其余 +N more）---
preview=$(printf '%s, ' "${changed[@]:0:4}" | sed 's/, *$//')
[ "$n" -gt 4 ] && preview="${preview}, +$((n-4)) more"

# --- body：完整 diffstat ---
body=$(git diff --cached --stat --no-color | sed 's/^/  /')

ts=$(date +%Y-%m-%d_%H:%M:%S)
subject="${ctype}(${scope}): ${preview}  ${stat}"

git commit -q -F - <<EOF >/dev/null 2>&1 || true
${subject}

${body}
EOF
exit 0
