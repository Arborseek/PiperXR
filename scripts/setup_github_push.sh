#!/usr/bin/env bash
# 一键配置 GitHub 推送（HTTPS Token 或 SSH 公钥）
set -euo pipefail

REPO="https://github.com/Arborseek/PiperXR.git"
SSH_REPO="git@github.com:Arborseek/PiperXR.git"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

cd "$ROOT"

echo "==> 1. 配置 remote"
git remote set-url origin "$REPO"

if [[ -n "${GITHUB_TOKEN:-}" ]]; then
  echo "==> 2. 用 GITHUB_TOKEN 写入凭据"
  LOGIN=$(curl -s -H "Authorization: Bearer $GITHUB_TOKEN" https://api.github.com/user \
    | python3 -c "import sys,json; print(json.load(sys.stdin).get('login',''))")
  if [[ -z "$LOGIN" ]]; then
    echo "ERROR: Token 无效"; exit 1
  fi
  echo "    Token 账号: $LOGIN"
  printf 'protocol=https\nhost=github.com\n' | git credential reject 2>/dev/null || true
  printf "protocol=https\nhost=github.com\nusername=$LOGIN\npassword=$GITHUB_TOKEN\n\n" \
    | git credential approve
  echo "==> 3. 推送"
  if git push -u origin master; then
    echo "OK: 推送成功 (HTTPS)"
    exit 0
  fi
  echo "WARN: HTTPS 推送失败（Token 可能无写权限），尝试 SSH..."
fi

echo "==> 2. 尝试 SSH 推送"
git remote set-url origin "$SSH_REPO"
if ssh -T -o BatchMode=yes git@github.com 2>&1 | grep -qi 'successfully authenticated'; then
  git push -u origin master
  echo "OK: 推送成功 (SSH)"
  exit 0
fi

echo ""
echo "SSH 未配置。请把下面公钥加到 GitHub → Settings → SSH keys："
echo "https://github.com/settings/ssh/new"
echo ""
cat ~/.ssh/id_ed25519.pub 2>/dev/null || ssh-keygen -t ed25519 -C "$(whoami)@github" -f ~/.ssh/id_ed25519 -N "" -q && cat ~/.ssh/id_ed25519.pub
echo ""
echo "加完后运行: git push -u origin master"
echo ""
echo "或者重新生成 Token（必须勾选 repo 写权限 / Contents: Read and write）："
echo "https://github.com/settings/tokens"
echo "然后: GITHUB_TOKEN=ghp_xxx bash scripts/setup_github_push.sh"
