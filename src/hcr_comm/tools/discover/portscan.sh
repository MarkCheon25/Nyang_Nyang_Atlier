#!/usr/bin/env bash
# TCP 커넥트 스캔 — 루트 불필요 (bash /dev/tcp)
# 사용법: portscan.sh <host> <start> <end> [parallel]
HOST="${1:-192.168.0.20}"
START="${2:-1}"
END="${3:-65535}"
PAR="${4:-300}"

scan_one() {
  local p=$1
  if timeout 1 bash -c "exec 3<>/dev/tcp/$HOST/$p" 2>/dev/null; then
    echo "OPEN $p"
  fi
}
export -f scan_one
export HOST

seq "$START" "$END" | xargs -P "$PAR" -I{} bash -c 'scan_one {}'
