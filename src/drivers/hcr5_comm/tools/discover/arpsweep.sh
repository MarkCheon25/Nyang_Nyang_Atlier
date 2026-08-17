#!/usr/bin/env bash
# ARP 스윕 — 루트 불필요. 커널이 ping 시도 시 ARP를 보내므로,
# ICMP가 필터링돼 있어도 살아있는 호스트는 이웃 캐시에 남는다.
# 사용법: arpsweep.sh <prefix, 예: 192.168.1> <our_ip_last_octet> [NIC]
PREFIX="${1:-192.168.1}"
SELF="${2:-100}"
# NIC 이름은 PC마다 다르다 — 04pc=enp3s0 · 03pc=enp131s0 (2026-08-15 인자화).
IFACE="${3:-${IFACE:-enp3s0}}"

ip neigh flush dev "$IFACE" 2>/dev/null

for i in $(seq 1 254); do
  [ "$i" = "$SELF" ] && continue
  ping -c1 -W1 -n -I "$IFACE" "$PREFIX.$i" >/dev/null 2>&1 &
done
wait

echo "=== $PREFIX.0/24 이웃 캐시 (FAILED/INCOMPLETE 제외) ==="
found=$(ip neigh show dev "$IFACE" | grep -viE "FAILED|INCOMPLETE")
if [ -n "$found" ]; then
  echo "$found"
else
  echo "(응답 호스트 없음)"
fi
