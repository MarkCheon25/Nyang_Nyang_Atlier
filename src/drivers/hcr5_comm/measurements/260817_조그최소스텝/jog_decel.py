#!/usr/bin/env python3
"""jog/stop 직후 감속 프로파일만 좁게 본다 (앞 스크립트의 정지판정 오염을 피해서).

앞 버전은 '연속 표본이 완전 고정' 을 정지로 봤는데, 조그 직후 다른 동작이 이어진
회차에서 훨씬 뒤의 정지를 잡아 stop후 Δ 가 52° 로 나왔다. 여기서는 stop 시각 기준
±0.5초 창의 속도 곡선을 그대로 찍어, 감속이 실제로 언제 끝나는지 눈으로 가른다.
"""
import csv, json, sys

CSV = "/home/markch03/hcr5_logs/_260816_실기1차/hcr5_20260816-073510.csv"
KEYS = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")
csv.field_size_limit(10**7)

pos, cmds = [], []
with open(CSV, newline="") as f:
    for row in csv.DictReader(f):
        t = row["topic"]
        if t == "motion/joint/position":
            try:
                d = json.loads(row["payload"])["data"]["data"]
            except Exception:
                continue
            pos.append((float(row["t_rel"]), [float(d[k]) for k in KEYS]))
        elif t in ("jog/start", "jog/stop"):
            cmds.append((float(row["t_rel"]), t))

def idx_at(t):
    lo, hi = 0, len(pos)
    while lo < hi:
        mid = (lo + hi) // 2
        if pos[mid][0] < t: lo = mid + 1
        else: hi = mid
    return lo

stops = [t for t, k in cmds if k == "jog/stop"]
print(f"jog/stop {len(stops)}회 — 각 stop 전후 속도(°/s, 6축 최대)\n")

for n, ts in enumerate(stops, 1):
    i = idx_at(ts)
    print(f"── stop #{n}  t_rel={ts:.3f}s")
    lo, hi = max(1, i - 6), min(len(pos) - 1, i + 22)
    slid_deg = 0.0
    prev_t = None
    for j in range(lo, hi):
        dt = pos[j][0] - pos[j - 1][0]
        if dt <= 0:
            continue
        dv = max(abs(pos[j][1][k] - pos[j - 1][1][k]) for k in range(6))
        rel = (pos[j][0] - ts) * 1000.0
        mark = "  ← stop 발행" if prev_t is not None and prev_t < ts <= pos[j][0] else ""
        if rel >= 0:
            slid_deg += dv
        print(f"   {rel:+8.1f}ms  Δ{dv:7.4f}°  {dv/dt:8.2f}°/s{mark}")
        prev_t = pos[j][0]
    print(f"   → stop 이후 누적 이동 {slid_deg:.4f}°\n")
