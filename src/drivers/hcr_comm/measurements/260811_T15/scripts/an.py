#!/usr/bin/env python3
"""mqtt_trace.jsonl 분석 — 발행 간격·도달오차·speed 대조. 읽기 전용."""
import json, sys, statistics as st

PATH = sys.argv[1]
KEYS = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]

rows = []
with open(PATH) as f:
    for line in f:
        line = line.strip()
        if line:
            rows.append(json.loads(line))

def inner(d):
    """봉투 data.data 두 겹을 벗긴다."""
    while isinstance(d, dict) and "data" in d and isinstance(d["data"], dict):
        d = d["data"]
    return d

here = [r for r in rows if r["topic"] == "move/joint/here"]
stop = [r for r in rows if r["topic"] == "move/stop"]
pos  = [(r["t"], inner(r["d"])) for r in rows if r["topic"] == "motion/joint/position"]
spd  = [(r["t"], inner(r["d"])) for r in rows if r["topic"] == "motion/joint/speed"]
evt  = [r for r in rows if r["topic"] == "event/motion"]

print(f"총 {len(rows)}건 / {len(set(r['topic'] for r in rows))}토픽")
print(f"move/joint/here = {len(here)}건 · move/stop = {len(stop)}건 · event/motion = {len(evt)}건")

# ── 발행 간격 (V-14 / §5 재발행 간격 ≥137ms) ────────────────────────────
if len(here) >= 2:
    ts = [r["t"] for r in here]
    gaps = [round((b - a) * 1000, 1) for a, b in zip(ts, ts[1:])]
    print(f"\n== move/joint/here 발행 간격 (ms) · n={len(gaps)} ==")
    print(f"  최소 {min(gaps)} · 중앙 {st.median(gaps)} · 평균 {round(st.mean(gaps),1)} · 최대 {max(gaps)}")
    print(f"  137ms 미만 위반: {sum(1 for g in gaps if g < 137)}건")
    print(f"  전체: {gaps}")
for r in here:
    d = inner(r["d"])
    ja = d.get("jointAngle") if isinstance(d, dict) else None
    print(f"  t={r['t']:8.3f}  {[round(x,4) for x in ja] if ja else d}")

# ── event/motion ────────────────────────────────────────────────────────
for r in evt:
    print(f"  event t={r['t']:8.3f}  {inner(r['d'])}")

# ── 자세 추이 ───────────────────────────────────────────────────────────
if pos:
    def vec(d):
        return [d.get(k) for k in KEYS]
    print(f"\n== motion/joint/position n={len(pos)} · 첫 {vec(pos[0][1])}")
    print(f"   마지막 {vec(pos[-1][1])}")
    for j, k in enumerate(KEYS):
        col = [p[1].get(k) for p in pos if p[1].get(k) is not None]
        if col:
            print(f"   {k:9s} p-p {max(col)-min(col):.6f}°  최종 {col[-1]:.6f}°")

# ── speed ↔ position 차분 대조 (T19) ────────────────────────────────────
if spd and len(pos) > 2:
    print(f"\n== T19 · motion/joint/speed n={len(spd)} ==")
    for k in KEYS:
        pairs = []
        for t, sd in spd:
            sv = sd.get(k)
            if sv is None:
                continue
            near = [p for p in pos if abs(p[0] - t) < 0.06]
            if len(near) < 2:
                continue
            a, b = near[0], near[-1]
            dt = b[0] - a[0]
            if dt <= 0:
                continue
            dv = (b[1].get(k, 0) - a[1].get(k, 0)) / dt   # 도/s
            if abs(dv) > 0.5 or abs(sv) > 0.5:
                pairs.append((sv, dv))
        if pairs:
            ratio = [s / d for s, d in pairs if abs(d) > 1e-6]
            sign = sum(1 for s, d in pairs if (s > 0) == (d > 0))
            print(f"   {k:9s} n={len(pairs):4d} · s/|v| 중앙 {st.median([abs(r) for r in ratio]):.5f}"
                  f" · 부호일치 {100*sign/len(pairs):.1f}%"
                  f" · |s|중앙 {st.median([abs(s) for s,_ in pairs]):.5f}"
                  f" · |v|중앙 {st.median([abs(d) for _,d in pairs]):.5f}")
        else:
            print(f"   {k:9s} 이동 표본 없음")
