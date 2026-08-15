#!/usr/bin/env python3
"""T19 판정 — motion/joint/speed 의 부호·단위. 값이 실제로 바뀐 구간으로만 차분한다."""
import json, sys, statistics as st

P = sys.argv[1]
K = "base"
rows = [json.loads(l) for l in open(P) if l.strip()]

def inner(d):
    while isinstance(d, dict) and "data" in d and isinstance(d["data"], dict):
        d = d["data"]
    return d

pos = [(r["t"], inner(r["d"]).get(K)) for r in rows if r["topic"] == "motion/joint/position"]
spd = [(r["t"], inner(r["d"]).get(K)) for r in rows if r["topic"] == "motion/joint/speed"]
pos = [p for p in pos if p[1] is not None]
spd = [s for s in spd if s[1] is not None]
print(f"position {len(pos)}건 · speed {len(spd)}건")
print(f"position 범위 {min(p[1] for p in pos):.4f} ~ {max(p[1] for p in pos):.4f}°")
print(f"speed    범위 {min(s[1] for s in spd):.6f} ~ {max(s[1] for s in spd):.6f}")
neg = sum(1 for s in spd if s[1] < 0)
print(f"speed 음수 표본: {neg} / {len(spd)}  ← 0 이면 **부호가 안 실린다**")

# 값이 바뀐 시각만 남긴다 (정지 구간 제거)
ch = []
for a, b in zip(pos, pos[1:]):
    if a[1] != b[1]:
        dt = b[0] - a[0]
        if dt > 0:
            ch.append((b[0], (b[1] - a[1]) / dt))       # (시각, 도/s)
print(f"\n값 변화 구간 {len(ch)}개 · 차분 범위 {min(c[1] for c in ch):+.4f} ~ {max(c[1] for c in ch):+.4f} °/s")

# speed 표본마다 가장 가까운 차분과 짝
pairs = []
for t, s in spd:
    near = min(ch, key=lambda c: abs(c[0] - t))
    if abs(near[0] - t) < 0.05 and (abs(near[1]) > 0.2 or abs(s) > 0.2):
        pairs.append((t, s, near[1]))
if not pairs:
    print("이동 표본 없음"); sys.exit()

pos_v = [(s, v) for _, s, v in pairs if v > 0]
neg_v = [(s, v) for _, s, v in pairs if v < 0]
print(f"\n짝지은 표본 {len(pairs)}개 — 차분 양수 {len(pos_v)} · 음수 {len(neg_v)}")
for name, grp in (("v>0 (정방향)", pos_v), ("v<0 (역방향)", neg_v)):
    if not grp:
        continue
    s_neg = sum(1 for s, _ in grp if s < 0)
    ratio = [abs(s) / abs(v) for s, v in grp if abs(v) > 1e-6]
    print(f"  {name}: n={len(grp)} · speed 음수 {s_neg}건 ({100*s_neg/len(grp):.1f}%)"
          f" · |s|/|v| 중앙 {st.median(ratio):.4f}")
    print(f"      |s| 중앙 {st.median([abs(s) for s,_ in grp]):.4f} · |v| 중앙 {st.median([abs(v) for _,v in grp]):.4f}")

print("\n판정 근거:")
print("  · v<0 구간에서 speed 가 음수면 → **부호 있음**")
print("  · v<0 구간에서도 speed 가 양수면 → **크기만 (부호 없음)**")
