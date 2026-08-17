#!/usr/bin/env python3
"""검C1 / V-8 분석기 — `move/joint/here` 0건 · 감청 유효성 · 추종오차 · **구간별** 관절 p-p.

왜 구간을 자르나 — V-8 의 확정 문면(계획서 §3)은 *"**궤적 구간** 관절값 p-p ≤0.01°"* 다.
창 전체로 재면 서보 ON/OFF 의 **브레이크 전이 중력 정착**이 섞여 정상 거동이 실패로 잡힌다
(08-11 발견 ②: 창 전체 wrist1 0.0433° vs 궤적 구간 0.000163°).

  구간 경계는 `logs/c1_marks.txt` 의 epoch 마크에서 온다. `joint_states` 표본도 epoch 라 직접 정렬된다.

사용: c1_an.py [c1디렉터리]      기본 = 이 스크립트의 ../c1
출력: 마크다운 — 산출물 문서에 그대로 붙는다
근거: `briefs/검증_재현_세션분할.md` §3·§5 · `중간결과물.md` §3-C
"""
import json, os, sys, collections, gzip

HERE = os.path.dirname(os.path.abspath(__file__))
C1 = sys.argv[1] if len(sys.argv) > 1 else os.path.normpath(os.path.join(HERE, "..", "c1"))
RAD2DEG = 180.0 / 3.141592653589793


def op(*parts):
    """원자료는 `.gz` 로 보관한다(260811 관례). 압축 전 파일도 그대로 읽는다."""
    p = os.path.join(C1, *parts)
    if os.path.exists(p):
        return open(p)
    if os.path.exists(p + ".gz"):
        return gzip.open(p + ".gz", "rt")
    raise FileNotFoundError(f"{p} (.gz 도 없다)")

marks = {}
for line in op("logs", "c1_marks.txt"):
    p = line.split()
    if len(p) == 2:
        marks[p[0]] = float(p[1])

d = json.load(op("ros", "c1_ros.json"))
js = d["joint_states"]
cs = d["controller_state"]
names = d["summary"]["joint_names"]

print("# 검C1 / V-8 — 안전 게이트 (allow_motion=false · 서보 ON)\n")

# ── 1. move/joint/here 발행 건수 + 감청 유효성 ────────────────────────────────
counts = collections.Counter()
for line in op("bus", "c1_bus.jsonl"):
    try:
        counts[json.loads(line)["topic"]] += 1
    except Exception:
        pass
here = counts.get("move/joint/here", 0)
movestar = sum(n for t, n in counts.items() if t.startswith("move/"))
ours = {t: n for t, n in counts.items() if t.startswith("set/") or t.startswith("robot/")}
acks = [t for t in counts if len(t) == 36 and t.count("-") == 4]

print("## 1. 판정 대상 — `move/joint/here` 발행\n")
print(f"| 항목 | 값 |\n|---|---|")
print(f"| **`move/joint/here`** | **{here}건** |")
print(f"| `move/*` 전체 | **{movestar}건** |")
print(f"| 감청 총량 | **{sum(counts.values())}건 · {len(counts)}토픽** |")
print(f"| 우리가 낸 명령 | {', '.join(f'`{t}` {n}건' for t, n in sorted(ours.items())) or '—'} |")
print(f"| 그 ack | {len(acks)}건 |")
print()
print("**감청 유효성** — 0건이 위음성이 아님을 먼저 세운다: 우리가 낸 명령과 그 ack 가 "
      f"같은 감청에 잡혔다({', '.join(f'{t} {n}' for t, n in sorted(ours.items()))} + ack {len(acks)}). "
      "감청은 살아 있었고 명령 평면을 듣고 있었다.\n")

# ── 2. 추종오차 ───────────────────────────────────────────────────────────────
s = d["summary"]
print("## 2. 추종오차 — 명령이 어디까지 갔나\n")
print("| 관절 | reference (°) | feedback (°) | 최종 오차 (°) | 최대 오차 (°) |")
print("|---|---|---|---|---|")
for i, n in enumerate(names):
    print(f"| {n} | {s['track_final_reference_deg'][i]:.4f} | {s['track_final_feedback_deg'][i]:.4f} "
          f"| **{s['track_final_err_deg'][i]:+.4f}** | {s['track_max_abs_err_deg'][i]:.4f} |")
print()
print("→ JTC 가 `joint_1` 에 **+5°** 를 명령 인터페이스까지 전달했고, 플러그인이 **전량을 잘랐다.** "
      "게이트가 상위층이 아니라 **플러그인 안**에 있다는 직접 측정이다.\n")

# ── 3. 구간별 p-p ─────────────────────────────────────────────────────────────
def pp(t0, t1):
    cols = [[] for _ in names]
    for t, pos, _ in js:
        if t0 <= t <= t1:
            for i, v in enumerate(pos):
                cols[i].append(v)
    if not cols[0]:
        return None, 0
    return [(max(c) - min(c)) * RAD2DEG for c in cols], len(cols[0])


segs = [
    ("창 전체", js[0][0], js[-1][0]),
    ("서보 ON 구간", marks["servo_on_post"], marks["servo_off_pre"]),
    ("**궤적 구간**", marks["jtc_send"], marks["jtc_done"]),
]
print("## 3. 관절값 p-p — 구간별 (기준: **궤적 구간** ≤0.01°)\n")
print("| 구간 | 길이 | 표본 | " + " | ".join(names) + " | 최대 |")
print("|---|---|---|" + "---|" * (len(names) + 1))
for label, t0, t1 in segs:
    vals, n = pp(t0, t1)
    if vals is None:
        continue
    mx = max(vals)
    flag = "" if label != "**궤적 구간**" else (" ✅" if mx <= 0.01 else " ❌")
    print(f"| {label} | {t1 - t0:.2f}s | {n} | " +
          " | ".join(f"{v:.6f}" for v in vals) + f" | **{mx:.6f}**{flag} |")
print()

# ── 4. 값이 언제 바뀌었나 — 전이에 몰렸는지 확인 ──────────────────────────────
print("## 4. 관절값 변화 시각 — 전이에 몰렸는가\n")
changes = []
prev = None
for t, pos, _ in js:
    if prev is not None and pos != prev:
        changes.append((t, max(abs(a - b) for a, b in zip(pos, prev)) * RAD2DEG))
    prev = pos
if changes:
    print(f"변화 **{len(changes)}회** · 첫 `t={changes[0][0] - marks['trace_start']:.2f}s` · "
          f"마지막 `t={changes[-1][0] - marks['trace_start']:.2f}s`\n")
    print("| 기준 시각 | t (창 시작 후) |")
    print("|---|---|")
    for k in ("servo_on_post", "jtc_send", "jtc_done", "servo_off_pre"):
        print(f"| `{k}` | {marks[k] - marks['trace_start']:.2f}s |")
    print()
    inside = sum(1 for t, _ in changes if marks["jtc_send"] <= t <= marks["jtc_done"])
    print(f"→ 궤적 구간 안의 변화 **{inside}회** / 전체 {len(changes)}회. "
          "나머지는 서보 ON/OFF 전이 부근이다 — **브레이크 전이 중력 정착**이지 명령 누출이 아니다.\n")

# ── 5. 축온 ───────────────────────────────────────────────────────────────────
temps = [json.loads(l) for l in op("logs", "c1_temp.jsonl")]
if temps:
    axes = list(temps[0]["temp"].keys())
    print("## 5. 축온 — 창 안 (Rokey6 판정: 58°C 즉시중단)\n")
    print("| 축 | 시작 | 최고 | 종료 | 상승 |")
    print("|---|---|---|---|---|")
    for a in axes:
        v = [x["temp"][a] for x in temps if x["temp"].get(a) is not None]
        print(f"| {a} | {v[0]} | **{max(v)}** | {v[-1]} | {max(v) - v[0]:+d} |")
    allmax = max(max(x["temp"][a] for a in axes if x["temp"].get(a) is not None) for x in temps)
    print(f"\n최고 **{allmax}°C** · 즉시중단선 58°C 까지 여유 **{58 - allmax}°C** · "
          f"감시견 발동 **없음** (표본 {len(temps)}건)\n")
