#!/usr/bin/env python3
"""관측점에서 참 TCP 를 역산한다 — 4점법을 우리가 직접 다시 푸는 것.

모델
  펜 끝을 늘 **같은 고정점 P** 에 맞췄다. 컨트롤러가 등록값 t_reg 로 계산한 tcp 좌표는
  참값 t_true 와의 차이 d = t_reg − t_true 만큼 어긋난다. 그 어긋남은 flange 좌표계에
  붙어 있으므로 base 좌표계에서는 flange 회전에 따라 돈다:

      tcp_i = P + R_i · d          R_i = 자세 i 의 flange 회전행렬 (ZYX, L16 확정)

  미지수 P(3) + d(3) = 6. 관측 한 점이 방정식 3개를 준다 → 자세 3개면 9방정식 6미지수.
  ⚠️ 자세들의 **J6(툴축) 회전이 서로 달라야** d 의 xy 성분이 결정된다. J6 가 고정이면
     d 의 xy 가 P 와 완전 축퇴해 아무 값이나 답이 된다 — 이번 세션이 그것에 당했다.

  풀고 나면  t_true = t_reg − d.

회전 규약 ZYX: R = Rz(rz)·Ry(ry)·Rx(rx) — 업무목록 L16 (잔차 6.9e-16).
"""
import json, math, os, sys


def rot_zyx(rx, ry, rz):
    rx, ry, rz = map(math.radians, (rx, ry, rz))
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                cy * cx],
    ]


def solve(A, b):
    """가우스-조던. A 는 n×n, b 는 n."""
    n = len(A)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-12:
            raise SystemExit("특이 행렬 — 자세가 축퇴했다 (J6 를 안 흔들었을 가능성)")
        M[c], M[p] = M[p], M[c]
        pv = M[c][c]
        M[c] = [v / pv for v in M[c]]
        for r in range(n):
            if r != c and M[r][c] != 0:
                f = M[r][c]
                M[r] = [v - f * w for v, w in zip(M[r], M[c])]
    return [M[i][n] for i in range(n)]


def fit(rows, t_reg, name):
    n = len(rows)
    Rs = [rot_zyx(*r["flange_ori"]) for r in rows]
    ps = [r["tcp_pos"] for r in rows]

    # 정규방정식 AtA x = Atb,  x = [P(3); d(3)],  A_i = [I | R_i]
    AtA = [[0.0] * 6 for _ in range(6)]
    Atb = [0.0] * 6
    for R, p in zip(Rs, ps):
        # 좌상 I·I = I 누적
        for i in range(3):
            AtA[i][i] += 1.0
            Atb[i] += p[i]
        # 우상 I·R, 좌하 R^T·I
        for i in range(3):
            for j in range(3):
                AtA[i][3 + j] += R[i][j]
                AtA[3 + i][j] += R[j][i]
        # 우하 R^T R = I
        for i in range(3):
            AtA[3 + i][3 + i] += 1.0
        # Atb 아래쪽 = R^T p
        for i in range(3):
            Atb[3 + i] += sum(R[k][i] * p[k] for k in range(3))

    x = solve(AtA, Atb)
    P, d = x[:3], x[3:]
    t_true = [t_reg[i] - d[i] for i in range(3)]

    # 잔차
    res = []
    for R, p in zip(Rs, ps):
        pred = [P[i] + sum(R[i][j] * d[j] for j in range(3)) for i in range(3)]
        res.append(math.dist(pred, p))
    rms = math.sqrt(sum(r * r for r in res) / len(res))

    print(f"── {name}  (n={n}, J6 = " + " ".join(f"{r['joint'][5]:.1f}" for r in rows) + ")")
    print(f"   등록값 t_reg  = ({t_reg[0]:8.3f}, {t_reg[1]:8.3f}, {t_reg[2]:8.3f})")
    print(f"   오차   d      = ({d[0]:8.3f}, {d[1]:8.3f}, {d[2]:8.3f})   |d| = {math.hypot(d[0],d[1]):.3f} mm (xy)")
    print(f"   ★ 참값 t_true = ({t_true[0]:8.3f}, {t_true[1]:8.3f}, {t_true[2]:8.3f})")
    print(f"     편심 크기   = {math.hypot(t_true[0], t_true[1]):.3f} mm · 축길이 {t_true[2]:.3f} mm")
    print(f"   고정점 P      = ({P[0]:8.3f}, {P[1]:8.3f}, {P[2]:8.3f})")
    print(f"   적합 잔차 RMS = {rms:.3f} mm  (점별 " + " ".join(f"{r:.2f}" for r in res) + ")")
    print()
    return t_true


STORE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tcp_points.json")

USAGE = """사용법
  python3 solve_tcp.py <라벨> <tx> <ty> <tz> [슬라이스]

    <라벨>        tcp_points.json 의 라벨 (예: 검산D)
    <tx ty tz>    그 회차를 **찍을 당시 컨트롤러에 등록돼 있던** 값 (mm).
                  차이를 푸는 것이라 등록값이 무엇이었는지 반드시 맞아야 한다
    [슬라이스]    파이썬 슬라이스 문법 (예: 4: 또는 :3). 무효점 잘라낼 때

  2026-08-16 세션이 실제로 돌린 것 (아래 값은 각 회차의 당시 등록값이다)
    검산A 그룹2  -1.410 -4.270 120.300  4:      ← 펜던트 4점법 직후
    검산B         0.000  0.000 120.300          ← xy 를 0 으로 만든 잘못된 등록 시절
    검산C        -1.410 -4.270 120.300          ← 재티칭이 '적용' 안 돼 값이 안 바뀐 회차
    검산D        -3.675 -1.725 119.491          ← 수동 입력 직후. 여기서 편심이 확정됐다
"""

a = sys.argv[1:]
if len(a) < 4:
    raise SystemExit(__doc__ + "\n" + USAGE)

label, t_reg = a[0], tuple(float(v) for v in a[1:4])
rows = [r for r in json.load(open(STORE)) if r["label"] == label]
if len(a) >= 5:
    lo, _, hi = a[4].partition(":")
    rows = rows[int(lo) if lo else None: int(hi) if hi else None]
if len(rows) < 3:
    raise SystemExit(f"표본이 {len(rows)}개다 — 미지수 6개라 자세 3개 이상이 필요하다")

print("참 TCP 역산 — 관측에서 직접 푼다\n")
fit(rows, t_reg, label)
