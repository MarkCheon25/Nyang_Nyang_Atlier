#!/usr/bin/env python3
"""컨트롤러 활성 툴 오프셋 측정기 — 로봇을 **전혀 움직이지 않는다**.

원리
  FK RPC(`robot/convertPose`)는 `poseType` 을 "tcp" / "flange" 둘 다 받는다(2026-08-16 확인).
  같은 관절각에 대해 두 번 부르면 컨트롤러가 **자기가 알고 있는 툴 오프셋**을 적용한 결과와
  적용 안 한 결과를 나란히 준다. 그 차이가 곧 등록된 툴이다.

  차이를 그냥 빼면 자세마다 다른 값이 나온다(base 좌표계라서). 플랜지 자세의 회전을 벗겨야
  **자세 무관 상수**가 된다:

      t = R_flange^T · (p_tcp − p_flange)          ← 이 t 가 툴 오프셋 (flange 좌표계, mm)

  여러 자세에서 t 가 같으면 측정이 맞은 것이다. 흩어지면 회전 규약이나 poseType 해석을 의심한다.

  회전 규약은 ZYX 오일러 `R = Rz(rz)·Ry(ry)·Rx(rx)` — 업무목록 L16 에서 확정(잔차 6.9e-16).

쓰임
  · 등록 **전**: t 가 (0,0,0) 이어야 한다. 아니면 활성 툴이 이미 있다(업무목록 L14).
  · 등록 **후**: t 가 4점법이 넣은 값과 같아야 한다. 캘리퍼 실측과도 대조한다.
  · `poseType` 이 실제로 갈리는지도 여기서 판명된다 — 등록 후에도 t=0 이면 컨트롤러가
    poseType 을 무시하고 늘 같은 것을 주는 것이다(그러면 이 도구는 못 쓴다).

사용법
  python3 tool_probe.py                # 기본 자세 묶음
  python3 tool_probe.py --json out.json
"""
import sys, os, json, math, argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mqtt_cmd as M

# ── 표본 자세 — 로봇은 안 움직인다. FK 계산용 관절각일 뿐이다 ──────────────────
# 회전 성분까지 분리하려면 자세가 다양해야 한다. ry≠0 자세를 반드시 넣는다
# (L16 함정: 홈은 ry=0 이라 ZYX 와 ZXY 가 동점이 된다).
POSES = [
    [0, -90, -90, -90, 90, 0],              # 홈
    [0.0584, -89.981, -90.277, -89.785, 90, -9.937],   # 현재 파킹 자세
    [30, -70, -60, -80, 55, 25],            # L16 이 쓴 ry≠0 자세
    [-45, -100, -70, -60, 120, -30],
    [15, -60, -110, -120, 80, 60],
    [60, -80, -50, -95, 40, -75],
    [-20, -120, -95, -45, 100, 15],
    [0, -90, -90, -90, 90, 90],             # 홈에서 J6 만 90°
]


def rot_zyx(rx, ry, rz):
    """ZYX 오일러 → 회전행렬. R = Rz(rz)·Ry(ry)·Rx(rx). 입력은 도."""
    rx, ry, rz = map(math.radians, (rx, ry, rz))
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                cy * cx],
    ]


def rt_mul(R, v):
    """R^T · v"""
    return [sum(R[k][i] * v[k] for k in range(3)) for i in range(3)]


def fk(m, joints, pose_type):
    """robot/convertPose — 읽기 전용. 로봇은 움직이지 않는다."""
    b = M.ack_body(M.cmd(m, "robot/convertPose",
                         {"info": {"joint": list(joints)}, "poseType": pose_type},
                         ack_timeout=5))
    if not b or b.get("code") != 0:
        raise SystemExit(f"FK 실패 (poseType={pose_type}, joint={joints}): {b}")
    d = b["data"]
    p, o = d["position"], d["orientation"]
    return [p["x"], p["y"], p["z"]], [o["x"], o["y"], o["z"]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", help="결과를 JSON 으로 저장할 경로")
    args = ap.parse_args()

    m = M.Mqtt(M.HOST, M.PORT)
    rows = []
    try:
        for j in POSES:
            p_t, o_t = fk(m, j, "tcp")
            p_f, o_f = fk(m, j, "flange")
            dp = [p_t[i] - p_f[i] for i in range(3)]
            do = [o_t[i] - o_f[i] for i in range(3)]
            t = rt_mul(rot_zyx(*o_f), dp)          # flange 좌표계로 되돌린 오프셋
            rows.append({"joint": j, "tcp": p_t, "flange": p_f,
                         "d_base": dp, "d_ori": do, "t_flange": t})
    finally:
        m.close() if hasattr(m, "close") else None

    print(f"{'관절각(도)':<44} {'|Δ|mm':>8}   툴 오프셋 t (flange 좌표계, mm)")
    print("-" * 100)
    for r in rows:
        j = " ".join(f"{v:6.1f}" for v in r["joint"])
        n = math.dist(r["tcp"], r["flange"])
        t = r["t_flange"]
        print(f"{j:<44} {n:8.4f}   x={t[0]:9.4f} y={t[1]:9.4f} z={t[2]:9.4f}")

    # ── 일관성 — t 가 자세 무관 상수여야 한다 ────────────────────────────────
    ts = [r["t_flange"] for r in rows]
    mean = [sum(t[i] for t in ts) / len(ts) for i in range(3)]
    spread = [max(t[i] for t in ts) - min(t[i] for t in ts) for i in range(3)]
    norm = math.sqrt(sum(v * v for v in mean))
    print("-" * 100)
    print(f"평균 t = ({mean[0]:.4f}, {mean[1]:.4f}, {mean[2]:.4f}) mm   |t| = {norm:.4f} mm")
    print(f"자세간 산포(최대−최소) = ({spread[0]:.4f}, {spread[1]:.4f}, {spread[2]:.4f}) mm")

    dori = [max(abs(r["d_ori"][i]) for r in rows) for i in range(3)]
    print(f"자세 차분 최대 |Δrx|={dori[0]:.4f} |Δry|={dori[1]:.4f} |Δrz|={dori[2]:.4f} 도")

    print()
    if norm < 1e-6:
        print("판정: 활성 툴 오프셋 = 0 (또는 컨트롤러가 poseType 을 무시한다).")
        print("      등록 후에도 이 값이 0 이면 poseType 해석이 갈리지 않는다는 뜻 —")
        print("      그때는 이 도구 대신 get/command/pos 차분만 쓸 수 있다.")
    elif max(spread) < 0.01:
        print(f"판정: 툴 오프셋이 자세 무관 상수다 (산포 < 0.01mm). t = {norm:.3f} mm 로 확정.")
    else:
        print("판정: ⚠️ t 가 자세마다 흔들린다. 회전 규약(ZYX) 또는 poseType 해석을 의심하라.")

    if args.json:
        with open(args.json, "w") as f:
            json.dump({"rows": rows, "mean": mean, "spread": spread, "norm": norm}, f, indent=2)
        print(f"\n저장: {args.json}")


if __name__ == "__main__":
    main()
