#!/usr/bin/env python3
"""P2 B3 — IK 실패율 스윕. **검사 대상 스크립트(`pose_goal.sh`)를 그대로 호출한다** (별도 경로로 재구현하지 않는다).

표본 설계 — 지시서 §6-B3 *"작업공간 안에서 고르게. 홈 근처만 찍으면 실패율이 낙관적으로 나온다"*:
  base 원점 기준 **구면 좌표**로 r × (방위각, 고도각) 격자. r 을 HCR-5 도달반경(≈915mm) 안팎으로
  걸쳐 **경계를 일부러 넘긴다** — 그래야 실패가 '경계'인지 'IK 품질'인지 error_code 로 갈린다.

사용: sweep.py <출력접두사> [--ori|--noori] [--tol POS_TOL]
"""
import math, subprocess, sys, json, os, time

# 자기 위치가 곧 VERIFY 다 — clone 경로가 PC마다 달라도 맞는다 (2026-08-15 이식).
VERIFY = os.environ.get("VERIFY") or os.path.dirname(os.path.abspath(__file__))
SH = os.path.join(VERIFY, "pose_goal.sh")

# 홈 자세 flange 자세(펜이 바닥을 향하는 자세) — TF 전정밀도 실측값
QUAT = ("0.707106666", "0.0", "0.707106897", "0.000000231")

# MoveItErrorCodes — 실패 사유를 갈라 읽는다
NAMES = {1: "SUCCESS", -1: "PLANNING_FAILED", -2: "INVALID_MOTION_PLAN", -10: "START_STATE_IN_COLLISION",
         -12: "GOAL_IN_COLLISION", -14: "GOAL_CONSTRAINTS_VIOLATED", -16: "INVALID_GOAL_CONSTRAINTS",
         -18: "INVALID_LINK_NAME", -31: "NO_IK_SOLUTION", 99999: "FAILURE"}


def points():
    """20점 = 5 반경 × 4 (방위각, 고도각). 반경이 도달경계(≈0.915m)를 가로지른다."""
    out = []
    for r in (0.35, 0.50, 0.65, 0.80, 0.95):
        for az, el in ((-45, -20), (-15, 10), (15, -20), (45, 10)):
            a, e = math.radians(az), math.radians(el)
            out.append((r, az, el,
                        r * math.cos(e) * math.cos(a),
                        r * math.cos(e) * math.sin(a),
                        0.441 + r * math.sin(e)))
    return out


def parse(txt):
    """result 의 error_code.val 을 뽑는다. `error_code:` 다음 첫 `val:` 만 본다."""
    lines, seen = txt.splitlines(), False
    val, ptime = None, None
    for ln in lines:
        s = ln.strip()
        if s.startswith("error_code:"):
            seen = True; continue
        if seen and s.startswith("val:"):
            val = int(s.split(":")[1].strip()); seen = False; continue
        if s.startswith("planning_time:"):
            try: ptime = float(s.split(":")[1])
            except ValueError: pass
    return val, ptime


def main():
    prefix = sys.argv[1]
    use_ori = "--noori" not in sys.argv
    tol = "0.0001"
    if "--tol" in sys.argv:
        tol = sys.argv[sys.argv.index("--tol") + 1]

    env = dict(os.environ, DOMAIN="2", POS_TOL=tol)
    rows = []
    for i, (r, az, el, x, y, z) in enumerate(points(), 1):
        args = [SH, f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"]
        if use_ori:
            args += list(QUAT)
        args += ["link6_1", "0.1", "true"]          # plan_only — 로봇 무동작
        t0 = time.time()
        p = subprocess.run(args, capture_output=True, text=True, env=env, timeout=120)
        val, ptime = parse(p.stdout)
        rows.append(dict(n=i, r=r, az=az, el=el, x=round(x, 6), y=round(y, 6), z=round(z, 6),
                         val=val, name=NAMES.get(val, str(val)), plan_time=ptime,
                         wall=round(time.time() - t0, 2)))
        print(f"{i:2d}/20 r={r:.2f} az={az:+3d} el={el:+3d} "
              f"({x:+.3f},{y:+.3f},{z:+.3f}) -> {NAMES.get(val, val)}", flush=True)

    ok = [r_ for r_ in rows if r_["val"] == 1]
    bad = [r_ for r_ in rows if r_["val"] != 1]
    mode = "자세구속 O" if use_ori else "자세구속 X"
    print(f"\n=== {mode} · POS_TOL={tol} ===")
    print(f"표본 {len(rows)} · 성공 {len(ok)} · 실패 {len(bad)} · 실패율 {len(bad)/len(rows)*100:.1f}%")
    for b in bad:
        print(f"  실패 #{b['n']:2d} r={b['r']:.2f} ({b['x']:+.3f},{b['y']:+.3f},{b['z']:+.3f}) {b['name']}")
    if ok:
        ts = [o["plan_time"] for o in ok if o["plan_time"] is not None]
        if ts:
            print(f"성공 계획시간 min/중앙/max = {min(ts):.4f} / {sorted(ts)[len(ts)//2]:.4f} / {max(ts):.4f} s")

    with open(f"{prefix}.json", "w") as f:
        json.dump(dict(mode=mode, pos_tol=tol, rows=rows), f, indent=1)


main()
