#!/usr/bin/env python3
"""M3 — 평면 조준 반복도 σ 실측 (대화형).

무엇을 재나: **같은 물리적 한 점을 사람이 눈으로 다시 맞출 때 얼마나 흩어지는가.**
`cal_script.md` N15 가 요구하는 짚기 오차 σ 의 실측값이고, 마커 크기가 여기 걸려 있다.

왜 새로 재나: 이미 있는 조준 반복도 **1.041mm**(`260816_펜TCP`)는 **허공의 고정점을 3D 로**
맞춘 값이다. 코너 짚기는 **평면 위 2D 조준**이라(z 는 작업평면이 잡아 주고 꼭짓점이라는 뚜렷한
시각 기준이 있다) 그 값을 그대로 쓰면 마커를 과하게 키우게 된다.

🔴 **핵심 절차 — 매 회차 전에 반드시 `scatter` 로 흩뜨린다.**
   안 흩뜨리고 연속으로 `mark` 하면 로봇이 안 움직였으므로 산포가 **0 으로 나온다.**
   그 0 은 조준 정밀도가 아니라 '아무것도 안 했음'이다.

이동 경로: 현재 TCP + (dx,dy,dz) → `robot/convertJointAngle`(IK, poseType="tcp")
           → `movej`. 조그는 쓰지 않는다 — 램프가 t³·³ 라 미세 위치결정에 못 쓴다(README §3).

⚠️ 로봇을 움직인다. 서보 ON + 비상정지 대기 상태에서만.

사용:
    python3 touch_repeat.py [표적이름]

    x +0.5      TCP 를 base 좌표 x 로 +0.5mm (y·z 도 같은 형식, 음수 가능)
    xy .3 -.2   x·y 동시
    mark        지금 펜 끝이 표적에 맞았다 — 한 점 기록
    scatter [r] 무작위로 흩뜨린다 (기본 반경 3mm, z 는 +2mm 띄운다)
    stat        지금까지의 산포
    pos         현재 자세만 다시 출력
    undo        마지막 기록 취소
    q           저장하고 종료
"""
import subprocess, json, sys, math, re, random, os, time

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools")
TOOLS = os.path.normpath(TOOLS)
CMD = os.path.join(TOOLS, "mqtt_cmd.py")
MAX_DELTA = 2.0          # movej 이동량 상한(도) — 미세조정용이라 좁게 잡는다
JOINT_NAMES = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")


def run(args, timeout=90):
    p = subprocess.run([sys.executable, CMD] + list(args), capture_output=True,
                       text=True, timeout=timeout, cwd=TOOLS)
    return p.returncode, p.stdout + p.stderr


def read_pos():
    rc, out = run(["pos"])
    for line in out.splitlines():
        if '"tcp"' not in line:
            continue
        m = re.search(r"(\{.*\})\s*$", line)
        if not m:
            continue
        try:
            d = json.loads(m.group(1))["data"]["data"]
        except Exception:
            continue
        return d["joint"], d["tcp"]["position"], d["tcp"]["orientation"]
    raise RuntimeError(f"pos 파싱 실패 (rc={rc})\n{out[:500]}")


def solve_ik(pos, ori):
    """TCP 포즈 → 관절각. mqtt_cmd.py ik 의 '해:' 블록을 파싱한다.

    '→ 그대로 실행하려면: movej ...' 줄은 소수점 2자리로 반올림돼 있어 쓰지 않는다
    (0.01° = 0.09mm — 미세조정에는 못 쓸 정밀도다). '해:' 줄은 4자리다.
    """
    rc, out = run(["ik", f"{pos['x']:.4f}", f"{pos['y']:.4f}", f"{pos['z']:.4f}",
                   f"{ori['x']:.4f}", f"{ori['y']:.4f}", f"{ori['z']:.4f}"])
    sol = []
    for name in JOINT_NAMES:
        m = re.search(rf"joint_\d\s+{name}\s+(-?\d+\.\d+)", out)
        if not m:
            return None, out
        sol.append(float(m.group(1)))
    return sol, out


def goto(pos, ori):
    sol, out = solve_ik(pos, ori)
    if sol is None:
        print("  ✗ IK 실패 — 그 자세는 못 간다 (특이점이거나 사거리 밖)")
        for ln in out.splitlines()[-3:]:
            print(f"    {ln}")
        return False
    rc, out = run(["movej"] + [f"{v:.6f}" for v in sol] +
                  ["--max-delta", str(MAX_DELTA), "--timeout", "30"])
    if rc != 0:
        print(f"  ✗ movej 거부 (rc={rc})")
        for ln in out.splitlines()[-4:]:
            print(f"    {ln}")
        return False
    return True


def show(j, p):
    print(f"  관절 {[round(v,4) for v in j]}")
    print(f"  TCP  x={p['x']:9.4f}  y={p['y']:9.4f}  z={p['z']:9.4f}")


def stat(marks):
    if len(marks) < 2:
        print(f"  기록 {len(marks)}점 — 2점 이상이어야 산포가 나온다")
        return
    n = len(marks)
    cx = sum(m["tcp"]["x"] for m in marks) / n
    cy = sum(m["tcp"]["y"] for m in marks) / n
    cz = sum(m["tcp"]["z"] for m in marks) / n
    dxy = [math.hypot(m["tcp"]["x"] - cx, m["tcp"]["y"] - cy) for m in marks]
    sx = math.sqrt(sum((m["tcp"]["x"] - cx) ** 2 for m in marks) / (n - 1))
    sy = math.sqrt(sum((m["tcp"]["y"] - cy) ** 2 for m in marks) / (n - 1))
    sz = math.sqrt(sum((m["tcp"]["z"] - cz) ** 2 for m in marks) / (n - 1))
    rms = math.sqrt(sum(d * d for d in dxy) / n)

    print(f"\n  ── {n}점 산포 ──")
    print(f"  중심      x={cx:9.4f}  y={cy:9.4f}  z={cz:9.4f}")
    print(f"  표준편차  σx={sx:.4f}  σy={sy:.4f}  σz={sz:.4f} mm")
    print(f"  xy 반경   RMS {rms:.4f}mm · 최악 {max(dxy):.4f}mm")
    sigma_xy = math.sqrt(sx * sx + sy * sy)
    print(f"  σ_xy = √(σx²+σy²) = {sigma_xy:.4f}mm")
    tgt = 0.3
    print(f"  → N15 요구 σ ≤ {tgt}mm : "
          f"{'✅ 통과' if sigma_xy <= tgt else f'❌ {sigma_xy/tgt:.1f}배 초과'}")
    print("  (참조: 260816_펜TCP 의 3D 조준 반복도 1.041mm)\n")


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "표적"
    outfile = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           f"m3_{name}.json")
    marks = []
    if os.path.exists(outfile):
        with open(outfile) as f:
            marks = json.load(f)
        print(f"기존 기록 {len(marks)}점을 이어서 씁니다 — {outfile}")

    j, p, o = read_pos()
    print(f"\n표적 '{name}' · 시작 자세")
    show(j, p)
    print("\n🔴 매 회차 전에 scatter 로 흩뜨릴 것. 안 그러면 산포가 0 으로 나온다.")
    print("명령: x/y/z <mm> · xy <dx> <dy> · mark · scatter [r] · stat · pos · undo · q\n")

    while True:
        try:
            line = input("m3> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        a = line.split()
        c = a[0].lower()

        if c == "q":
            break
        elif c == "pos":
            j, p, o = read_pos()
            show(j, p)
        elif c == "stat":
            stat(marks)
        elif c == "undo":
            if marks:
                marks.pop()
                print(f"  취소 — 남은 {len(marks)}점")
            else:
                print("  기록이 없다")
        elif c in ("x", "y", "z", "xy"):
            j, p, o = read_pos()
            tgt = dict(p)
            try:
                if c == "xy":
                    tgt["x"] += float(a[1]); tgt["y"] += float(a[2])
                else:
                    tgt[c] += float(a[1])
            except (IndexError, ValueError):
                print(f"  사용법: {c} <mm>" + (" <mm>" if c == "xy" else ""))
                continue
            if goto(tgt, o):
                time.sleep(0.3)
                j, p, o = read_pos()
                show(j, p)
        elif c == "scatter":
            r = float(a[1]) if len(a) > 1 else 3.0
            j, p, o = read_pos()
            tgt = dict(p)
            th = random.uniform(0, 2 * math.pi)
            d = random.uniform(0.4 * r, r)
            tgt["x"] += d * math.cos(th)
            tgt["y"] += d * math.sin(th)
            tgt["z"] += 2.0                       # 표면에서 띄운다
            print(f"  흩뜨림 {d:.2f}mm @ {math.degrees(th):.0f}° · z +2.0mm")
            if goto(tgt, o):
                time.sleep(0.3)
                j, p, o = read_pos()
                show(j, p)
                print("  → 펜 끝을 표적에 다시 맞춘 뒤 mark")
        elif c == "mark":
            j, p, o = read_pos()
            marks.append({"n": len(marks) + 1, "joint": j, "tcp": p, "ori": o,
                          "t": time.strftime("%Y-%m-%dT%H:%M:%S")})
            print(f"  ✔ {len(marks)}번째 기록  "
                  f"x={p['x']:.4f} y={p['y']:.4f} z={p['z']:.4f}")
            with open(outfile, "w") as f:
                json.dump(marks, f, ensure_ascii=False, indent=1)
            if len(marks) >= 2:
                stat(marks)
        else:
            print("  ? x/y/z <mm> · xy <dx> <dy> · mark · scatter [r] · stat · pos · undo · q")

    if marks:
        with open(outfile, "w") as f:
            json.dump(marks, f, ensure_ascii=False, indent=1)
        print(f"\n저장 — {outfile} ({len(marks)}점)")
        stat(marks)


if __name__ == "__main__":
    main()
