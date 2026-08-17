#!/usr/bin/env python3
"""조그/movej 최소스텝 실측 — M1·M2.

왜 재나: cal_script.md N15 가 짚기 오차 σ ≤ 0.3mm 를 요구한다. 현재 자세의 base 축
수평반경이 519.1mm 라 0.3mm = 0.0331°. 그 눈금을 실제로 낼 수 있는 수단이 무엇인지
(movej 인가 조그인가, 조그라면 speed 를 얼마까지 낮춰야 하는지) 가른다.

M1  movej 미세스텝  — 지령 Δ 대비 실제 TCP 변위. 목표각을 직접 주므로 정지지연 무관
M2  조그 speed 하한 — 개루프 속도 조그. 컨트롤러가 저speed 를 받는지 + 실제 이동량

⚠️ 로봇을 움직인다. 서보 ON + 비상정지 대기 상태에서만.
    mqtt_cmd.py 를 subprocess 로 부른다 — 그쪽 가드(이동량 검증·도착대기·정지보장)를 그대로 쓰기 위해.
"""
import subprocess, json, sys, math, time, re

TOOLS = "/home/markch03/WorkSpace_260814/Nyang_Nyang_Atlier/src/drivers/hcr5_comm/tools"
CMD = f"{TOOLS}/mqtt_cmd.py"
R_MM = 519.1                      # 현재 자세 TCP 수평반경 (base 축 회전반경)
DEG2MM = R_MM * math.pi / 180.0   # base 1° 가 TCP 에서 몇 mm 인가 = 9.060mm
TARGET_MM = 0.3                   # N15 요구
TARGET_DEG = TARGET_MM / DEG2MM   # = 0.0331°


def run(args, timeout=90):
    p = subprocess.run([sys.executable, CMD] + args, capture_output=True,
                       text=True, timeout=timeout, cwd=TOOLS)
    return p.returncode, p.stdout + p.stderr


def read_pos():
    """관절각·TCP 를 읽는다. 응답 JSON 줄을 그대로 파싱한다."""
    rc, out = run(["pos"])
    for line in out.splitlines():
        if '"tcp"' not in line:
            continue
        m = re.search(r'(\{.*\})\s*$', line)
        if not m:
            continue
        try:
            d = json.loads(m.group(1))["data"]["data"]
        except Exception:
            continue
        return d["joint"], d["tcp"]["position"], d["flange"]["position"]
    raise RuntimeError(f"pos 파싱 실패 (rc={rc})\n{out[:600]}")


def dist(a, b):
    return math.sqrt(sum((a[k] - b[k]) ** 2 for k in ("x", "y", "z")))


def banner(s):
    print(f"\n{'='*72}\n{s}\n{'='*72}")


def m1(steps):
    """movej 로 base 를 Δ 씩 옮기며 실제 TCP 변위를 잰다."""
    banner("M1 — movej 미세스텝 (base 축)")
    print(f"환산: base 1° = {DEG2MM:.3f}mm · 목표 {TARGET_MM}mm = {TARGET_DEG:.4f}°\n")
    print(f"{'지령Δ도':>9} {'지령mm':>8} {'실측관절Δ도':>12} {'실측TCPmm':>10} "
          f"{'오차mm':>8} {'판정':>6}")
    print("-" * 62)

    j0, t0, _ = read_pos()
    rows = []
    for d in steps:
        target = list(j0)
        target[0] = j0[0] + d
        rc, out = run(["movej"] + [f"{v:.6f}" for v in target] +
                      ["--max-delta", "1.0", "--timeout", "30"])
        if rc != 0:
            print(f"{d:>9.4f}  ── movej 거부/실패 (rc={rc}) ──")
            for ln in out.splitlines()[-4:]:
                print(f"      {ln}")
            continue
        time.sleep(0.4)                      # 정착 대기
        j1, t1, _ = read_pos()
        dj = j1[0] - j0[0]
        dmm = dist(t1, t0)
        want = d * DEG2MM
        err = dmm - abs(want)
        ok = "✅" if abs(err) < 0.05 else "⚠️"
        print(f"{d:>9.4f} {want:>8.3f} {dj:>12.6f} {dmm:>10.4f} "
              f"{err:>+8.4f} {ok:>6}")
        rows.append((d, dj, dmm))
        j0, t0 = j1, t1
    return rows


def m2(trials):
    """조그 speed 하한 — (speed, ms) 마다 실제 이동량."""
    banner("M2 — 조그 speed 하한 (jogJoint, base 축)")
    print(f"{'speed°/s':>9} {'ms':>6} {'ack':>5} {'실측관절Δ도':>12} "
          f"{'실측TCPmm':>10} {'예상mm':>8}")
    print("-" * 58)

    rows = []
    for spd, ms in trials:
        j0, t0, _ = read_pos()
        rc, out = run(["jog", "1", "positive", str(spd), str(ms)], timeout=60)
        ack = "OK" if rc == 0 and "ack 없음" not in out else ("없음" if rc == 0 else f"rc{rc}")
        if rc != 0:
            print(f"{spd:>9} {ms:>6} {ack:>5}  ── 거부 ──")
            for ln in out.splitlines()[-3:]:
                print(f"      {ln}")
            continue
        time.sleep(1.0)                      # 감속 정착 (실측 감속 ~250ms)
        j1, t1, _ = read_pos()
        dj = j1[0] - j0[0]
        dmm = dist(t1, t0)
        print(f"{spd:>9} {ms:>6} {ack:>5} {dj:>12.6f} {dmm:>10.4f} "
              f"{spd*ms/1000.0*DEG2MM:>8.3f}")
        rows.append((spd, ms, dj, dmm))
    return rows


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "both"

    j, t, f = read_pos()
    print("시작 자세:")
    print(f"  관절 {[round(v,4) for v in j]}")
    print(f"  TCP   x={t['x']:.3f} y={t['y']:.3f} z={t['z']:.3f}")
    r = math.sqrt(t["x"]**2 + t["y"]**2)
    print(f"  TCP 수평반경 {r:.1f}mm  (스크립트 상수 {R_MM}mm — 차이 {r-R_MM:+.1f}mm)")

    if what in ("m1", "both"):
        m1([0.05, 0.02, 0.01, 0.005, -0.005, -0.01, -0.02, -0.05])
    if what in ("m2", "both"):
        m2([(1, 100), (1, 50), (0.5, 100), (0.1, 100), (0.1, 50), (0.05, 100)])
