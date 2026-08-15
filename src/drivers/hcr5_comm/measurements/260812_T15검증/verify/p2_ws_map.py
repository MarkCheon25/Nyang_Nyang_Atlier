#!/usr/bin/env python3
"""P2 B3 보강 — **소묘 평면 도달지도**. 20점 표본으로는 "작업공간 안 실패 0건"의 근거가 얇다.

펜이 바닥을 향하는 자세를 고정하고 수평면을 격자로 훑어 **어디에 닿는지**를 IK 로 직접 잰다.

사용: p2_ws_map.py [link6_1|pen_tip] [timeout초]

  pen_tip  ★ 기본. **z 가 곧 종이 높이**다 — 환산이 없다
  link6_1  플랜지. z 는 종이 높이 + (tool0~pen_tip 길이). `10d8615` 기준선 재현용

⚠️ **여기 나오는 도달률·구간은 그때 URDF 에 들어 있던 공구 치수의 함수다.**
   홀더·펜 치수가 자리표시 값인 동안에는 **로봇에 대한 사실이 아니라 기준선**이다.
   충돌 형상이 도달 범위를 크게 바꾼다는 것이 실측됐으므로(08-12: 같은 평면에서
   펜 없음 550mm → 펜 있음 300mm), **치수를 실측하면 반드시 다시 뽑는다.**
   절차는 `../p2/README.md` "치수를 재고 나면" 절.

⚠️ **`pen_tip` 은 IK 시드(로봇 현재 자세)에 따라 결과가 달라진다** — `p2_ik_probe.py` 머리말 참조.
   **기준선은 전부 홈에서 잰 것이다.** 비교하려면 먼저 홈으로 보내라:
      ./verify/c2_moveaction.sh 1.570796 0 1.570796 0 1.570796 0 0.1 false
"""
import math, sys
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
LINK = ARGS[0] if ARGS else "pen_tip"
TIMEOUT = float(ARGS[1]) if len(ARGS) > 1 else 0.005

# 회전이라 **치수와 무관**하다 — 홀더 길이를 바꿔도 이 표는 그대로다.
# 다시 뽑으려면: p2_tf_read.py <링크> base_link (홈 자세에서)
QUAT = {"link6_1": (0.707106666, 0.0, 0.707106897, 0.000000231),
        "pen_tip": (1.0, 0.0, 0.0, 0.0)}
# pen_tip 은 z 가 종이 높이 그 자체라 종이가 놓일 만한 높이를 훑는다.
# link6_1 은 그보다 공구 길이만큼 위 — 두 평면이 같은 자세임을 08-12 에 격자째 대조했다
# (pen z+0.10 ↔ flange z+0.30 이 336칸 중 불일치 0칸).
ZS = {"pen_tip": (0.00, 0.05, 0.10, 0.20), "link6_1": (0.10, 0.20, 0.30, 0.40)}
if LINK not in QUAT:
    print(f"모르는 링크 '{LINK}' — 자세를 먼저 p2_tf_read.py 로 재서 QUAT·ZS 에 넣어라"); sys.exit(2)

SHOULDER_Z = 0.148778
XS = [round(0.20 + 0.05 * i, 3) for i in range(16)]    # 0.20 ~ 0.95
YS = [round(-0.50 + 0.05 * i, 3) for i in range(21)]   # -0.50 ~ +0.50

rclpy.init()
n = Node("p2_ws_map")
cli = n.create_client(GetPositionIK, "/compute_ik")
if not cli.wait_for_service(timeout_sec=15.0):
    print("SERVICE_UNAVAILABLE"); sys.exit(2)

req = GetPositionIK.Request()
req.ik_request.group_name = "hcr_arm"
req.ik_request.ik_link_name = LINK
req.ik_request.avoid_collisions = True
req.ik_request.timeout.sec = int(TIMEOUT)
req.ik_request.timeout.nanosec = int((TIMEOUT % 1) * 1e9)
ps = PoseStamped(); ps.header.frame_id = "base_link"
ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = QUAT[LINK]

print(f"=== link={LINK} · timeout={TIMEOUT}s · avoid_collisions=True ===")
grand = {"ok": 0, "n": 0}
per_z = []
for z in ZS[LINK]:
    ok_pts, rows = [], []
    for x in XS:
        line = ""
        for y in YS:
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
            req.ik_request.pose_stamped = ps
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(n, fut, timeout_sec=20.0)
            r = fut.result()
            good = bool(r) and r.error_code.val == 1
            line += "#" if good else "."
            grand["n"] += 1
            if good:
                grand["ok"] += 1
                ok_pts.append((x, y))
        rows.append(f"  x={x:.2f} |{line}|")
    tot = len(XS) * len(YS)
    if ok_pts:
        xs = [p[0] for p in ok_pts]; ys = [p[1] for p in ok_pts]
        d = [math.sqrt(p[0]**2 + p[1]**2 + (z - SHOULDER_Z)**2) for p in ok_pts]
        info = (f"도달 {len(ok_pts)}/{tot} ({len(ok_pts)/tot*100:.1f}%) · "
                f"x {min(xs):.2f}~{max(xs):.2f} · y {min(ys):.2f}~{max(ys):.2f} · "
                f"어깨거리 {min(d):.3f}~{max(d):.3f} m")
    else:
        info = f"도달 0/{tot}"
    # 완전히 채워진 x 행 = 그 폭 전체가 뚫린 구간. 놓을 수 있는 직사각형을 여기서 읽는다
    full = [XS[i] for i, rr in enumerate(rows) if "." not in rr.split("|")[1]]
    per_z.append((z, info, full))
    print(f"\n=== z = {z:+.2f} m ===   {info}")
    print(f"  (y = {YS[0]:+.2f} … {YS[-1]:+.2f}, 0.05 간격 · '#'=IK 성공)")
    print("\n".join(rows), flush=True)
    if full:
        print(f"  ★ y 전폭(±0.50) 도달 x = {min(full):.2f}~{max(full):.2f} m ({len(full)}행)")

print(f"\n=== 총계 · link={LINK} · timeout={TIMEOUT}s ===")
print(f"표본 {grand['n']} · IK 성공 {grand['ok']} · 실패 {grand['n']-grand['ok']} "
      f"· 성공률 {grand['ok']/grand['n']*100:.1f}%")
for z, info, full in per_z:
    tail = f" · 전폭도달 x {min(full):.2f}~{max(full):.2f}" if full else ""
    print(f"  z={z:+.2f}: {info}{tail}")
rclpy.shutdown()
