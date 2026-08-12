#!/usr/bin/env python3
"""P2 B3 보강 — **소묘 평면 도달지도**. 20점 표본으로는 "작업공간 안 실패 0건"의 근거가 얇다.

펜이 바닥을 향하는 자세(홈 flange 자세)를 고정하고 수평면을 격자로 훑어
**종이를 어디에 놓을 수 있는지**를 IK 로 직접 잰다. `/compute_ik` 기본 timeout(5ms) 그대로.

사용: ws_map.py [timeout초]
"""
import math, sys, json
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

TIMEOUT = float(sys.argv[1]) if len(sys.argv) > 1 else 0.005
QUAT = (0.707106666, 0.0, 0.707106897, 0.000000231)   # 펜이 바닥을 향하는 자세
SHOULDER_Z = 0.148778

ZS = (0.10, 0.20, 0.30, 0.40)
XS = [round(0.20 + 0.05 * i, 3) for i in range(16)]    # 0.20 ~ 0.95
YS = [round(-0.50 + 0.05 * i, 3) for i in range(21)]   # -0.50 ~ +0.50

rclpy.init()
n = Node("ws_map")
cli = n.create_client(GetPositionIK, "/compute_ik")
if not cli.wait_for_service(timeout_sec=15.0):
    print("SERVICE_UNAVAILABLE"); sys.exit(2)

req = GetPositionIK.Request()
req.ik_request.group_name = "hcr_arm"
req.ik_request.ik_link_name = "link6_1"
req.ik_request.avoid_collisions = True
req.ik_request.timeout.sec = int(TIMEOUT)
req.ik_request.timeout.nanosec = int((TIMEOUT % 1) * 1e9)
ps = PoseStamped(); ps.header.frame_id = "base_link"
ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = QUAT

grand = {"ok": 0, "n": 0}
per_z = []
for z in ZS:
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
    if ok_pts:
        xs = [p[0] for p in ok_pts]; ys = [p[1] for p in ok_pts]
        d = [math.sqrt(p[0]**2 + p[1]**2 + (z - SHOULDER_Z)**2) for p in ok_pts]
        info = (f"도달 {len(ok_pts)}/{len(XS)*len(YS)} ({len(ok_pts)/(len(XS)*len(YS))*100:.1f}%) · "
                f"x {min(xs):.2f}~{max(xs):.2f} · y {min(ys):.2f}~{max(ys):.2f} · "
                f"어깨거리 {min(d):.3f}~{max(d):.3f} m")
    else:
        info = "도달 0"
    per_z.append((z, len(ok_pts), info))
    print(f"\n=== z = {z:.2f} m ===   {info}")
    print(f"  (y = {YS[0]:+.2f} … {YS[-1]:+.2f}, 0.05 간격 · '#'=IK 성공)")
    print("\n".join(rows), flush=True)

print(f"\n=== 총계 · timeout={TIMEOUT}s ===")
print(f"표본 {grand['n']} · IK 성공 {grand['ok']} · 실패 {grand['n']-grand['ok']} "
      f"· 성공률 {grand['ok']/grand['n']*100:.1f}%")
for z, c, info in per_z:
    print(f"  z={z:.2f}: {info}")
rclpy.shutdown()
