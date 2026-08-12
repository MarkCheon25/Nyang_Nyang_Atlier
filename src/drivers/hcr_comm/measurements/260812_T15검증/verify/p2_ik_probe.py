#!/usr/bin/env python3
"""P2 B3 보강 — **IK 실패**와 **경로계획 실패**를 갈라 낸다.

`pose_goal.sh` 가 낸 FAILURE 는 move_group 로그상 OMPL 타임아웃/자기충돌이지만,
pose goal 에서는 **IK 가 목표 표본을 못 만들면 그것도 OMPL 실패로 보인다.**
그래서 `/compute_ik` 를 직접 호출해 IK 자체의 성패를 따로 잰다.

사용: ik_probe.py [IK_TIMEOUT초] [--noori] [--collide|--nocollide]
"""
import math, sys, json
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

TIMEOUT = float(sys.argv[1]) if len(sys.argv) > 1 else 0.005
AVOID = "--nocollide" not in sys.argv
QUAT = (0.707106666, 0.0, 0.707106897, 0.000000231)   # 홈 flange 자세 (펜이 바닥)

# sweep.py 와 **같은 표본**이어야 대조가 성립한다
def points():
    out = []
    for r in (0.35, 0.50, 0.65, 0.80, 0.95):
        for az, el in ((-45, -20), (-15, 10), (15, -20), (45, 10)):
            a, e = math.radians(az), math.radians(el)
            out.append((r, az, el, r*math.cos(e)*math.cos(a), r*math.cos(e)*math.sin(a),
                        0.441 + r*math.sin(e)))
    return out

ERR = {1: "SUCCESS", -31: "NO_IK_SOLUTION", -12: "GOAL_IN_COLLISION", -10: "START_STATE_IN_COLLISION",
       -17: "INVALID_ROBOT_STATE", -18: "INVALID_LINK_NAME", 99999: "FAILURE", -1: "PLANNING_FAILED"}

rclpy.init()
n = Node("ik_probe")
cli = n.create_client(GetPositionIK, "/compute_ik")
if not cli.wait_for_service(timeout_sec=15.0):
    print("SERVICE_UNAVAILABLE"); sys.exit(2)

SHOULDER_Z = 0.148778          # T16 기록의 어깨 높이(mm→m). 도달거리는 여기서 재야 의미가 있다
rows = []
for i, (r, az, el, x, y, z) in enumerate(points(), 1):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "hcr_arm"
    req.ik_request.ik_link_name = "link6_1"
    req.ik_request.avoid_collisions = AVOID
    req.ik_request.timeout.sec = int(TIMEOUT)
    req.ik_request.timeout.nanosec = int((TIMEOUT % 1) * 1e9)
    ps = PoseStamped()
    ps.header.frame_id = "base_link"
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = QUAT
    req.ik_request.pose_stamped = ps

    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=20.0)
    res = fut.result()
    val = res.error_code.val if res else None
    d_sh = math.sqrt(x*x + y*y + (z - SHOULDER_Z)**2)
    rows.append(dict(n=i, r=r, x=round(x, 4), y=round(y, 4), z=round(z, 4),
                     d_shoulder=round(d_sh, 4), val=val, name=ERR.get(val, str(val))))
    print(f"{i:2d}/20 r={r:.2f} 어깨거리={d_sh:.3f}m ({x:+.3f},{y:+.3f},{z:+.3f}) -> {ERR.get(val, val)}", flush=True)

ok = [q for q in rows if q["val"] == 1]
bad = [q for q in rows if q["val"] != 1]
print(f"\n=== /compute_ik · timeout={TIMEOUT}s · avoid_collisions={AVOID} ===")
print(f"표본 {len(rows)} · IK 성공 {len(ok)} · IK 실패 {len(bad)} · 실패율 {len(bad)/len(rows)*100:.1f}%")
if ok:
    print(f"성공 최대 어깨거리 = {max(q['d_shoulder'] for q in ok):.4f} m")
if bad:
    print(f"실패 최소 어깨거리 = {min(q['d_shoulder'] for q in bad):.4f} m")
    for b in bad:
        print(f"  IK실패 #{b['n']:2d} 어깨거리={b['d_shoulder']:.3f} ({b['x']:+.3f},{b['y']:+.3f},{b['z']:+.3f}) {b['name']}")
print(json.dumps(rows))
rclpy.shutdown()
