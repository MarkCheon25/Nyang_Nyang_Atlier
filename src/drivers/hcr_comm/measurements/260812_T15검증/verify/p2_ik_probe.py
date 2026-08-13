#!/usr/bin/env python3
"""P2 B3 보강 — **IK 실패**와 **경로계획 실패**를 갈라 낸다.

`pose_goal.sh` 가 낸 FAILURE 는 move_group 로그상 OMPL 타임아웃/자기충돌이지만,
pose goal 에서는 **IK 가 목표 표본을 못 만들면 그것도 OMPL 실패로 보인다.**
그래서 `/compute_ik` 를 직접 호출해 IK 자체의 성패를 따로 잰다.

사용: p2_ik_probe.py [link6_1|pen_tip] [IK_TIMEOUT초] [--nocollide]

  link6_1  플랜지. `10d8615` 기준선을 재현할 때
  pen_tip  ★ 펜 끝(TCP). **소묘의 진짜 질문은 이쪽이다**

⚠️ **위치만 재는 모드는 없다** (2026-08-12 실측). `GetPositionIK` 의 `pose_stamped` 는
   위치·자세 한 덩어리고 자세를 빼는 플래그가 없다. 단위 쿼터니언을 넣는 것은
   "자세 무시" 가 아니라 **"엉뚱한 자세로 풀어라"** 라서 실패율이 오히려 오른다.
   자세 구속 없는 실패율은 `pose_goal.sh`(계획 단계)로 재야 한다 — 거기서는 정말로 뺀다.
   구 판본의 `--noori` 는 문서에만 있고 구현이 없었다. 지웠다.

⚠️ **여기 나오는 실패율은 그때 URDF 에 들어 있던 공구 치수의 함수다.**
   홀더·펜 치수가 자리표시 값인 동안에는 **로봇에 대한 사실이 아니라 기준선**이다.
   치수를 실측해 `nyang_pen.xacro` 에 넣고 재빌드한 뒤 같은 명령으로 다시 뽑아 비교한다.

⚠️ **`pen_tip` 실패율은 로봇의 현재 자세(IK 시드)에 따라 달라진다** (2026-08-12 실측).
   같은 표본·같은 timeout 인데 홈에서 **55.0%**, 다른 자세에서 **50.0%** 가 나왔다.
   KDL 은 현재 자세에서 출발하는 국소 해법이고, 200mm 공구가 붙으면 목표가 경계 근처로
   몰려 출발점이 결과를 뒤집는다(가설). **`link6_1` 은 같은 조건에서 20.0% 로 불변이었다** —
   시드 무관성은 플랜지에서만 성립하고 공구에는 **일반화되지 않는다.**
   → **비교하려면 시드 자세를 맞춰라.** 기준선은 전부 **홈**에서 잰 것이다:
      ./verify/c2_moveaction.sh 1.570796 0 1.570796 0 1.570796 0 0.1 false
"""
import math, sys, json
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
LINK = ARGS[0] if ARGS else "pen_tip"
TIMEOUT = float(ARGS[1]) if len(ARGS) > 1 else 0.005
AVOID = "--nocollide" not in sys.argv

# 같은 물리적 자세(펜이 바닥)를 링크마다 다른 값으로 표현한 것이다. 회전이라 **치수와 무관** —
# 홀더 길이를 바꿔도 이 표는 그대로다. 다시 뽑으려면: p2_tf_read.py <링크> base_link (홈 자세에서)
QUAT = {"link6_1": (0.707106666, 0.0, 0.707106897, 0.000000231),
        "pen_tip": (1.0, 0.0, 0.0, 0.0)}
if LINK not in QUAT:
    print(f"모르는 링크 '{LINK}' — 자세를 먼저 p2_tf_read.py 로 재서 QUAT 에 넣어라"); sys.exit(2)

# p2_sweep.py 와 **같은 표본**이어야 대조가 성립한다. 링크를 바꿔도 표본은 고정이다
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
n = Node("p2_ik_probe")
cli = n.create_client(GetPositionIK, "/compute_ik")
if not cli.wait_for_service(timeout_sec=15.0):
    print("SERVICE_UNAVAILABLE"); sys.exit(2)

SHOULDER_Z = 0.148778          # T16 기록의 어깨 높이(mm→m). 도달거리는 여기서 재야 한 값으로 모인다
rows = []
for i, (r, az, el, x, y, z) in enumerate(points(), 1):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "hcr_arm"
    req.ik_request.ik_link_name = LINK
    req.ik_request.avoid_collisions = AVOID
    req.ik_request.timeout.sec = int(TIMEOUT)
    req.ik_request.timeout.nanosec = int((TIMEOUT % 1) * 1e9)
    ps = PoseStamped()
    ps.header.frame_id = "base_link"
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = QUAT[LINK]
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
print(f"\n=== /compute_ik · link={LINK} · timeout={TIMEOUT}s · avoid_collisions={AVOID} ===")
print(f"표본 {len(rows)} · IK 성공 {len(ok)} · IK 실패 {len(bad)} · 실패율 {len(bad)/len(rows)*100:.1f}%")
if ok:
    print(f"성공 최대 어깨거리 = {max(q['d_shoulder'] for q in ok):.4f} m")
if bad:
    print(f"실패 최소 어깨거리 = {min(q['d_shoulder'] for q in bad):.4f} m")
    for b in bad:
        print(f"  IK실패 #{b['n']:2d} 어깨거리={b['d_shoulder']:.3f} ({b['x']:+.3f},{b['y']:+.3f},{b['z']:+.3f}) {b['name']}")
print(json.dumps(rows))
rclpy.shutdown()
