#!/usr/bin/env python3
"""ros2_control 쪽 추적 — 추종오차·값 갱신빈도·velocity 품질을 한 번에 산출한다. 읽기 전용.

⚠️ **컨테이너 안에서 돈다** (rclpy·control_msgs 필요). 그런데 `compose.yml` 은 `ws_moveit2` 와
   `hanwha_robot_arm` 만 마운트하고 **`src/drivers/` 는 마운트하지 않는다** — 이 파일은 컨테이너에
   안 보인다. 넣어서 쓴다:

       docker cp src/drivers/hcr_comm/tools/ros_trace.py markch_moveit2_dev:/tmp/ros_trace.py
       docker exec markch_moveit2_dev bash -lc 'source install/setup.bash && python3 /tmp/ros_trace.py 45'

   (마운트 범위는 T15 곁가지로 열려 있다. 그게 닫히면 이 복사 단계가 없어진다.)

왜 이 세 값인가:
  - **추종오차** — `ros2_controllers.yaml` 에 `constraints` 블록이 없어 JTC 허용오차가 전부 0.0(=검사 안 함)이다.
    즉 **로봇이 안 움직여도 `SUCCEEDED` 가 나온다**(260811 실측 — 5° 오차를 두고 성공 판정). '명령이 관통했는가'는
    액션 `error_code` 가 아니라 `controller_state.error` 로 재야 한다.
  - **값 갱신 빈도** — `/joint_states` 는 브로드캐스터가 100Hz 로 낸다. 발행률이 아니라 **값이 실제로 바뀐 횟수**를
    세야 `rw_rate` 가 비친다. 로봇이 움직여야 셀 수 있다 — 정지 상태에서는 **0.574Hz** 밖에 안 나온다.
  - **velocity 품질** — 코드가 낸 `state.velocity` ↔ position 수치미분 대조 (부호 일치율 · 상대오차).

사용법: ros_trace.py [sec] [out.json]            기본 30초 · /tmp/ros_trace.json
출력  : stdout 에 요약 JSON, 파일에는 요약 + 전 표본(joint_states · controller_state)
구독  : /joint_states · /hcr_arm_controller/controller_state — **아무것도 발행하지 않는다**
근거  : `ros2_control_hw_interface/중간결과물.md` §3-C (260811-오일리스부싱)
"""
import json
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/ros_trace.json"


class Cap(Node):
    def __init__(self):
        super().__init__("ros_trace")
        self.js = []      # (t, positions, velocities)
        self.cs = []      # (t, reference, feedback, error)
        self.names = []
        self.create_subscription(JointState, "/joint_states", self.on_js, 200)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/hcr_arm_controller/controller_state", self.on_cs, 200)

    def on_js(self, m):
        if not self.names:
            self.names = list(m.name)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.js.append((t, list(m.position), list(m.velocity)))

    def on_cs(self, m):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.cs.append((t, list(m.reference.positions), list(m.feedback.positions),
                        list(m.error.positions)))


def summarize(cap):
    out = {"joint_names": cap.names, "n_joint_states": len(cap.js), "n_controller_state": len(cap.cs)}

    # ── /joint_states: 값 갱신 빈도(V-6) · 이동 폭 ──────────────────────────
    if len(cap.js) >= 2:
        span = cap.js[-1][0] - cap.js[0][0]
        changed = 0
        for a, b in zip(cap.js, cap.js[1:]):
            if any(abs(x - y) > 0 for x, y in zip(a[1], b[1])):
                changed += 1
        out["js_span_s"] = round(span, 4)
        out["js_publish_hz"] = round(len(cap.js) / span, 3) if span > 0 else None
        out["js_value_change_count"] = changed
        out["js_value_change_hz"] = round(changed / span, 3) if span > 0 else None
        nj = len(cap.js[0][1])
        pp = []
        for j in range(nj):
            col = [s[1][j] for s in cap.js]
            pp.append(round((max(col) - min(col)) * 180.0 / 3.141592653589793, 6))
        out["js_peak_to_peak_deg"] = pp

    # ── velocity 품질(V-13): state.velocity ↔ position 수치미분 ──────────────
    moving = []
    for a, b in zip(cap.js, cap.js[1:]):
        dt = b[0] - a[0]
        if dt <= 0:
            continue
        for j in range(len(a[1])):
            num = (b[1][j] - a[1][j]) / dt          # 수치미분
            rep = b[2][j] if j < len(b[2]) else 0.0  # 코드가 낸 velocity
            if abs(num) > 1e-4 or abs(rep) > 1e-4:
                moving.append((j, num, rep))
    if moving:
        sign_ok = sum(1 for _, n, r in moving if (n > 0) == (r > 0) or abs(n) < 1e-9)
        rel = [abs(r - n) / abs(n) for _, n, r in moving if abs(n) > 1e-3]
        out["vel_moving_samples"] = len(moving)
        out["vel_sign_match_pct"] = round(100.0 * sign_ok / len(moving), 2)
        if rel:
            rel.sort()
            out["vel_rel_err_median"] = round(rel[len(rel) // 2], 4)
            out["vel_rel_err_p90"] = round(rel[int(len(rel) * 0.9)], 4)
    else:
        out["vel_moving_samples"] = 0

    # ── 추종오차 (V-15 의 error_code 를 대신하는 실측 기준) ──────────────────
    if cap.cs:
        nj = len(cap.cs[0][3])
        rad2deg = 180.0 / 3.141592653589793
        mx = [0.0] * nj
        for _, _, _, err in cap.cs:
            for j in range(min(nj, len(err))):
                mx[j] = max(mx[j], abs(err[j]))
        out["track_max_abs_err_deg"] = [round(e * rad2deg, 4) for e in mx]
        # 궤적 끝 기준 정착 오차 = 마지막 표본
        out["track_final_err_deg"] = [round(e * rad2deg, 4) for e in cap.cs[-1][3]]
        out["track_final_reference_deg"] = [round(x * rad2deg, 4) for x in cap.cs[-1][1]]
        out["track_final_feedback_deg"] = [round(x * rad2deg, 4) for x in cap.cs[-1][2]]
    return out


def main():
    rclpy.init()
    cap = Cap()
    end = cap.get_clock().now().nanoseconds * 1e-9 + DURATION
    while rclpy.ok() and cap.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(cap, timeout_sec=0.05)
    res = summarize(cap)
    with open(OUT, "w") as f:
        json.dump({"summary": res,
                   "joint_states": [[round(t, 6), p, v] for t, p, v in cap.js],
                   "controller_state": [[round(t, 6), r, fb, e] for t, r, fb, e in cap.cs]},
                  f, ensure_ascii=False)
    print(json.dumps(res, ensure_ascii=False, indent=2))
    cap.destroy_node()
    rclpy.shutdown()


main()
