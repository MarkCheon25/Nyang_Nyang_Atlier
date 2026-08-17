#!/usr/bin/env bash
# V-11 재현 — 실기 없이 실기 모드 로드 3경로 (260812-감압밸브, 검B)
#
# 기준(확정 문면, 세션분할계획 §3): on_configure 가 ≤3.1s 안에 ERROR,
#   **실패 사유마다** 다른 메시지 (TCP 미도달 / 표본 미수신).
#   OS 타임아웃(수십 초)에 잡히면 실패.
#
# ★ 측정량 = 로그의 `'configure' hardware` → `Failed to 'configure'` 직전 ERROR 까지의 시간.
#   프로세스 wall time 이 아니다 (노드 기동 오버헤드가 섞인다).
#
# ★ 기동 방식 — `controller_manager` 는 **`/robot_description` 토픽을 구독**한다.
#   URDF 를 `--ros-args -p` 로 넘기면 XML 이 인자 파서를 깨뜨린다("Failed to parse global arguments").
#   → 노드를 먼저 띄우고 URDF 를 **transient_local 로 발행**한다.
#
# 방법: 전개된 URDF 의 <hardware> 에 host/port 를 직접 주입한다 — 소스 무변경.
#   (host·port 는 xacro 로 관통돼 있지 않다. 260811-진공펌프 발견 5)
# 실기 링크는 **내려가 있어야** 한다 — ① 은 실기에 닿으면 안 되는 시험이다.
#
# ⚠️ set -u 를 쓰지 않는다 — ROS 의 setup.bash 가 미정의 변수를 참조해 즉사한다.
# ⚠️ 프로세스 정리에 pgrep -f 를 쓰지 않는다 — 패턴이 자기 셸 명령줄에 걸려 셸을 죽인다(S2 사고).
#    ps -eo pid,comm + awk 가 이 환경의 유일한 정답. (-x 는 comm 15자 절단으로 조용히 빠진다)

BASE=/tmp/v11_base.urdf
OUT=/tmp/v11_out
mkdir -p "$OUT"

source /opt/ros/jazzy/setup.bash
source ~/ws_moveit2/install/setup.bash

kill_ros() {
  for n in ros2_control_node robot_state_pub controller_manag; do
    for p in $(ps -eo pid,comm | awk -v n="$n" '$2==n {print $1}'); do
      kill -9 "$p" 2>/dev/null
    done
  done
  sleep 1
}

# <hardware> 블록 안에 <param name="host"|"port"> 를 끼워 넣는다
mk_urdf() {   # $1=host $2=port $3=경로
  python3 - "$1" "$2" "$3" "$BASE" <<'PY'
import sys
host, port, dst, base = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
s = open(base).read()
inject = f'      <param name="host">{host}</param>\n      <param name="port">{port}</param>\n'
i = s.index('<plugin>hcr5_bridge/HcrSystemInterface</plugin>')
j = s.index('\n', i) + 1
open(dst, 'w').write(s[:j] + inject + s[j:])
PY
}

run_case() {  # $1=태그 $2=host $3=port
  local tag=$1 host=$2 port=$3
  local urdf=/tmp/v11_${tag}.urdf log=$OUT/v11_${tag}.log
  mk_urdf "$host" "$port" "$urdf"
  kill_ros

  echo "### [$tag] host=$host port=$port"
  # ros2_control_node 를 먼저 띄운다 (URDF 를 토픽으로 기다린다)
  ( timeout 60 ros2 run controller_manager ros2_control_node --ros-args \
      -p update_rate:=100 \
      --params-file ~/ws_moveit2/install/hcr_moveit_config/share/hcr_moveit_config/config/ros2_controllers.yaml \
      >"$log" 2>&1 ) &
  local node_pid=$!
  sleep 3
  # URDF 를 transient_local 로 1회 발행 → CM 이 받아 configure 를 시작한다
  timeout 15 ros2 topic pub --once --qos-durability transient_local \
      /robot_description std_msgs/msg/String "{data: $(python3 -c '
import json,sys; print(json.dumps(open(sys.argv[1]).read()))' "$urdf")}" \
      >>"$log" 2>&1
  # ⚠️ 인자 없는 `wait` 를 쓰지 않는다 — 경로 ③ 은 스텁 브로커도 이 셸의 백그라운드 작업이라
  #    `wait` 가 스텁을 영원히 기다린다. 노드 PID 만 지목해 기다린다.
  wait "$node_pid"
  kill_ros
}

run_case noresp   192.168.0.99 1883    # ① 무응답 호스트
run_case refused  127.0.0.1    18830   # ② 접속 거부

# ③ 브로커만 — 스텁을 먼저 띄운다
python3 /tmp/mqtt_stub.py 1884 >"$OUT/stub.log" 2>&1 &
STUB=$!
sleep 1
run_case stubonly 127.0.0.1    1884
kill $STUB 2>/dev/null

echo "=== 완료 — $OUT ==="
