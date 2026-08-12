#!/usr/bin/env bash
# V-12 재현 — CM abort 완화책 2종 (260812-감압밸브, 검B)
#
# 기준(`검증.md` V-12): 프로세스가 **SIGABRT 하지 않는다**. 여전히 abort 하면 실패 = 완화 불가.
#
# 대조군이 있어야 판정이 선다 — 완화책 **없이** 같은 URDF 를 태우면 abort 하는 것이
# V-11 3경로에서 이미 나왔다(`terminate called` + `Stack trace` + `Aborted`).
# 여기서는 그 abort 가 두 파라미터로 사라지는지를 본다.
#
# (a) hardware_components_initial_state.shutdown_on_initial_state_failure:=false
#       → configure 를 시도해 ERROR 를 내지만 throw 하지 않는다
# (b) hardware_components_initial_state.unconfigured:=[hcr_robot_system]
#       → configure 자체를 건너뛴다 (MQTT 접속 시도 없음)
# ⚠️ 정확한 이름은 **둘 다 `hardware_components_initial_state.` 하위**다.
#
# ⚠️ 낡은 robot_state_publisher 를 먼저 완전히 죽인다 — 전송지속(transient_local) QoS 로 남은
#    옛 URDF 가 새 노드에 흘러들어 엉뚱한 것을 재는 사고가 있었다(S1 실패 원인).
# ⚠️ 실기 링크는 내려가 있어야 한다 — 이 시험은 실기에 닿으면 안 된다.

BASE=/tmp/v11_base.urdf
OUT=/tmp/v12_out
mkdir -p "$OUT"

source /opt/ros/jazzy/setup.bash
source ~/ws_moveit2/install/setup.bash
CFG=~/ws_moveit2/install/hcr_moveit_config/share/hcr_moveit_config/config/ros2_controllers.yaml

kill_ros() {
  for n in ros2_control_node robot_state_pub controller_manag; do
    for p in $(ps -eo pid,comm | awk -v n="$n" '$2==n {print $1}'); do
      kill -9 "$p" 2>/dev/null
    done
  done
  sleep 1
}

# 무응답 호스트로 configure 를 반드시 실패시킨다 (V-11 ① 과 같은 픽스처)
python3 - "$BASE" <<'PY'
import sys
s = open(sys.argv[1]).read()
inject = '      <param name="host">192.168.0.99</param>\n      <param name="port">1883</param>\n'
i = s.index('<plugin>hcr_bridge/HcrSystemInterface</plugin>')
j = s.index('\n', i) + 1
open('/tmp/v12.urdf', 'w').write(s[:j] + inject + s[j:])
PY

run_case() {  # $1=태그  $2..=추가 --ros-args 파라미터
  local tag=$1; shift
  local log=$OUT/v12_${tag}.log
  kill_ros
  echo "### [$tag] extra: $*"
  ( timeout 45 ros2 run controller_manager ros2_control_node --ros-args \
      -p update_rate:=100 --params-file "$CFG" "$@" \
      >"$log" 2>&1 ) &
  local node_pid=$!
  sleep 3
  timeout 15 ros2 topic pub --once --qos-durability transient_local \
      /robot_description std_msgs/msg/String "{data: $(python3 -c '
import json; print(json.dumps(open("/tmp/v12.urdf").read()))')}" \
      >>"$log" 2>&1
  # 살아남는지 보는 것이 목적이라 잠깐 재운 뒤 생존을 확인하고 정상 종료시킨다
  sleep 6
  local alive=0
  ps -eo pid,comm | awk '$2=="ros2_control_node"{print $1}' | grep -q . && alive=1
  echo "--- 생존(SIGTERM 전): $alive" | tee -a "$log"
  kill_ros
  wait "$node_pid" 2>/dev/null
}

run_case control                                                                  # 대조군 — 완화책 없음
run_case a -p hardware_components_initial_state.shutdown_on_initial_state_failure:=false
run_case b -p hardware_components_initial_state.unconfigured:="[hcr_robot_system]"

echo "=== 완료 — $OUT ==="
