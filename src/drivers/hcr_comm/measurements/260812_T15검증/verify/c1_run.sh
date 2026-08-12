#!/bin/bash
# 검C1 / V-8 안전 게이트 — 서보 ON 창 **1회**. allow_motion=false 로 기동한 스택에 JTC 궤적을 태운다.
#
# 판정 대상 = `move/joint/here` 발행 **0건**. 근거의 힘은 두 가지가 받친다:
#   ① 감청 유효성 — 우리가 낸 set/operation·set/velocity 가 같은 감청에 잡혀야 0건이 위음성이 아니다
#   ② 추종오차가 그대로 남는 것 — JTC 가 명령 인터페이스까지 보냈고 플러그인이 전량을 잘랐다는 직접 측정
#
# ⚠️ 창 길이 = 서보 ON **17초** (Rokey6 판정 2026-08-12: 착수 축온 55°C 라 창 ≤20초 · 58°C 즉시중단).
#    58°C 차단은 c1_tempwatch.py 가 병렬로 건다 — 이 스크립트는 시간만 지킨다.
# ⚠️ p-p 는 창 전체로 재면 안 된다 — 브레이크 전이 중력 정착이 섞인다(§3, 08-11 발견 ②).
#    그래서 아래 mark 로 **구간 경계 시각**을 남긴다. 자르는 것은 분석기 몫이다.
set -u

REPO=/home/markch04/Markch_ws_260809/Nyang_Nyang_Atlier/Nyang_Nyang_Atlier
TOOLS=$REPO/src/drivers/hcr_comm/tools
BASE=$REPO/src/drivers/hcr_comm/measurements/260812_T15검증
OUTD=$BASE/c1
DUR=40

# 착수 시 실기 자세 (10:5x 실측) — joint_1 만 +5°, 나머지는 제자리
J1=1.5708409413351383
J2=-0.00023066666550499253
J3=1.5708665709646388
J4=-0.002605678999224139
J5=1.570838093598527
J6=2.7528120574680443e-05
TGT=1.6581074039348548        # J1 + 5°

MARKS=$OUTD/logs/c1_marks.txt
mark() { echo "$1 $(date +%s.%N)" >> "$MARKS"; echo "[$(date +%H:%M:%S.%3N)] $1"; }

: > "$MARKS"

echo "── 계측기 3종 기동 (${DUR}s) ──"
python3 "$TOOLS/mqtt_trace.py" 192.168.0.20 $DUR "$OUTD/bus/c1_bus.jsonl" \
    > "$OUTD/logs/c1_bus.log" 2>&1 &
BUS=$!
docker exec markch_moveit2_dev bash -lc \
    "cd ~/ws_moveit2 && source install/setup.bash && export ROS_DOMAIN_ID=0 && python3 /tmp/ros_trace.py $DUR /tmp/c1_ros.json" \
    > "$OUTD/logs/c1_ros.log" 2>&1 &
ROS=$!
python3 "$BASE/verify/c1_tempwatch.py" 192.168.0.20 $DUR 58 "$OUTD/logs/c1_temp.jsonl" \
    > "$OUTD/logs/c1_tempwatch.log" 2>&1 &
TW=$!

mark trace_start
sleep 3

echo "── 서보 ON ──"
mark servo_on_pre
python3 "$TOOLS/mqtt_cmd.py" servo on 2>&1 | tail -2
mark servo_on_post

sleep 5

echo "── JTC 궤적 : joint_1 +5° (4s) ──"
mark jtc_send
docker exec markch_moveit2_dev bash -lc \
  "cd ~/ws_moveit2 && source install/setup.bash && export ROS_DOMAIN_ID=0 && \
   ros2 action send_goal /hcr_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
   '{trajectory: {joint_names: [joint_1,joint_2,joint_3,joint_4,joint_5,joint_6], points: [{positions: [$TGT,$J2,$J3,$J4,$J5,$J6], time_from_start: {sec: 4, nanosec: 0}}]}}'" \
   2>&1 | tail -8 | tee "$OUTD/logs/c1_jtc.log"
mark jtc_done

echo "── 정착 대기 ──"
sleep 5

echo "── 서보 OFF ──"
mark servo_off_pre
python3 "$TOOLS/mqtt_cmd.py" servo off 2>&1 | tail -2
mark servo_off_post

echo "── 창 종료. 계측기 종료 대기 ──"
wait $BUS $ROS $TW          # ⚠️ 인자 없는 wait 금지 (검B 교훈)
mark all_done
echo "완료 — 원자료: $OUTD"
