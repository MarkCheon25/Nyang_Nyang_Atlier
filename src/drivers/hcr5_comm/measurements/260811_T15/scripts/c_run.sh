#!/bin/bash
# C 본체 — 서보 ON 창 1회. base 축만 총 10°, 감속 0.5.
# C2/V-7 단발 점대점 → C3+C4/V-14·V-15 느린 궤적 → 서보 OFF
set -u
TOOLS=/home/markch04/Markch_ws_260809/Nyang_Nyang_Atlier/Nyang_Nyang_Atlier/src/drivers/hcr5_comm/tools
J0=1.570850433790509        # 현재 joint_1 (real base 0.0031°)
J2=-0.00014808230378112066
J3=1.5708865051209169
J4=-0.0015956817477945035
J5=1.5708399920896012
J6=2.088340181527482e-05
C2_TGT=1.6581169            # +5°
C4_TGT=1.7453834            # +10° (누적)

ts() { date +%H:%M:%S.%3N; }
say() { echo "[$(ts)] $*"; }

goal() {  # $1=목표 joint_1, $2=초
  docker exec markch_moveit2_dev bash -lc "cd ~/ws_moveit2 && source install/setup.bash && export ROS_DOMAIN_ID=0 && \
    ros2 action send_goal /hcr_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
    '{trajectory: {joint_names: [joint_1,joint_2,joint_3,joint_4,joint_5,joint_6], points: [{positions: [$1,$J2,$J3,$J4,$J5,$J6], time_from_start: {sec: $2, nanosec: 0}}]}}'" 2>&1 | tail -12
}

say "── 서보 ON ──"
python3 $TOOLS/mqtt_cmd.py servo on 2>&1 | tail -3
sleep 3

say "── C2/V-7 : 단발 점대점 base +5° (4s) ──"
goal $C2_TGT 4
say "C2 목표 전송 완료 — 정착 대기 8s"
sleep 8

say "── C3+C4 / V-14·V-15 : 느린 궤적 base +5°→+10° (8s) ──"
goal $C4_TGT 8
say "C4 목표 전송 완료 — 정착 대기 6s"
sleep 6

say "── 서보 OFF ──"
python3 $TOOLS/mqtt_cmd.py servo off 2>&1 | tail -3
say "── 창 종료 ──"
