#!/bin/bash
# `/move_action`(MoveIt MoveGroup)으로 관절 목표 1개를 Plan&Execute — V-15 가 `검증.md` 에서 지정한 방법.
#
# 사용: c2_moveaction.sh J1 J2 J3 J4 J5 J6 [scale] [plan_only]
#       scale     — velocity·acceleration scaling factor 를 같은 값으로 준다
#       plan_only — "true" 면 계획만 하고 실행하지 않는다 (**로봇 무동작 · 발열 0**). 창 열기 전 시간계획 교정용
#
# ⚠️ `planning_options.planning_scene_diff.is_diff` 를 안 주면 MoveIt 이 빈 씬을 '전 세계'로 받아
#    계획이 깨진다. `robot_state.is_diff` 도 같이 줘야 현재 자세를 시작점으로 쓴다.
# ⚠️ 목표 허용오차를 1e-4 rad 로 좁게 준다 — 넓으면 MoveIt 이 목표에 못 미친 계획도 성공으로 낸다.
# 근거: 계획서 `briefs/검증_재현_세션분할.md` §5 검C2 · `검증.md` V-15
set -u
J1=$1; J2=$2; J3=$3; J4=$4; J5=$5; J6=$6; VS=${7:-0.014}; PO=${8:-false}

jc() { echo "{joint_name: $1, position: $2, tolerance_above: 0.0001, tolerance_below: 0.0001, weight: 1.0}"; }

GOAL="{request: {group_name: hcr_arm, num_planning_attempts: 5, allowed_planning_time: 5.0,
 max_velocity_scaling_factor: $VS, max_acceleration_scaling_factor: $VS,
 goal_constraints: [{joint_constraints: [$(jc joint_1 $J1), $(jc joint_2 $J2), $(jc joint_3 $J3), $(jc joint_4 $J4), $(jc joint_5 $J5), $(jc joint_6 $J6)]}]},
 planning_options: {planning_scene_diff: {is_diff: true, robot_state: {is_diff: true}}, plan_only: $PO, replan: false}}"

docker exec markch_moveit2_dev bash -lc \
  "cd ~/ws_moveit2 && source install/setup.bash && export ROS_DOMAIN_ID=0 && \
   ros2 action send_goal /move_action moveit_msgs/action/MoveGroup '$GOAL'" 2>&1
