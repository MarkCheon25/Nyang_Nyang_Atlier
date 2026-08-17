#!/bin/bash
# `/move_action`(MoveIt MoveGroup) 으로 **좌표 목표**(pose goal) 1개를 Plan&Execute — P2.
#
# 원본 = 같은 폴더 `c2_moveaction.sh`(관절 목표, 24줄). **`goal_constraints` 안만 교체했고
# `planning_options`·scaling·액션 이름·`docker exec` 래핑은 그대로 두었다.**
#
# 사용: pose_goal.sh X Y Z [QX QY QZ QW] [LINK] [SCALE] [PLAN_ONLY]
#       X Y Z      — 목표 위치 (m, $FRAME 기준)
#       QX..QW     — 목표 자세(쿼터니언). **넷을 다 주거나 아예 생략한다.**
#                    생략하면 자세 구속을 **걸지 않는다** → 위치만 맞추고 자세는 IK 가 아무렇게나 고른다.
#                    ⚠️ 소묘는 펜 자세가 중요하다. 뺀 채로 잰 성공률을 "성공"이라 부르지 말 것 —
#                    넣은 경우와 **따로** 센다 (지시서 §7-4)
#       LINK       — 좌표에 맞출 링크. P1 전 = link6_1(플랜지) / P1 후 = pen_tip. 기본 link6_1
#       SCALE      — velocity·acceleration scaling factor. 기본 0.1 은 **mock 기준**이다.
#                    ⚠️ 실기(P3)에서는 c2_moveaction.sh 와 같이 0.014 급으로 낮춰 부를 것
#       PLAN_ONLY  — "true" 면 계획만 하고 실행하지 않는다. IK 성패만 빨리 볼 때 쓴다
#
# 환경변수(기본값) — 인자 자리를 늘리지 않으려고 뺐다:
#       FRAME=base_link  POS_TOL=0.0001  ORI_TOL=0.001  DOMAIN=0  ATTEMPTS=5  PLAN_TIME=5.0
#
# ⚠️ 원본이 주석으로 박아 둔 함정 2개를 그대로 지킨다 (c2_moveaction.sh:8-10):
#    `planning_options.planning_scene_diff.is_diff` 를 안 주면 MoveIt 이 빈 씬을 '전 세계'로 받아
#    계획이 깨진다. `robot_state.is_diff` 도 같이 줘야 현재 자세를 시작점으로 쓴다.
# ⚠️ **허용 반경을 크게 잡지 않는다** — 원본이 관절에 1e-4 rad 를 준 것과 같은 이유다.
#    넓으면 목표에 못 미친 계획이 성공으로 잡힌다. 기본값은 MoveIt 자체 기본값과 같게 두었다
#    (위치 1e-4 m = 0.1mm · 자세 1e-3 rad).
# ⚠️ 위치 목표는 별도 필드가 아니라 `constraint_region` 의 **구(SolidPrimitive.SPHERE=2) 중심**으로
#    들어간다. 그 **반경이 곧 위치 허용오차**다. 링크 원점 + target_point_offset 이 구 안에 들어오면 만족.
# 근거: 지시서 `briefs/P2_pose_goal_경로.md` §1·§2 · 계획서 `briefs/좌표계_제어_작업계획.md` §5-P2
set -u
X=$1; Y=$2; Z=$3

FRAME=${FRAME:-base_link}; POS_TOL=${POS_TOL:-0.0001}; ORI_TOL=${ORI_TOL:-0.001}
DOMAIN=${DOMAIN:-0}; ATTEMPTS=${ATTEMPTS:-5}; PLAN_TIME=${PLAN_TIME:-5.0}

if [ $# -ge 7 ]; then
  QX=$4; QY=$5; QZ=$6; QW=$7; LINK=${8:-link6_1}; VS=${9:-0.1}; PO=${10:-false}
  ORI="{header: {frame_id: $FRAME}, link_name: $LINK,
   orientation: {x: $QX, y: $QY, z: $QZ, w: $QW},
   absolute_x_axis_tolerance: $ORI_TOL, absolute_y_axis_tolerance: $ORI_TOL,
   absolute_z_axis_tolerance: $ORI_TOL, parameterization: 0, weight: 1.0}"
else
  LINK=${4:-link6_1}; VS=${5:-0.1}; PO=${6:-false}
  ORI=""            # 자세 구속 없음 — 위치만
fi

POS="{header: {frame_id: $FRAME}, link_name: $LINK,
 target_point_offset: {x: 0.0, y: 0.0, z: 0.0},
 constraint_region: {primitives: [{type: 2, dimensions: [$POS_TOL]}],
  primitive_poses: [{position: {x: $X, y: $Y, z: $Z}, orientation: {w: 1.0}}]},
 weight: 1.0}"

GOAL="{request: {group_name: hcr_arm, num_planning_attempts: $ATTEMPTS, allowed_planning_time: $PLAN_TIME,
 max_velocity_scaling_factor: $VS, max_acceleration_scaling_factor: $VS,
 goal_constraints: [{position_constraints: [$POS], orientation_constraints: [$ORI]}]},
 planning_options: {planning_scene_diff: {is_diff: true, robot_state: {is_diff: true}}, plan_only: $PO, replan: false}}"

# 컨테이너명은 PC마다 다르다 — 04pc=markch_moveit2_dev · 03pc=moveit2_dev. 떠 있는 쪽을
# 잡고, CONTAINER= 로 덮어쓸 수 있다 (2026-08-15 이식).
CONTAINER=${CONTAINER:-$(docker ps --format '{{.Names}}' | grep -xE '(markch_)?moveit2_dev' | head -1)}
[ -z "$CONTAINER" ] && { echo "❌ moveit2 컨테이너가 안 떠 있다 — run_container.sh up 먼저"; exit 1; }

docker exec "$CONTAINER" bash -lc \
  "cd ~/ws_moveit2 && source install/setup.bash && export ROS_DOMAIN_ID=$DOMAIN && \
   ros2 action send_goal /move_action moveit_msgs/action/MoveGroup '$GOAL'" 2>&1
