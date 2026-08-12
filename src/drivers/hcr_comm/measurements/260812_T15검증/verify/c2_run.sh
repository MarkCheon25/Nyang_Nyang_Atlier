#!/bin/bash
# 검C2 / 서보 ON 창② — allow_motion:=true 로 기동한 스택에 실제 명령을 태운다. 쓰기 5행.
#
#   w1    JTC 액션으로 base +5°        → V-7(도달오차) · V-13(velocity 품질) · V-14(재발행 선점)
#   w2    /move_action 으로 base +5° 더 → V-15(JTC 궤적 관통) — `검증.md` 가 지정한 방법
#   home  홈 복귀 (base 10° → 0°)
#
# 왜 창을 셋으로 쪼개는가 — 착수 축온이 **55°C**(게이트 ≤50°C)다. Rokey6 판정은
# *"진행하되 총 서보 ON ≤45초 · 58°C 즉시중단"*(2026-08-12). 창 사이마다 축온을 보고
# 다음 창을 열지 결정한다. 58°C 차단은 사람 눈이 아니라 c1_tempwatch.py 가 10Hz 로 건다.
#
# 왜 w1 은 JTC 이고 w2 는 /move_action 인가 — `검증.md` V-15 의 지정 방법이 `/move_action`
# (MoveIt, SUCCESS=**1**)인데 08-11 단자대는 JTC 액션(control_msgs, SUCCESSFUL=**0**)으로 쟀다.
# 중간결과물 §3-D 가 그 차이를 '방법 차이'로 남겨 뒀다 — 이 세션이 지정 방법으로 닫는다.
#
# ⚠️ 관절값은 실행 시점에 버스에서 읽어 넣는다(아래 CUR_*). 자세가 세션마다 다르므로 하드코딩하지 않는다.
set -u

STAGE=${1:?사용법: c2_run.sh {w1|w2|home}}

REPO=/home/markch04/Markch_ws_260809/Nyang_Nyang_Atlier/Nyang_Nyang_Atlier
TOOLS=$REPO/src/drivers/hcr_comm/tools
BASE=$REPO/src/drivers/hcr_comm/measurements/260812_T15검증
OUTD=$BASE/c2
DEX="docker exec markch_moveit2_dev bash -lc"
RSRC="cd ~/ws_moveit2 && source install/setup.bash && export ROS_DOMAIN_ID=0"

VS=${VS:-0.014}     # /move_action scaling — 창 열기 전 plan_only 로 교정한 값 (아래 w2)
case "$STAGE" in
  w1)   DUR=22; DELTA=5  ; MODE=jtc  ;;
  w2)   DUR=22; DELTA=5  ; MODE=moveit ;;
  home) DUR=20; DELTA=-10; MODE=jtc  ;;
  *) echo "STAGE 는 w1|w2|home 이어야 한다"; exit 2 ;;
esac

mkdir -p "$OUTD/bus" "$OUTD/ros" "$OUTD/logs"   # git 은 빈 디렉터리를 안 나른다 (검A 교훈)
MARKS=$OUTD/logs/c2_${STAGE}_marks.txt
mark() { echo "$1 $(date +%s.%N)" >> "$MARKS"; echo "[$(date +%H:%M:%S.%3N)] $1"; }
: > "$MARKS"

# ── 현재 관절값을 URDF rad 로 읽어 목표를 만든다 (규약 변환은 코드가 이미 하고 있다) ──
read -r CUR1 CUR2 CUR3 CUR4 CUR5 CUR6 <<<"$($DEX "$RSRC && timeout 5 ros2 topic echo /joint_states --once --field position" \
    | tr -d '\r' | grep -oE '[-0-9.e+]+' | head -6 | tr '\n' ' ')"
[ -z "${CUR6:-}" ] && { echo "❌ /joint_states 를 못 읽었다 — 스택이 떠 있는지 확인"; exit 1; }
TGT=$(python3 -c "print(repr($CUR1 + $DELTA*3.141592653589793/180))")

echo "── 현재 joint_1 = $CUR1 rad → 목표 $TGT rad (${DELTA}°) ──"
echo "$STAGE cur=$CUR1 tgt=$TGT delta=${DELTA}deg" >> "$MARKS"

# ── 계측기 3종 ──
echo "── 계측기 3종 기동 (${DUR}s) ──"
python3 "$TOOLS/mqtt_trace.py" 192.168.0.20 $DUR "$OUTD/bus/c2_${STAGE}_bus.jsonl" \
    > "$OUTD/logs/c2_${STAGE}_bus.log" 2>&1 &
BUS=$!
$DEX "$RSRC && python3 /tmp/ros_trace.py $DUR /tmp/c2_${STAGE}_ros.json" \
    > "$OUTD/logs/c2_${STAGE}_ros.log" 2>&1 &
ROS=$!
python3 "$BASE/verify/c1_tempwatch.py" 192.168.0.20 $DUR 58 "$OUTD/logs/c2_${STAGE}_temp.jsonl" \
    > "$OUTD/logs/c2_${STAGE}_tempwatch.log" 2>&1 &
TW=$!

mark trace_start
sleep 3

echo "── 서보 ON ──"
mark servo_on_pre
python3 "$TOOLS/mqtt_cmd.py" servo on 2>&1 | tail -2
mark servo_on_post
sleep 2

if [ "$MODE" = jtc ]; then
  SEC=4; [ "$STAGE" = home ] && SEC=6
  echo "── JTC 궤적 : joint_1 ${DELTA}° (${SEC}s) ──"
  mark move_send
  $DEX "$RSRC && \
     ros2 action send_goal /hcr_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
     '{trajectory: {joint_names: [joint_1,joint_2,joint_3,joint_4,joint_5,joint_6], points: [{positions: [$TGT,$CUR2,$CUR3,$CUR4,$CUR5,$CUR6], time_from_start: {sec: $SEC, nanosec: 0}}]}}'" \
     2>&1 | tail -10 | tee "$OUTD/logs/c2_${STAGE}_action.log"
  mark move_done
else
  echo "── /move_action Plan&Execute : joint_1 ${DELTA}° (vel/acc scale $VS) ──"
  mark move_send
  bash "$BASE/verify/c2_moveaction.sh" "$TGT" "$CUR2" "$CUR3" "$CUR4" "$CUR5" "$CUR6" "$VS" \
     2>&1 | tee "$OUTD/logs/c2_${STAGE}_action.log"
  mark move_done
fi

echo "── 정착 대기 ──"
sleep 6

echo "── 서보 OFF ──"
mark servo_off_pre
python3 "$TOOLS/mqtt_cmd.py" servo off 2>&1 | tail -2
mark servo_off_post

echo "── 창 종료. 계측기 종료 대기 ──"
wait $BUS $ROS $TW          # ⚠️ 인자 없는 wait 금지 (검B 교훈)
docker cp markch_moveit2_dev:/tmp/c2_${STAGE}_ros.json "$OUTD/ros/c2_${STAGE}_ros.json" 2>&1 | tail -1
mark all_done
echo "완료 — 원자료: $OUTD"
