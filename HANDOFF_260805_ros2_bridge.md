# 인수인계 — ROS2 제어 브릿지 (2026-08-05, 세션 260805-봉선화)

> **임시 문서다.** 세션 기록은 리포 밖이 원칙이나(07-25 방침) 타 PC 전달 채널이 git 뿐이라
> 08-03 과 같은 예외를 쓴다. **작업이 안착하면 이 파일을 삭제**하고 내용을 각 README 로 옮긴다.
>
> 09pc 의 `업무목록.md`·`작업일지.md` 는 git 밖이라 다른 PC 에서 볼 수 없다. 그래서 재개에
> 필요한 것만 여기 옮겨 적는다.

---

## 0. 지금 상태 한 줄

**실기 제어 프로토콜은 전부 확보됐고 URDF 정렬도 끝났다. 남은 건 궤적 실행 층 구현과 빌드 검증이다.**

| | 상태 |
|---|---|
| 실기 명령 프로토콜 | ✅ 20여 종 역설계·실증 |
| 절대 관절이동 PC 실증 | ✅ 도달오차 0.0000° |
| 연속궤적(블렌딩) | ✅ 실증 — 소묘 스트로크 실행 가능 확인 |
| URDF ↔ 실기 정렬 | ✅ 교정 완료 (RMS 0.006mm) |
| `hcr_bridge` 상태 층 | ✅ 빌드·**실기 연결 검증 완료** (`/joint_states` 발행, 규약 변환 실증) |
| `hcr_bridge` 궤적 층 | ⬜ 미착수 |

## 1. 다른 PC 에서 재개할 때 — 순서대로

### ① 환경 (실기 없이 가능)
```bash
git fetch && git switch markch/hcr5_ros2 && git pull
cd Nyang_Nyang_Atlier/src/moveit2
docker compose build            # libmosquitto·nlohmann-json·uuid 추가돼 있다
./run_container.sh shell
# ── 컨테이너 안 ──
cd ~/ws_moveit2 && colcon build --symlink-install && source install/setup.bash
```
09pc 에서 **빌드·실기 연결까지 검증했다** — 컨테이너 재빌드 후 `colcon build` 통과(15.8s),
실기에 MQTT 접속해 `/joint_states` 발행 확인, `joint_1 = 1.5708 rad`(=90.00°)로 홈 자세의
규약 변환값과 일치했다. 다른 PC 에서는 이미지가 없으므로 `docker compose build` 부터 새로 한다.
CMake 가 `pkg_check_modules(MOSQUITTO libmosquitto)`·`(UUID uuid)` 로 라이브러리를 찾는다.

### ② 실기 연결 (로봇이 필요할 때)
- 랜선을 그 PC 이더넷 포트 ↔ 컨트롤러에 직결. 컨트롤러 IP `192.168.0.20`
- PC 를 같은 서브넷 고정 IP 로:
```bash
nmcli connection add type ethernet ifname <NIC> con-name hcr5 \
  ipv4.method manual ipv4.addresses 192.168.0.100/24 ipv6.method disabled autoconnect no
nmcli connection up hcr5
ping -c2 192.168.0.20
```
- **함정**: 컨트롤러를 재부팅하면 링크가 끊겨 이 프로필이 내려간다(autoconnect=no) → 다시 `up`
- 읽기 전용 확인: `python3 src/drivers/hcr_comm/tools/mqtt_cmd.py pos`

### ③ 브릿지 첫 검증 (읽기 전용 — 안전)
```bash
ros2 launch hcr_bridge state_bridge.launch.py     # allow_motion 기본 false
ros2 topic echo /joint_states --once
```
RViz 에 RobotModel 을 띄우면 **실기 자세가 그대로 보여야 한다**. 안 맞으면 규약 변환 의심.

## 2. 다음 구현 — 궤적 실행 층 (T15)

`hcr_bridge` 에 이어 붙인다. 설계는 `src/moveit2/ws_moveit2/src/hcr_bridge/README.md` §5.

1. **`FollowJointTrajectory` 액션 서버** — MoveIt2 의 `moveit_controllers.yaml` 이 이미
   `/hcr_arm_controller/follow_joint_trajectory` 를 기대한다. `demo.launch.py` 에서
   `ros2_control_node`+spawner 를 빼고 브릿지가 이 액션과 `/joint_states` 를 직접 제공하면
   MoveIt 쪽 설정은 무수정이다
2. **궤적 → `program/plan` 변환** — 아래 §3 규칙대로
3. `program/play` 실행 후 `program/index`·`program/end` 로 진행 추적, `event/motion` 으로 도착 판정

**대안 경로**: 단발 자세 이동은 `move/joint/here` 로 충분하다(도달오차 0.0000°). 촬영 자세·홈 복귀는
이쪽이 간단하다. 연속 스트로크만 `program/plan` 이 필요하다.

## 3. 소묘 스트로크를 프로그램으로 만드는 규칙 (08-05 실증)

```
시작 노드:  {startVelocity:0, endVelocity:V, continues:true,  radius:0}
중간 노드:  {startVelocity:V, endVelocity:V, continues:true,  radius:0}
종료 노드:  {startVelocity:V, endVelocity:0, continues:false, radius:0}
```

- `continues:true` 로 이은 전환에서 TCP 속도가 **0 을 안 찍고 19~20mm/s 유지**됨을 실측했다.
  `continues:false`+`endVelocity:0` 전환은 0.02mm/s 로 완전히 멈춘다
- ⚠️ **`radius`>0 을 스트로크 내부에 쓰지 마라.** radius=50 이면 궤적이 그 웨이포인트를
  **22.3mm 떨어져** 지나간다(모서리를 깎는다). radius=0 은 0.34mm 로 정확 통과한다.
  선 모양을 지키려면 내부는 radius:0 + continues:true 조합이다
- linear 직선성은 179mm 구간에서 **이탈 0.045mm** — 로봇 반복정밀도(±0.1mm)보다 좋다
- 전역 velocity(`set/velocity`, 0~1)는 노드 velocity 에 **곱해진다**: 50mm/s × 0.51 = 실측 20~31mm/s
- **속도·가속도가 안 맞으면 "특이점" 으로 오진하기 쉽다** — §5 참조

## 4. 반드시 지킬 것

### 관절 규약 변환 — 빼먹으면 조용히 틀린다
```
q_URDF[i](도) = SIGN[i] * q_real[i](도) + DELTA[i]
SIGN  = (+1, +1, -1, +1, +1, +1)     ← J3 만 부호 반전
DELTA = ( 90,  90,  0,  90,  0,  0)
```
`include/hcr_bridge/joint_convention.hpp` 에 구현돼 있다. 실기 홈 `[0,-90,-90,-90,90,0]`
= URDF `[90,0,90,0,90,0]`, 그 자세의 flange 는 (490.0, -170.5, 441.5) rx=-180.
**에러가 안 나고 엉뚱한 자세로 가므로 가장 위험한 실패 방식이다.**

### 안전
- **PC MQTT 명령은 펜던트 데드맨(인에이블 스위치)을 거치지 않는다.** 물리 e-stop 만이 최후 수단
- `move/joint/here` 는 한 번 발행하면 목표까지 자율 주행한다 — 시간 기반 정지가 성립하지 않는다.
  `mqtt_cmd.py movej` 의 3중 가드(이동량 상한·관절한계·도착 타임아웃 후 `move/stop`)를 계승할 것
- **ack ≠ 도착.** ack 는 접수일 뿐이고 도착은 `event/motion {"event":"moveHere"}` 가 알린다
- 충돌은 **래치된다**(PAUSED) — `event/collision/clear` 로 명시 해제
- **서보를 켠 채 오래 두지 마라** — §5 참조

## 5. 08-05 에 겪은 함정 (재발 방지)

**드라이브 0x40 장애** — 아무 명령도 없는 상태에서 6축 드라이브가 0.8초 만에 전부 트립
(`error/network` 280002 `EVENT_ERROR_DRIVE_ERROR`). 드라이브 1부터 순차 전파(EtherCAT
데이지체인). 이후 `EVENT_WARNING_COMMUNICATION_SDO_READ` 122건·Working Counter 이상
3160회로 필드버스 두절. 소프트 리셋으로 안 붙어 **컨트롤러 재부팅으로 복구**했다.
배경: 축 온도가 세션 시작 37°C → 장애 직전 **58~61°C**(서보를 1시간 반 켜둔 채 방치).
단정은 못 한다(가장 뜨거운 축은 wrist3 인데 먼저 죽은 건 base). **서보를 오래 켜두지 말 것.**

> **2026-08-05 세션 종료 시점에 Mark 가 서보를 켜둔 채로 두었다.** 다음 세션에서 실기를 만지기
> 전에 **축 온도부터 확인**하라: `python3 tools/mqtt_cmd.py pos` 로 통신을 보고,
> `mqtt_sub.py ... monitor/robot` 으로 temp 를 볼 것. 60°C 를 넘었거나 `-1` 이면
> (통신 두절) 위 장애가 재발한 것이다.

**"특이점" 오진** — `error/command` 150033 의 메시지가 *"there might be singular points"* 로
**추정형**이다. 08-05 의 실제 원인은 불가능한 속도 프로파일이었다: `acceleration:100`mm/s² 로
`velocity:500`mm/s 에 도달하려면 1250mm 가 필요한데 구간이 112~261mm 였고, `startVelocity:500`
은 직전 구간이 낼 수 없는 속도였다. **velocity 를 50 으로 낮추자 무에러 완주.** 기구학적으로도
전 웨이포인트가 J5=90° 라 손목 특이점(J5=0)에서 가장 먼 자세였다.

**설정 화면은 MQTT 를 안 쓸 때가 있다** — 툴·속도 설정 조작이 캡처에 전혀 안 잡힌 세션이 있었다
(mongoLog 0건). REST(4000/8000)로 가는 것으로 보이고 라우트 추측 탐색은 전부 404 였다.
단 **적용까지** 하면 `robot/setup/tcp` 가 MQTT 로 실린다. 등록만 하고 적용을 안 하면 안 실린다.

**관리자 계정** — 안전 설정 화면은 로그인이 필요하다. 기본값 `Admin`/`170502`, `user`/`hcr5`.

## 6. 아직 모르는 것 (우선순위 순)

1. **`program/plan` 크기 한계** — 스트로크 수백 개 × waypoint 당 tcp+flange+joint 3표현이면
   수 MB 급 JSON 이 된다. Mosquitto 1.4.7·컨트롤러 파서가 받아줄지 미실측. **완주(AC1)의 잠재
   차단 요인**이고 캡처가 아니라 **부하 실험**으로만 확정된다. 한계 발견 시 `program/play
   {selectedIndex:[a,b]}` 부분 실행으로 배치 분할이 가능하다
2. **즉시정지 지연** — `program/stop` 은 확보됐으나 실행 중 유효성·지연(BRD N4 "즉시")이 미실측
3. **펜 장착 시 TCP** — `robot/setup/tcp` 로 설정하거나, 전 구간 flange 기준으로 제어하고 펜
   오프셋을 PC 가 소유하는 우회가 가능하다. 현재 `tool1` = `{X:-52.25, Y:-13.49, Z:116.12}`(128.05mm)
4. **필압 ↔ 충돌감지** — 펜이 종이를 누르는 반력이 전류 기반 충돌감지를 트립시키면 PAUSED 래치로
   작화가 반복 중단된다. 실험 필요
5. Script 노드 컨테이너 형식 · arc/circle middlePoint · waypoint relative/variable · REST 라우트

## 7. 참조

| 무엇 | 어디 |
|---|---|
| 브릿지 설계·규약·안전 | `src/moveit2/ws_moveit2/src/hcr_bridge/README.md` |
| **명령 프로토콜 (원본)** | `src/drivers/hcr_comm/README.md` — 08-05 실측 반영 완료 |
| 실기 제어 CLI | `src/drivers/hcr_comm/tools/mqtt_cmd.py` (movej·pos·fk·ik 등) |
| 명령 캡처 도구 | `src/drivers/hcr_comm/tools/capture.py` |
| 기구학 교정 근거 | `hanwha_robot_arm/HCR_5/hcr_robot_description/urdf/hcr_robot.xacro` 상단 주석 |
| 커밋 | `6846588` URDF 교정 · `b650b27` hcr_bridge · `e2b9b4b` movej·FK/IK |
