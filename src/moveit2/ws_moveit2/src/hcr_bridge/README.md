# hcr_bridge — HCR-5 실기 ↔ ROS2 브릿지

> 한화 HCR-5(2018 1세대, Rodi 1.003.005)를 **벤더 플러그인 없이** ROS2 에 붙인다.
> 컨트롤러의 네이티브 MQTT 버스를 직접 말하는 방식이며, 프로토콜은 펜던트 관찰로
> 역설계했다 → [`src/drivers/hcr_comm/README.md`](../../../../drivers/hcr_comm/README.md)

*2026-08-05 착수. 상태 브릿지(독립 노드) + **ros2_control 하드웨어 플러그인**(08-09 신설, 실기 미검증) 구현.*

---

## 0. 왜 ros2_control 이 아닌가 → **부분 재검토 중** (08-09)

커뮤니티 드라이버(`hcr_control/RobotSystem`)는 Rodi 2.x 플러그인을 요구해 이 로봇에 설치할 수 없다.
네이티브 MQTT 경로는 1.x 에서 그대로 동작하므로 이쪽을 쓴다. **이 부분은 그대로다.**

08-05 에 `SystemInterface` 플러그인 대신 **독립 노드**를 택한 이유는 셋이었다:

| | 이유 | 08-09 현재 |
|---|---|---|
| 주기 불일치 | 상태 버스가 **29.1Hz**, `controller_manager` 는 100Hz. `read()`/`write()` 를 100Hz 로 돌리면 없는 데이터를 만들어내야 한다 | ⚠️ **무너졌다** — ros2_control **4.45.2** 의 컴포넌트별 `rw_rate`·`is_async` 로 주기를 분리할 수 있다(사실, mock 확인) |
| 실행 모델 | `move/joint/here` 는 **한 번 발행하면 목표까지 자율 주행**한다(실측: 명령 없이 6.3초간 61° 이동 후 도착). 매 주기 목표를 밀어넣는 `write()` 모델과 맞지 않는다 | **유효** — 그래서 플러그인의 명령 경로는 **점대점 한정**이다 |
| 연속 궤적 | 소묘 스트로크는 `program/plan` 의 블렌딩(`radius`·`continues`)으로 실행한다 — 컨트롤러가 보간을 소유하는 구조라 ros2_control 의 궤적 소유권과 충돌한다 | **유효** — 스트로크는 `program/plan` 에 그대로 남긴다 |

그래서 08-09 에 **`HcrSystemInterface` 를 실제로 구현했다**(`src/hcr_system_interface.cpp`). 둘은 배타가 아니다 —
플러그인은 **점대점 + 상태 피드백** 계층이고, 연속 스트로크는 여전히 `program/plan` 이다.

> **이것이 08-05 결정의 '반전'인지 '병행'인지는 아직 판정하지 않았다.** 판정에는 실기 실측(B·C)이 필요하고,
> 기록 자리는 `ros2_control_hw_interface/최종결과물.md` 의 '차이' 절이다. 계약·실측 현황은
> [`중간결과물.md`](../../../../drivers/hcr_comm/ros2_control_hw_interface/중간결과물.md).

MoveIt2 는 **계획·검증**을 맡는다. 실행은 — 점대점이면 JTC→플러그인, 연속 스트로크면 컨트롤러에 위임한다.

## 1. 관절 규약 — 빠뜨리면 조용히 틀린다 ⚠️

실기와 URDF 는 **영점 규약이 다르다**. `include/hcr_bridge/joint_convention.hpp` 가 이를 담는다.

```
q_URDF[i](도) = SIGN[i] * q_real[i](도) + DELTA[i]
  SIGN  = (+1, +1, −1, +1, +1, +1)      ← J3 만 부호 반전
  DELTA = ( 90,  90,   0,  90,   0,  0)
```

- 실기 zero = 팔이 **수평으로 뻗은** 자세 (flange z = −2.5mm)
- URDF zero = 팔이 **수직으로 선** 자세 (flange z = +1063mm)
- 실기 홈 `[0, −90, −90, −90, 90, 0]` = URDF `[90, 0, 90, 0, 90, 0]`

이 변환과 URDF origin 교정을 함께 적용하면 109개 자세에서 **위치오차 RMS 0.0060mm**.
**변환을 빼면 에러 없이 전혀 다른 자세로 간다** — 가장 위험한 실패 방식이다. 근거: 업무목록 T16.

관절 매핑(6축 조그 실측 확정): `joint_1~6` ↔ `base·shoulder·elbow·wrist1·wrist2·wrist3`,
축간 간섭 없음, 부호는 6축 모두 증가 방향 일치, 영점 오프셋 없음(펜던트 표시 = 버스 값).

## 2. 구성

| 파일 | 역할 |
|---|---|
| `include/hcr_bridge/joint_convention.hpp` | 규약 변환·가동범위·홈 자세 (헤더 온리) |
| `include/hcr_bridge/mqtt_client.hpp` · `src/mqtt_client.cpp` | MQTT 버스 클라이언트 + `pubWithAck` RPC |
| `src/state_bridge_node.cpp` | 상태 → `/joint_states`, 서보·홈 서비스 (**독립 노드** 경로) |
| `include/hcr_bridge/hcr_system_interface.hpp` · `src/hcr_system_interface.cpp` | **ros2_control 하드웨어 플러그인** (§0) — MoveIt2 실행 경로용 |
| `hcr_bridge_plugins.xml` | pluginlib 등록 — 클래스명 `hcr_bridge/HcrSystemInterface` |
| `launch/state_bridge.launch.py` | 기동 (기본 읽기 전용) |

두 경로는 **동시에 쓰지 않는다.** 독립 노드는 `/joint_states` 를 직접 내고, 플러그인은
`joint_state_broadcaster` 를 통해 낸다 — 같이 띄우면 같은 토픽에 둘이 발행한다.

### 봉투 규약

```
보낼 때  {"type":"pub"|"pubWithAck", "uuid":U, "data":{"thng_id":1, ...}}
pubWithAck → **uuid 와 같은 이름의 토픽**으로 응답:
         {"type":"pub", "uuid":.., "data":{"code":0,"data":{...},"msg":"success"}}
```

응답 알맹이는 **한 겹 안쪽**(`data.data`)이다. `MqttClient::request()` 가 벗겨서 돌려준다.

**ack ≠ 도착.** ack 는 명령 접수일 뿐이고, 이동 완료는 로봇이 `event/motion {"event":"moveHere"}` 로 알린다.

## 3. 쓰기

```bash
# 읽기 전용 (기본) — 로봇을 절대 움직이지 않는다
ros2 launch hcr_bridge state_bridge.launch.py

# 동작 지령 서비스까지 열기 — e-stop 대기 상태에서만
ros2 launch hcr_bridge state_bridge.launch.py allow_motion:=true
```

```bash
ros2 topic echo /joint_states --once
ros2 service call /hcr_state_bridge/set_servo std_srvs/srv/SetBool "{data: true}"
ros2 service call /hcr_state_bridge/go_home  std_srvs/srv/Trigger
```

RViz 에 `RobotModel` 을 띄우면 **실기의 실제 자세가 그대로 보인다**(교정된 URDF 기준).

## 4. 안전 ⚠️

- **PC MQTT 명령은 펜던트 인에이블 스위치(데드맨)를 거치지 않는다.** 그래서 동작 지령은
  `allow_motion` 파라미터로 잠가 뒀고 기본값이 `false` 다
- 상태 수신이 1초 이상 끊기면 `/joint_states` 발행을 **멈춘다** — 낡은 자세를 참으로 믿게 두지 않는다
- 필드버스 상태(`controllerStatus`)를 감시한다. `FIELD_BUS_SW_STATE_CONNECTED` 가 아니면 에러 로그.
  08-05 에 드라이브 6축이 동시에 트립해(에러 0x40) 필드버스가 두절된 적이 있다 — 컨트롤러 재부팅으로 복구
- 충돌은 **래치된다**(`PAUSED`). `event/collision/clear` 로 명시 해제해야 재개된다
- 서보를 켠 채 오래 두지 말 것 — 홀딩 토크로 축 온도가 37→60°C 까지 오른다

## 5. 다음 (T15)

**ros2_control 경로 — 점대점 + 상태 피드백** (08-09 신설)

### 기동 전 — 컨테이너 진입

아래 명령은 전부 **개발 컨테이너 안에서** 돈다. 환경 구축은
[`src/moveit2/README.md`](../../../README.md) 가 원본이고, 여기서는 **들어가는 법만** 든다.

```bash
# 표준 경로 — 이 PC에 이 클론 하나뿐이면 이걸로 끝난다
cd src/moveit2 && ./run_container.sh shell
```

⚠️ **같은 리포의 클론이 이 PC에 둘 이상이면 위 명령을 쓰면 안 된다.**
compose 는 **디렉터리 이름**으로 프로젝트를 식별하는데 양쪽 다 `moveit2` 라, 클론이 둘이면
프로젝트명이 겹쳐 **compose 가 남의 컨테이너를 자기 것으로 인식해 지우고 다시 만든다.**
`container_name` 만 바꿔서는 못 막는다 — compose 는 이름이 아니라 **라벨**로 찾기 때문이다.
(04pc 실측, 2026-08-10. 트리 밖에 08-05 이전 클론이 살아 있어 실제로 성립해 있었다)

격리하려면 PC 로컬 `compose.override.yml`(커밋 대상 아님)에 **셋을 다 갈라 놓는다** —
최상위 `name:`(프로젝트) · `container_name:` · `image:`. 이미지 태그까지 가르는 이유는
`compose.yml` 의 `image: moveit2_dev:jazzy` 가 **남의 이미지 이름이기도 해서** 그냥 build 하면 덮어쓰기 때문이다.

```bash
# 격리한 PC — 자기 컨테이너 이름부터 확인한다
docker ps -a --filter name=moveit2_dev --format '{{.Names}}\t{{.Image}}\t{{.Status}}'
#   markch_moveit2_dev   moveit2_dev:jazzy-markch   Up 15 minutes        ← 내 것
#   moveit2_dev          moveit2_dev:jazzy          Exited (137) 42시간   ← 남의 것

# 이름이 비슷해 헷갈리면 마운트 경로가 판정한다 — 내 클론을 물고 있는 쪽이 내 것이다
docker inspect <이름> --format '{{range .Mounts}}{{println .Source}}{{end}}'

docker exec -it <이름> bash        # ← 진입
```

> `--filter ancestor=moveit2_dev` 는 쓰지 마라 — **태그까지 정확히** 맞아야 걸려서 빈 결과가 나온다.
> 위처럼 `name=` 부분일치가 안전하다(실측).

> ⚠️ 격리한 PC에서는 **`run_container.sh` 를 쓰지 마라.** 그 스크립트는 `compose.override.yml` 을
> 자동으로 읽지만 내부 `CONTAINER="moveit2_dev"` 가 **하드코딩**이라 `shell`·`logs` 가 남의 컨테이너를 가리킨다.
> `docker compose -f compose.yml -f compose.override.yml <up|down|build>` 와 `docker exec` 를 직접 쓴다.

**이미지를 재사용해도 되는지는 크기가 아니라 `Dockerfile` 대조로 판단한다.**
04pc 에서 기존 `moveit2_dev:jazzy`(6.13GB, 크기 동일)를 그대로 썼다가
`libmosquitto-dev`·`nlohmann-json3-dev` 가 빠져 있어 `find_package(nlohmann_json)` 에서 빌드가 깨졌다 —
**남의 Dockerfile 로 만들어진 이미지**였기 때문이다(그 2종은 2026-08-05 에 우리 쪽에만 들어갔다).
진입 후 한 줄로 확인할 수 있다:

```bash
ls /usr/share/cmake/nlohmann_json/nlohmann_jsonConfig.cmake /usr/include/mosquitto.h
```

### 기동

```bash
# 실기 전환은 인자 하나다. 기본값은 mock 이라 인자 없이 띄우면 로봇이 필요 없다.
ros2 launch hcr_moveit_config demo.launch.py \
    hardware_plugin:=hcr_bridge/HcrSystemInterface rw_rate:=30 is_async:=true

# 실기 쓰기 — ⚠️ 로봇이 움직인다. 서보 ON·e-stop 대기·입회를 갖추고서만
ros2 launch hcr_moveit_config demo.launch.py \
    hardware_plugin:=hcr_bridge/HcrSystemInterface rw_rate:=30 is_async:=true \
    allow_motion:=true
```

`allow_motion` 은 §4 의 잠금과 **같은 이름·같은 기본값(`false`)** 이지만 전달 경로가 다르다 —
독립 노드는 ROS 파라미터로 받고, 플러그인은 **URDF `<hardware>` 의 `<param>`** 으로 받는다.
xacro 4단 관통은 `cc5a319`(2026-08-10). 인자를 잊으면 `false` 라 로봇은 움직이지 않는다.

- [x] `HcrSystemInterface` 구현 — 라이프사이클 9행·안전게이트 7건. 빌드·플러그인 로드까지 실측
- [ ] **실기 읽기 검증(B)** — 상태 정합·`read()` 실효 주기·통신 두절 거동·차분 velocity 품질
- [ ] **실기 쓰기 검증(C)** — 게이트가 실제로 막는가 → 단발 점대점 → 재발행 선점 → JTC 궤적 관통
- [ ] 실기 없이 실기 모드를 띄우면 **controller_manager 가 abort** 한다 — 완화책 확정 필요

**연속 스트로크 경로 — 이 계층 밖이다**

- [ ] `FollowJointTrajectory` 액션 서버 — MoveIt2 궤적 수신
- [ ] 궤적 → `program/plan` 변환. 스트로크 내부는 `continues:true`·`radius:0`(정확 통과),
      스트로크 경계에서만 정지. **`radius`>0 은 모서리를 22mm 깎으므로 선 안에서 쓰면 안 된다**(실측)
- [ ] `program/play` 실행 + `program/index`·`program/end` 로 진행 추적
- [ ] `program/plan` 크기 한계 실측 — 스트로크 수백 개면 수 MB 급 JSON 이 된다 (완주의 잠재 차단 요인)
- [ ] 즉시정지(`program/stop`) 지연 실측 — BRD N4 "즉시" 요건
