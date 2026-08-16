# hcr5_bridge — HCR-5 실기 ↔ ROS 2 브릿지

한화 HCR-5(2018 1세대, Rodi 1.003.005)를 **벤더 플러그인 없이** ROS 2 에 붙인다.
컨트롤러의 네이티브 MQTT 버스를 직접 말하며, 프로토콜은 펜던트 관찰로 역설계했다
(→ [`hcr5_comm/README.md`](../hcr5_comm/README.md)).

> 2026-08-16 재조립. 종전 `README.md` · `운전절차.md` · `RUNTIME.md` 셋을 해체해
> **1.개요 / 2.설치 / 3.실행** 세 항목으로 합쳤다. 원본은 `WorkSpace_260814/_보관_260816/` 에 있다.

---

# 1. 개요

## 1.1 명령어 모음집 — 진입해서 돌린다

컨테이너에 **들어가서 치는** 형태만 적는다. 호스트에서 한 줄로 던지는 형태는 §3.9.
⚠️ **한 줄씩** 친다 — 블록을 통째로 붙여넣으면 첫 줄이 새 셸을 열어 뒤가 빨려 들어간다(§3.1).

**터미널 1 — 진입**

```bash
cd ~/WorkSpace_260814/Nyang_Nyang_Atlier/src/drivers/hcr5_bridge && ./run_container.sh shell
```

> ⚠️ **2026-08-16 재작성.** 종전 이 자리는 `bringup.launch.py` 를 띄운 뒤 `movej` 를 쏘라고
>안내했다. **지금 그렇게 하면 명령이 부딪힌다** — `movej`·`movel`·`servo` 셋 다 스택과 **배타**다
> (§1.2 · [`movej.md`](movej.md) §1.3). 수동 운전에 스택은 **띄우지 않는다.**

**서보 ON**

```bash
ros2 run hcr5_bridge servo on
```

**movej — 관절 목표 (홈 → P1, base +10°)**

```bash
ros2 run hcr5_bridge movej 1.745329 0 1.570796 0 1.570796 0 --speed 2 --dry-run
```

⚠️ 숫자는 **URDF 라디안**이 기본이다(실기 도 아님). 같은 점을 실기 도로 주려면 `--deg 10 -90 -90 -90 90 0`.
`--speed` 는 **실기 도/s** — 기본 2, 상한 10.
🔴 그 단위가 아직 미확정이다 — **첫 이동 전에 [`movej.md`](movej.md) §2.3 을 읽는다.**
`--dry-run` 을 빼면 **움직인다.**

**정지**: `ros2 run hcr5_bridge servo off` · 이상 시 정지 사다리 §3.7

> 🔴 이 컨테이너는 아직 안 세웠다 — 진입 줄은 지금 `cd ~/WorkSpace_260814/Nyang_Nyang_Atlier/src/moveit2 && ./run_container.sh shell` 이다(§2.1).
> 착수 전 확인 §3.2 · 기동 §3.4 · 지령 §3.5 · 정지 사다리 §3.7.

### movel — 같은 문, 직교 직선

`movej` 와 **같은 자리**다(스택 없이 단독). 다른 것은 구속 대상뿐이다 —
`movej` 는 관절을, `movel` 은 엔드이펙터 궤적을 구속한다.

```bash
ros2 run hcr5_bridge movel world 0 0 20 --dry-run
```

```
ros2 run hcr5_bridge movel <좌표계> <x> <y> <z> [옵션]
```

| 좌표계 | 기준 | 예 |
|---|---|---|
| `base` | 실기 base **절대** (mm) | `movel base 490 -170.5 441.5` |
| `world` | base 축 방향 **상대** Δ | `movel world 0 0 20` — 수직 20mm ↑ |
| `tool` | 현재 툴 축 기준 **상대** Δ | `movel tool 0 0 -10` — 툴 방향 10mm |

- **단위는 늘 mm.** 인자에 **자세가 없다** — 현재 자세를 그대로 유지한다([`movel.md`](movel.md) §2.3).
- 주요 옵션: `--speed`(기본 **50** mm/s) · `--accel`(1000 mm/s²) · `--max-dist`(100mm) ·
  `--tcp`｜`--flange`(제어점, 기본 flange) · `--dry-run` · `--plan-only`(발행하되 **play 안 함**)
- ⚠️ `--dry-run` 을 빼면 **움직인다.** 겁나면 `--plan-only` 로 적재까지만 해 보고 `program/clear` 로 지운다.

✅ **경로는 직선이다** — 컨트롤러가 보간을 소유한다. 실측 수직편차 **정속구간 최대 0.0184mm ·
RMS 0.0074mm / 100mm 구간**(2026-08-16, 로봇 반복정밀도 ±0.1mm 안). 근거는 [`movel.md`](movel.md) §1.4.

> 🔴 **`--speed` 기본값을 500 으로 올리지 마라.** 컨트롤러 기본이 500mm/s 인데,
> 가속 100mm/s² 로 500 에 도달하려면 1250mm 가 필요해 짧은 구간에서 프로파일이 성립하지 않는다.
> 08-05 에 이것이 `error/command` **150033 "there might be singular points"** 로 나왔다 —
> **특이점이 아니었다**(§3.8 · `hcr5_comm/README.md` §8 함정 ①). movel 은 발행 전에 이 조건을 검사한다.

## 1.2 실행 경로는 셋이고, 서로 배타다 ⚠️

| | ros2_control 경로 | 독립 노드 경로 | **`program/plan` 경로** |
|---|---|---|---|
| 진입 | `bringup.launch.py` | `state_bridge.launch.py` | **스택 불필요** — 실행기만 돌린다 |
| 구현 | `HcrSystemInterface` (하드웨어 플러그인) | `state_bridge_node` | `movej` · `movel` · `servo` (rclcpp 없음) |
| `/joint_states` | `joint_state_broadcaster` 가 낸다 | 노드가 **직접** 낸다 | **없다** — MQTT `get/command/pos` 로 읽는다 |
| 지령 | `FollowJointTrajectory` 액션 (**MoveIt 전용**) | ROS 서비스 (`set_servo` · `go_home`) | MQTT `program/clear`→`plan`→`play` |
| 보간 소유 | JTC (관절공간) | — | **컨트롤러 (관절·직교 양쪽)** |
| 쓰임 | MoveIt 계획의 실행 | 상태 확인·초기 검증 | **실기 수동 운전 전부** |

**셋을 동시에 띄우지 않는다.** 앞 둘은 같은 `/joint_states` 에 겹쳐 발행해서 안 되고,
셋째는 **명령 인터페이스가 부딪혀서** 안 된다 — 플러그인이 관절 지령을 계속 쓰는 동안
컨트롤러가 자기 궤적을 실행하면 두 지령이 싸운다.

> ⚠️ **운전 실행기 셋이 2026-08-16 에 전부 첫째 칸에서 셋째 칸으로 옮겨졌다.**
> 오전에 `movel`(종전에는 JTC 액션을 쐈고 플랜지 경로가 직선이 아니었다 — [`movel.md`](movel.md)),
> 오후에 `movej`([`movej.md`](movej.md), 세션 260816-호크니)가 옮겨졌다.
> **첫째 칸에는 이제 사람이 직접 쏘는 실행기가 없다.** 스택을 띄웠다면 이 셋을 **쓰면 안 된다.**

**MoveIt2 는 계획·검증을 맡는다.** 그 실행만 JTC→플러그인으로 가고, 사람이 터미널에서 하는
수동 운전은 관절이든 직교든 `program/plan` 으로 컨트롤러에 위임한다. 스트로크 경로는 **이 계층 밖**이다.

## 1.3 관절 규약 — 빠뜨리면 조용히 틀린다 ⚠️

실기와 URDF 는 **영점 규약이 다르다**. `include/hcr5_bridge/joint_convention.hpp` 가 이를 담는다.

```
q_URDF[i](도) = SIGN[i] * q_real[i](도) + DELTA[i]
  SIGN  = (+1, +1, −1, +1, +1, +1)      ← J3 만 부호 반전
  DELTA = ( 90,  90,   0,  90,   0,  0)
```

- 실기 zero = 팔이 **수평으로 뻗은** 자세 (flange z = −2.5mm)
- URDF zero = 팔이 **수직으로 선** 자세 (flange z = +1063mm)
- 실기 홈 `[0, −90, −90, −90, 90, 0]` = URDF `[90, 0, 90, 0, 90, 0]`

이 변환과 URDF origin 교정을 함께 적용하면 109개 자세에서 **위치오차 RMS 0.0060mm**.
**변환을 빼면 에러 없이 전혀 다른 자세로 간다** — 가장 위험한 실패 방식이다.

관절 매핑(6축 조그 실측 확정): `joint_1~6` ↔ `base·shoulder·elbow·wrist1·wrist2·wrist3`,
축간 간섭 없음, 부호는 6축 모두 증가 방향 일치, 영점 오프셋 없음(펜던트 표시 = 버스 값).

## 1.4 봉투 규약

```
보낼 때  {"type":"pub"|"pubWithAck", "uuid":U, "data":{"thng_id":1, ...}}
pubWithAck → uuid 와 같은 이름의 토픽으로 응답:
         {"type":"pub", "uuid":.., "data":{"code":0,"data":{...},"msg":"success"}}
```

응답 알맹이는 **한 겹 안쪽**(`data.data`)이다. `MqttClient::request()` 가 벗겨서 돌려준다.
**ack ≠ 도착** — ack 는 명령 접수일 뿐이고, 이동 완료는 로봇이 `event/motion` 으로 알린다.

## 1.5 구성

| 파일 | 역할 |
|---|---|
| `include/hcr5_bridge/joint_convention.hpp` | 규약 변환·가동범위·홈 자세 (헤더 온리) |
| `include/hcr5_bridge/mqtt_client.hpp` · `src/mqtt_client.cpp` | MQTT 버스 클라이언트 + `pubWithAck` RPC |
| `include/hcr5_bridge/hcr_system_interface.hpp` · `src/hcr_system_interface.cpp` | **ros2_control 하드웨어 플러그인** |
| `include/hcr5_bridge/pose_convention.hpp` | 직교 자세 규약 — **ZYX 오일러** (2026-08-16 실측 확정) |
| `include/hcr5_bridge/servo_gate.hpp` | **게이트 ⑤** — `program/play` 전 서보 상태 확인. `movej`·`movel` 공용 |
| `include/hcr5_bridge/move_common.hpp` | ⚠️ **고아** — JTC 경로의 공통 게이트였다. 2026-08-16 `movej` 이관으로 사용처가 없어졌다 ([`movej.md`](movej.md) §4) |
| `src/movej.cpp` · `src/movel.cpp` · `src/servo.cpp` | 운전 실행기 (`ros2 run hcr5_bridge …`) — 셋 다 **rclcpp 없음** |
| `src/state_bridge_node.cpp` | 독립 노드 경로 |
| `hcr5_bridge_plugins.xml` | pluginlib 등록 — 클래스명 `hcr5_bridge/HcrSystemInterface` |
| `launch/bringup.launch.py` · `launch/state_bridge.launch.py` | 두 경로의 진입 |
| `config/ros2_controllers.yaml` · `rviz/hcr5.rviz` | 컨트롤러 설정 · RViz2 레이아웃 |
| `Dockerfile` · `compose.yml` · `run_container.sh` · `entrypoint.sh` | **개발 컨테이너** (§2) |

`servo` 만 `rclcpp` 의존이 없다 — 스택이 죽은 뒤에도 단독으로 돌아야 하기 때문이다(§3.7 정지 사다리 3순위).

---

# 2. 설치

## 2.1 컨테이너는 이 디렉터리에 있다

**docker 진입에 필요한 것이 전부 여기 있다** — `Dockerfile` · `compose.yml` · `run_container.sh` ·
`entrypoint.sh`. 종전에는 `src/moveit2/` 것을 빌려 썼고, 2026-08-16 에 이리로 복제해 이름을 갈랐다.

| 항목 | `src/moveit2/` (팀 공용) | 여기 |
|---|---|---|
| compose 프로젝트 | `moveit2` (디렉터리명) | `nyang_nyang_atlier` |
| 이미지 | `moveit2_dev:jazzy` | `nyang_nyang_atlier:jazzy` |
| 컨테이너 | `moveit2_dev` | `nyang_nyang_atlier` |
| 워크스페이스 | `~/ws_moveit2` | `~/ws_atlier` |

**셋(프로젝트·이미지·컨테이너)을 다 갈라야 격리된다.** compose 는 컨테이너를 이름이 아니라
**라벨**로 찾기 때문에 `container_name` 만 바꾸면 남의 컨테이너를 자기 것으로 인식해
지우고 다시 만든다(04pc 실측 2026-08-10). 이미지까지 가르는 이유는 같은 태그로 build 하면
남의 이미지를 덮어쓰기 때문이다.

⚠️ **이미지·프로젝트 이름에 대문자를 못 쓴다** — Docker repository name 규칙이다.
`Nyang_Nyang_Atlier` 로는 build 가 `invalid reference format` 으로 죽는다. 컨테이너 이름만
대문자가 허용되지만 헷갈리지 않게 소문자로 통일했다.

> 🔴 **이 컨테이너는 아직 안 세웠다 (2026-08-16 현재).** 파일만 준비된 상태다.
> 지금 당장 실기를 돌리려면 기존 `moveit2_dev` 를 쓴다 — 그때는 워크스페이스가
> `~/ws_moveit2` 이고 진입이 `cd ../../moveit2 && ./run_container.sh shell` 이다.

## 2.2 세우기

```bash
cd ~/WorkSpace_260814/Nyang_Nyang_Atlier/src/drivers/hcr5_bridge && ./run_container.sh build
```

20~40분. 그 다음부터는 진입만 하면 된다:

```bash
cd ~/WorkSpace_260814/Nyang_Nyang_Atlier/src/drivers/hcr5_bridge && ./run_container.sh shell
```

`shell` 은 컨테이너가 없으면 띄우고 `docker exec -it nyang_nyang_atlier bash` 로 붙는다.
`docker compose` 를 직접 부르지 않는 이유는 UID/GID·렌더 GID·`XAUTHORITY` 가 안 맞을 수 있어서다.

**들어가면 `source` 가 필요 없다** — `.bashrc` 가 `/opt/ros/jazzy/setup.bash` 와
`~/ws_atlier/install/setup.bash` 를 둘 다 물어준다.

병합 결과 확인(`compose.override.yml` 이 제대로 얹혔는지):

```bash
./run_container.sh config
```

## 2.3 컨테이너가 보는 파일 — 볼륨 매핑

**호스트 배치는 그대로 두면서** 컨테이너 안에서만 colcon 워크스페이스 모양으로 보이게 하는 것이 요점이다.

| compose.yml | 호스트 | 컨테이너 |
|---|---|---|
| `./ws_atlier` | `src/drivers/hcr5_bridge/ws_atlier` | `~/ws_atlier` |
| `../../../hanwha_robot_arm` | `hanwha_robot_arm/` (리포 최상위) | `~/ws_atlier/src/hanwha_robot_arm` |
| **`../`** | **`src/drivers/`** | **`~/ws_atlier/src/drivers`** |

- `hanwha_robot_arm` 매핑이 빠지면 **아무것도 안 뜬다** — URDF 원본 `hcr_robot_description` 이 거기 있다
- `src/drivers/` 아래는 어떻게 재배치해도 컨테이너에서 그대로 보인다
- `package.xml` 이 없는 디렉터리(`hcr5_comm/` 통째)는 colcon 이 자동으로 건너뛴다 —
  `src/drivers/` 아래 **ROS 패키지는 `hcr5_bridge` 와 `nyang_pen` 둘**이다
- `hcr5_comm/tools/` 가 컨테이너에서 보이므로 `docker cp` 우회가 필요 없다

⚠️ **`ws_atlier/COLCON_IGNORE` 를 지우지 마라.** `../` 매핑 때문에 워크스페이스가 자기 자신을
한 겹 안쪽에서 다시 보고(`~/ws_atlier/src/drivers/hcr5_bridge/ws_atlier/install/…/package.xml`),
colcon 이 그걸 **두 번째 hcr5_bridge** 로 잡는다. 그 한 줄 파일이 막는다.

⚠️ **`compose.yml` 을 고치면 컨테이너를 재생성해야 붙는다.** 재시작만으로는 마운트가 안 바뀐다:

```bash
./run_container.sh down && ./run_container.sh shell
```

## 2.4 빌드

```bash
# 컨테이너 안 (진입했으면 source 불필요)
cd ~/ws_atlier && flock /tmp/atlier_colcon.lock colcon build --packages-select hcr5_bridge
```

- **`flock`** — 두 세션이 동시에 빌드하면 깨진다. 이 프로젝트는 병렬 세션이 흔해서 락을 건다
- **완전 재빌드**는 `build/hcr5_bridge` `install/hcr5_bridge` 를 지우고 한다. 소스 경로가 바뀌면
  `CMakeCache.txt` 가 stale 이 되어 **반드시** 필요하다 (실측: 단일 패키지 5~8초)
- `build/` `install/` `log/` 는 git 에 안 올린다 — 지워도 재생성된다
- ⚠️ **실기 스택이 떠 있는 중에 재빌드하지 않는다.** 플러그인 `.so` 를 갈아끼우는 것이라 위험하다

빌드 결과 확인:

```bash
ros2 pkg executables hcr5_bridge          # movej · movel · servo · state_bridge_node
ros2 control list_hardware_components     # 스택이 떠 있을 때만
```

**이미지를 재사용해도 되는지는 크기가 아니라 `Dockerfile` 대조로 판단한다.** 04pc 에서 크기가
같은 남의 이미지를 그대로 썼다가 `libmosquitto-dev`·`nlohmann-json3-dev` 가 빠져 있어
`find_package(nlohmann_json)` 에서 깨졌다. 진입 후 한 줄로 확인한다:

```bash
ls /usr/share/cmake/nlohmann_json/nlohmann_jsonConfig.cmake /usr/include/mosquitto.h
```

## 2.5 GPU · GUI

GPU 설정은 **PC 로컬 자산이라 git 에 안 올린다**(`compose.override.yml`, `.gitignore` 대상).
공용 `compose.yml` 에는 GPU 설정을 넣지 않는다 — 벤더가 PC마다 갈려서 공용 파일에 두면
pull 마다 충돌한다. override 없이 띄우면 RViz2 3D 가 소프트웨어 렌더링으로 느려진다(동작은 한다).

⚠️ **override 의 서비스 키는 `atlier` 다** (moveit2 쪽 스니펫은 `moveit2`). 키가 다르면 조용히 안 얹힌다.

RViz2 는 컨테이너가 호스트 X 서버에 그린다.

| 항목 | 값 | 확인 |
|---|---|---|
| 호스트 `DISPLAY` | `:1` | `echo $DISPLAY` |
| 컨테이너 `DISPLAY` | `:1` (compose 가 물려줌) | `docker exec nyang_nyang_atlier printenv DISPLAY` |
| X11 소켓 | `/tmp/.X11-unix` 마운트 | `docker exec nyang_nyang_atlier ls /tmp/.X11-unix` → `X1` |
| 접근 허용 | `LOCAL:` 항목 필요 | `xhost` |

안 뜨면 호스트에서 한 번 `xhost +local:root`.
⚠️ **컨테이너의 `DISPLAY` 는 생성 시점 값이 박힌다.** 호스트 `DISPLAY` 가 바뀌었으면(재로그인 등)
컨테이너를 재생성해야 한다.

## 2.6 이 PC 의 값 (03pc — markch03 / MSI Vector 16)

| 항목 | 값 |
|---|---|
| 리포 | `~/WorkSpace_260814/Nyang_Nyang_Atlier` |
| PC IP (실기 랜) | `192.168.0.100/24` · NIC `enp131s0` |
| 실기 컨트롤러 | `192.168.0.20` · MQTT `1883` |
| GPU | RTX 5080 Laptop (NVIDIA 스니펫 override) |

**실기 랜 프로필** — 컨트롤러를 재부팅하면 링크가 끊겨 프로필이 내려간다(`autoconnect no`):

```bash
nmcli connection up hcr5 && ping -c2 192.168.0.20
```

---

# 3. 실행

## 3.1 명령은 두 자리에서 돈다 — 이게 사고 지점이다 ⚠️

| 층 | 명령 | 도는 자리 |
|---|---|---|
| **인프라** | `run_container.sh` · `docker exec` · `xhost` · `nmcli` · `ping` | **호스트** |
| **로봇 제어** | `ros2 launch` · `ros2 run` · `python3 tools/…` · `colcon build` | **컨테이너 안** |

같은 로봇 명령을 **두 방식으로** 전달할 수 있고, 둘은 대체 관계가 아니다:

| 방식 | 형태 | 쓸 때 |
|---|---|---|
| **대화형** (§3.3~3.7) | 진입 후 `ros2 run hcr5_bridge movej …` | **사람이 로봇 앞에서.** 이게 기본이다 |
| **비대화형** (§3.9) | 호스트에서 `docker exec … bash -lc "source … && ros2 run …"` | 에이전트·스크립트·재현 |

⚠️ **두 형태를 섞으면 되던 게 안 된다.** `.bashrc` 는 **대화형 셸에서만** 읽히므로
`docker exec … bash -lc "ros2 …"` 는 `ros2: command not found` 가 난다. 비대화형은
`source` 를 매번 명시해야 한다 — 그것이 유일한 차이다.

⚠️ **여러 줄을 통째로 붙여넣지 않는다.** 블록 첫 줄이 `run_container.sh shell` 이나
`docker exec -it … bash` 면 **새 셸이 열리고 뒤따르는 줄들이 그 셸의 입력으로 빨려 들어간다.**
그래서 이 문서의 명령은 전부 **한 줄로 완결**되고 셸을 바꾸지 않는다. 함수 정의(`GO() { … }`)도
쓰지 않는다 — 그 셸에서만 살아서 창을 새로 열 때마다 다시 붙여넣어야 한다.

## 3.2 착수 게이트

| 항목 | 확인 | 기준 |
|---|---|---|
| 컨테이너 | `docker ps --filter name=nyang_nyang_atlier` | `Up` |
| 실기 도달 | `ping -c2 192.168.0.20` | PC 가 `192.168.0.100/24` 에 있어야 한다 |
| 축온 | §3.4 감시견 또는 `tools/mqtt_baseline.py` | **≤50°C** |
| 필드버스 | `status/operation.controllerStatus` | `FIELD_BUS_SW_STATE_CONNECTED` |
| `move_group` 중복 | `docker exec nyang_nyang_atlier ps -ef \| grep move_group` | **0개** |
| 사람 | — | **물리 e-stop 앞에 있을 것.** PC 명령은 펜던트 데드맨을 안 거친다 |

## 3.3 명령의 성격이 셋으로 갈린다

창 배치가 그 성격을 따른다 — 편의가 아니라 구조다.

| | 성격 | 수명 | 하는 일 | 실패하면 |
|---|---|---|---|---|
| **기동** | **상주** — 창을 점유 | 창이 사는 동안 | 로봇을 ROS 제어 **아래로 넘긴다**. `unconfigured→configured→active`, 명령 인터페이스 **claim** | 아무것도 못 한다 |
| **지령** | **일회** — 붙었다 뜬다 | 수 초 | 선 계에 **액션 goal 하나**를 얹는다 | 그 goal 만 실패, 계는 산다 |
| **정지** | 한 번 | — | 기동이 만든 상태를 **되돌린다**(`on_deactivate`) | ⚠️ **서보가 남는다** |

**지령을 백 번 해도 기동 상태는 안 바뀐다.** 반대로 기동을 다시 하면 지령이 하던 것이 끊긴다.

| 창 | 무엇 | 왜 전용 창인가 |
|---|---|---|
| **1** | 기동 | 창을 점유한다. **Ctrl+C 로 끝내는 창** |
| **2** | 축온 감시견 | 운전 내내 살아 있어야 한다 |
| **3** | 지령 | 아무 창에서 몇 번이든 — 여기가 자유롭다 |

세 창 모두 §2.2 로 진입한 뒤 시작한다.

## 3.4 기동 — 창 1

```bash
ros2 launch hcr5_bridge bringup.launch.py real:=true allow_motion:=true
```

**`move_group` 을 안 띄운다.** MoveIt 없이 `controller_manager` · `robot_state_publisher` ·
컨트롤러 2개 · RViz2 만 선다. 그것이 이 launch 의 요점이다.

| 모드 | 인자 | 로봇 |
|---|---|---|
| sim (mock) | 없음 | 안 움직임 · **실기 불필요** |
| 실기 읽기 | `real:=true` | 안 움직임 |
| 실기 쓰기 | `real:=true allow_motion:=true` | **움직인다 — e-stop 대기 필수** |

`real:=true` 하나가 xacro 인자 셋(`hardware_plugin:=hcr5_bridge/HcrSystemInterface` ·
`rw_rate:=30` · `is_async:=true`)을 자동으로 넣는다 — 손으로 줄 필요 없다.
`use_rviz:=false` 로 RViz2 를, `auto_servo:=false` 로 서보 자동화를 끈다.

**이 순서로 나와야 한다** — 하나라도 빠지면 멈춘다:

```
Configured and activated joint_state_broadcaster
Configured and activated hcr_arm_controller
활성화 — 명령 버퍼 = 실기 [...]°          ← 플러그인이 실기를 잡았다
서보 ON — ack 수신
```

그리고 **RViz2 창에 로봇 자세**가 뜬다.

> **서보 ON 이 컨트롤러 활성 *뒤*인 것이 안전 설계다.** `allow_motion=true` 로 활성화하면 궤적을
> 안 줘도 `move/joint/here` 2건이 자동 발행되는데, `on_activate` 가 `get/command/pos` 로
> 명령 버퍼를 **현재 실기 자세로 채우므로**(`hcr_system_interface.cpp:306-330`) 그 2건이
> 현재 자세 그대로다 — 로봇이 안 움직인다. 순서가 뒤집히면 그 보장이 깨진다.

**축온 감시견 — 창 2, 기동 직후**

```bash
cd ~/ws_atlier && python3 'src/drivers/hcr5_comm/measurements/260812_T15검증/verify/c1_tempwatch.py' 192.168.0.20 600 58 /tmp/manual_temp.jsonl
```

임계 도달 시 스스로 `servo off` 를 쏘고 죽는다. 정상 종료 메시지
`✅ 창 종료 — 임계 미도달` 은 *"임계에 안 닿았다"* 는 뜻이다 — 트립으로 오독하기 쉽다.

## 3.5 지령 — 창 3

### 넣는 숫자는 URDF **라디안**이다 ⚠️

`movej` 의 인자는 ROS 표준 SI 라 rad 이고, 그 rad 는 **URDF 좌표계** 기준이라
실기 도(度)와 한 겹 더 어긋나 있다. **두 겹을 한꺼번에** 넘겨야 한다.

> 📌 `movej` 가 2026-08-16 에 MQTT 경로로 옮겨간 뒤에도 **이 규약은 그대로다**(Mark 판단).
> MQTT 의 네이티브 단위는 도지만 변환은 실행기 **안쪽**에서 한다 — 밖으로는 계속 rad 로 말한다.
> 이유는 [`movej.md`](movej.md) §2.2. 발행 전에 rad·URDF도·실기도 **세 칸을 다 찍는다.**

```
실기 도 → 넣을 값(rad) :  q_URDF = (SIGN·q_실기 + DELTA) × π/180
넣을 값(rad) → 실기 도 :  q_실기 = (q_URDF × 180/π − DELTA) / SIGN
```

joint_1 만 펴서 적으면 —

| 넣는 값 (rad) | URDF 도 | **실기 도** | |
|---|---|---|---|
| `1.570796` | 90° | **0°** | 홈 |
| `1.745329` | 100° | **10°** | P1 |
| `1.919862` | 110° | **20°** | P2 |
| `15` | 859.44° | **769.44°** | ← 2.1 바퀴. 실제로 낸 사고다 |

⚠️ **실기 도를 그대로 넣으면 안 된다.** `15` 는 15° 가 아니라 15 rad 다. joint_1 을 실기 X° 로
보내려면 `(X + 90) × π/180` 을 넣는다 — 실기 15° 는 `1.832596` 이다.

### 속도 — 이제 컨트롤러가 받는다

> ⚠️ **2026-08-16 에 의미가 바뀌었다.** 종전에는 `--speed` 로 PC 가 `time_from_start` 를 계산해
> JTC 에 실었고 **실기에는 속도라는 개념이 전달되지 않았다.** 지금은 `move.joint.velocity` 로
> 컨트롤러에 **그대로 실린다.** 숫자와 기본값은 같지만 도달 경로가 다르다 ([`movej.md`](movej.md) §2.4).
> 그래서 **`--sec` 는 없어졌다** — 시간을 지정하는 칸이 `program/plan` 에 없다.

`--speed` 는 **각속도**, `--accel` 은 **각가속도**다. `movej` 가 관절 공간이라 그렇다.
선속도(mm/s)는 `movel` 의 것이다.

| 인자 | 뜻 | 기본 |
|---|---|---|
| `--speed D` | 각속도 **도/s** → `move.joint.velocity` | **30** |
| `--accel A` | 각가속도 **도/s²** → `move.joint.acceleration` | **100** (컨트롤러 joint 기본값) |
| `--max-speed D` | 각속도 상한 — 넘으면 거부 | **50** (펜던트 joint 기본값과 같은 자리) |
| `--max-delta D` | 최대 이동 축의 각변위 상한 — 넘으면 거부 | **90°** |

✅ **단위는 도/s 로 확정됐다** (2026-08-16 실기 **182점 · 6축 전부** — [`movej.md`](movej.md) §2.3,
`hcr5_comm/README.md` §6.1.1). **지령 속도는 오차 0.13% 로 그대로 지켜진다.**
다만 가감속이 사다리꼴이 아니라 램프에 `v/a` 의 1.54배가 걸린다:

```
소요시간 t = d/v + 1.54·(v/a) + 0.20초       (RMS 잔차 0.037초 — 실행기의 "예상" 이 이 식이다)
```

즉 **지령 가속도의 실효값은 0.65·a** 다. `--accel 100` 은 65°/s² 처럼 거동한다.
**축을 안 가린다** — J1(팔 전체 회전)부터 J6(툴축 회전)까지 같은 식이고, 중력 방향(올림/내림)도 무관하다.
**시각도 안 가린다** — 같은 지령 80회를 29분에 걸쳐 돌려도 세 항 다 끌리지 않는다.
다만 `0.20초` 항의 **소수 셋째 자리는 믿지 마라** — 회차 산포가 0.035초다.
실행기가 도착 후 실측/예상 비율을 계속 찍는데, 이제는 가설 검증이 아니라 **회귀 감시**다 —
비율이 1.00 근처를 벗어나면 전역 배율(`get/velocity`)부터 의심한다.

**속도 상한 기준은 「가장 많이 도는 축」이다.** P2 는 base 10° · wrist3 20° 라 wrist3 가 기준이다.
다축을 동시에 돌려도 **소요시간은 그 선행축 하나가 정한다** (벡터 노름 아님 — 실측 확인).

⚠️ **기본 30°/s 는 게이트 ③ 이 9°(`v²/a`)를 요구한다.** 그보다 짧은 이동은 거부되고,
그 구간에서 낼 수 있는 최대 속도를 알려준다 — `--speed` 를 낮추거나 `--accel` 을 올린다.
(실측 램프각은 `0.77·v²/a` = 6.9° 라 게이트가 30% 보수적이다. 안전 방향이라 그대로 뒀다.)

**상한 50°/s 위로 가려면** `--max-speed` 를 올리거나 `--force` 를 **명시**한다. (HCR-5 공식 최대는 180°/s.)
참고로 `move/joint/here` 자율주행 실측이 9.7°/s 였다 — 그것은 전역 배율에 묶인 다른 경로의 값이라
이 기본값의 근거는 아니다.

**`--max-delta` 는 새 칸이다.** `--speed` 가 각속도만 보므로 총 이동량은 이것만 본다 —
`movel` 의 직교 거리 상한과 같은 자리다.

### 좌표

| 점 | joint_1 | joint_2 | joint_3 | joint_4 | joint_5 | joint_6 | 실기 (도) | 비고 |
|---|---|---|---|---|---|---|---|---|
| **홈** | 1.570796 | 0 | 1.570796 | 0 | 1.570796 | 0 | `[0, −90, −90, −90, 90, 0]` | SRDF `hcr_home` |
| **P1** | **1.745329** | 0 | 1.570796 | 0 | 1.570796 | 0 | `[10, −90, −90, −90, 90, 0]` | base +10° |
| **P2** | **1.919862** | 0 | 1.570796 | 0 | 1.570796 | **0.349066** | `[20, −90, −90, −90, 90, 20]` | base +20° · wrist3 +20° |
| **1회전** | **7.853981** | 0 | 1.570796 | 0 | 1.570796 | 0 | `[360, −90, −90, −90, 90, 0]` | base 한 바퀴 |

**P1·P2 를 이 두 축으로만 잡은 이유** — 홈 자세에서 플랜지 높이가 **수학적으로 불변인 축은
joint_1(수직축 회전)과 joint_6(아래를 향한 툴축 회전) 둘뿐**이다. 나머지 축(2·3·4·5)은 플랜지를
내릴 수 있어 작업대 클리어런스를 사람이 따로 확인해야 한다. 첫 수동 운전은 이 둘로만 한다.

### 명령

```bash
ros2 run hcr5_bridge movej --deg 10 -90 -90 -90 90 0        # 홈 → P1
```
```bash
ros2 run hcr5_bridge movej --deg 20 -90 -90 -90 90 20       # P1 → P2
```
```bash
ros2 run hcr5_bridge movej home                             # P2 → 홈
```

한 줄씩, 앞 명령이 **`← 도착`** 을 찍은 뒤 다음으로 간다. 겁나면 뒤에 **`--dry-run`** 을 붙여
계산만 먼저 본다 — 발행 없이 현재·목표·각변위·속도·FK 가 다 찍힌다.

> 📌 **`← 도착` 은 2026-08-16 에 생겼다.** 종전 JTC 판은 `← 종료 code=` 만 찍었고 그 code 는
> 도착 판정력이 없어서 *"도착은 `mqtt_cmd.py pos` 로 본다"* 고 사람에게 떠넘겼다. 지금은
> `program/end`·`event/motion` 으로 판정하고 **관절 오차까지 되읽어 찍는다** ([`movej.md`](movej.md) §1.3).

**`--deg` 를 빼면 라디안이다.** `10` 을 그냥 넣으면 10 rad(실기 483°)로 읽힌다 — 게이트가 막지만
막히는 것에 기대지 말고 단위를 맞춰 넣는다.

> ⚠️ **스택(`bringup.launch.py`)을 띄운 채로 이 절을 실행하지 마라.** `movej`·`movel` 둘 다
> 그 스택과 **배타**다(§1.2). 수동 운전은 스택을 내린 뒤 단독으로 돌린다 — §1.1.

**자세 확인 — 도착 판정은 이걸로 한다** (읽기 전용, 로봇이 안 움직인다):

```bash
cd ~/ws_atlier && python3 src/drivers/hcr5_comm/tools/mqtt_cmd.py pos
```

⚠️ **`error_code: 0`(SUCCESSFUL)을 도착 판정으로 쓰면 안 된다.** JTC `constraints` 가 미설정이라
추종에 실패해도 성공이 나온다.

## 3.6 정지 — 창 1 에서 `Ctrl+C`

**하나로 끝난다.** 이 순서로 나오면 정상이다:

```
서보 OFF — ack 수신                              ← on_deactivate
비활성화 — move/stop 발행·발행 잠금. 서보는 껐다   ← on_deactivate
```

**둘 다 플러그인 `on_deactivate` 안에서 일어난다.** 순서는 발행 잠금 → `move/stop` → 워커 회수 →
서보 OFF 이고, 로그는 ack 를 받은 뒤에 찍히므로 위 두 줄이 다 나와야 실제로 꺼진 것이다.

> **서보 OFF 자동화는 종전 설계를 뒤집은 것이다.** 원래 주석은 *"서보는 끄지 않는다 —
> 브레이크 판단은 사람 몫"* 이었다. 바꾼 이유는 **서보를 켠 채 방치하는 쪽이 더 위험**하기 때문이다.
> 자세가 위험해 브레이크를 걸고 싶지 않으면 기동 때 `auto_servo:=false` — 그러면 기동 서보 ON 도,
> 종료 서보 OFF 도 둘 다 안 한다.

⚠️ **`Ctrl+C` 없이 창을 닫으면(창 X 버튼·SSH 끊김) 어떻게 되는지는 확인 안 했다.** SIGHUP 이
controller_manager 를 정상 종료로 이끄는지 미검증이다. 확인 전까지는 **반드시 `Ctrl+C`** 로 끝낸다.

**뒷정리** — 종료 시 `move_group` 이 exit −11(Segfault)로 죽는 것은 알려진 정상 거동이지만
(`MoveItCpp` 소멸자) 그때 **고아 프로세스가 남을 수 있다**. 비어 있어야 정상이다:

```bash
docker exec nyang_nyang_atlier ps -ef | grep -E "move_group|ros2_control_node|robot_state_pub|static_transform|rviz2" | grep -v grep
```

남았으면 `docker exec nyang_nyang_atlier pkill -INT -f "ros2 launch"`.
⚠️ 고아를 둔 채 재기동하면 **낡은 `/robot_description`(transient_local QoS)이 새 스택을 오염시킨다**.

## 3.7 정지 사다리 — 위에서부터

| 순위 | 수단 | 거동 |
|---|---|---|
| 1 | **물리 e-stop** | 유일하게 확실한 수단. PC 명령은 데드맨을 안 거친다 |
| 2 | `docker exec nyang_nyang_atlier pkill -INT -f "ros2 launch"` | `on_deactivate` 가 `move/stop` 발행 + 발행 **영구 잠금** + **서보 OFF** |
| 3 | `ros2 run hcr5_bridge servo off` (또는 `… mqtt_cmd.py servo off`) | 컨트롤러가 `150026 …_BUT_SERVO_OFF` 로 명령을 거부한다 (실기측 2차 방어선) |

**3 순위는 2 가 서보를 끄게 된 뒤에도 남는다.** ROS 에 기대지 않는 유일한 칸이기 때문이다 —
`servo` 실행기는 `rclcpp` 의존이 없어 스택이 죽은 뒤에도 단독으로 돈다.
2 가 끄는 것은 **정상 종료 경로**이고, 3 은 그 경로가 통째로 죽었을 때의 칸이다.

⚠️ **`mqtt_cmd.py movestop` 단독은 소용이 적다** — JTC 가 200ms 마다 새 목표를 계속 밀어넣어 곧 재개된다.

## 3.8 안전 — 실측으로 확인된 것들 ⚠️

**축온** — 2026-08-05 에 **58~61°C** 에서 6축 드라이브가 0.8초 만에 전부 트립했고 컨트롤러
재부팅으로만 복구됐다. 감시견의 58°C 즉시중단이 그 선이다. **서보를 켠 채 오래 두지 말 것** —
홀딩 토크로 축 온도가 37→60°C 까지 오른다.

**단위 착각이 명령을 폭주시킨다** — 홈에서 `positions: [15, …]` · `sec: 5` 를 발행했더니
`15 rad = 실기 769.44°` 로 해석돼 **명령 속도가 153.9°/s**(통상 2°/s 의 77배)가 됐다.
막아 준 안전망은 **URDF 관절한계 하나뿐**이었다 — 실기 267.77° 에서 발행이 끊겼다.
에러 로그는 **한 줄뿐**이다(`warned_limits_` 가 래치라 이후 차단은 조용하다) — **에러 1줄 = 사고 1건이 아니다.**
그런데도 3.2초 뒤 `Goal reached, success!` 가 떴다.

**관절한계는 URDF 와 실기가 어긋날 수 있다** — URDF 한계가 URDF 좌표에서 대칭 ±360° 였는데,
규약 오프셋 δ=90° 가 붙는 **j1·j2·j4** 는 그 대칭이 실기에서 깨져 양(+) 방향으로 **한 바퀴가 안 됐다**
(실기 +270° 에서 막혔다). 2026-08-15 에 `hcr_robot.xacro`(팀 공용)를 고쳐 정합시켰고,
홈 0° → 280° → 360° → 0° 완주로 실증했다(축온 최고 55°C).

원칙: **URDF 한계는 실기 한계보다 항상 안쪽**으로 둔다. 끝자리는 올리지 말고 **내린다** —
`write()` 가 한계를 두 번 보기 때문이다(`withinLimits()` 실기 도 ±360° · URDF `info_.limits` rad).
URDF 한계가 실기보다 바깥이면 URDF 검사는 통과하는데 실기 검사가 막아 두 게이트가 싸운다.

```
정확한 450° = 7.853981633974483 rad
반올림 7.853982 rad → 실기 +360.000021°  → withinLimits() 차단
내림   7.853981 rad → 실기 +359.999964°  → 통과 (여유 0.000036°)
```

**안전망이 이 경로에 다 걸리는 건 아니다**:

| 안전망 | 걸리나 |
|---|---|
| 실행기 게이트 (단위 표시 · `withinLimits()` · 속도 상한) | ✅ 발행 **전에** 건다 |
| 관절한계 (실기 ±360° · J3 ±165°) | ✅ 하드웨어 게이트 6 — URDF 한계와 **이중 검사** |
| MoveIt 속도 스케일 `0.1` | ❌ planning 을 안 거친다 |
| `joint_limits.yaml` 180°/s | ❌ MoveIt 계획용이라 JTC 가 안 읽는다 |
| JTC `constraints` | ❌ `ros2_controllers.yaml` 에 항목 자체가 없다 |
| `mqtt_cmd.py` 3중 가드 | ❌ 그건 MQTT 직결 경로의 가드다 |

그 밖에:

- 상태 수신이 1초 이상 끊기면 `/joint_states` 발행을 **멈춘다** — 낡은 자세를 참으로 믿게 두지 않는다
- 필드버스 상태(`controllerStatus`)를 감시한다. `FIELD_BUS_SW_STATE_CONNECTED` 가 아니면 에러 로그
- 충돌은 **래치된다**(`PAUSED`). `event/collision/clear` 로 명시 해제해야 재개된다

## 3.9 호스트에서 한 줄로 — 에이전트·스크립트용

창을 유지할 필요가 없다. 대신 `source` 를 매번 명시해야 한다(§3.1).
**사람이 로봇 앞에서 쓰는 방식은 §3.2~3.7 이다** — 이쪽은 재현·자동화용이다.

> ⚠️ **2026-08-16 수정.** 종전 이 자리는 `bringup.launch.py` 를 띄운 뒤 `movej` 를 쏘는 예였다.
> 두 실행기가 MQTT 로 옮겨간 지금 그 조합은 **배타 위반**이다(§1.2). 스택 없이 돌린다.

```bash
docker exec nyang_nyang_atlier bash -lc "source /opt/ros/jazzy/setup.bash && source ~/ws_atlier/install/setup.bash && ros2 run hcr5_bridge servo on"
```
```bash
docker exec nyang_nyang_atlier bash -lc "source /opt/ros/jazzy/setup.bash && source ~/ws_atlier/install/setup.bash && ros2 run hcr5_bridge movej --deg 10 -90 -90 -90 90 0 --dry-run"
```
```bash
docker exec nyang_nyang_atlier bash -lc "source /opt/ros/jazzy/setup.bash && source ~/ws_atlier/install/setup.bash && ros2 run hcr5_bridge servo off"
```

⚠️ 위 가운데 줄에서 **`--dry-run` 을 빼면 로봇이 움직인다.** 자동화에서 빼기 전에
[`movej.md`](movej.md) §2.3 의 🔴 단위 문제를 먼저 닫는다.

**스택을 쓰는 경우(MoveIt 계획 실행)에만** 아래가 유효하다:

```bash
docker exec -d nyang_nyang_atlier bash -lc "source /opt/ros/jazzy/setup.bash && source ~/ws_atlier/install/setup.bash && ros2 launch hcr5_bridge bringup.launch.py real:=true allow_motion:=true > /tmp/launch.log 2>&1"
```
```bash
docker exec nyang_nyang_atlier grep -E "Configured and activated|활성화 —|서보 ON|ERROR" /tmp/launch.log
```
```bash
docker exec nyang_nyang_atlier pkill -INT -f "ros2 launch"
```

**이 방식도 서보는 꺼진다** — `pkill -INT` 가 controller_manager 를 정상 종료로 이끌고,
서보 OFF 는 플러그인 `on_deactivate` 가 하기 때문이다. 그래도 내린 뒤 **자세를 확인한다** —
어디서 멈췄는지는 로그가 아니라 실기가 안다:

```bash
docker exec nyang_nyang_atlier bash -lc "cd ~/ws_atlier && python3 src/drivers/hcr5_comm/tools/mqtt_cmd.py pos"
```

## 3.10 독립 노드 경로 (배타 ⚠️)

§3.4~3.9 의 ros2_control 경로와 **동시에 쓰지 않는다** — 같은 `/joint_states` 에 둘이 발행한다.

```bash
# 읽기 전용 (기본) — 로봇을 절대 움직이지 않는다
ros2 launch hcr5_bridge state_bridge.launch.py
```
```bash
# 동작 지령 서비스까지 열기 — e-stop 대기 상태에서만
ros2 launch hcr5_bridge state_bridge.launch.py allow_motion:=true
```
```bash
ros2 topic echo /joint_states --once
```
```bash
ros2 service call /hcr_state_bridge/set_servo std_srvs/srv/SetBool "{data: true}"
```
```bash
ros2 service call /hcr_state_bridge/go_home  std_srvs/srv/Trigger
```

`allow_motion` 은 §3.4 의 잠금과 **같은 이름·같은 기본값(`false`)** 이지만 전달 경로가 다르다 —
독립 노드는 ROS 파라미터로 받고, 플러그인은 **URDF `<hardware>` 의 `<param>`** 으로 받는다.

## 3.11 다른 경로

| 하고 싶은 것 | 방법 |
|---|---|
| **직교 좌표로 직선 이동** | `ros2 run hcr5_bridge movel <base\|world\|tool> <x y z>` — `program/plan` linear. 컨트롤러가 보간을 소유한다. ✅ **직선이다** (실측 수직편차 정속구간 최대 0.0184mm/100mm). ⚠️ **ros2_control 스택과 배타** — 띄운 채로 쓰지 않는다(§1.2). 사용법 §1.1 끝 · 근거 [`movel.md`](movel.md) |
| MoveIt 계획·충돌검사를 태운 이동 | `hcr5_comm/measurements/260812_T15검증/verify/c2_moveaction.sh <j1..j6> <scale>` — `/move_action` Plan&Execute. ⚠️ **`move_group` 이 필요하다** — `bringup.launch.py` 는 안 띄우므로 `hcr_moveit_config/demo.launch.py` 로 띄워야 한다 |
| 마커를 끌어서 대화형 | RViz MotionPlanning 패널 — 위와 같이 `move_group` 이 필요하다 |
| 계측까지 붙은 창 1회 | `verify/c2_run.sh {w1\|w2\|home}` — 계측기 3종 + 서보 사이클 + 원자료 회수 |
| ros2_control 안 쓰는 단발 이동 | `hcr5_comm/tools/mqtt_cmd.py movej <j1..j6>` — 3중 가드 내장, **실기 도(度)** 로 준다 |
| 연속 스트로크 | 이 계층 밖이다 — `program/plan` |

---

## 관련 문서

| 무엇 | 어디 |
|---|---|
| MQTT 명령 프로토콜 (역설계 원본) | [`hcr5_comm/README.md`](../hcr5_comm/README.md) |
| 계약·실측 현황 | [`중간결과물.md`](../hcr5_comm/guideline/중간결과물.md) · [`검증.md`](../hcr5_comm/guideline/검증.md) |
| 진척 (실기 읽기 B · 쓰기 C · 검증 집계) | [`hcr5_comm/README.md` §12](../hcr5_comm/README.md) |
| 팀 공용 moveit2 컨테이너 · GPU 스니펫 원본 | [`src/moveit2/README.md`](../../moveit2/README.md) |

**남은 것** — 연속 스트로크 경로(`FollowJointTrajectory` → `program/plan` 변환, `program/play` 실행,
`program/plan` 크기 한계 실측, 즉시정지 지연 실측)는 이 계층 밖이다.
