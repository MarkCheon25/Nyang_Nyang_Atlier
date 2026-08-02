# simulation

**MuJoCo 시뮬이 실제로 돈다.** 확인 하네스·MJCF 변환에 이어 컨트롤러 스폰과 궤적
실행까지 통과했다. 다만 **궤적 추종은 아직 성립하지 않는다** — 관성이 큰 관절이
목표를 못 따라간다. 접촉 실험 이전에 이것부터 풀어야 한다 ("구현 상태" 참조).

## 이 모듈이 답하는 질문

**"펜이 종이에 실제로 어떻게 닿는가."**

mock hardware(`mock_components/GenericSystem`)는 궤적을 그대로 따라간다고 가정하므로
계획과 실제의 차이가 언제나 0이다. **MuJoCo 를 쓰는 이유는 계획대로 안 되는 것을 보기
위해서다.**

| | mock hardware | MuJoCo |
|---|---|---|
| 궤적 계획·실행·RViz | ✅ | ✅ |
| 파이프라인 완주 (AC1 경로) | ✅ | ✅ |
| **접촉·필압** | ❌ 없음 | ✅ **R2 검증** |
| **동역학**(관성·마찰 → 실제 소요) | ❌ | ✅ N2 근거 |
| **실제로 그어진 선** | 계획과 동일 | ✅ 차이가 보인다 |

BRD **R2(필압 제어 실패)는 가능성 "높음"**이다 — 펜을 테이프로 고정해 z 방향 완충이
없기 때문이다. 그 위험을 실물 전에 보는 것이 이 환경의 존재 이유다.

> **역할을 갈라 두지 않으면 MuJoCo 가 mock 의 중복이 된다.** 파이프라인 완주는 mock 이
> 담당하고(SA §1 MVP 정의), MuJoCo 는 그 위에 얹는다.

## 경계 타입 — `atlier/sim/contact_trace.hpp`

**ROS 에도 MuJoCo 에도 의존하지 않는다.** 궤적을 기록하는 쪽과 읽는 쪽이 타입만
공유하면 되므로, 시뮬 연동이 막혀도 확인 경로는 따로 선다.

```cpp
namespace atlier::sim {

struct Point2        { double x_mm, y_mm; };          // vision 과 같은 규약 — 종이 좌상단 원점
struct ContactSample { double t_s, x_mm, y_mm, force_n; bool in_contact; };
struct DrawnTrace    { std::vector<ContactSample> samples; double paper_w_mm, paper_h_mm; };
struct PaperFrame    { double origin_x_m, origin_y_m, surface_z_m, yaw_rad; };

Point2 WorldToPaper(const PaperFrame &, double world_x_m, double world_y_m);

std::size_t SegmentCount(const DrawnTrace &);   // 실제로 그어진 선 조각 수 — 끊김 검출
double      DrawnLengthMm(const DrawnTrace &);
bool        ForceStats(const DrawnTrace &, double & min_n, double & max_n, double & mean_n);
double      ContactDutyRatio(const DrawnTrace &);
}
```

**`SegmentCount` 가 R2 진단의 핵심이다** — 이 값이 계획 스트로크 수보다 많으면 한
스트로크가 도중에 끊겼다는 뜻이고, 필압 부족을 의심할 근거가 된다.

### vision 과는 파일로 붙는다

계획 지도를 겹쳐 그려야 "계획 vs 실제"가 보이는데, **`vision_core` 에 링크하지
않는다.** 두 모듈은 별도 브랜치·별도 컨테이너에서 자라고 있고, 필요한 것은 좌표
몇 개뿐이다. 코드로 붙이면 이 모듈이 OpenCV 까지 끌고 온다.

```
vision:  stroke_map_cli → map.csv ─┐
                                   ├─→ trace_cli → drawn.svg (겹쳐 그림)
simulation: MuJoCo → trace.csv ────┘
```

결합이 CSV 형식 하나로 줄어든다. 그 형식이 바뀌면 `planned_map.cpp` 가 깨지는 것이 대가다.

## 연동 방식 — BRD D5 결정 (2026-07-28)

**`ros-jazzy-mujoco-ros2-control` (apt 바이너리)를 쓴다.** D5 는 "1주차 결정"으로
남아 있던 항목인데, 소스 빌드 없이 apt 한 줄로 끝난다.

| | 값 |
|---|---|
| 패키지 | `ros-jazzy-mujoco-ros2-control` **0.0.3** (ros-controls 조직) |
| MuJoCo | `ros-jazzy-mujoco-vendor` = **3.4.0** |
| 하드웨어 플러그인 | `mujoco_ros2_control/MujocoSystemInterface` |

**이것이 하드웨어 추상화 경계(N6·F7.3)를 그대로 성립시킨다** — 원본이 쓰던
`mock_components/GenericSystem` 자리에 이 플러그인을 넣는 것이 전부이고, MoveIt2 도
`joint_trajectory_controller` 도 그대로다. SA 가 경계를 F4.3 에 둔 설계와 정확히 맞물린다.

> 대안: [dfki-ric/mujoco_ros2_control](https://github.com/dfki-ric/mujoco_ros2_control) —
> FT 센서 지원이 명시적이고 xacro 를 런타임 변환한다. 소스 빌드라 동료 PC 재현 비용이
> 늘지만, **하드웨어 인터페이스 경계라 교체 비용이 작다.** 필압 측정 경로가 막히면 후보.

## 구성

```
simulation/
├── README.md            # (이 파일) 인터페이스 원본 + 세팅 절차
├── Dockerfile           # ros-base + mujoco-ros2-control — MoveIt2 는 넣지 않는다
├── compose.yml / entrypoint.sh / run_container.sh
├── compose.override.yml # ⛔ git 제외 — PC별 GPU 설정 (뷰어용, §3.1)
├── data/                # ⛔ git 제외 — 궤적·SVG·변환된 MJCF
└── ws_simulation/src/
    ├── sim_core/        # ROS·MuJoCo 무관 C++ 라이브러리 + 확인용 CLI
    └── sim_bringup/     # 펜 래퍼 URDF · MJCF 씬 · 컨트롤러 설정 · launch
```

**MoveIt2 를 이미지에 넣지 않은 이유** — 이 환경이 답하는 질문은 접촉이고, 궤적
계획(F4.2)은 moveit2 환경의 몫이다. 궤적을 넣어 돌리는 데는
`joint_trajectory_controller` 만 있으면 된다.

## 1. 이미지 빌드 (최초 1회)

```bash
cd src/simulation
./run_container.sh build      # 5~12분
```

전제 조건은 `src/moveit2/README.md` §0과 같다. GPU 설정은 **MuJoCo 뷰어를 띄울 때만**
필요하다 — §3.1 참조.

## 2. 빌드와 확인

```bash
./run_container.sh shell
colcon build --symlink-install && source install/setup.bash
```

### 확인 — `trace_cli`

**계획과 실제를 한 장에 겹쳐 본다.** mm 를 그대로 쓰므로 브라우저에서 A4 실제 비율로
보이고 인쇄하면 자로 실측된다.

```bash
# MuJoCo 없이 하네스만 검증 — 시뮬 연동이 막혀도 확인 경로가 서는지 본다
ros2 run sim_core trace_cli --dummy --case weak-force --out-dir ~/data/out

# 실제 시뮬 결과 + vision 계획 지도
ros2 run sim_core trace_cli --trace ~/data/trace.csv --plan ~/data/map.csv --out-dir ~/data/out
```

| 더미 시나리오 | 무엇을 보여주나 |
|---|---|
| `ideal` | 계획대로 그어진 경우 — 조각 수가 계획 스트로크 수와 같다 |
| `weak-force` | **필압 부족 → 선 끊김.** 조각 수가 계획보다 많아지고 SVG 에서 한 선이 여러 색으로 쪼개진다 |
| `drift` | 위치 오차 누적 — 바운딩박스가 밀린다 (캘리브레이션 오차 R5 의 모습) |

| 출력 | 내용 |
|---|---|
| `drawn.svg` | **주력.** 계획(회색)과 실제(색)를 겹쳐 그림. `--show-force` 로 필압을 굵기에, `--show-travel` 로 공중 이동을 점선으로 |
| `trace.csv` | `t_s,x_mm,y_mm,force_n,in_contact` |
| `summary.txt` | 조각 수·길이·필압·접촉 비율 + 계획 대비 |

---

## 3. 시뮬 기동

### 3.0 ⚠️ `ROS_DOMAIN_ID` 를 0 이 아닌 값으로 — **컨테이너를 띄우기 전에**

```bash
export ROS_DOMAIN_ID=42          # 0 이 아니면 된다
./run_container.sh up            # 이 값이 컨테이너 환경에 박힌다
```

컨테이너가 `network_mode: host` 라서 **기본값 0 을 쓰면 같은 LAN 에서 도는 남의 ROS
그래프에 합류한다.** 2026-08-02 에 실제로 겪었다 — 다른 PC 의 MoveIt2 Panda 데모와
`/controller_manager`·`/joint_state_broadcaster`·`/robot_state_publisher` 이름이 겹쳐
**`/joint_states` 가 빈 배열로 나왔다.** 에러가 아니라 조용히 빈 값이라 알아채기 어렵다.

> **순서가 중요하다.** `run_container.sh shell` 은 이미 있는 컨테이너에 `docker exec`
> 할 뿐이라 **호스트에서 export 해도 그때는 안 먹는다.** 컨테이너가 이미 0 으로 떠
> 있으면 `./run_container.sh down` 후 다시 `up` 하거나, 셸마다 안에서 export 한다.
> 확인: 컨테이너 안에서 `echo $ROS_DOMAIN_ID`

### 3.1 뷰어를 띄우려면 — GPU 오버라이드

**MuJoCo Simulate 창을 띄울 때만 필요하다.** `headless:=true` 로 돌리면 없어도 된다
(확인 경로가 SVG 파일 출력이라 GPU 와 무관).

같은 폴더에 `compose.override.yml` 을 만든다 — 스니펫은 `src/moveit2/README.md` §2와
같고 **서비스 이름만 `simulation` 으로** 바꾼다. `run_container.sh` 가 있으면 자동으로
함께 읽는다. `.gitignore` 대상이라 커밋되지 않는 PC 로컬 자산이다.

```bash
./run_container.sh config | grep -A2 nvidia    # 오버라이드가 얹혔는지 확인
docker exec simulation_dev glxinfo -B | grep renderer
```

`OpenGL renderer` 가 `llvmpipe` 로 나오면 GPU 가 안 붙은 것이다 — 뷰어가 아주 느리다.

### 3.2 MJCF 변환 — **`data/mjcf/` 가 비어 있을 때만**

launch 는 변환하지 않는다. 산출물이 이미 있으면 건너뛴다.

```bash
S=$(ros2 pkg prefix sim_bringup --share)
xacro $S/urdf/hcr_robot_pen.xacro > /tmp/hcr_pen.urdf
/opt/ros/jazzy/share/mujoco_ros2_control/scripts/robot_description_to_mjcf.sh \
  -u /tmp/hcr_pen.urdf -m $S/mjcf/mujoco_inputs.xml --scene $S/mjcf/scene.xml \
  -o ~/data/mjcf -s -c --no-fuse
```

> ⚠️ **`--no-fuse` 와 `-m` 은 선택이 아니다.** 없으면 각각 펜이 병합돼 사라지고
> actuator 가 0 개로 나온다 (아래 "함정 3건" 참조).

> ⚠️ **변환 스크립트의 python 의존이 이미지에 없다** (2026-08-02 확인). 위 명령은
> `requirements.txt`(mujoco·obj2mjcf·coacd·trimesh 등 17개)를 요구하는데 Dockerfile
> 이 설치하지 않는다. **`data/mjcf/` 산출물이 이미 있으면 변환을 건너뛰어도 된다** —
> 경로 참조가 전부 상대경로라 디렉터리째 옮겨도 무사하다.

### 3.3 실행 (A) — ROS 연동 + 뷰어

컨트롤러까지 올라오므로 **궤적을 넣어 볼 수 있다.** 이쪽이 주력이다.

```bash
./run_container.sh shell
source install/setup.bash
ros2 launch sim_bringup mujoco_sim.launch.py        # headless:=true 로 창 없이
```

기동이 끝나면 컨트롤러 2개가 active 여야 한다:

```bash
ros2 control list_controllers
#  joint_trajectory_controller  ... active
#  joint_state_broadcaster      ... active
ros2 topic echo /joint_states --once      # name 이 비어 있으면 §3.0 을 볼 것
```

**궤적 넣어 보기** — 셸을 하나 더 열고(`./run_container.sh shell`):

```bash
source install/setup.bash
ros2 topic pub -1 /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
  points: [{positions: [0.5, -0.0059, 1.5728, -1.5906, -1.2, 0.0], time_from_start: {sec: 3}}]}"
```

> **`joint_5`(손목)는 휙 도는데 `joint_1`(베이스)은 기어간다** — 아래 "궤적 추종이
> 성립하지 않는다" 가 화면에서 그대로 보인다. 추종 오차는 여기서 본다:
> `ros2 topic echo /joint_trajectory_controller/controller_state`

### 3.4 실행 (B) — 씬만 볼 때, MuJoCo 단독 뷰어

ROS 없이 배치만 확인한다. 마우스로 관절을 직접 끌어볼 수 있다.

```bash
/opt/ros/jazzy/opt/mujoco_vendor/bin/simulate ~/data/mjcf/scene.xml
```

로봇·책상·A4 종이·지그 배치를 눈으로 본다. **초기 자세에서 펜이 종이 반대편을
향하는 것**(미결 "작화 자세 · 종이 배치")도 여기서 바로 보인다.

---

## 구현 상태

| 항목 | 상태 |
|---|---|
| 접촉 궤적 타입·통계 (`sim_core`) | ✅ 구현됨 |
| 확인 하네스 (`trace_cli` · SVG·CSV·요약) | ✅ 구현됨 — 시나리오 3종 관통 |
| vision 계획 지도 읽기 (`map.csv`) | ✅ 구현됨 |
| 펜 래퍼 URDF (`hcr_robot_pen.xacro`) | ✅ 파싱 통과 — ros2_control 블록 1개(MuJoCo 단독) |
| **URDF → MJCF 변환** | ✅ **통과** |
| **MJCF 씬 (책상·종이·지그)** | ✅ **로드 통과** — body 12 · joint 6 · geom 22 · **actuator 6** · **sensor 1** |
| **물리 실행** | ✅ **안정** — 초기 자세 2초 유지, 관절 오차 0.003 rad |
| **필압 측정 경로** | ✅ **열림** — 종이 `touch` 센서가 접촉을 잡는다 (아래 참조) |
| 필압 → ros2_control 인터페이스 노출 | ⬜ 미확인 — 센서가 MJCF 에 있는 것과 ROS 에서 읽는 것은 별개 |
| **컨트롤러 스폰** | ✅ **통과** — 두 컨트롤러 active · 6축 command interface claimed |
| **궤적 명령 수신·실행** | ✅ **통과** — 궤적 토픽으로 관절이 실제로 움직인다 |
| **궤적 추종** | ⚠️ **성립 안 함** — 관성 큰 관절이 목표를 못 따라간다 (아래) |
| 궤적 기록 노드 (MuJoCo → trace.csv) | ⬜ 미착수 |
| MJCF 변환 재현 (새 컨테이너에서) | ⚠️ **불가** — 변환 스크립트의 python 의존이 Dockerfile 에 없다 |

**검증된 것 (2026-07-28)**
- `ros-jazzy-mujoco-ros2-control` 0.0.3 · `mujoco-vendor` 3.4.0 apt 설치 확인
- HCR-5 URDF **8개 링크 전부 inertial 보유** — MuJoCo 변환의 주요 장애물 없음
- 펜 래퍼 xacro: ros2_control 블록 **1개**(MuJoCo 단독) · world 고정 · `mujoco_model` param 주입
- **원본 인자화 후에도 MoveIt2 경로 무영향** — 인자 없이 파싱하면 여전히 `mock_components/GenericSystem`
- **씬 로드 통과** — body 12(world·base·link1~6·pen_link·pen_tip·paper·jig) · joint 6 ·
  geom 22 · **actuator 6** · **sensor 1**(pen_pressure)
- **물리 안정** — 초기 자세(joint_3=1.5669 등)로 2초 시뮬, 관절 오차 최대 0.003 rad
- **접촉·필압 감지** — 종이 `touch` 센서가 접촉을 잡음 (값의 해석은 위 R2 항목)
- 하네스 3시나리오: `ideal` 조각 3/계획 3 · `weak-force` **조각 10 → 끊김 7회 검출** · `drift` 바운딩박스 이동 확인

**검증된 것 (2026-08-02) — 시뮬이 실제로 돌았다**
- **하드웨어 추상화 경계가 성립함을 실측.** mock 자리에 플러그인만 갈아끼운 채
  `joint_trajectory_controller` 가 그대로 붙는다 — 로그가 그대로 보여준다:
  `Loaded hardware 'hcr_robot_mujoco' from plugin 'mujoco_ros2_control/MujocoSystemInterface'`
- MuJoCo actuator 6개가 `joint_1`~`joint_6` 에 1:1 등록 · 물리 스레드 기동
- 컨트롤러 2개 active · command interface 6개 claimed · state interface 18개(pos·vel·eff)
- **궤적 토픽 명령으로 관절이 실제로 움직인다** — mock 이 아니라 물리가 도는 상태에서

**⚠️ 궤적 추종이 성립하지 않는다 (2026-08-02) — 접촉 실험의 선결 과제**

| 관절 | 명령 | 결과 |
|---|---|---|
| `joint_5` (손목) | −1.4695 → **−1.2** | **정확히 도달** |
| `joint_1` (베이스 — 74.6 kg 전체를 회전) | 0.5 → **0.0**, 3초 궤적 | **20초 후 0.288** (오차 0.288 rad 잔존) |

`joint_1` 실측 각속도 **≈0.011 rad/s** — 0.5 rad 도는 데 30초 이상. **관성이 큰
관절일수록 못 따라간다.** 관련 값이 전부 튜닝 전 자리값이다:

| 값 | 출처 | 문제 |
|---|---|---|
| `actuatorfrcrange="-100 100"` | URDF `<limit effort="100">` → 변환기가 이관 | **전 관절 동일** — CAD 변환 자리값. link2 만 32 kg 인데 손목과 같은 토크 |
| `damping` 20/20/10/5/5/2 · `frictionloss` 5/5/3/1/1/0.5 | `mjcf/mujoco_inputs.xml` (URDF 엔 `<dynamics>` 없음) | 출발점 값 |
| `kp` 25000/25000/25000/10000/10000/5000 | `mjcf/mujoco_inputs.xml` | 출발점 값 |

> **이것이 BRD 의 "관절별 최대 가속도·토크 미확보"와 같은 뿌리다.** 공식 매뉴얼에
> 항목이 없어 열어 둔 값이 URDF 에 `effort="100"` 자리값으로 들어가 있고, 그것이
> 지금 시뮬 궤적을 막는다. 벤더 문의의 근거가 하나 더 생긴 셈이다.
>
> **N2(15분) 예산 추정도 이 상태로는 성립하지 않는다** — 3초 궤적이 30초 걸린다.
>
> 지배 항이 `actuatorfrcrange` 포화인지 `dampratio` 과감쇠인지는 미확정 (가설).
> 이미지에 python `mujoco` 가 없어 직접 못 봤다.

**아직 확인 못 한 것** — 초기 자세에서 펜 끝이 `(-0.18, -0.42, 0.27)` 로 종이(0.55, 0)
반대편을 향한다. **작화 자세를 푸는 것은 MoveIt2(F4.2)의 일**이라 여기서는 종이를
런타임에 펜 아래로 옮겨 센서만 검증했다. 실제 접촉 시퀀스는 궤적 계획이 붙어야 한다.

**변환에서 잡힌 함정 3건 — 전부 해결됨 (2026-07-28)**

1. **원본 `hcr_robot.xacro` 가 mock 을 무조건 호출하던 것** → **원본을 인자화**했다.
   `ros2_control:=false` 로 블록 생성을 끄고, `hardware_plugin:=<이름>` 으로 교체한다.
   **기본값이 종전과 같아 MoveIt2 쪽은 달라지지 않는다** (인자 없이 파싱하면 여전히
   `mock_components/GenericSystem` — 실측 확인). 이것이 SA 가 말한 하드웨어 추상화
   경계의 실제 구현 지점이다.
2. **`--fuse` 가 펜을 병합하던 것** → **`--no-fuse`** 로 해결. 변환 결과에
   `pen_link`·`pen_tip` body 가 살아 있다 (body 10 → 씬 포함 12).
3. **actuator 0 개** → **`mjcf/mujoco_inputs.xml` 을 `-m` 으로 넘겨** 해결. actuator 6 개
   생성 확인. 같은 파일의 `default class="visual"/"collision"` 이 빠져 있던 것이
   직전 세션의 `unknown default class 'visual'` 로드 실패 원인이었다.

**기동에서 잡힌 함정 1건 — 해결됨 (2026-08-02)**

launch 가 첫 줄에서 죽었다 — `Unable to parse the value of parameter
robot_description as yaml`. **`Command()` 치환은 타입이 정해지지 않은 채로 오므로**
launch 가 URDF 문자열을 YAML 로 파싱하려 든다. `ParameterValue(..., value_type=str)`
로 감싸 해결. **launch 를 한 번도 안 돌려 본 것이 첫 줄에서 드러난 셈이다.**

**필압 측정 — 종이 쪽에서 잰다 (2026-07-28)**

펜이 아니라 **종이에 `<site>` 를 두고 `touch` 센서**를 걸었다. 변환된 로봇 MJCF 를
후처리해 site 를 심을 필요가 없어져 재현이 쉽다. 검증에서 접촉을 정확히 잡았다.

> ⚠️ **그런데 그 값이 R2 를 그대로 보여준다.**
> 펜 끝을 종이에 **4 mm** 밀어 넣었더니 필압이 **408 N** 으로 올라갔다. 실제 연필
> 필압은 1~5 N 대이므로 **1 mm 오차가 약 100 N** 이라는 뜻이다 — 연필심은 즉시
> 부러지고 종이는 찢어진다.
>
> 원인은 설계 그대로다. **위치 제어 액추에이터는 목표 자세를 향해 힘을 무한정
> 올리고, 펜은 테이프로 고정돼 z 방향 완충이 없다**(BRD §2.3). 종이 높이 측정
> 오차가 곧바로 필압 오차로 나타난다는 R2 의 서술이 숫자로 확인된 것이다.
>
> 이 값 자체는 `kp`·`solref`·`solimp` 튜닝 전이라 절대값을 믿을 것은 아니다.
> 다만 **방향은 분명하다 — 위치 제어 + 강체 펜은 필압이 폭주한다.** R2 의 2차
> 완화책(스프링 내장 펜홀더)이 왜 준비된 경로여야 하는지를 시뮬이 뒷받침한다.

## 미결

- **필압을 ros2_control 로 읽는 경로** — MJCF 에 센서가 있는 것과 ROS 쪽에서 값을
  받는 것은 별개다. `mujoco_ros2_control` 의 센서 매핑을 쓸지, 플러그인으로 토픽을
  낼지 미정. 래퍼에 `effort` state_interface 는 미리 열어 두었다
- **관절 게인·토크 한계 — 우선순위 최상 (2026-08-02 승격)** — 접촉 이전에 **궤적
  추종부터 안 된다.** `actuatorfrcrange`(URDF `effort="100"` 자리값 유래) · `kp` ·
  `damping`/`frictionloss` 를 실제 링크 질량(합 74.6 kg, link2 만 32 kg)에 맞춰야
  한다. **`effort` 의 근거값은 벤더 문의(관절별 최대 토크)에 걸려 있다**
- **접촉·강성 파라미터 튜닝 — 우선순위 높음** — `solref`/`solimp`/`friction` 이 전부
  출발점 값이다. **4 mm 침투에 408 N** 이 나오는 현재 설정으로는 필압 실험 자체가
  성립하지 않는다. 실물 연필 필압 1~5 N 대를 재현하는 것이 목표. **다만 위 궤적
  추종이 먼저다** — 펜을 종이까지 못 가져가면 접촉 실험을 시작할 수 없다
- **MJCF 변환 의존을 Dockerfile 에 반영** — `robot_description_to_mjcf.sh` 의
  `requirements.txt` 17개가 이미지에 없어 **새 컨테이너에서 변환 절차를 재현할 수
  없다.** 이미지에 넣을지(용량 증가) 변환용 스크립트를 따로 둘지 미정
- **`ROS_DOMAIN_ID` 기본값을 0 이 아닌 값으로** — `network_mode: host` 라 기본값 0 은
  같은 LAN 의 남의 그래프에 합류한다. `compose.yml` 기본값을 바꿀지, README 절차로
  둘지 정할 것. moveit2·vision·operator 컨테이너와 값이 같아야 서로 통신한다는
  제약과 함께 봐야 한다
- **작화 자세 · 종이 배치** — 초기 자세에서 펜이 종이 반대편을 향한다. 종이 위치를
  로봇 작업 범위에 맞출지, MoveIt2 가 자세를 풀게 할지 (F4.1·F4.2 와 함께 정할 일)
- **펜 치수·장착 위치** — `hcr_robot_pen.xacro` 의 값은 실측 전 임시값
- **`paper_frame.yaml` ↔ `scene.xml` 일치** — 두 곳에 같은 배치가 적혀 있다. 어긋나면
  선이 엉뚱한 자리에 찍히는데 알아채기 어렵다. 한쪽에서 생성하는 방법을 검토할 것
- **BRD 마일스톤 ↔ SA MVP 드리프트** — BRD §7.3 은 1~2주차를 "MuJoCo 에서 완주"로,
  SA §1 은 MVP 를 "sim(MoveIt2 mock)"으로 둔다. 어느 것이 먼저인지 문서가 어긋나 있다

## 설계 참조

| 문서 | 경로 |
|---|---|
| 요구사항 | `docs/Business Requirements.md` — F7 검증환경 · N6 이식성 · **R2 필압** · D5 |
| 시스템 구조 | `docs/System Architecture.md` — §4 검증 환경, §5.1 기술 스택 |
| 처리 흐름 | `docs/Process Flow.md` — **§5 시뮬 3단계 분기**(F4.3 이 경계) |
| 경계 상대편 | `src/vision/README.md`(계획 지도 `map.csv`) · `src/moveit2/README.md`(궤적 F4.2) |
