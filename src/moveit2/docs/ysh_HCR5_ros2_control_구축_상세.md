# HCR-5 MoveIt2 + ros2_control 신규 구축 — 상세 보고서

> **한 줄 요약** — 물려받은 `hanwha_robot_arm` 자산을 고치는 대신 **`hcr5_description` · `hcr5_moveit_config` 두 패키지를 새로 구성**했다.
> 관절 한계를 실기 공식값으로 교체하고, **펜 끝(`pen_tip`) TCP를 신설해 IK 기준점을 플랜지에서 펜 끝으로 옮겼다.**
> mock 하드웨어에서 **컨트롤러 2개 active · 6개 인터페이스 claimed** 까지 확인했다.

**작업일** 2026-08-08 (2026-08-09 기구학 정본 교정 반영) · **브랜치** `ysh_moveit_ros2_control` · **담당** ysh (moveit2 모듈)
**환경** ROS 2 Jazzy · MoveIt2 · `moveit2_dev:jazzy` 컨테이너

> ### 2026-08-09 갱신 — 이 문서의 두 가지가 뒤집혔다
> 인천 쪽(`origin/markch/hcr5_ros2`, 08-05) 성과를 반영했다.
>
> 1. **§5.1 의 관절 한계표는 부분적으로 틀렸다** — "실기는 ±360°지만 넓힐 실익 없음" 으로
>    유지했던 joint_2·4·5 를 **±360° 로 넓혔다.** 실기 wrist2 가 **−267°** 로 관측되어,
>    종전 ±120° 로는 실기의 현재 자세조차 URDF 로 표현할 수 없었다 → **§5.5**
> 2. **§10.1·§10.2 의 캡처 요청·블로커는 해소됐다** — 08-05 에 절대 관절이동·연속 블렌딩이
>    모두 실증됐다 → **§10.1(개정)**
>
> 그리고 이 문서가 다루지 않았던 **43mm 기구학 오차**가 발견되어 교정했다 → **§5.5**

---

## 0. 현재 상태 (실측)

| 항목 | 상태 |
|---|---|
| `colcon build` (두 패키지) | ✅ 성공 |
| xacro 파싱 · `check_urdf` | ✅ `base_link → … → link6_1 → tool0 → pen_tip` |
| MoveIt 계획 (OMPL/RRTConnect) | ✅ 성공 |
| 시간 매개변수화 (TOTG) | ✅ 성공 (가속도 한계 보정 후) |
| `joint_state_broadcaster` | ✅ **active** |
| `hcr_arm_controller` (JTC) | ✅ **active** |
| 하드웨어 `hcr5_system` | ✅ active · 100Hz · position 6개 **`[claimed]`** |
| **기구학 ↔ 실기 정합** | ✅ **08-09 교정** — 홈 자세 flange 오차 **0.01mm** (교정 전 43mm) → §5.5 |
| RViz Plan & Execute 10회 연속 | ⬜ **미검증** — 육안 확인 남음 |
| 펜 방향 RViz 육안 확인 | ⬜ **미검증** — FK·TF 로는 확인됨(§5.2·§5.5) |

> **주의** — 위 두 개(⬜)가 1단계 완료 판정의 마지막 조건이다. 컨트롤러가 하드웨어를
> 정상 claim 한 상태이므로 실행 자체는 성립할 것으로 보이나, **확인 전까지 완료로 보지 않는다.**

---

## 1. 배경 — 왜 이 작업인가

### 1.1 "MQTT를 ROS2로 바꾼다"는 성립하지 않는다

출발점의 문제 인식은 *"실제 제어가 확인된 `mqtt_cmd.py` 방식이 좋지 않으니 ROS2 제어로 바꾸자"* 였다.
**그러나 MQTT와 ROS2는 대체 관계가 아니다.**

| | 정체 |
|---|---|
| **MQTT** | 로봇 컨트롤러로 들어가는 **통신 채널** (물리 통로) |
| **ROS2** | PC 쪽 **계획·제어 프레임워크** |

그리고 [`hanwha_robot_arm/docs/실기_연결_현황.md`](../../../hanwha_robot_arm/docs/실기_연결_현황.md) §4.1 이 확인했듯,
**Rodi 1.003.005 에서는 MQTT가 유일하게 열려 있는 제어 통로다.** TCP 7000(RoboDK·커뮤니티 드라이버)은
Rodi 2.x 를 요구하고, Modbus TCP 는 I/O 전용이다.

→ **결론: MQTT를 버리는 게 아니라 ros2_control 아래에 감춘다.** 최종 사슬은 이렇게 된다.

```
MoveIt2 (계획)
   ↓ FollowJointTrajectory
JointTrajectoryController
   ↓ ros2_control
[하드웨어 플러그인]          ← 여기가 sim / 실기 교체 지점
   ↓
MQTT 1883 → HCR-5 컨트롤러
```

### 1.2 다만 현재 `mqtt_cmd.py` 사용 형태는 제어에 쓸 수 없다

나쁜 것은 MQTT가 아니라 지금의 **사용 방식**이다.

| 문제 | 내용 |
|---|---|
| 원샷 CLI | 명령마다 TCP 연결을 새로 맺고 끊음 |
| 개루프 속도 조그 | `jogJoint/start` → sleep → `stop`. 위치 제어가 아님 |
| 피드백 루프 없음 | 30Hz로 오는 `motion/joint/position` 을 읽지 않음 |
| 단축 제어 | 6축 동시 궤적 추종 불가 |
| 안전 감시 없음 | `event/collision` 래치를 코드가 보지 않음 |

### 1.3 아키텍처 결정 — ros2_control 유지

**결정: ros2_control 을 쓴다** (액션 서버 직결 방식을 쓰지 않는다).

근거:
- sim 팀도 같은 URDF·같은 ros2_control 을 쓴다 → **플러그인만 갈아끼우면 sim/실기가 같은 코드로 검증된다** (SA 문서의 하드웨어 추상화 경계 N6·F7.3)
- 컨트롤러 교체·다른 컨트롤러 타입 등 생태계를 그대로 쓸 수 있다
- 1·2단계에서 검증한 구성이 4단계 실기에서 그대로 살아남는다

---

## 2. 전체 작업 계획과 현재 위치

```
1-0   아키텍처 결정 (ros2_control)                            ✅ 완료
 │
1단계  description + moveit_config + mock 실행                🔶 거의 완료 (§0 의 ⬜ 2건)
 │
1-5   기구학 정본 교정 (43mm)                                 ✅ 08-09 완료 → §5.5
 │
2단계  2-1~2-3 궤적 시각화  ───  2-4 역방향 검증(선→궤적)     🔶 2-1 완료 (hcr5_viz)
 ║          (관찰 방향)              (그리기 가능성 판정)
 ║
[인천] MQTT 프로토콜 · 연속성 · 상태 브릿지                   ✅ 08-05 완료 → §10.1
 │
3단계  궤적 실행 층 (T15) — FollowJointTrajectory 액션 서버   ⬜ ★ 유일한 미착수 · 담당 확정 필요
 │       + 궤적 → program/plan 변환 (linear 체인)
 │
4단계  실기 환경 구성 + 제어                                   ⬜
 │
5단계  vision → base 캘리브레이션                              ⬜
```

### 2.1 2-4(역방향 검증)를 계획에 추가한 이유

당초 2단계는 **로봇이 움직임 → 자취를 선으로**(관찰 방향)만 있었다.
그러나 프로젝트가 실제로 해야 하는 것은 **선을 주면 → 로봇이 그대로 움직임**(생성 방향)이다
(`각자역할_작업순서.txt` 3번: "그림 엣지에 따른 경로 생성").

관찰 방향만 하고 3·4단계로 가면, **"선대로 그릴 수 있는가"를 4단계 실기 앞에서 처음 시험**하게 된다.
거기서 터질 수 있는 문제가 전부 무겁다:

- 데카르트 경로 중간에 **IK 해가 끊김** (관절이 튐 → 선이 망가짐)
- **특이점** 통과 불가
- 종이 평면 유지 실패 (펜이 파고들거나 뜸)
- A4를 놓을 수 있는 위치가 예상보다 좁음

→ **2-4 추가**: 사각형·원·지그재그를 `computeCartesianPath` 로 궤적화하고, 2-1이 그린 자취선과 원본 도형을 대조한다.
비용은 거의 없다(데카르트 경로는 MoveIt 내장, 검증 도구는 2-1에서 이미 만듦). 얻는 것은 크다:

- "평면에 선을 그릴 수 있는가"가 **sim 에서 판정**된다
- **A4를 로봇 앞 어디에 놓아야 하는지**가 숫자로 나온다 → 4단계 실기 셋업 결정 + sim 팀 캘리브레이션에도 필요
- 실패 시 **3·4단계 설계가 바뀐다** (IK 솔버 교체, 자세 제약 추가, 작업영역 재배치)

---

## 3. 만든 것 — 패키지 구성

```
ws_moveit2/src/
├── hanwha_robot_arm/          ← 마운트로 들어온 기존 자산. 건드리지 않음
├── hcr5_description/          ← 신설
└── hcr5_moveit_config/        ← 신설 (Setup Assistant 생성 + 손보정)
```

> **패키지 이름을 `hcr5_*` 로 한 이유** — `compose.yml` 이 `hanwha_robot_arm/` 을 컨테이너 안
> `ws_moveit2/src/` 아래로 마운트한다. 즉 기존 `hcr_robot_description` · `hcr_moveit_config` 가
> 이미 워크스페이스에 있어, 같은 이름을 쓰면 colcon 이 충돌한다.

### 3.1 `hcr5_description`

```
hcr5_description/
├── CMakeLists.txt              urdf · meshes · config 를 install
├── package.xml                 MIT · xacro 의존
├── LICENSE                     원본(MIT, (c)2018 Toshinori Kitamura) 승계
├── config/joint_limits.yaml    실기 공식값 + 근거 주석 (원본 근거용)
├── meshes/                     STL 7개 (base_link · link1_1~link6_1)
└── urdf/
    ├── hcr5.urdf.xacro         ★ 최상위 — 인자 3개
    ├── hcr5_arm.xacro          링크·조인트
    ├── hcr5_tool.xacro         ★ 신규 — tool0 · pen_tip
    ├── hcr5_materials.xacro    색상
    └── hcr5.ros2_control.xacro 플러그인 교체 지점
```

### 3.2 `hcr5_moveit_config`

Setup Assistant 생성본에 **손보정 2건**(§7). 주요 산출물:

| 파일 | 내용 |
|---|---|
| `config/hcr5.srdf` | `<chain base_link="base_link" tip_link="pen_tip"/>` ★ |
| `config/kinematics.yaml` | KDL |
| `config/joint_limits.yaml` | **손보정** — 가속도 한계 |
| `config/ros2_controllers.yaml` | **손보정** — 인터페이스 목록 |
| `config/moveit_controllers.yaml` | FollowJointTrajectory |
| `launch/demo.launch.py` | move_group + controller_manager + rsp + RViz |

---

## 4. 물려받은 것 / 버린 것 / 새로 만든 것

| 구분 | 대상 | 판단 근거 |
|---|---|---|
| **물려받음** | STL 메쉬 7개, 링크 원점·축 | **CAD 변환 산출물이라 원본 CAD 없이 재생성 불가.** 치수 신뢰성은 §6.3에서 교차검증됨 |
| **버림** | `hcr_robot.trans` | **ROS1 잔재.** `transmission_interface/SimpleTransmission` + `EffortJointInterface` 형식은 ROS2 ros2_control 이 쓰지 않는다. 게다가 Effort 인터페이스인데 실제 제어는 position 이라 내용도 안 맞는다 |
| **버림** | `hcr_robot.gazebo` | **Gazebo Classic 용.** 우리는 mock → 실기 경로만 쓰고, sim 팀은 MuJoCo 로 확정 |
| **버림** | `ros2 pkg create` 의 `include/` · `src/` | 데이터 전용 패키지라 C++ 골격 불필요 |
| **교체** | 관절 한계 · 속도 | §5.1 |
| **신설** | `tool0` · `pen_tip` 툴 체인 | §5.2 |
| **신설** | 최상위 `hcr5.urdf.xacro` | 인자 3개로 sim/실기/펜길이를 한 곳에서 가른다 |

---

## 5. 기술 결정과 근거

### 5.1 관절 한계 — 실기 공식값으로 교체

출처: **HCR-5 Collaborative Robot User Manual v2.0 (2018-11, ENG)** Appendix F(제원)·G(정지거리)

| 관절 | 기존 (CAD 기본값) | **교체값** | 판단 |
|---|---|---|---|
| joint_1 | `0° ~ +360°` | **±360°** (`±6.283185`) | ⛔ **음수 불가 해소** |
| joint_2 | ±90° | ~~유지~~ → **±360°** | 08-09 정정 (§5.5) |
| joint_3 | ±130° | **±165°** (`±2.879793`) | 실기 공식값. 유일하게 별도 제한 있는 관절 |
| joint_4 | ±170° | ~~유지~~ → **±360°** | 08-09 정정 (§5.5) |
| joint_5 | ±120° | ~~유지~~ → **±360°** | ⛔ 08-09 정정 — 실기 wrist2 가 **−267°** 로 관측된다 |
| joint_6 | `0° ~ +360°` | **±360°** | ⛔ **음수 불가 해소** |
| 최대속도 | `100 rad/s` (5729°/s) | **`3.1416 rad/s`** (180°/s) | 실기의 **약 32배**였다 |
| 최대가속 | `100 rad/s²` | **`3.5 rad/s²`** (≈200°/s²) | **추정치 — 벤더 확인 대상** |

**joint_1·6 의 음수 불가가 왜 치명적이었나**

`revolute` 조인트는 범위 밖으로 감기지(wrap) 못하므로 플래너가 반드시 범위 **안쪽**을 통과해야 한다.

```
joint_1 을  +0.1 rad → -0.1 rad  로 옮기고 싶다   (실제 이동량 0.2 rad = 11°)
  ↓  -0.1 은 범위 밖 → 등가값 6.183 rad 로 가야 함
실제 계획되는 이동:  0.1 → 6.183 rad = 6.08 rad = 348° 대회전
```

**11° 움직이면 될 것을 348° 돌린다.** 그리기 작업에서는 펜이 종이 위를 지나다 base 가 한 바퀴 도는 셈이다.
IK 솔버가 `(-π, π]` 로 해를 내면 **해가 있는데도 계획 실패**로 떨어진다.
`lower` 를 `-6.283185` 로 바꿔 **0을 범위 중앙에** 두어 해결했다.

**속도 한계가 왜 문제였나**

MoveIt 은 이 값으로 궤적의 시간을 매개변수화한다. `100 rad/s` 면 "5729°/s 로 움직일 수 있다"고 믿고
구간 시간을 짧게 배분한다. 기존 실측(`실기_연결_현황.md` §5.1)에서 **scaling 0.1 에서도 peak 343.9°/s
= 공식 한계의 1.91배**, scaling 1.0 이면 약 19배가 나왔다. 그대로 실기에 보내면 추종 오차(→ 선 일그러짐)
또는 매뉴얼 8.5 의 관절 안전한계 초과 자체정지가 난다.

**가속도 3.5 는 추정치임을 명시**

공식 매뉴얼에 **관절 최대 가속도 항목이 아예 없다.** Appendix G 의 정지거리·정지시간
(J1 34.67°/0.61s, J2 31.60°/0.55s, J3 29.08°/0.51s, 브레이크 체결 0.03s)에서 역산했다.
비상제동 기준이라 통상 운전 가속도는 이보다 작아야 한다. **벤더 문의 항목으로 남긴다.**

### 5.2 펜 TCP 신설 — 이 작업의 핵심 신설분

**문제**: 기존 URDF 의 체인 끝이 `link6_1` = **플랜지**였고, SRDF 에 엔드이펙터·TCP 정의가 **아예 없었다.**
그리기 작업은 **펜 끝 프레임**이 있어야 성립한다.

**해결**: 툴 체인을 신설하고 SRDF 의 IK 기준점을 펜 끝으로 옮겼다.

```
link6_1 ──fixed──> tool0 ──fixed──> pen_tip
(플랜지)          (ROS-I 관례:       (실제 TCP)
                   +Z = 진출 방향)
```

**펜 축 방향은 어떻게 정했나 — 기하 추론 + FK 검증**

이 URDF 는 **모든 joint 의 `rpy` 가 0** 이라 링크 프레임이 전부 base_link 와 축 정렬돼 있다.
그리고 `joint_6` 의 회전축이 `(-1, 0, 0)` 이다. 6축 팔에서 마지막 관절의 회전축 = 공구 진출축이므로,
**플랜지 법선 = `link6_1` 의 -X** 로 추론했다.

→ `flange_p = -π/2` 로 `tool0` 의 +Z 를 그 -X 에 맞췄다.

**FK 로 검증한 결과** (zero pose, ROS 런타임 없이 URDF 에서 직접 계산):

```
프레임        base_link 기준 위치 (m)
base_link     [  0.0000   0.0000   0.0000 ]
link1_1       [  0.0000   0.0000   0.0300 ]
link2_1       [ -0.0785   0.0000   0.1488 ]
link3_1       [ -0.0778   0.0010   0.5728 ]
link4_1       [ -0.1079   0.0010   0.9113 ]
link5_1       [ -0.1699   0.0010   1.0009 ]
link6_1       [ -0.2595   0.0010   1.0629 ]
tool0         [ -0.2595   0.0010   1.0629 ]
pen_tip       [ -0.4095   0.0010   1.0629 ]

link6_1 → pen_tip  translation = [-0.1500  0.0000  0.0000]
tool0 의 +Z 방향   = [-1. 0. 0.]   (link6_1 기준)  → joint_6 회전축과 일치
```

손목이 -X 로 계속 밀려나고(link4 −0.108 → link5 −0.170 → link6 −0.260), 펜이 그 연장선인 −0.410 으로
나간다. **팔 바깥을 향한다.** 부호가 반대였다면 펜이 팔 안쪽을 파고들어 기하적으로 불합리했을 것이다.

> ⚠️ **RViz 육안 확인이 아직 남았다.** 어긋나면 `hcr5_tool.xacro` 의 `flange_p` 를 `+1.5707963` 로
> 바꾸면 끝난다 (그 하나만 고치면 되도록 인자화해 두었다).

**펜홀더 설계 미확정 대응** — `pen_length` 기본값 0.15m 는 임시값이고 xacro 인자다.
설계가 확정되면 `pen_length:=0.182` 처럼 숫자만 교체하면 된다 (교체 동작 검증 완료, §6.2).

### 5.3 ros2_control 블록을 `description` 에 둔 이유

**MoveIt 표준 관례는 moveit_config 가 ros2_control 블록을 소유하는 것이다. 여기서는 반대로 갔다.**

근거: **sim 팀은 moveit_config 없이 description 만 쓴다.** 하드웨어 블록이 moveit_config 에 들어가면
sim 팀이 그 블록을 쓸 수 없다. 물려받은 `hcr_robot.ros2_control.xacro` 의 주석도 정확히 이 이유를
적고 있었다 (*"시뮬레이션은 hardware 에 `<param>` 을 넣어야 해서 이 경로를 쓴다"*).

→ 블록은 `hcr5_description` 에 두고, Setup Assistant 의 "ros2_control URDF Modifications" 탭은 건드리지 않았다.
**단, 이 선택이 §7.2 의 문제를 만들었다.**

### 5.4 하드웨어 추상화 경계 — 인자 3개

`hcr5.urdf.xacro` 최상위에서 전부 갈린다.

| 인자 | 기본값 | 용도 |
|---|---|---|
| `hardware_plugin` | `mock_components/GenericSystem` | **mock / MuJoCo / 실기 드라이버 교체 지점** |
| `ros2_control` | `true` | `false` 면 블록을 만들지 않는다 (감싸는 쪽이 자체 블록을 정의할 때) |
| `pen_length` | `0.15` | 펜 끝까지 거리 |

4단계에서 실기 연결은 `hardware_plugin:=<우리드라이버>/HcrSystem` **한 줄 교체**로 끝나도록 설계했다.

> **08-09 단서** — 인천 실증 결과 실기는 `ros2_control` 의 100Hz 스트리밍 모델과 맞지 않는다
> (상태 29.1Hz, 이동은 명령 한 번에 자율 주행, 보간을 컨트롤러가 소유). 그래서 실기 경로는
> `SystemInterface` 플러그인이 아니라 **독립 브릿지 노드가 `FollowJointTrajectory` 액션을 직접
> 제공**하는 형태가 된다. 위 인자 3개는 mock·MuJoCo 경로에서 그대로 유효하다.

---

### 5.5 ⚠️ 기구학 정본 교정 — 43mm (2026-08-09)

**이 절이 이 문서에서 가장 중요하다.** 여기까지의 모든 작업이 **43mm 틀린 로봇 모델** 위에서
이루어졌다.

#### 무엇이 틀렸나

`hcr5_arm.xacro` 는 물려받은 CAD 변환본(`hcr_robot.xacro`)에서 가져왔는데, **그 CAD 치수가
실기와 달랐다.** 인천 팀이 08-05 에 컨트롤러의 순기구학 RPC(`robot/convertPose`)로 6축을
20°씩 전 회전 스윕(108 샘플)해 실기 기구학을 추출하고 최소제곱으로 교정했다.

| 관절 | 기존 (CAD) | **정본 (실측)** | 차이 |
|---|---|---|---|
| `joint_2.x` | −0.0785 | **−0.059138** | +19.36mm |
| `joint_2.z` | 0.118778 | **0.119001** | +0.22mm |
| `joint_3.x` | 0.000652 | **−0.019348** | −20.00mm |
| `joint_3.y` | 0.001 | **0** | −1.00mm |
| `joint_3.z` | 0.424 | **0.425001** | +1.00mm |
| `joint_4.z` | 0.338567 | **0.338499** | −0.07mm |
| `joint_5.z` | 0.089596 | **0.089504** | −0.09mm |
| **`joint_6.x`** | **−0.089596** | **−0.132498** | **−42.90mm ← 지배적** |

joint_2.x 와 joint_3.x 는 축이 평행해 합만 의미 있어 보이지만, 두 origin 사이에 J2 회전이
끼어 있어 개별 식별된다(합만 맞추면 RMS 0.28mm 가 남는다).

교정 결과 109개 자세 위치오차 **RMS 43.4mm → 0.0060mm** (최대 0.0130mm).

#### 왜 위험한가

**계획도 실행도 에러 없이 성공한다.** 달성률 100%, TOTG 성공, 컨트롤러 SUCCESS —
전부 정상으로 보이고 **펜만 43mm 옆에 그린다.** A4 폭이 210mm 이므로 그림이 성립하지 않는다.
§7 의 문제들이 전부 "에러가 나서 알 수 있었던" 것과 대조적이다.

#### 이 브랜치에 반영한 것

1. **관절 origin 8개** 를 정본으로 교체
2. **가동범위** joint_2·4·5 를 ±360° 로 (§5.1 정정 — 실기 wrist2 가 −267° 로 관측된다)
3. **mesh visual/collision origin 재계산** — 이 파일의 mesh origin 은 "영점 자세 누적변환의 역"
   이라는 유도값이다. 관절 origin 을 고쳤으므로 함께 다시 계산해 파일 내부 정합성을 유지했다.
   ⚠️ 그 결과 `link5_1 ↔ link6_1` 사이에 **약 43mm 의 시각적 틈**이 생긴다. **실기 손목이 CAD
   모델보다 그만큼 길다**는 뜻이며 기구학은 옳다. STL 재작업은 별도 과제 — 계획·실행 무영향
4. **SRDF `home` 자세** 를 실기가 `move/joint/home` 으로 실제 가는 자세로 교체
5. **가드 스크립트에 [F]·[G] 신설** — 값 대조에 그치지 않고 **순기구학을 직접 풀어** 실측
   flange 좌표와 대조한다

#### 독립 검증 (반영 후 재계산 — 인천 수치를 그대로 믿지 않았다)

| 검증 | 결과 |
|---|---|
| 홈 자세 FK flange (계산) | (490.00, −170.50, 441.50) mm |
| 홈 자세 flange (펜던트 실측) | (490.00, −170.50, 441.50) mm |
| **차이** | **0.01 mm** |
| 컨테이너 `tf2_echo base_link link6_1` | `[0.490, −0.171, 0.441]` ✅ 일치 |
| 컨테이너 `tf2_echo base_link pen_tip` | `[0.490, −0.171, 0.291]` = flange −150mm (펜이 바닥 향함) |

#### 덤 — `flange_p = −π/2` 가 벤더 규약과 일치했다

`pen_tip` 의 TF 자세가 **RPY (180°, 0, 0)** 으로 나왔는데, 펜던트가 같은 자세에서 보고한
flange **`rx = −180°`** 와 일치한다. 즉 §5.2 에서 URDF 축 방향만 보고 유도했던 `tool0` 프레임이
**실기 flange 프레임 규약과 같다.**

의미: 3단계에서 MoveIt 의 데카르트 자세를 실기 `program/plan` 의 flange 웨이포인트로 넘길 때
**미지의 회전 보정이 끼지 않는다.** (다만 문서에 남은 각도는 `rx` 하나뿐이라 세 축 전부가
확정된 것은 아니다 — 실기에서 `get/command/pos` 로 세 각 모두 대조할 것.)

#### 실기 ↔ URDF 영점 규약 (이 URDF 에는 담기지 않는다)

```
q_URDF[i](도) = SIGN[i] * q_real[i](도) + DELTA[i]
  SIGN  = (+1, +1, −1, +1, +1, +1)     ← J3 만 부호 반전
  DELTA = ( 90,  90,   0,  90,   0,  0)
```

실기 zero = 팔이 수평으로 뻗은 자세, URDF zero = 팔이 수직으로 선 자세.
구현은 브릿지가 소유한다: `hcr_bridge/include/hcr_bridge/joint_convention.hpp`.
⚠️ **이 변환을 빼먹으면 로봇이 에러 없이 전혀 다른 자세로 간다.**

출처: `origin/markch/hcr5_ros2` — `hanwha_robot_arm/HCR_5/hcr_robot_description/urdf/hcr_robot.xacro`
관절 정의부 주석 · `HANDOFF_260805_ros2_bridge.md`

---

## 6. 검증 결과 (전부 실측)

### 6.1 빌드·구조

```
colcon build            → Finished <<< hcr5_description / hcr5_moveit_config
xacro 파싱              → exit 0
check_urdf              → robot name is: hcr5
                          base_link → link1_1 → … → link6_1 → tool0 → pen_tip
관절 한계 (생성 URDF)    → velocity="3.1416" × 6
SRDF 그룹               → <chain base_link="base_link" tip_link="pen_tip"/>
```

### 6.2 인자 교체 동작

| 시험 | 결과 |
|---|---|
| `hardware_plugin:=mujoco_ros2_control/MujocoSystemInterface` | ✅ `<plugin>` 이 실제로 바뀜 |
| `ros2_control:=false` | ✅ `<ros2_control` 블록 0개 |
| `pen_length:=0.182` | ✅ `origin xyz="0 0 0.182"` 로 즉시 반영 |

### 6.3 런타임

```
list_controllers
  joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster          active
  hcr_arm_controller       joint_trajectory_controller/JointTrajectoryController  active

list_hardware_components
  name: hcr5_system · type: system · plugin: mock_components/GenericSystem
  state: active · read/write rate: 100 Hz
  joint_1~6/position  [available] [claimed]     ← 컨트롤러가 하드웨어를 정상 확보
```

### 6.4 덤으로 나온 교차검증 — CAD 치수가 실기 제원과 일치

FK 결과로 최대 작업반경을 계산하면:

```
상완 0.424 + 전완 0.3386 + 손목 (0.0896 + 0.062) ≈ 0.915 m
```

**HCR-5 공식 작업반경 915mm 와 정확히 일치한다.** 물려받은 CAD 링크 치수를 신뢰해도 된다는 뜻이며,
4단계 좌표계 정합 때 큰 안심 재료다.

> 단 이것은 **치수**의 검증이지 **관절 부호·영점 규약**의 검증이 아니다. 후자는 여전히 실기 대조 대상이다(§9.2).

---

## 7. 부딪힌 문제와 해결 ★ 재현 시 가장 중요한 절

### 7.1 시간 매개변수화 실패 — 가속도 한계 없음

**증상**
```
[ERROR] [moveit.core.time_optimal_trajectory_generation]:
  No acceleration limit was defined for joint joint_1!
  You have to define acceleration limits in the URDF or joint_limits.yaml
[ERROR] [move_group]: PlanningResponseAdapter 'AddTimeOptimalParameterization'
  failed with error code FAILURE
```
계획(OMPL)은 성공하는데 실행이 안 된다.

**원인** — **URDF `<limit>` 태그에는 가속도 항목이 아예 없다.** `effort` 와 `velocity` 만 지원한다.
그래서 Setup Assistant 가 `has_acceleration_limits: false` / `max_acceleration: 0` 으로 생성한다.

`AddTimeOptimalParameterization` 은 궤적에 **시간을 입히는** 단계다. 계획은 "어떤 경로로 갈지"만
정하고, 여기서 "각 지점을 언제 지날지"를 계산한다. 그러려면 속도와 **가속도**가 둘 다 필요하다.

**해결** — `hcr5_moveit_config/config/joint_limits.yaml` 의 6개 관절 전부:
```yaml
has_acceleration_limits: true
max_acceleration: 3.5
```

### 7.2 실행 실패 — 컨트롤러 인터페이스 목록이 빈 리스트

**증상**
```
[ERROR] [follow_joint_trajectory_controller_handle]:
  Action client not connected to action server: hcr_arm_controller/follow_joint_trajectory
[ERROR] [trajectory_execution_manager]: Failed to send trajectory part 1 of 1
[INFO]  [move_group]: CONTROL_FAILED
```

**진단 근거** (증상만으로는 원인이 안 보인다)
```
list_controllers          → joint_state_broadcaster 하나뿐. hcr_arm_controller 없음
list_hardware_components  → joint_1~6/position [available] [unclaimed]   ← 아무도 안 잡아감
```

**원인** — `ros2_controllers.yaml` 이 이렇게 생성돼 있었다:
```yaml
hcr_arm_controller:
  ros__parameters:
    command_interfaces: []      ← 비어 있음
    state_interfaces:   []      ← 비어 있음
```
인터페이스가 없으니 컨트롤러가 **configure 단계에서 실패**해 아예 올라오지 않고, 따라서 액션 서버가
열리지 않아 MoveIt 이 붙지 못했다.

**왜 비었나** — §5.3 의 결정에 따라 Setup Assistant 의 **"ros2_control URDF Modifications" 탭을
건너뛰었기 때문이다.** 그 탭은 블록 생성만 하는 게 아니라 **컨트롤러의 인터페이스 목록도 채운다.**
블록 중복을 피하려던 판단이 이 부작용을 낳았다.

**해결** — `hcr5_description/urdf/hcr5.ros2_control.xacro` 의 선언과 일치시켰다:
```yaml
command_interfaces:
  - position
state_interfaces:
  - position
  - velocity
```

### 7.3 ⚠️ Setup Assistant 재생성 시 되돌아가는 값

**7.1·7.2 의 손보정은 Setup Assistant 를 다시 돌리면 전부 초기화된다.**

| 파일 | 항목 | 넣어야 할 값 |
|---|---|---|
| `config/joint_limits.yaml` | `has_acceleration_limits` / `max_acceleration` | `true` / `3.5` (6개 전부) |
| `config/ros2_controllers.yaml` | `command_interfaces` / `state_interfaces` | `[position]` / `[position, velocity]` |

두 파일 모두 **상단에 이유를 주석으로 박아 두었다.**

**그리고 검증 스크립트를 두었다** — `src/moveit2/tools/ysh_check_moveit_config.py`

```bash
python3 src/moveit2/tools/ysh_check_moveit_config.py
echo $?     # 0 = 전부 통과, 1 = 문제 있음
```

**Setup Assistant 를 재실행한 직후에는 반드시 한 번 돌릴 것.** 검사 항목:

| | 대상 | 잡아내는 것 |
|---|---|---|
| A | `hcr5_arm.xacro` | 관절 속도, **joint_1·6 음수 방향**, joint_3 ±165° |
| B | `joint_limits.yaml` | **가속도 한계** (§7.1) |
| C | `ros2_controllers.yaml` | **인터페이스 빈 리스트** (§7.2) |
| D | `hcr5.srdf` | 그룹이 chain 인지, `tip_link == pen_tip` 인지 |
| E | `hcr5_tool.xacro` | `tool0` · `pen_tip` 정의 |

문제가 있으면 **고칠 값까지 함께 출력**한다. 스크립트는 moveit_config 패키지 **바깥**
(`src/moveit2/tools/`)에 두었으므로 Setup Assistant 재생성에도 살아남는다.
실제로 값을 깨뜨려 `exit=1` 과 지적 내용이 나오는 것까지 확인했다.

### 7.3.1 구조적으로 더 굳히려면 (2단계에서 흡수 예정)

`MoveItConfigsBuilder` 소스를 확인한 결과, 두 파일의 사정이 다르다.

| 파일 | 오버라이드 | 근거 |
|---|---|---|
| `joint_limits.yaml` | **가능** | `.joint_limits(file_path=...)` 가 *"Absolute or relative path"* 를 받는다. 절대경로를 주면 `pathlib` 에서 오른쪽이 이기므로 **`hcr5_description` 쪽 단일 정본**을 가리킬 수 있다 |
| `ros2_controllers.yaml` | **불가** | `launches.py:346` 이 `moveit_config.package_path / "config/ros2_controllers.yaml"` 로 **경로를 하드코딩**한다. `generate_demo_launch` 를 쓰는 한 못 바꾼다 |

→ **해법은 자체 launch 파일이다.** Setup Assistant 는 `demo.launch.py` 등 **정해진 이름만** 생성하므로,
다른 이름(`hcr5_bringup.launch.py` 등)으로 만들면 재생성에도 살아남는다.

2단계에서 `hcr_viz` 노드를 함께 띄우려면 **어차피 자체 launch 가 필요하므로**, 그때 두 config 경로를
`hcr5_description` 쪽으로 고정하면 별도 작업 없이 이 문제가 해소된다.

### 7.4 사소한 것들

| 증상 | 원인·해결 |
|---|---|
| `cp: target '.../meshes/': No such file or directory` | `ros2 pkg create` 는 `meshes/`·`urdf/`·`config/` 를 만들지 않는다 → `mkdir -p` |
| `cd $DST` 실패 | `$DST` 가 상대경로였다 → 절대경로로 지정 |
| `robot_state_publisher` 가 `-p robot_description:="$(cat ...)"` 로 죽음 | URDF 주석의 특수문자가 ROS 인자 파서를 깨뜨린다. FK 검증은 ROS 런타임 없이 URDF 직접 파싱으로 우회했다 |
| `Overrun detected! ... 100 Hz ... took 10.119192 ms` | **문제 아님.** RT 커널이 아닌 Docker 에서는 정상. 다만 §9.1 의 시사점이 있다 |

---

## 8. 재현 절차

```bash
# ── 호스트 ──
cd ~/Nyang_Nyang_Atlier/src/moveit2
./run_container.sh shell          # X11 허용(xhost)까지 스크립트가 처리한다

# ── 컨테이너 안 ──
cd ~/ws_moveit2
colcon build --symlink-install --packages-select hcr5_description hcr5_moveit_config
source install/setup.bash
ros2 launch hcr5_moveit_config demo.launch.py     # ⚠️ 두 번 실행 금지
```

**상태 확인** (다른 터미널):
```bash
docker exec -it moveit2_dev bash
source ~/ws_moveit2/install/setup.bash
ros2 control list_controllers            # 둘 다 active
ros2 control list_hardware_components    # position 6개가 [claimed]
```

**설정 검증** (호스트에서, Setup Assistant 재실행 직후에는 필수):
```bash
python3 src/moveit2/tools/ysh_check_moveit_config.py
```

**RViz 조작**: MotionPlanning 패널 → 인터랙티브 마커 드래그 → `Plan` → `Execute`

> `--symlink-install` 이 걸려 있어 **config YAML 수정은 재빌드 없이 런치 재기동만으로 반영된다.**

---

## 9. 남은 것 · 다음 단계

### 9.1 1단계 마무리 (⬜ 2건)

- [ ] RViz 에서 임의 목표 **10회 연속 Plan & Execute 성공**
- [ ] **인터랙티브 마커가 `pen_tip` 에 붙어 있는지** 확인 (SRDF 는 이미 `tip_link="pen_tip"` 확인됨)
- [ ] Displays 에서 `TF` 켜고 **펜이 팔 바깥을 향하는지** 육안 확인 (§5.2)

현재 스케일링 0.1 이라 실제 동작은 **속도 18°/s · 가속 20°/s²** 정도로 느리다. 초기 검증엔 이게 맞다.

> **`Overrun detected` 경고의 시사점** — 실기 MQTT 상태 브로드캐스트는 **30Hz** 다. 어차피 100Hz
> 실시간 루프를 맞출 이유가 없으므로, 4단계에서 `update_rate` 를 실측 지연에 맞춰 낮춘다.
> **이것이 C++ 실시간 SystemInterface 대신 Python 브릿지로 충분하다고 보는 근거이기도 하다.**

### 9.2 2단계 이후

§2 의 흐름도 참조. 특히:

- **3-1 가짜 HCR-5 스텁**을 2단계와 **병렬로** 만든다 → 실기 없이 브릿지 개발/디버깅이 가능해지고,
  주말의 귀한 실기 시간을 "개발"이 아니라 "검증"에 쓸 수 있다. 실기 연결 시 **IP 한 줄만 교체**.
- **관절 부호·영점 규약 검증** — 현 URDF 의 관절 축 부호가 뒤죽박죽이다
  (`joint_2` 는 `-1 0 0`, `joint_3` 은 `+1 0 0`). CAD 변환 산출물이라 **로봇 자체 규약과 일치한다는
  보장이 없다.** 어긋나면 MoveIt 은 멀쩡한 계획을 세우고 로봇은 엉뚱한 데로 간다.
  검증 수단은 공짜로 있다 — MQTT 가 `motion/tool/position` · `motion/flange/position` 을 30Hz 로 쏘므로,
  **로봇이 계산한 FK 와 우리 URDF 의 FK 를 대조**하면 된다.

---

## 10. 팀 공유 사항

### 10.1 ✅ 해소됨 — 캡처 요청 · movej 연속성 (2026-08-09 갱신)

> 종전 §10.1(캡처 요청)·§10.2(movej 연속성 — "프로젝트 성패가 걸린 질문")를 이 절로 대체한다.
> **둘 다 2026-08-05 인천 세션에서 답이 나왔다.** 원문은 git 이력에 남는다.

| 당시 질문 | 답 (08-05 실증) |
|---|---|
| 절대 관절이동 명령이 무엇인가 | `move/joint/here {jointAngle:[j1..j6]}` — **도달오차 0.0000°** |
| movej 를 연속 실행하면 멈추는가 | **안 멈춘다.** `program/plan` 의 `continues:true` 연쇄에서 전환 시 TCP 최저속도 **19~20mm/s** 유지 (`continues:false` 는 0.02mm/s = 완전정지) |
| 서보 스트리밍 경로를 찾아야 하는가 | **불필요.** 보간을 컨트롤러가 소유하는 `program/plan` 경로로 스트로크가 나온다 |
| 명령 분해능(0.01°)이 선 품질 병목인가 | **아니다.** 그건 펜던트 UI 표시 정밀도였고, 전정밀도로 보내면 컨트롤러가 그대로 반영한다 (TCP 오차 0.001mm) |
| 직선 품질은 | `linear` 179mm 구간에서 이탈 **0.045mm** — 로봇 반복정밀도(±0.1mm)보다 좋다 |

**즉 3단계 설계가 확정됐다.** 궤적을 관절점으로 잘게 썰어 스트리밍하는 방식이 아니라,
**데카르트 웨이포인트를 `program/plan` 의 `linear` 체인으로 넘기고 보간은 컨트롤러에 맡긴다.**

```
시작 노드:  {startVelocity:0, endVelocity:V, continues:true,  radius:0}
중간 노드:  {startVelocity:V, endVelocity:V, continues:true,  radius:0}
종료 노드:  {startVelocity:V, endVelocity:0, continues:false, radius:0}
```

> ⚠️ **`radius`>0 을 스트로크 내부에 쓰지 마라.** `radius:50` 이면 궤적이 그 웨이포인트를
> **22.3mm 떨어져** 지나간다(모서리를 깎는다). `radius:0` 은 0.34mm 로 정확 통과한다.

#### 남은 질문 (우선순위 순)

1. **`program/plan` 크기 한계** — 웨이포인트마다 tcp+flange+joint 3표현이 실려 스트로크
   수백 개면 수 MB JSON 이 된다. Mosquitto 1.4.7 이 받아줄지 미실측. **완주의 잠재 차단 요인.**
   → 우리 쪽 변환기의 `--dry-run` 으로 실제 바이트 수를 뽑아 부하 실험을 한 번에 끝낸다
2. **펜 장착 후 TCP** — `robot/setup/tcp` 로 등록할지, flange 기준으로 두고 펜 오프셋을 PC 가
   소유할지. 후자면 §5.5 의 `tool0` ≡ flange 규약 일치가 그대로 쓰인다
3. **필압 ↔ 충돌감지** — 펜이 종이를 누르는 반력이 전류 기반 충돌감지를 트립시키면 `PAUSED`
   래치로 작화가 반복 중단된다
4. **`program/stop` 지연** — BRD N4 "즉시" 요건 대비 미실측
5. ⚠️ **08-05 세션이 서보를 켠 채 끝났다.** 실기를 만지기 전에 `monitor/robot` 의 축 온도부터
   확인할 것 — 60°C 초과 또는 `-1`(통신 두절)이면 드라이브 0x40 장애 재발이다

#### 역할 정리 — 지금 못 박아야 한다

`HANDOFF_260805_ros2_bridge.md` 기준으로 **`hcr_bridge` 의 궤적 실행 층(T15)이 유일한 미착수
항목**이고, 이것이 moveit2 모듈 담당 범위(궤적 최적화·동작 알고리즘·좌표 변환)와 정확히 겹친다.
**중복 작업을 막으려면 누가 쓰는지 확정이 필요하다.**

### 10.3 중복 조사 방지

`limitCheck` ON/OFF, 충돌 감지·`event/collision/clear` 복구는 **이미 2026-07-29 에 PC 에서 실증됐다**
([`src/drivers/hcr_comm/README.md`](../../drivers/hcr_comm/README.md) §6). "로봇 안전설정 조작여부"
담당자가 중복 조사하지 않도록 이 문서를 공유할 것.

### 10.4 vision 팀과 합의 필요

`각자역할_작업순서.txt` 3번에 **좌표 변환**이 moveit2 몫으로 잡혀 있다. vision 이 A4 위치와 그림을
**vision 좌표계**로 넘기므로, **산출물 포맷(A4 좌상단·우하단을 어떤 단위·어떤 원점으로 줄 것인지)** 을
지금 합의해 두어야 한다. 나중에 어긋나면 양쪽 다 재작업이다.

---

## 참고

- 실기 연결 조사 · 공식 스펙 대조: [`hanwha_robot_arm/docs/실기_연결_현황.md`](../../../hanwha_robot_arm/docs/실기_연결_현황.md)
- MQTT 명령 프로토콜 · 제어 실증: [`src/drivers/hcr_comm/README.md`](../../drivers/hcr_comm/README.md)
- 스펙 출처: **HCR-5 Collaborative Robot User Manual v2.0 (2018-11, ENG)** Appendix F(제원) · G(정지거리) · §16.6(SW Update)
- 역할 분담: `각자역할_작업순서.txt`
- 하드웨어 추상화 경계(N6·F7.3): `docs/System Architecture.md` §6
