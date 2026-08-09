# HCR-5 MoveIt2 워크스페이스 — 작업 정리

> 이 워크스페이스(`ws_moveit2/src/`)에서 만든 것, 실행 방법, 디버깅, 실기 통합 준비를 한 곳에 모았다.
>
> **왜 이렇게 만들었는가**(설계 근거·의사결정 경위)는 [`../../../docs/ysh_HCR5_ros2_control_구축_상세.md`](../../../docs/ysh_HCR5_ros2_control_구축_상세.md) 에 있다.
> **코드를 한 줄씩 뜯어본 해설**은 [`../../../ysh_debug/`](../../../ysh_debug/) 에 있다.
> 이 문서는 **"무엇을 어떻게 돌리는가"** 에 집중한다.

**브랜치** `ysh_moveit_ros2_control` · **환경** ROS 2 Jazzy · `moveit2_dev:jazzy` 컨테이너

---

## 0. 한눈에

```
MoveIt2 (계획)
   ↓ FollowJointTrajectory
JointTrajectoryController
   ↓ ros2_control
[하드웨어 플러그인]  ← mock / MuJoCo / 실기 를 인자 한 줄로 교체
   ↓
(실기) MQTT 1883 → HCR-5 컨트롤러
```

**핵심 3가지**

| | 물려받은 것 | 새로 만든 것 |
|---|---|---|
| 체인 끝 | `link6_1` (플랜지) | **`pen_tip` (펜 끝)** — 그리기의 IK 기준점 |
| 엔드이펙터 | 정의 없음 | **`tool0` · `pen_tip` 툴 체인** |
| 관절/속도 한계 | CAD 기본값 (속도 실기의 32배) | **실기 공식값** (180°/s, ±360°·J3 ±165°) |
| 기구학 치수 | CAD 변환값 (실기와 **43mm** 차이) | **실기 FK 스윕 교정값** (오차 0.01mm) ★08-09 |

**현재 상태** — mock 하드웨어에서 계획·실행 전 구간 통과. 데카르트 경로 달성률 100%.
기구학이 실기와 0.01mm 로 정합됐다.

**실기 쪽** — MQTT 프로토콜·상태 브릿지는 인천에서 08-05 에 완료됐다.
**남은 것은 궤적 실행 층(T15) 하나**다 (§4).

---

## 1. 작업한 파일과 내용

패키지 4개. 전부 `ws_moveit2/src/` 아래.

```
hcr5_description/     로봇 기술서 (URDF · 메쉬 · 관절한계)
hcr5_moveit_config/   MoveIt 설정 (Setup Assistant 생성 + 손보정 3건)
hcr5_examples/        C++ 예제 4종 (관절목표 → IK → 데카르트 경로)
hcr5_viz/             펜 끝 자취 시각화 (Python, 읽기 전용)
```

> `hanwha_robot_arm/` (= `hcr_robot_description`, `hcr_moveit_config`) 은 컨테이너 마운트로
> 들어온 **물려받은 자산**이다. 건드리지 않는다. 패키지 이름을 `hcr5_*` 로 지은 것도
> 이름 충돌을 피하기 위해서다.

### 1.1 `hcr5_description` — 로봇 기술서

| 파일 | 내용 |
|---|---|
| `urdf/hcr5.urdf.xacro` | ★ **최상위.** 인자 3개로 sim/실기/펜길이를 가른다 |
| `urdf/hcr5_arm.xacro` | 링크 7 · 관절 6. **관절 origin·한계 모두 실기 실측 정본** (08-09 교정) |
| `urdf/hcr5_tool.xacro` | ★ **신설.** `tool0` · `pen_tip` 툴 체인 |
| `urdf/hcr5.ros2_control.xacro` | ★ **sim / 실기 갈림길.** 플러그인이 인자 |
| `urdf/hcr5_materials.xacro` | 색상 |
| `config/joint_limits.yaml` | 원본 근거 (출처 주석 포함). MoveIt 이 읽는 것은 moveit_config 쪽 |
| `meshes/*.stl` | 7개. CAD 변환 산출물이라 재생성 불가 |

**최상위 인자 3개**

| 인자 | 기본값 | 용도 |
|---|---|---|
| `hardware_plugin` | `mock_components/GenericSystem` | **실기 연결 지점** |
| `ros2_control` | `true` | `false` 면 블록을 만들지 않음 (sim 이 자체 정의할 때) |
| `pen_length` | `0.15` | 펜 끝까지 거리 [m] |

**교체한 관절 한계**

| 관절 | 기존 | 교체 | 이유 |
|---|---|---|---|
| joint_1·6 | `0°~+360°` | **±360°** | 음수 방향이 막혀 11° 이동이 **348° 대회전**으로 계획됐다 |
| joint_2·4 | ±90° / ±170° | **±360°** | CAD 더미값 → 공식값 (08-09) |
| joint_5 | ±120° | **±360°** | ⛔ **실기 wrist2 가 −267° 로 관측된다** — 종전 값으로는 실기의 현재 자세조차 표현 못 한다 (08-09) |
| joint_3 | ±130° | **±165°** | 실기 공식값. 유일하게 별도 제한 있는 관절 |
| 전 관절 | `100 rad/s` | **`3.141593`** (180°/s) | 실기의 약 32배였다 |

**⚠️ 08-09 기구학 정본 교정 — 43mm**

물려받은 CAD 치수가 실기와 달랐다. 인천이 컨트롤러 FK RPC 로 108 샘플 스윕해 최소제곱 교정한
값을 이식했다 — **109개 자세 RMS 43.4mm → 0.006mm.**

| 관절 | 기존 (CAD) | 정본 (실측) | 차이 |
|---|---|---|---|
| `joint_2.x` / `.z` | −0.0785 / 0.118778 | **−0.059138 / 0.119001** | +19.36 / +0.22 mm |
| `joint_3.x` / `.y` / `.z` | 0.000652 / 0.001 / 0.424 | **−0.019348 / 0 / 0.425001** | −20.00 / −1.00 / +1.00 mm |
| `joint_4.z` | 0.338567 | **0.338499** | −0.07 mm |
| `joint_5.z` | 0.089596 | **0.089504** | −0.09 mm |
| **`joint_6.x`** | **−0.089596** | **−0.132498** | **−42.90 mm ← 지배적** |

**이것이 위험한 이유는 에러가 안 나기 때문이다.** 달성률 100%, TOTG 성공, 실행 SUCCESS —
전부 정상으로 보이고 펜만 43mm 옆에 그린다. A4 폭이 210mm 다.

mesh visual/collision origin 도 함께 재계산했다(이 파일의 mesh origin 은 "영점 자세 누적변환의
역"이라는 유도값이다). 그 결과 RViz 에서 **`link5_1 ↔ link6_1` 사이에 약 43mm 틈이 보인다** —
실기 손목이 CAD 모델보다 그만큼 길다는 뜻이고, **기구학은 옳다.** STL 재작업은 별도 과제.

**실기 ↔ URDF 영점 규약** (URDF 에는 담기지 않는다 — 브릿지가 소유):

```
q_URDF[i](도) = SIGN[i] * q_real[i](도) + DELTA[i]
  SIGN  = (+1, +1, −1, +1, +1, +1)     ← J3 만 부호 반전
  DELTA = ( 90,  90,   0,  90,   0,  0)
```

⚠️ **이 변환을 빼먹으면 로봇이 에러 없이 전혀 다른 자세로 간다.**

### 1.2 `hcr5_moveit_config` — MoveIt 설정

Setup Assistant 생성본 + **손보정 3건**.

| 파일 | 손본 내용 | 왜 |
|---|---|---|
| `config/joint_limits.yaml` | `has_acceleration_limits: true` / `max_acceleration: 3.5` | URDF `<limit>` 에 가속도 항목이 **없어서** TOTG 가 실패한다 |
| `config/ros2_controllers.yaml` | `command_interfaces: [position]` / `state_interfaces: [position, velocity]` | 빈 `[]` 로 생성되어 컨트롤러가 configure 에 실패한다 |
| `config/moveit.rviz` | `Loop Animation: false` / `Robot Alpha: 1.0` | 실제 로봇이 흐리게 보여 목표 미리보기와 뒤바뀌어 읽혔다 |

⚠️ **Setup Assistant 를 다시 돌리면 위 3건이 전부 초기화된다.** 검증 스크립트로 확인할 것 (§3.1).

**SRDF 핵심**
```xml
<group name="hcr_arm">
  <chain base_link="base_link" tip_link="pen_tip"/>   <!-- ★ 펜 끝 기준 IK -->
</group>
```

### 1.3 `hcr5_examples` — C++ 예제

| 실행파일 | 소스 | 무엇을 확인하나 |
|---|---|---|
| `joint_goal` | `01_joint_goal.cpp` | IK 없이 **계획→실행 사슬만** |
| `pose_goal` | `02_pose_goal.cpp` | **IK 가 펜 끝 기준**으로 풀리는지 |
| `cartesian_square` | `03_cartesian_square.cpp` | ★ **데카르트 직선 경로** — 그리기의 원형 |
| `4_cartesian_square` | `04_cartesian_square.cpp` | 실험용 (곡선+대각선+계단 복합 도형) |
| **`draw_contour`** | **`05_draw_contour.cpp`** | ★★ **비전 좌표를 그대로 그린다** — 실제 작업 |

- 01~04 는 **절대 좌표를 박지 않고 현재 자세 기준 상대 이동** → 어디서 실행해도 대체로 도달 가능
- **05 는 다르다.** SRDF `home`(펜이 수직으로 아래를 향하는 자세)로 먼저 이동한 뒤,
  그 자세의 pen_tip 위치를 중심으로 수평 XY 평면에 그린다

**05 가 하는 일**

```
① SRDF home 으로 이동          펜이 −Z 를 향하게 (RPY 180,0,0)
② 픽셀 → 로봇 좌표 변환         x = cx − (v−v_c)·s ,  y = cy − (u−u_c)·s
③ 입력 도형을 초록 선으로 발행   /hcr5_examples/target_shape
④ 펜 든 채 시작점 위로 이동      (이 선이 자취에 남지 않게 그리기와 분리)
⑤ /pen_trail/clear 호출         자취에 "그린 것만" 남기려고
⑥ 하강 → 윤곽선 → 상승          한 번의 데카르트 경로
```

② 의 **u·v 양쪽에 음부호가 붙는 것이 핵심**이다. 한쪽만 뒤집으면 거울상이 된다
(이미지 v 가 아래로 증가하는 보정 하나, 관찰자 오른쪽이 −Y 인 보정 하나).

좌표는 지금 `kContour` 에 박혀 있다(고양이 70점). vision 이 넘겨주면 **거기만 갈아끼우면 된다.**
- `launch/example.launch.py` 가 `robot_description`·SRDF·kinematics·joint_limits 를 주입한다.
  **`ros2 run` 으로는 뜨지 않는다** (§3.3)

### 1.4 `hcr5_viz` — 펜 끝 자취 시각화

`pen_trail` 노드 하나. **로봇을 제어하지 않는 읽기 전용.**

```
30Hz 타이머 → TF(base_link→pen_tip) 조회 → 직전 점에서 2mm 이상 움직였나?
   → self.points 에 추가 → MarkerArray 발행
        LINE_STRIP (빨강)  지나간 자취
        SPHERE     (노랑)  현재 펜 끝
```

- Python(rclpy) — MoveIt C++ API 가 필요 없고, `--symlink-install` 덕에 **재빌드 없이** 값을 만질 수 있다
- launch 파일 없음 — 파라미터를 전부 기본값과 함께 선언해 외부 주입이 불필요하다
- 기동 시 `DELETEALL` 을 1회 발행해 이전 실행의 자취를 지운다

---

## 2. 실행 명령어 모음

### 2.1 컨테이너 진입

```bash
# 호스트에서
cd ~/Nyang_Nyang_Atlier/src/moveit2
./run_container.sh shell          # X11 허용까지 스크립트가 처리한다

# 컨테이너 안 — 새 터미널마다 매번
source ~/ws_moveit2/install/setup.bash
```

| 명령 | 용도 |
|---|---|
| `./run_container.sh shell` | 진입 (인자 없이 실행해도 동일) |
| `./run_container.sh up` / `down` | 기동 / 종료 |
| `./run_container.sh logs` | 로그 |
| `./run_container.sh build` | 이미지 재빌드 (20~40분) |

### 2.2 빌드

```bash
cd ~/ws_moveit2
colcon build --symlink-install --packages-select \
    hcr5_description hcr5_moveit_config hcr5_examples hcr5_viz
source install/setup.bash
```

**재빌드가 필요한가**

| 고친 파일 | 재빌드 |
|---|---|
| **C++ (`.cpp`)** | **필요** |
| Python · YAML · xacro · launch · rviz | 불필요 (노드/런치만 다시 띄우면 됨) |

`--symlink-install` 이 텍스트 파일을 심볼릭 링크로 걸어주지만, C++ 은 컴파일 결과물이 실행되므로 예외다.

### 2.3 기본 3터미널 구성

| 터미널 | 명령 | 역할 |
|---|---|---|
| 1 | `ros2 launch hcr5_moveit_config demo.launch.py` | MoveIt + 컨트롤러 + RViz |
| 2 | `ros2 run hcr5_viz pen_trail` | 자취 시각화 |
| 3 | `ros2 launch hcr5_examples example.launch.py example:=cartesian_square` | 도형 그리기 |

⚠️ **1번을 두 번 띄우지 말 것.** controller_manager 가 둘이 되어 `/joint_states` 가 충돌한다.

### 2.4 MoveIt 기동 옵션

```bash
ros2 launch hcr5_moveit_config demo.launch.py                 # RViz 포함
ros2 launch hcr5_moveit_config demo.launch.py use_rviz:=false # 헤드리스 (테스트용)
ros2 launch hcr5_moveit_config demo.launch.py --show-args     # 인자 목록
```

### 2.5 예제 실행

```bash
ros2 launch hcr5_examples example.launch.py example:=joint_goal
ros2 launch hcr5_examples example.launch.py example:=pose_goal
ros2 launch hcr5_examples example.launch.py example:=cartesian_square
ros2 launch hcr5_examples example.launch.py example:=4_cartesian_square
ros2 launch hcr5_examples example.launch.py example:=draw_contour
```

**`draw_contour` 인자** — 재빌드 없이 실험한다.

| 인자 | 기본 | 뜻 |
|---|---|---|
| `draw_size` | `0.15` | 도형의 긴 변 [m] |
| `eef_step` | `0.002` | 데카르트 보간 간격 [m] — 작을수록 매끄럽다 |
| `hover` | `0.03` | 펜을 들고 이동할 높이 [m] |
| `vel_scale`·`acc_scale` | `0.1` | 속도·가속도 스케일 |
| `execute` | `true` | `false` 면 계획만 (도달 가능성만 빠르게 확인) |
| `go_home` | `true` | `false` 면 현재 자세에서 바로 |

```bash
ros2 launch hcr5_examples example.launch.py example:=draw_contour draw_size:=0.25
ros2 launch hcr5_examples example.launch.py example:=draw_contour execute:=false
```

> ⚠️ **`ros2 launch` 에 `--ros-args -p x:=y` 를 붙여도 노드로 전달되지 않는다.**
> 위처럼 `key:=value` 런치 인자로 줘야 한다. (launch 가 `ParameterValue(..., value_type=)`
> 로 타입을 붙여 노드에 실어 준다 — 타입을 안 붙이면 문자열로 들어가 노드가 예외를 낸다)

### 2.5.1 05 검증 결과 (mock, 2026-08-09)

고양이 윤곽선 70점, `draw_size:=0.15` 기본값.

```
입력 도형   70 점 · bbox 475×535 px → 133.2×150.0 mm (배율 0.00028 m/px)
달성률      100.0%  ·  궤적 구간 195  ·  소요 19.3 초
종료점 오차 0.01 mm
```

`pen_trail` 자취(249점, 펜 다운 구간)를 입력 도형과 대조한 결과:

| 항목 | 값 |
|---|---|
| 입력 대비 편차 평균 | **0.064 mm** |
| 〃 중앙 / 95% / 최대 | 0.040 / 0.211 / **0.404 mm** |
| **평면 이탈 (z 폭)** | **0.261 mm** |
| 자취 bbox | 150.0 × 133.0 mm (입력 150.0 × 133.2) |

**편차의 정체** — mock 하드웨어는 명령을 그대로 따르므로 이 오차는 로봇이 아니라
**우리 파이프라인**에서 나온다. `computeCartesianPath` 가 2mm 간격으로 IK 를 풀고, 그
사이는 컨트롤러가 **관절 공간에서 선형 보간**한다. 관절 공간 직선은 데카르트 공간에서
직선이 아니라서 살짝 부풀고, 그것이 평면 이탈 0.26mm 로 나타난다.
→ **`eef_step` 을 줄이면 줄어든다.** 실기 선 품질 예산을 짤 때 이 값을 먼저 확보할 것.

**크기 스윕** — 어디서 깨지는지 확인했다 (`execute:=false` 로 계획만):

| `draw_size` | 실제 크기 | 달성률 |
|---|---|---|
| 0.15 | 133×150 mm | 100.0% |
| 0.20 | 178×200 mm | 100.0% |
| 0.28 | 249×280 mm | 100.0% |
| 0.35 | 311×350 mm | 100.0% |
| 0.45 | 400×450 mm | 100.0% |
| 0.55 | 488×550 mm | 100.0% |

**A4(210×297) 는 여유롭게 통과한다.** 홈 자세 기준 그리기 평면(z=291.5mm, 중심
x=490mm)이 작업반경 915mm 안쪽 한가운데라 550mm 까지도 IK 가 끊기지 않는다.
실기에서는 이보다 **관절 속도·가속도와 특이점 회피가 먼저 한계가 될 가능성이 높다.**

### 2.6 자취 시각화

```bash
ros2 run hcr5_viz pen_trail                                            # 기본
ros2 run hcr5_viz pen_trail --ros-args -p min_point_distance:=0.0002   # 세밀한 도형용
ros2 run hcr5_viz pen_trail --ros-args -p tip_frame:=link6_1           # 플랜지 자취와 비교

ros2 service call /pen_trail/clear std_srvs/srv/Empty                  # 자취 지우기
```

**RViz 설정** — `Add` → `By topic` → `/pen_trail/trail` → `MarkerArray`

| 파라미터 | 기본 | 뜻 |
|---|---|---|
| `base_frame` | `base_link` | 기준 좌표계 |
| `tip_frame` | `pen_tip` | 따라갈 프레임 |
| `min_point_distance` | `0.002` | 이만큼 움직여야 점을 찍는다 [m] |
| `max_points` | `20000` | 최대 점 개수 |
| `publish_rate` | `30.0` | 발행 주기 [Hz] |
| `line_width` | `0.002` | 선 굵기 [m] |
| `tip_size` | `0.008` | 현재 위치 구슬 지름 [m] |

⚠️ **`min_point_distance` 가 도형의 스텝보다 크면 세부가 뭉개진다.** 1mm 단위 도형이면 `0.0002` 정도로 낮출 것.

### 2.7 URDF 확인

```bash
# xacro 펼치기
xacro $(ros2 pkg prefix hcr5_description)/share/hcr5_description/urdf/hcr5.urdf.xacro > /tmp/hcr5.urdf

# 링크 트리 (끝이 pen_tip 이어야 한다)
check_urdf /tmp/hcr5.urdf

# 인자 교체 확인
xacro ... hardware_plugin:=mujoco_ros2_control/MujocoSystemInterface | grep "<plugin>"
xacro ... ros2_control:=false | grep -c "<ros2_control"      # 0 이면 정상
xacro ... pen_length:=0.182 | grep -A3 tool0_to_pen_tip
```

기대되는 트리:
```
base_link → link1_1 → … → link6_1 → tool0 → pen_tip
```

---

## 3. 테스트 · 디버깅

### 3.1 설정 검증 스크립트 ★ 가장 먼저

```bash
# 호스트에서
python3 ~/Nyang_Nyang_Atlier/src/moveit2/tools/ysh_check_moveit_config.py
echo $?     # 0 = 통과 / 1 = 문제
```

**Setup Assistant 를 재실행한 직후에는 반드시 돌릴 것.** 검사 항목:

| | 대상 | 잡아내는 것 |
|---|---|---|
| A | `hcr5_arm.xacro` | 관절 속도, **가동범위가 공식값보다 좁지 않은지** (좁으면 IK 해가 있어도 계획 실패) |
| B | `joint_limits.yaml` | **가속도 한계** |
| C | `ros2_controllers.yaml` | **인터페이스 빈 리스트** |
| D | `hcr5.srdf` | 그룹이 chain 인지, `tip_link == pen_tip` 인지 |
| E | `hcr5_tool.xacro` | `tool0` · `pen_tip` 정의 |
| **F** | `hcr5_arm.xacro` | ★ **관절 origin 이 실측 정본과 같은지** + **순기구학을 직접 풀어** 홈 자세 flange 를 실측(490.0, −170.5, 441.5)과 대조 |
| **G** | `hcr5.srdf` | `home` 이 실기 `move/joint/home` 자세인지 |

**[F] 가 가장 중요하다.** 다른 검사는 전부 "에러가 나서 알 수 있는" 것을 앞당겨 잡는 것이지만,
기구학이 되돌아가면 **아무 에러 없이 펜이 43mm 옆에 그린다.** 그래서 [F] 는 값 대조에 그치지 않고
FK 를 직접 풀어 실측 좌표와 대조한다 (통과 시 출력: `차이 0.01mm`).

문제가 있으면 **고칠 값까지 함께 출력**한다. moveit_config 패키지 바깥에 있어 재생성에도 살아남는다.

### 3.2 런타임 상태 확인

```bash
ros2 control list_controllers            # 2개 모두 active 여야 함
ros2 control list_hardware_components    # ★ 지금 mock 인지 실기인지 판별하는 가장 빠른 방법
ros2 node list
ros2 topic echo /joint_states --once --field position
ros2 run tf2_ros tf2_echo base_link pen_tip
ros2 topic info /pen_trail/trail          # Publisher/Subscription count
```

**정상 출력**
```
joint_state_broadcaster  ... active
hcr_arm_controller       ... active

name: hcr5_system
plugin name: mock_components/GenericSystem
joint_1~6/position  [available] [claimed]      ← claimed 여야 컨트롤러가 하드웨어를 잡은 것
```

### 3.3 진단표 — 증상 → 원인

| 증상 | 원인 |
|---|---|
| `Failed to fetch parameter 'robot_description'` | `ros2 run` 으로 예제를 띄웠다. **launch 를 써야 한다** |
| 관절값을 못 받음 / 무한 대기 | `demo.launch.py` 미실행, 또는 스핀 스레드 누락 |
| `end effector` 가 `link6_1` | SRDF 그룹이 chain 이 아니다 → `Add Kin. Chain` 으로 재설정 |
| `No acceleration limit was defined for joint joint_1!` | `joint_limits.yaml` 가속도 한계 (§1.2) |
| `Action client not connected to action server` | `ros2_controllers.yaml` 인터페이스가 빈 `[]` (§1.2) |
| pose 목표가 **항상** 실패 | kinematics 파라미터 누락 |
| pose 목표가 **가끔** 실패 | KDL timeout(0.005s). 재실행하거나 TRAC-IK 검토 |
| 데카르트 달성률 < 100% | 시작 자세 / 도형 크기 / 특이점 / IK 솔버 |
| `executable '...' not found` | 실행파일 이름 오타. `ls install/hcr5_examples/lib/hcr5_examples/` |
| **실행은 성공인데 RViz 가 안 움직임** | **주황색은 `Query Goal State`(목표 미리보기)다.** 실제 로봇은 `Scene Robot` |
| 노드를 껐는데도 선이 보임 | Marker `lifetime` 기본값 0 = **영구 유지**. RViz 가 들고 있는 것 |
| `/joint_states` 가 튐 | `demo.launch.py` 가 **두 번** 떠 있다 |
| C++ 고쳤는데 반영 안 됨 | **재빌드 필요** (§2.2) |

### 3.4 중복 실행 확인 · 정리

```bash
# ⚠️ pgrep -cf "패턴" 은 자기 명령줄까지 세므로 쓰지 말 것 (오진 유발)
ps -eo pid,args --no-headers | grep -E "[m]ove_group|[r]os2_control_node|[r]viz2|[p]en_trail"

# 정리 — PID 로 직접 (pkill -f 는 자기 자신을 먼저 죽인다)
kill <PID> ...
```

각각 **1개씩**이면 정상이다.

### 3.5 자취 데이터 수치 확인

```bash
ros2 topic echo /pen_trail/trail --once --full-length > /tmp/m.yaml
```
```python
import yaml
d = [x for x in yaml.safe_load_all(open("/tmp/m.yaml")) if x][0]
for m in d["markers"]:
    pts = m.get("points") or []
    print(m["ns"], len(pts))
    for ax in "xyz":
        v = [p[ax] for p in pts]
        if v: print(f"  {ax}: {min(v):+.4f} ~ {max(v):+.4f}  폭 {max(v)-min(v):.4f}")
```

**검증 예시** (XZ 평면 10cm 사각형):
```
x 폭 0.1000 m   ← 정확히 10cm
z 폭 0.1000 m   ← 정확히 10cm
y 폭 0.0000 m   ← ★ 평면을 벗어나지 않았다
```
`y 폭 0` 이 핵심이다 — 그리기의 요구가 "종이 평면 유지"이므로 이 값이 곧 품질 지표다.

### 3.6 알아둘 함정

- **`ros2 topic echo` 출력은 YAML 문서가 여러 개**다. `safe_load` 는 실패한다 → `safe_load_all`
- **`robot_description` 을 `-p` 로 넘기면 안 된다** — URDF 주석의 특수문자가 ROS 인자 파서를 깨뜨린다
- **`TOTG 가 궤적점을 병합한다`** — `EEF_STEP` 을 촘촘히 해도 관절값 차이가 미미한 점들은 합쳐진다.
  1mm 스텝 도형에서 226점 예상 → 실제 42점. **그리기 해상도의 하한이 여기서 결정될 수 있다**
- **`Overrun detected! ... 100 Hz`** 경고는 문제가 아니다. RT 커널이 아닌 Docker 에서는 정상

---

## 4. 이후 로봇과 통합할 때

### 4.1 지금 어디까지 됐고, 무엇이 남았나

> **2026-08-09 전면 갱신.** 종전 §4.1~§4.4 는 "movej 프로토콜 미포착" 을 블로커로 두고 있었으나
> **인천 08-05 세션에서 전부 해소됐다.** 원문은 git 이력에 남는다.

```
── 우리 쪽 (ysh, 이 브랜치) ──────────────────────────
✅ 계획 · 실행 사슬 전 구간 (move_group → JTC → ros2_control → mock)
✅ 펜 끝 기준 IK · 데카르트 경로 달성률 100%
✅ 자취 시각화 (hcr5_viz)
✅ 기구학 정본 교정 — 실기와 0.01mm 정합            ← 08-09

── 인천 쪽 (markch/hcr5_ros2, 08-05) ────────────────
✅ MQTT 명령 프로토콜 20여 종 역설계
✅ 절대 관절이동 move/joint/here — 도달오차 0.0000°
✅ 연속 스트로크 블렌딩 (program/plan) 실증
✅ hcr_bridge 상태 층 — MQTT → /joint_states, 실기 검증 완료

── 남은 것 ───────────────────────────────────────────
⬜ hcr_bridge 궤적 실행 층 (T15)   ← ★ 유일한 미착수 · 담당 확정 필요
⬜ 펜 장착 후 TCP 정책
⬜ 필압 ↔ 충돌감지 실험
```

### 4.2 ✅ 해소됨 — 실기 명령 프로토콜

| 당시 질문 | 답 (08-05 실증) |
|---|---|
| 절대 관절이동 명령 | `move/joint/here {jointAngle:[j1..j6]}` — 도달오차 **0.0000°** |
| movej 연속 실행 시 멈추는가 | **안 멈춘다.** `program/plan` 의 `continues:true` 연쇄에서 전환 시 TCP 최저속도 **19~20mm/s** 유지 (`continues:false` 는 0.02mm/s = 완전정지) |
| 서보 스트리밍을 찾아야 하나 | **불필요** |
| 0.01° 분해능이 병목인가 | **아니다.** 펜던트 UI 표시 정밀도였다. 전정밀도 전송 시 TCP 오차 0.001mm |
| 직선 품질 | `linear` 179mm 구간 이탈 **0.045mm** — 반복정밀도(±0.1mm)보다 좋다 |

출처: `origin/markch/hcr5_ros2` — `src/drivers/hcr_comm/README.md` · `HANDOFF_260805_ros2_bridge.md`

### 4.3 ⚠️ 실기는 ros2_control 모델과 맞지 않는다 — 설계가 바뀌었다

`hardware_plugin` 인자 한 줄 교체로 실기가 붙을 것으로 설계했으나, 실측 결과 맞지 않는다.

| | mock (현재) | 실기 |
|---|---|---|
| 보간 소유자 | 우리 (TOTG) | **컨트롤러** |
| 명령 단위 | 100Hz 관절 목표 스트림 | 웨이포인트 노드 (`program/plan`) |
| 상태 주기 | 100Hz | **29.1Hz** |
| 이동 모델 | 매 주기 push | 한 번 쏘면 **자율 주행** |

→ 실기 경로는 `SystemInterface` 플러그인이 아니라 **독립 브릿지 노드**가 된다.
`hardware_plugin` 인자는 **mock · MuJoCo 경로에서 그대로 유효**하다.

```bash
# mock (현재)
xacro hcr5.urdf.xacro
# MuJoCo
xacro hcr5.urdf.xacro hardware_plugin:=<sim 플러그인>
```

### 4.4 만들 것 — 궤적 실행 층 (T15)

**ROS 인터페이스 계약은 `/hcr_arm_controller/follow_joint_trajectory` 를 유지한다.**
브릿지가 이 액션을 직접 제공하면 `moveit_controllers.yaml` 무수정이고,
**`hcr5_examples` 예제 4개가 코드 한 줄 안 고치고 실기에서 돈다.**

```
MoveIt2 ─┬─ [A] 이동 · 자세잡기 · 홈복귀
         │      계획 → 마지막 관절점만 → move/joint/here
         │
         └─ [B] 그리기 스트로크                    ★ 본 경로
                MoveIt = 검증 전용 (IK 가부 · 충돌 · 한계 · 달성률)
                실행   = program/plan 의 linear 체인
```

**스트로크 체인 구성 규칙 (08-05 실증 — 그대로 쓸 것)**

```
시작 노드:  {startVelocity:0, endVelocity:V, continues:true,  radius:0}
중간 노드:  {startVelocity:V, endVelocity:V, continues:true,  radius:0}
종료 노드:  {startVelocity:V, endVelocity:0, continues:false, radius:0}
```

> ⚠️ **`radius`>0 을 스트로크 내부에 쓰지 마라.** `radius:50` 이면 궤적이 그 웨이포인트를
> **22.3mm 떨어져** 지나간다(모서리를 깎는다). `radius:0` 은 0.34mm 로 정확 통과한다.

> ⚠️ **속도 프로파일이 물리적으로 가능해야 한다.** 안 그러면 `error/command` 150033 이
> *"there might be singular points"* 로 **오진**시킨다. 이 에러가 나면 자세를 의심하기 전에
> **속도·가속도·구간거리를 먼저 계산하라.**

**작업 순서 — 안전 단계별**

| # | 할 일 | 로봇 필요? |
|---|---|---|
| 1 | 변환기를 **MQTT 없는 순수 함수**로. `--dry-run` 으로 JSON 과 **바이트 크기** 출력 | ❌ |
| 2 | `FollowJointTrajectory` 액션 서버. 1차 목표는 "goal 받고 변환하고 출력하고 **거절**" | ❌ |
| 3 | 실기 단발 검증 — 경로 A 만. `mqtt_cmd.py movej` 의 **3중 가드** 계승 | ✅ |
| 4 | 스트로크 — 직선 1개 → 사각형 → 04 도형 | ✅ |

1번이 미해결 과제 하나를 미리 잡는다: **`program/plan` 크기 한계.** 웨이포인트마다
tcp+flange+joint 3표현이 실려 스트로크 수백 개면 수 MB JSON 이 된다. Mosquitto 1.4.7 이
받아줄지 미실측이고 **완주의 잠재 차단 요인**이다. `--dry-run` 이 실제 바이트 수를 뽑아주면
부하 실험이 한 번에 끝난다. 한계에 걸리면 `program/play {selectedIndex:[a,b]}` 로 배치 분할한다.

**4단계 계측기는 이미 있다** — `hcr5_viz/pen_trail` 이 실기에서 **수정 없이 그대로 돈다.**
상태 브릿지가 `/joint_states` 를 쏘면 `robot_state_publisher` 가 TF 를 만들고, `pen_trail` 은
TF 만 읽기 때문이다. 2단계로 만든 시각화가 그대로 실기 선품질 측정기가 된다.

### 4.5 실기 연결 전 반드시 확인할 것

**① ✅ 영점 · 부호 규약 — 확정됐다 (08-05/08-09)**

우려했던 대로 **URDF 와 실기의 영점 규약이 실제로 달랐다.** 변환식이 확정돼 있다:

```
q_URDF[i](도) = SIGN[i] * q_real[i](도) + DELTA[i]
  SIGN  = (+1, +1, −1, +1, +1, +1)     ← J3 만 부호 반전
  DELTA = ( 90,  90,   0,  90,   0,  0)
```

실기 zero = 팔이 수평으로 뻗은 자세, URDF zero = 팔이 수직으로 선 자세.
구현: `hcr_bridge/include/hcr_bridge/joint_convention.hpp` (이 URDF 에는 담기지 않는다).

⚠️ **이 변환을 빼먹으면 에러 없이 전혀 다른 자세로 간다 — 가장 위험한 실패 방식이다.**

그리고 **치수도 틀렸었다.** 종전 이 문단은 *"치수는 이미 교차검증됐다(작업반경 915mm 일치)"*
라고 적었는데, **작업반경이 맞는 것과 각 관절 origin 이 맞는 것은 다른 문제였다.**
`joint_6.x` 가 42.9mm 틀려 전체 위치오차 RMS 가 43.4mm 였다 → §1.1 참조. 지금은 0.006mm 다.

**② 안전 절차**

- **PC MQTT 명령은 펜던트 인에이블 스위치(데드맨)를 거치지 않는다.** e-stop 에 손, 로봇 반경 정리 필수
- `set/limitCheck {mode:true}` 로 소프트 관절제한을 켜 둘 것
- 충돌은 **래치된다** — `PAUSED` 로 멈추고 `event/collision/clear` 로 명시적 해제가 필요하다
  (감지·정지·해제·재개 전 과정을 PC 에서 처리 가능함이 실증돼 있다)
- 워치독 — 상태 수신이 N ms 끊기면 정지
- 속도·가속 스케일을 **0.1 이하**로 유지하며 시작

**③ 펜홀더 TCP**

설계가 확정되면 `pen_length` 만 교체하면 된다.
```bash
xacro hcr5.urdf.xacro pen_length:=0.182
```
단 툴 링크에 실제 메쉬·충돌형상이 생기면 **자기충돌 매트릭스를 다시 생성해야 하므로
Setup Assistant 재실행이 필요하다.** 그때 §1.2 의 손보정 3건이 초기화되므로 §3.1 검증을 반드시 돌릴 것.

**④ 펜 축 방향 — 벤더 규약과 일치했다 (08-09)**

`flange_p = -π/2` 는 기하 추론 + FK 검증을 거쳤고, 08-09 에 **실측과도 맞았다.**
홈 자세에서 `pen_tip` TF 자세가 **RPY (180°, 0, 0)** 으로 나오는데, 펜던트가 같은 자세에서
보고한 flange **`rx = −180°`** 와 일치한다. 즉 우리 `tool0` 이 **실기 flange 프레임 규약과 같다.**

의미: 3단계에서 MoveIt 의 데카르트 자세를 `program/plan` 의 flange 웨이포인트로 넘길 때
**미지의 회전 보정이 끼지 않는다.**

> 다만 문서에 남은 각도는 `rx` 하나뿐이라 세 축 전부가 확정된 것은 아니다 —
> 실기에서 `mqtt_cmd.py pos` 로 세 각 모두 대조할 것.

### 4.6 실기 검증 순서

```
0. 축 온도 확인 ⚠️      mqtt_cmd.py pos → monitor/robot 의 temp
                        60°C 초과 또는 -1(통신 두절)이면 드라이브 0x40 장애 재발
1. 네트워크 직결        PC 192.168.0.100/24 ↔ 컨트롤러 192.168.0.20
                        컨트롤러 재부팅 시 프로필이 내려간다 → nmcli connection up hcr5
2. 읽기만          ✅   hcr_bridge 상태 층 (allow_motion:=false 기본) — 08-05 검증 완료
3. 영점 · 부호 대조 ✅   확정 (§4.5-①). RViz RobotModel 이 실기 자세와 같은지로 재확인
4. 무동작 쓰기          set/limitCheck 같은 안전한 명령으로 왕복 확인
5. 저속 단축 이동       move/joint/here + 3중 가드 (이동량 상한·관절한계·도착 타임아웃)
6. 다축 이동
7. 짧은 스트로크        program/plan 직선 1개
8. 실제 그리기 궤적
```

각 단계에서 **`pen_trail` 자취를 남겨** 계획 대비 실제 편차를 수치로 확보할 것.
그것이 실기 선 품질의 유일한 계측 수단이다.

---

## 5. 함께 볼 문서

| 문서 | 내용 |
|---|---|
| [`../../../docs/ysh_HCR5_ros2_control_구축_상세.md`](../../../docs/ysh_HCR5_ros2_control_구축_상세.md) | 설계 근거 · 의사결정 경위 · 문제 해결 기록 |
| [`../../../docs/ysh_HCR5_ros2_control_요약.md`](../../../docs/ysh_HCR5_ros2_control_요약.md) | 1페이지 요약 (팀 공유용) |
| [`../../../ysh_debug/`](../../../ysh_debug/) | 코드 한 줄씩 해설본 |
| [`../hcr5_examples/README.md`](../hcr5_examples/README.md) | 예제 상세 |
| [`../hcr5_viz/README.md`](../hcr5_viz/README.md) | 자취 시각화 상세 |
| `../../../../drivers/hcr_comm/README.md` | **MQTT 명령 프로토콜** (실기 통합의 핵심 자료) |
| `../../../../../hanwha_robot_arm/docs/실기_연결_현황.md` | 실기 연결 조사 · 공식 스펙 대조 |
