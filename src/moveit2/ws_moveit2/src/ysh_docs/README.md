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
| 관절/속도 한계 | CAD 기본값 (속도 실기의 32배) | **실기 공식값** (180°/s) |

**현재 상태** — mock 하드웨어에서 계획·실행 전 구간 통과. 데카르트 경로 달성률 100%.
실기 연결은 미착수 (movej 프로토콜 미포착이 블로커, §4).

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
| `urdf/hcr5_arm.xacro` | 링크 7 · 관절 6. CAD 산출물에서 가져오고 **`<limit>` 만 실기값으로 교체** |
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
| joint_3 | ±130° | **±165°** | 실기 공식값 |
| 전 관절 | `100 rad/s` | **`3.1416`** (180°/s) | 실기의 약 32배였다 |

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

- 셋 다 **절대 좌표를 박지 않고 현재 자세 기준 상대 이동** → 어디서 실행해도 대체로 도달 가능
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
```

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
| A | `hcr5_arm.xacro` | 관절 속도, **joint_1·6 음수 방향**, joint_3 ±165° |
| B | `joint_limits.yaml` | **가속도 한계** |
| C | `ros2_controllers.yaml` | **인터페이스 빈 리스트** |
| D | `hcr5.srdf` | 그룹이 chain 인지, `tip_link == pen_tip` 인지 |
| E | `hcr5_tool.xacro` | `tool0` · `pen_tip` 정의 |

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

```
✅ 계획 · 실행 사슬 전 구간 (move_group → JTC → ros2_control → mock 하드웨어)
✅ 펜 끝 기준 IK
✅ 데카르트 경로 달성률 100%
✅ 자취 시각화

⬜ movej 명령 프로토콜 포착        ← 🔴 블로커
⬜ MQTT 브릿지 (SystemInterface)
⬜ 실기 안전 절차
⬜ 영점 · 부호 규약 검증
```

### 4.2 🔴 블로커 — movej 프로토콜 미포착

MQTT 명령 프로토콜은 역설계되어 있으나(`src/drivers/hcr_comm/README.md`),
**절대 관절이동(movej)·직선이동(movel)이 아직 안 잡혔다.** 지금 잡힌 것은 개루프 속도 조그뿐이다.

MoveIt 이 만드는 것은 **6축 동시 시간 매개변수화 궤적**이라, 단축 개루프 조그로는 실행할 방법이 없다.

**해결 경로** — 펜던트에서 movej/movel 을 조작하며 `capture.py` 로 캡처:
```bash
python3 src/drivers/hcr_comm/tools/capture.py 192.168.0.20 1883 ~/cap_movej
```
읽기 전용이라 로봇에 아무것도 쓰지 않는다. 절차는 `docs/ysh_HCR5_ros2_control_구축_상세.md` §10 참조.

**함께 확인해야 할 것 ★** — movej 를 연속 2~3개 실행했을 때 **중간에 멈췄다 가는지, 끊김 없이 이어지는지(blending)**.
"매번 정지"라면 획마다 마디가 져서 선 품질이 결정적으로 나빠지고, **서보 스트리밍 경로를 따로 찾아야 한다.**

### 4.3 교체 지점은 이미 준비돼 있다

```bash
# mock (현재)
xacro hcr5.urdf.xacro

# 실기
xacro hcr5.urdf.xacro hardware_plugin:=<우리드라이버>/HcrSystem
```

`hcr5.ros2_control.xacro` 가 플러그인을 인자로 받으므로 **URDF 는 그대로 두고 한 줄만 바꾸면 된다.**
sim 팀(MuJoCo)도 같은 지점을 쓴다.

### 4.4 만들어야 할 것 — MQTT 브릿지

**권장 구조**
```
MoveIt2 → JTC → ros2_control
                    ↓ 하드웨어 플러그인
                    ↓ (토픽 or 직접)
              hcr_bridge (Python)     ← 기존 hcr_comm MQTT 코드 재사용
                    ↓ MQTT 1883
                 HCR-5
```

- **기존 `hcr_comm` 의 Python MQTT 구현을 그대로 살릴 것.** 순수 stdlib 으로 와이어 프로토콜을 직접 구현했고,
  56,723건 캡처로 검증된 자산이다. C++ 로 재구현하면 어렵게 잡은 버그가 다시 살아난다.
- 실기 MQTT 상태 브로드캐스트는 **30Hz** 다. `update_rate: 100` 을 맞출 이유가 없으므로
  **실측 지연에 맞춰 낮춘다** (`ros2_controllers.yaml`).

**단계**
1. **가짜 HCR-5 스텁** — 캡처된 프로토콜대로 행동하는 가짜 로봇. 실기 없이 브릿지를 개발·디버깅할 수 있고,
   실기 연결 시 **IP 한 줄만 교체**하면 된다
2. **상태 경로** — MQTT `motion/joint/position` → `/joint_states` (읽기 전용, 위험 0)
3. **명령 경로** — 궤적 → movej (프로토콜 확보 후)

### 4.5 실기 연결 전 반드시 확인할 것

**① 영점 · 부호 규약**

현 URDF 의 관절 축 부호가 뒤죽박죽이다 (`joint_2` 는 `-1 0 0`, `joint_3` 은 `+1 0 0`).
CAD 변환 산출물이라 **로봇 자체 규약과 일치한다는 보장이 없다.**
어긋나면 MoveIt 은 멀쩡한 계획을 세우고 로봇은 엉뚱한 데로 간다.

검증 수단은 공짜로 있다 — MQTT 가 `motion/tool/position`·`motion/flange/position` 을 30Hz 로 쏘므로,
**로봇이 계산한 FK 와 우리 URDF 의 FK 를 대조**하면 된다.

> 참고: **치수**는 이미 교차검증됐다. FK 로 계산한 작업반경이 **915mm** 로 공식 제원과 정확히 일치한다.
> 부호·영점만 별도 확인 대상이다.

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

**④ 펜 축 방향 최종 확인**

`flange_p = -π/2` 는 기하 추론 + FK 검증을 거쳤다 (`link6_1 → pen_tip = [-0.15, 0, 0]`).
실기에서 육안으로 한 번 더 확인하고, 어긋나면 `hcr5_tool.xacro` 의 `flange_p` 를 `+1.5707963` 로 바꾸면 된다.

### 4.6 실기 검증 순서

```
1. 네트워크 직결        PC 192.168.0.100/24 ↔ 컨트롤러 192.168.0.20
2. 읽기만               상태 퍼블리셔로 /joint_states 확인 · 지연/지터 실측
3. 영점 · 부호 대조     로봇 FK vs URDF FK
4. 무동작 쓰기          set/limitCheck 같은 안전한 명령으로 왕복 확인
5. 저속 단축 이동
6. 다축 이동
7. 짧은 궤적
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
