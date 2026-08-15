# drawing_cat

MoveIt2 로 고양이 외곽선을 그리고, MuJoCo 로 실제 물리를 태워 확인하는 패키지.

담당 범위: **F3.1**(순서 최적화) · **F4.1**(좌표 변환) · **F4.2**(Cartesian Path)

### 문서가 여러 개인 이유 — 어디를 고쳐야 하나

| 문서 | 읽는 사람 | 무엇이 있나 |
|---|---|---|
| **이 문서** | 이 패키지를 만지는 사람 | 실행법 · 파라미터 · 인터페이스 · 알려진 문제. **기술 사실의 주인** |
| [`CHANGELOG.md`](CHANGELOG.md) | 왜 이렇게 됐는지 찾는 사람 | 작업 단위별 배경 · 변경 · 근거 · 검증 |
| [`moveit2/docs/Pipeline Integration Status.md`](../../../docs/Pipeline%20Integration%20Status.md) | **다른 파트 사람** | 현황 요약 + 팀별 요청. 여기 수치는 이 문서에서 가져온 사본 |
| [`test_tools/README.md`](test_tools/README.md) | 연결을 확인하려는 사람 | 시험 도구 사용법 + **제거 방법** |
| [`../vision_interfaces/README.md`](../vision_interfaces/README.md) | 토픽이 안 들어올 때 | 미러 패키지 주의사항 + **타입 해시 기준값의 주인** |

> ⚠️ 같은 수치가 여러 문서에 나온다. **수정할 때는 이 문서를 먼저 고치고** 팀 문서를
> 맞춘다. 타입 해시만은 예외로 `vision_interfaces/README.md` 가 주인이다.

---

## 1. 구조 — 두 컨테이너가 DDS 로 붙는다

```
┌─ moveit2_dev ──────────────┐         ┌─ simulation_dev ─────────────────┐
│  move_group   (계획·실행)   │         │  ros2_control_node (MuJoCo 물리) │
│  rviz2        (시각화)      │         │  robot_state_publisher           │
│  draw_cat     (응용)        │         │  joint_state_broadcaster         │
│                            │         │  joint_trajectory_controller     │
└────────────────────────────┘         └──────────────────────────────────┘
              │                                        │
              │──[ FollowJointTrajectory 목표 ]───────▶│   명령 (머리 → 몸통)
              │◀─[ /joint_states 500Hz, /tf ]──────────│   상태 (몸통 → 머리)
```

두 컨테이너 모두 `network_mode: host` 라 같은 ROS 그래프에 있다. **`ROS_DOMAIN_ID` 가
같기만 하면** 붙는다.

> ⚠️ **"MoveIt2 가 머리"는 명령 방향을 말하는 것이고, 기동 순서는 상태 방향을 따른다.**
> 플래너는 "지금 로봇이 어디 있는지"에서 출발하고, 그 현재 상태의 출처가 MuJoCo 다.
> 실물도 같다 — 로봇 전원·드라이버를 먼저 올리고 MoveIt 을 띄운다.
>
> 다만 이건 **강제가 아니라 관례다**(실측). move_group 은 시뮬 없이도 기동을 마치고,
> 시뮬이 뒤늦게 떠도 재시작 없이 복구된다. 지켜야 하는 건 "계획·실행을 시도하는
> 시점에 시뮬이 떠 있을 것" 하나뿐이다.

### 1-1. 좌표 하나가 로봇 움직임이 되기까지

```
비전 픽셀 (u, v)
   │
   │  draw_cat — letterbox 변환 (5절)
   ▼
펜 끝 목표점 (mm) → 로봇 좌표 (m)
   │
   │  MoveIt — 경로 만들기
   │    · 홈 이동 : OMPL RRT — 관절공간 탐색
   │    · 그리기  : computeCartesianPath — 직선 보간 + 점마다 IK
   ▼
관절각 궤적 (시각 + 6 관절 각도 목록)
   │
   │  AddTimeOptimalParameterization — 시간 배분만 계산
   ▼
시간까지 붙은 궤적
   │
   │  ═══ DDS 액션 FollowJointTrajectory ═══  ← 여기가 컨테이너 경계
   ▼
joint_trajectory_controller — 스플라인 보간, 500 Hz
   │
   │  매 2 ms "지금 이 각도로 가라" 위치 지령
   ▼
MuJoCo position actuator → 물리 엔진 (관성·중력·접촉)
   │
   │  실제 각도
   ▼
/joint_states 500 Hz → move_group 으로 되돌아감
```

> ⚠️ **"최적 궤적"이 아니다.** 흔한 오해라 짚어 둔다.
>
> | 단계 | 실제 | 최적인가 |
> |---|---|---|
> | 홈 이동 | OMPL **RRT** (샘플링 기반) | ❌ 실행 가능한 경로 하나를 찾을 뿐. 매번 다른 경로가 나온다 |
> | **스트로크 순서** | **F3.1 Nearest Neighbor** | ⚠️ **근사.** 펜업 이동 거리를 줄이지만 최적해는 보장하지 않는다 (9절) |
> | 그리기 | `computeCartesianPath` | ❌ **플래너가 아니다.** 직선 보간 + IK. 탐색도 최적화도 없다 |
> | 시간 배분 | `AddTimeOptimalParameterization` | ⭕ 단 **경로 모양이 아니라 시간만**. 이미 정해진 경로 위에서 속도·가속 프로파일을 관절 한계 안에 맞춘다 |
>
> **그리는 경로 모양은 여전히 비전이 준 좌표 그대로**다. F3.1 이 정하는 것은 **스트로크를
> 어떤 순서로, 어느 점부터 그릴지**이지 선 모양이 아니다. MoveIt 은 그걸 관절 각도로 바꿀 뿐이다.
>
> 게다가 `planning.velocity_scaling`·`acceleration_scaling` 이 기본 **0.1**(10 %)이라,
> 시간 최적화도 "한계의 10 % 안에서 최적"이다. 실제로는 일부러 느리게 돈다. 단
> **펜업 이동만은** `travel_velocity_scaling`(기본 0.5)으로 따로 빠르게 간다 — 종이에
> 닿지 않는 구간이라 경로도 속도도 그림에 영향을 주지 않기 때문이다.

> ⚠️ **MuJoCo 는 궤적을 "재생"하지 않는다 — 이것이 추종 오차의 원인이다.**
> 컨트롤러가 500 Hz 로 푼 위치 지령을 MuJoCo 의 position actuator 가 목표로 삼아 토크를
> 만들고, 물리 엔진이 관성·중력·접촉을 풀어 **실제 각도**를 낸다. 즉 **"가라는 각도"를
> 받을 뿐 그 각도가 되는 것은 보장되지 않는다.**
>
> | 오늘 잰 문제 | 위 사슬의 어디서 생기나 |
> |---|---|
> | joint_2 오차 14.96° | **물리 엔진** — 관성이 커서 지령을 못 따라감 |
> | joint_1 눌림 (−7.07° → −0.03°) | **MJCF 관절 한계** — 물리적으로 갈 수 없음 |
>
> 속도 배율이 이미 10 % 인데도 이만큼 벌어진다는 점에 유의할 것. 자세한 것은 9 절.

---

## 2. 실행

### 2-0. 처음 받았다면 — 빌드부터

아래 실행 절차는 전부 `source install/setup.bash` 로 시작한다. **저장소를 새로 받았으면
`install/` 이 없으므로** 먼저 빌드해야 한다.

```bash
cd ~/Nyang_Nyang_Atlier/src/moveit2
export ROS_DOMAIN_ID=42        # ⚠️ 컨테이너를 만들기 전에. 10 절 참조
./run_container.sh shell

cd ~/ws_moveit2
colcon build --packages-select vision_interfaces drawing_cat
source install/setup.bash
```

> ⚠️ **`vision_interfaces` 를 같이 빌드해야 한다.** `drawing_cat` 이 이 메시지 타입에
> 의존하는데, vision 팀 정의를 **복제해 둔 미러 패키지**라 이 워크스페이스 안에 있다.
> 빼먹으면 `drawing_cat` 이 헤더를 못 찾아 빌드가 실패한다. 복제하는 이유와 주의사항은
> [`vision_interfaces/README.md`](../vision_interfaces/README.md).

**로봇 없이 먼저 확인할 수 있는 것** — 컨테이너만 있으면 되고 시뮬레이터도 필요 없다:

```bash
colcon test --packages-select drawing_cat && colcon test-result --verbose
#   F3.1 순서 최적화 단위시험 5 케이스. 몇 밀리초면 끝난다 (9 절)
```

시뮬레이션 컨테이너(`sim_bringup`)는 별도 워크스페이스라 그쪽 절차를 따른다
(`src/simulation/README.md`).

### 순서 (양쪽 컨테이너의 `ROS_DOMAIN_ID` 가 같아야 한다)

```bash
# ── ① simulation 컨테이너 — 몸통
cd ~/Nyang_Nyang_Atlier/src/simulation
export ROS_DOMAIN_ID=42        # ⚠️ 컨테이너가 없으면 여기서 만들어진다. 10절 참조
./run_container.sh shell
cd ~/ws_simulation && source install/setup.bash
ros2 launch sim_bringup mujoco_sim.launch.py headless:=true
#   기다릴 것: Configured and activated joint_trajectory_controller
#   MuJoCo 창을 보려면 headless:=true 를 뺀다

# ── ② moveit2 컨테이너 — 머리 (move_group + RViz)
cd ~/Nyang_Nyang_Atlier/src/moveit2
export ROS_DOMAIN_ID=42
./run_container.sh shell
ros2 control list_controllers
#   ⚠️ 여기서 joint_trajectory_controller 가 나와야 한다. 10절 참조
cd ~/ws_moveit2 && source install/setup.bash
ros2 launch drawing_cat mujoco_moveit.launch.py
#   기다릴 것: You can start planning now!

# ── ③ moveit2 컨테이너 (다른 셸) — 그리기
cd ~/Nyang_Nyang_Atlier/src/moveit2 && ./run_container.sh shell
cd ~/ws_moveit2 && source install/setup.bash
ros2 launch drawing_cat draw_cat.launch.py \
  params_file:=$(ros2 pkg prefix drawing_cat --share)/config/topic_example.yaml
#   기다릴 것: '/vision/strokes' 구독 중 ... 최대 120초 대기

# ── ④ moveit2 컨테이너 (또 다른 셸) — 비전이 없을 때만
cd ~/Nyang_Nyang_Atlier/src/moveit2 && ./run_container.sh shell
cd ~/ws_moveit2 && source install/setup.bash
ros2 run drawing_cat fake_vision_publisher      # ⚠️ test_tools, 확인 전용
```

> ③ · ④ 는 **셸을 새로 여는 것**이므로 컨테이너 진입부터 다시 한다. 이미 떠 있는
> 컨테이너에 붙는 것이라 `export` 는 필요 없다 — `docker exec` 가 컨테이너의
> `ROS_DOMAIN_ID` 를 물려받는다 (10절).
>
> `.bashrc` 가 `/opt/ros/jazzy` 와 워크스페이스를 자동으로 source 하므로 `ros2` 명령은
> 바로 쓸 수 있다. 그래도 `source install/setup.bash` 를 한 번 더 치는 이유는, 셸을 연
> **뒤에** 빌드했다면 자동 source 가 그 결과를 못 잡기 때문이다.

> ⚠️ **③ 이 대기에 들어간 뒤 120 초 안에 ④ 를 실행해야 한다** (`wait_timeout_s`).
> 넘기면 종료되며, 그때는 ③ 만 다시 실행하면 된다. 셸을 옮겨 다니다 보면 생각보다 촉박하다
> — 실제로 두 번 놓쳤다.

> ⚠️ **`ros2 launch` 는 `--ros-args -p` 를 받지 않는다.** 그건 `ros2 run` 전용이다.
> 파라미터를 바꾸려면 설정 파일을 만들어 `params_file:=` 로 넘긴다:
> ```bash
> cp $(ros2 pkg prefix drawing_cat --share)/config/topic_example.yaml /tmp/my.yaml
> vim /tmp/my.yaml        # scale, pen_lift 등 수정
> ros2 launch drawing_cat draw_cat.launch.py params_file:=/tmp/my.yaml
> ```

> ⚠️ **원격(AnyDesk 등)에서 세션이 끊기면 그 안의 launch 도 같이 죽는다.**
> `./run_container.sh shell` 은 `docker exec -it` 이라 터미널이 닫히면 자식 프로세스가
> 함께 정리된다. 실제로 시뮬레이션이 이렇게 사라져 처음부터 다시 한 적이 있다.
>
> **위 절차대로 해도 아무 문제 없다** — 평범한 셸 4 개로 성공한다. 아래는 원격이 자꾸
> 끊길 때만 고려하면 되는 **선택지**이고, 정규 절차가 아니다.
>
> <details>
> <summary>원격이 불안정할 때 — 오래 도는 것만 분리 실행</summary>
>
> `docker exec -d` 는 TTY 에 매이지 않아 터미널을 닫아도 살아 있다. 도커 기본 기능이라
> 따로 설치할 것은 없다. ① 시뮬레이션 · ② move_group 처럼 오래 띄워둘 것에만 쓴다.
>
> ```bash
> # 호스트에서 — 컨테이너 안으로 들어가지 않는다
> docker exec -d simulation_dev bash -lc \
>   'source /opt/ros/jazzy/setup.bash && source ~/ws_simulation/install/setup.bash && \
>    ros2 launch sim_bringup mujoco_sim.launch.py headless:=true > /tmp/sim.log 2>&1'
>
> docker exec simulation_dev tail -f /tmp/sim.log     # 로그 보기 (Ctrl-C 해도 안 죽음)
> docker exec simulation_dev pkill -f "ros2 launch"   # 끝내기
> ```
>
> ③ 그리기와 ④ 가짜 비전은 짧게 끝나고 로그를 봐야 하므로 이렇게 하지 않는다.
> </details>

> ⚠️ **`hcr_moveit_config` 의 `demo.launch.py` 를 ② 대신 쓰면 안 된다.** 저건
> mock_hardware 단독 데모용이라 몸통까지 자기가 띄운다. 5 개가 시뮬과 정면으로
> 겹친다 — `ros2_control_node`(컨트롤러 매니저 2개) · `joint_state_broadcaster`
> (`/joint_states` 오염) · `hcr_arm_controller` spawner(그 이름의 컨트롤러가 없음) ·
> `robot_state_publisher`(`/tf` 이중 발행) · `static_transform_publisher`
> (`hcr_robot_pen.xacro` 가 이미 `world → base_link` 를 갖고 있음).

### ③ 에 넘기는 설정은 무엇인가

`params_file:=` 로 넘기는 `topic_example.yaml` 의 내용이다.

```yaml
strokes:
  source: "topic"              # 입력을 어디서 받을지. params · csv · topic (5절)
  topic: "/vision/strokes"
  use_latched: false           # 살아 있는 발행분만 받는다 (5-2절)
  frame_timeout_ms: 1500       # 이만큼 조용하면 한 이미지분이 다 왔다고 본다
  wait_timeout_s: 120.0        # 첫 프레임을 기다리는 한도

paper:
  width_mm: 210.0              # ┐ 종이 규격. 픽셀 → mm 변환에 쓰인다
  height_mm: 297.0             # │ A4 에서 여백을 뺀 작화영역 180 × 267 이
  margin_mm: 15.0              # ┘ letterbox 계산의 기준이 된다 (5-2절)
  auto_center: true            # 그림의 경계상자 중심을 원점에 맞춘다
  scale: 0.0004                # mm → m 축척. A4 를 그대로 그릴 수 없으니 줄인다
  pen_lift: 0.008              # 스트로크 사이 펜 드는 높이 (m)

planning:
  min_fraction: 0.9            # 이 값 미만이면 그 스트로크는 실행하지 않는다

marker:
  color_rgba: [0.1, 0.8, 0.2, 1.0]   # RViz 자취 색
  line_width: 0.002                  # RViz 자취 선 굵기 (m)

output:
  map_csv_path: "/tmp/map_from_vision.csv"   # 변환 결과를 남긴다 (7절 검증용)
```

크게 네 덩어리다:

| 그룹 | 정하는 것 |
|---|---|
| `strokes` | 입력을 어디서 어떻게 받을지 |
| `paper` | 종이 규격과 축척 — **F4.1(좌표 변환)이 여기 들어 있다** |
| `planning` · `marker` | 실행 기준과 RViz 표시 |
| `output` | 검증용 `map.csv` 저장 경로 |

값을 바꾸는 방법은 위 "순서" 의 주의사항을, 전체 파라미터 목록과 기본값은 **4 절**을 본다.

### RViz 에서 펜 자취 보기

`mujoco_moveit.launch.py` 가 쓰는 `rviz/draw_cat.rviz` 에 **`PenTrace` 디스플레이가
이미 들어 있다** — Add 를 누를 필요 없다.

> ⚠️ 마커는 volatile QoS 로 스트로크마다 한 번씩만 발행된다. **RViz(②)가
> `draw_cat`(③)보다 먼저 떠 있어야** 자취가 보인다. 순서가 바뀌면 그냥 안 나온다.

> ⚠️ 기본 축척(`scale: 0.00025`)에서 그림은 **1.5cm × 1.2cm** 다. RViz 기본 시점에서는
> 점 하나로 보인다. Views 패널의 `Focal Point` 를 로그에 찍히는 "펜 끝" 좌표로 맞추고
> `Distance` 를 0.05 쯤으로 줄이거나, `scale` 을 키운다(0.001 = 4배, fraction 1.00 유지 확인됨).

> ⚠️ **마커가 안 보이면 `MotionPlanning` 디스플레이를 꺼 보라.** 자취는 펜 끝 위치에
> 그려지는데 거기엔 로봇 팔 자체가 있어서 **로봇 메시에 가려진다.** 실제로 이것 때문에
> "마커가 안 그려진다"고 한참 헤맸다. 끄는 즉시 초록 선이 드러난다.
>
> 그래도 안 보이면 Displays 패널에서 `PenTrace` 를 펼쳐 `Namespaces` 아래
> `cat_strokes` 가 있는지 본다 — 있으면 받은 것이고 화면 문제, 없으면 못 받은 것이다.
> `ros2 topic info /pen_trace --verbose` 의 Subscription count 가 1 이어야 한다.

> ⚠️ **RViz 의 초록 선은 "계획"이지 "실제"가 아니다.** 마커는 입력받은 펜 끝 목표점으로
> 그려지며, MuJoCo 가 그걸 따라갔는지와 무관하게 발행된다. 그래서 **자취가 멀쩡해 보여도
> 실제 로봇은 전혀 다른 곳을 지났을 수 있다** (9절의 joint_1 눌림이 그 경우다).
> 실제 움직임은 MuJoCo 창에서 보거나 `controller_state` 로 재야 한다.

### 종료

> ⚠️ **`ros2 launch` 는 Ctrl-C 로 끝낼 것.** launch 부모 프로세스만 `kill` 하면
> 자식 노드가 남는다. 실제로 점검 중 `robot_state_publisher` 가 4 개 쌓여 있었고,
> 내용이 같아 에러는 안 나지만 `/tf` 이중 발행 상태였다. 의심되면:
> ```bash
> ps -eo pid,etime,args | grep -E "robot_state_publisher|ros2_control_node"
> ```

---

## 3. 파일

| 파일 | 역할 |
|---|---|
| `src/draw_cat.cpp` | 응용 노드. 스트로크를 읽어 Cartesian Path 로 그리고 마커를 발행 |
| `launch/mujoco_moveit.launch.py` | **머리** — `move_group` + `rviz2` 만 띄운다 |
| `launch/draw_cat.launch.py` | 응용 노드만 띄운다 (move_group 이 이미 떠 있어야 함) |
| `config/draw_cat_params.yaml` | 기본 파라미터 (가짜 고양이 5 스트로크) |
| `config/csv_example.yaml` | vision `map.csv`(파일)를 읽는 예시 설정 |
| `config/topic_example.yaml` | vision `/vision/strokes`(토픽)를 받는 예시 설정 |
| `config/moveit_controllers_mujoco.yaml` | MuJoCo 용 컨트롤러 설정 |
| `config/joint_limits_mujoco.yaml` | joint_1 · joint_6 위치 한계 덮어쓰기 (아래 9절) |
| `rviz/draw_cat.rviz` | `PenTrace` 마커가 포함된 RViz 설정 |
| `data/sample_cat_map.csv` | 샘플 외곽선 (96점 폐곡선). `data/gen_sample_map.py` 로 생성 |
| `test_tools/` | ⚠️ **연결 확인 전용.** vision 없이 `/vision/strokes` 경로를 시험한다. 실제 vision 이 붙으면 폴더째 지운다 — `test_tools/README.md` |
| `CHANGELOG.md` | 작업 기록 — 무엇을 왜 바꿨고 무엇으로 확인했는지 |

> ⚠️ **`hanwha_robot_arm/` 은 건드리지 않는다.** 외부에서 가져와 포팅한 공용 자산이고
> mock_hardware 데모가 계속 동작해야 한다. MuJoCo 전용 설정은 전부 이 패키지 안에 두고
> 절대경로로 주입한다 (`MoveItConfigsBuilder` 의 `file_path` 는 절대경로를 받는다).

---

## 4. 파라미터

`config/draw_cat_params.yaml` 을 고치면 **재빌드 없이** 동작이 바뀐다. 다른 파일을
쓰려면 `params_file:=<경로>`.

| 그룹 | 키 | 기본값 | 뜻 |
|---|---|---|---|
| `planning` | `group` / `home_pose` | `hcr_arm` / `hcr_home` | SRDF 의 그룹·자세 이름 |
| | `eef_step` | `0.002` | Cartesian 보간 간격 (m) |
| | `min_fraction` | `0.9` | 이 값 미만이면 그 스트로크를 **실행하지 않는다** |
| | `velocity_scaling` / `acceleration_scaling` | `0.1` | **그리는 구간**의 속도·가속 배율 |
| | `optimize_order` | `true` | F3.1 — NN 으로 스트로크 순서를 다시 정한다 |
| | `rotate_closed_start` | `true` | 폐곡선을 가장 가까운 정점에서 시작하도록 회전 |
| | `travel_mode` | `cartesian` | 펜업 이동 방식. `cartesian`(직선) 또는 `joint`(관절공간) |
| | `travel_velocity_scaling` | `0.5` | 펜업 이동 속도 배율. **지금 축척에서는 효과 없음** (§9) |
| | `travel_acceleration_scaling` | `1.0` | 펜업 이동 가속 배율. **여기가 진짜 손잡이다.** 🔴 실물 이관 전 하드웨어 확인 필요 |
| | `travel_eef_step` | `0.01` | 펜업 이동의 보간 간격 (m) |
| `paper` | `use_current_pose_as_origin` | `true` | 현재 자세를 종이 원점으로 |
| | `origin_x/y/z` | `0.0` | 위가 `false` 일 때의 종이 원점 (F4.1 입력) |
| | `auto_center` | `false` | 좌표 경계상자 중심을 자동으로 원점에 맞춤 |
| | `center_x_mm` / `center_y_mm` | `81.0` / `51.0` | 수동 중심 (`auto_center` 면 무시) |
| | `scale` | `0.00025` | mm → m 축척 |
| | `pen_lift` | `0.008` | 펜업 높이 (m) |
| `pen` | `tip_offset_z` | `0.0` | 플랜지 → 펜 끝 거리 (m), 플랜지 로컬 +z |
| `marker` | `topic` / `ns` / `line_width` / `color_rgba` / `hold_seconds` | | RViz 마커 |
| | `width_mm` / `height_mm` / `margin_mm` | `210` / `297` / `15` | 용지 규격. `source: topic` 의 픽셀→mm 변환에만 쓰임 |
| `strokes` | `source` | `params` | `params` · `csv` · `topic` |
| | `csv_path` | `""` | `source: csv` 일 때 읽을 `map.csv` |
| | `names` / `closed` | | `source: params` 일 때의 스트로크와 폐곡선 여부 |
| | `topic` | `/vision/strokes` | `source: topic` 일 때 구독할 토픽 |
| | `use_latched` | `false` | `true` 면 `TRANSIENT_LOCAL` 로 구독(밀린 이력도 받음) |
| | `frame_timeout_ms` | `1500` | 이 시간 동안 조용하면 한 프레임 완성으로 본다 |
| | `wait_timeout_s` | `30.0` | 첫 프레임 대기 한도 |
| `output` | `map_csv_path` | `""` | 비우면 저장 안 함. 지정하면 변환 결과를 `map.csv` 로 남긴다 (`trace_cli` 검증용) |

> ⚠️ `use_current_pose_as_origin: true` 인 동안에는 **`pen.tip_offset_z` 를 바꿔도
> 결과가 안 바뀐다.** 원점도 펜 끝 기준으로 같이 밀려서 상쇄되기 때문이다. 종이 원점을
> 명시(`false`)할 때부터 의미를 갖는다.

---

## 5. vision 인터페이스 — 경로가 **두 개**다

vision 에서 좌표를 받는 길이 둘 있고, `strokes.source` 로 고른다.

| | `source: "csv"` | `source: "topic"` |
|---|---|---|
| 전달 | 파일 `map.csv` | ROS 2 토픽 `/vision/strokes` |
| 단위 | **mm** (A4 종이 좌표) | **픽셀** |
| 만드는 쪽 | `vision_core/stroke_map_cli` | `contour_pixel_node` (SAM3) |
| 언제 | 오프라인 확인·재현 | 실전 연결 |

**둘 다 결국 mm 로 수렴한다** — `topic` 으로 받아도 `output.map_csv_path` 로
`map.csv` 를 남기면 simulation 의 `trace_cli` 검증(7절)이 그대로 이어진다.

```
/vision/strokes (픽셀) ──▶ draw_cat ──┬─▶ 로봇 좌표로 그리기
                          (F4.1 변환)  └─▶ map.csv (mm) ──▶ trace_cli 검증
```

### 5-1. `map.csv` (파일)

```
stroke_idx,point_idx,x_mm,y_mm,closed
```

헤더 줄은 있어도 없어도 된다. `stroke_idx` 가 바뀌면 새 스트로크다.

```bash
# vision 컨테이너에서 만들기
ros2 run vision_core stroke_map_cli --in ~/data/cat.png --out-dir ~/data/out
# → ~/data/out/map.csv

# moveit2 에서 읽기
ros2 launch drawing_cat draw_cat.launch.py \
  params_file:=$(ros2 pkg prefix drawing_cat --share)/config/csv_example.yaml
```

### 5-2. `/vision/strokes` (토픽)

| 항목 | 값 |
|---|---|
| 메시지 | `vision_interfaces/msg/Stroke` |
| 발행 | **부위 하나 = 메시지 하나.** 5 개 부위면 5 번 연달아 |
| 발행자 QoS | `RELIABLE` · `TRANSIENT_LOCAL` · depth 20 |
| 좌표 | `int32 u, v` — 이미지 **픽셀**. 좌상단 원점, `v` 는 아래로 증가 |
| 폐곡선 | 항상 닫힘. 첫 점이 끝에 중복돼 있지 않다 (받는 쪽이 닫는다) |

```bash
# 셸 3 — 토픽을 기다린다 (최대 120초)
ros2 launch drawing_cat draw_cat.launch.py \
  params_file:=$(ros2 pkg prefix drawing_cat --share)/config/topic_example.yaml

# 셸 4 — "구독 중" 로그가 뜨면. vision 없이 시험할 때는 가짜 발행자를 쓴다
ros2 run drawing_cat fake_vision_publisher          # ⚠️ test_tools — 확인 전용
```

**픽셀 → mm 변환은 임의로 정하지 않는다.** vision 의
`vision_core/src/image_to_map.cpp` `ScaleToPaper` 와 같은 letterbox 식을 쓴다 —
축척 기준이 컨투어 bbox 가 아니라 **이미지 전체 크기**이고, 그 값(`image_width` ·
`image_height`)이 메시지에 실려 온다.

```
scale    = min(180 / image_width, 267 / image_height)     [mm/px]
offset_x = 15 + (180 - image_width  * scale) * 0.5
offset_y = 15 + (267 - image_height * scale) * 0.5
x_mm = offset_x + u * scale        y_mm = offset_y + v * scale
```

180 × 267 은 A4(210×297)에서 여백 15mm 를 뺀 작화영역이다 (vision `params.hpp` 기본값).
세 값 모두 `paper.width_mm` · `height_mm` · `margin_mm` 파라미터로 바꿀 수 있다.

> ⚠️ **어느 이미지의 부위인지 메시지만으로는 알 수 없다.** `Header` 도 프레임 번호도
> 없다. 그래서 받는 쪽이 경계를 판정한다 — ① `instance_label` 이 중복되면 새 이미지
> 시작 ② 마지막 수신 후 `frame_timeout_ms` 동안 조용하면 프레임 확정. 프레임이 여러 개
> 쌓였으면 **가장 최신 것**만 쓰고 경고를 남긴다. 개수(`expected_parts` 같은 것)에
> 의존하지 않는다.

> ⚠️ **구독 durability 선택이 중요하다.** 발행자가 `TRANSIENT_LOCAL` 이어도 구독자는
> `VOLATILE` 로 붙을 수 있다 (durability 는 "구독자가 덜 요구하는" 방향이 호환).
>
> | `strokes.use_latched` | 동작 | 쓸 때 |
> |---|---|---|
> | `false` (기본) | `VOLATILE` — 살아 있는 발행분만 | 섞일 여지 없음. **먼저 띄워두고** vision 을 돌린다 |
> | `true` | `TRANSIENT_LOCAL` — 밀린 이력도 받음 | vision 이 한 번 쏘고 끝나는 운용 |
>
> 실측: 발행자가 미리 5 개를 쏜 뒤 `VOLATILE` 로 구독하면 밀린 이력 0 개, 발행자가 보는
> 구독자 수는 1, 이후 새로 발행한 것은 정상 수신된다.

> ⚠️ **`vision_interfaces` 는 미러다.** vision 원본 정의를 `ws_moveit2/src/vision_interfaces/`
> 에 복제해 두었다. 필드가 하나라도 다르면 타입 해시가 달라져 **토픽은 보이는데 메시지가
> 0 개** 들어온다. 에러가 안 나서 QoS 문제로 오해하기 쉽다.
>
> ```bash
> ros2 topic info /vision/strokes --verbose | grep -i "type hash"
> ```
>
> **기준 해시값은 [`vision_interfaces/README.md`](../vision_interfaces/README.md) 에만 적어
> 둔다** — 여러 곳에 베껴 두면 정의가 바뀌었을 때 어느 값이 맞는지 알 수 없게 된다.

> ⚠️ **이 형식이 vision ↔ moveit2 ↔ simulation 의 유일한 결합점이다.** 코드로 링크하지
> 않고 파일만 주고받는 것이 팀 설계다 (근거: `sim_core/include/atlier/sim/planned_map.hpp`
> 주석). `src/draw_cat.cpp` 의 파싱 규칙은 `sim_core/src/planned_map.cpp` 와 **일부러
> 동일하게** 맞춰 두었다 — 한쪽만 고치면 두 소비자가 어긋난다.

> ⚠️ **폐곡선은 첫 점을 끝에 중복해 넣지 않는다**(vision 규약). 마지막 점에서 첫 점으로
> 돌아오는 구간을 만드는 것은 **소비자 몫**이고, 이 노드가 알아서 한 점을 더 붙인다.
> 좌표에 첫 점을 또 넣으면 그 자리에서 멈췄다 간다.

> ⚠️ **스트로크들 사이에는 순서가 없다**(vision 규약). 그래서 여기 적은 순서에도 아무
> 의미가 없고, 실제 그리는 순서는 **F3.1**(`optimizeStrokeOrder`)이 다시 정한다.
> 주어진 순서를 그대로 쓰려면 `planning.optimize_order: false`. → 9절

> ⚠️ **moveit2 컨테이너에는 `~/data` 마운트가 없다.** `simulation/compose.yml` 에는
> `./data:/home/rosuser/data:rw` 가 있는데 `moveit2/compose.yml` 에는 없다. 마운트를
> 추가하기 전까지는 `docker cp map.csv moveit2_dev:/tmp/map.csv` 로 넣어야 한다.

---

## 6. 펜 끝(pen tip) 값 — 실측하면 **여기만** 바꾼다

> ⚠️ **지금 값은 전부 실측 전 임시값이다.** 실물 펜을 재고 나면 아래 표의 자리만
> 교체하면 되도록 모아 두었다. 코드에는 펜 치수가 박혀 있지 않다.

| 무엇 | 파일 | 위치 | 현재 값 |
|---|---|---|---|
| 펜 전체 길이 | `src/simulation/ws_simulation/src/sim_bringup/urdf/hcr_robot_pen.xacro` | `pen_length` (50행) | `0.150` m |
| 펜 반지름 | 〃 | `pen_radius` (51행) | `0.004` m |
| 펜 질량 | 〃 | `pen_mass` (52행) | `0.010` kg |
| 플랜지 장착 위치 | 〃 | `<joint name="pen_joint">` origin (75행~) | `xyz="0 0 0"` — `link6_1` 직결 |
| 펜 끝 위치 | 〃 | `<joint name="pen_tip_joint">` origin (83행~) | `xyz="0 0 ${pen_length}"` |
| **MoveIt 쪽 TCP 오프셋** | `drawing_cat/config/draw_cat_params.yaml` | `pen.tip_offset_z` | `0.0` m (펜 무시) |

**파생물 — 직접 고치지 말 것**

| 무엇 | 어디서 나오나 |
|---|---|
| `src/simulation/data/mjcf/*` | `hcr_robot_pen.xacro` 에서 변환된 것. `pen_length` 를 바꾸면 **반드시 재변환**해야 MuJoCo 에 반영된다 |

### 실측값이 생겼을 때의 교체 절차

```bash
# 1. 물리 치수 — 진실의 출처는 여기 하나뿐이다
vim src/simulation/ws_simulation/src/sim_bringup/urdf/hcr_robot_pen.xacro   # pen_length

# 2. MJCF 재변환 (simulation 컨테이너)
S=$(ros2 pkg prefix sim_bringup --share)
xacro $S/urdf/hcr_robot_pen.xacro > /tmp/hcr_pen.urdf
/opt/ros/jazzy/share/mujoco_ros2_control/scripts/robot_description_to_mjcf.sh \
  -u /tmp/hcr_pen.urdf -m $S/mjcf/mujoco_inputs.xml --scene $S/mjcf/scene.xml \
  -o ~/data/mjcf -s -c --no-fuse

# 3. MoveIt 쪽 TCP 오프셋을 같은 값으로
vim ws_moveit2/src/drawing_cat/config/draw_cat_params.yaml   # pen.tip_offset_z
```

> ⚠️ **`pen_length` 와 `pen.tip_offset_z` 는 같은 값이어야 한다.** 지금은 서로 다른
> 파일에 있고 어긋나도 아무 경고가 없다 — 조용히 그림이 틀어질 뿐이다. 한쪽만
> 고치지 말 것. (MoveIt 쪽 URDF 에 펜이 들어가면 이 이중화는 사라진다. 9절 참조)

> ⚠️ `paper.use_current_pose_as_origin: true` 인 동안에는 **`tip_offset_z` 를 바꿔도
> 결과가 안 바뀐다.** 원점도 펜 끝 기준으로 같이 밀려 상쇄되기 때문이다. 종이 원점을
> 명시(`false`)할 때부터 값이 살아난다.

---

## 7. 계획 대비 실제 확인 — `trace_cli`

"계획한 대로 그어졌는가"는 시뮬레이션팀이 만든 `trace_cli` 로 본다. **직접 만들지
않는다** — 계획 지도(`map.csv`)와 접촉 궤적(`trace.csv`)을 한 장에 겹쳐 SVG 로
뽑아 주는 도구가 이미 있다.

```bash
# moveit2 쪽에서 만든 계획 지도를 simulation 의 data 로 옮긴다
cp ws_moveit2/src/drawing_cat/data/sample_cat_map.csv src/simulation/data/map.csv

# simulation 컨테이너에서
ros2 run sim_core trace_cli --dummy --case ideal \
  --plan /home/rosuser/data/map.csv --out-dir /home/rosuser/data/out
# → data/out/{drawn.svg, trace.csv, summary.txt}
```

`drawn.svg` 는 A4 실제 비율이라 브라우저에서 열거나 인쇄해 자로 잴 수 있다. 계획은
회색, 실제는 색으로 겹쳐 그려진다.

**파이프라인 확인 결과 (실측)**

| 시나리오 | 결과 |
|---|---|
| `ideal` | 계획 1 스트로크 → 실제 1 조각, 길이 304.9 → 303.6 mm (**99.6 %**) |
| `weak-force` | 계획 1 → 실제 **2 조각**, 길이 82.7 % — 선 끊김 1 회를 잡아낸다 (BRD R2) |

계획 길이 304.9 mm 는 **폐곡선의 닫는 구간까지 포함한 값**이다. 즉 `draw_cat` 이 쓴
`closed` 플래그가 CSV 를 건너 그쪽 도구까지 제대로 전달됐다는 교차 검증이 된다.
바운딩박스도 원본 `map.csv` 와 일치한다.

> ⚠️ **아직 `--dummy` 로만 돌 수 있다.** 실제 접촉 궤적(`trace.csv`)은 펜이 종이에
> 닿아야 나오는데, **펜이 MoveIt 의 TCP 가 아니라서**(9절) 실제 접촉이 발생하지
> 않는다. 지금 확인된 것은 "계획 지도가 도구까지 흘러가고 산출물이 나온다"는
> 파이프라인이지, 그림의 실제 품질이 아니다.

---

## 8. 확인된 동작

| 항목 | 결과 |
|---|---|
| 컨테이너 간 DDS | ✅ `moveit2_dev` 에서 시뮬 노드 6 개 관측 (그쪽 프로세스는 0 개) |
| `/joint_states` | ✅ **500 Hz**, σ=0.34ms, 발행자 1 개 |
| 액션 연결 | ✅ client `/moveit_simple_controller_manager` ↔ server `/joint_trajectory_controller` |
| 가짜 고양이 (5 스트로크) | ✅ fraction 1.00 × 5, 실행 6/6 SUCCEEDED |
| `map.csv` 폐곡선 (96 점) | ✅ fraction 1.00, 실행 성공, 관절 실제 이동 |
| **`/vision/strokes` 토픽 수신** | ✅ 부위 5 개(cat·눈2·코·입), 이미지 1280×960 → letterbox 0.14062 mm/px · 오프셋 (15.0, 81.0) — 손계산과 일치 |
| **토픽 → map.csv → `trace_cli`** | ✅ 계획 5 개 → 실제 조각 5 개, 길이 98.7 %, 바운딩박스 일치. **파일 검증 경로가 끊기지 않는다** |
| QoS durability 동작 | ✅ 발행자 `TRANSIENT_LOCAL` + 구독자 `VOLATILE` 호환 확인 (밀린 이력 0, 새 발행분은 수신) |
| **전 구간 수동 실증** (2026-08-14) | ✅ 셸 4개로 직접 기동 → 부위 5개 수신 → 5/5 실행. `scale` 0.0004·0.001 양쪽 fraction 1.00 |
| RViz 마커 렌더링 | ✅ 그려진다. 단 **`MotionPlanning` 을 꺼야 보인다**(로봇 메시에 가림) |
| **joint_1 · joint_6 눌림** | ❌ **실측 확인.** 지령 −7.07° vs 실제 −0.03°, 펜 끝 56.7 mm 편차 (9절) |
| `trace_cli` 파이프라인 | ✅ `map.csv` → `drawn.svg`·`summary.txt` (7절). `closed` 플래그가 계획 길이에 반영되는 것까지 교차 검증 |
| 추종 오차 — 정지 시 | ✅ joint_2 **0.16°** — 자세 유지는 문제없다 |
| 추종 오차 — 그리는 중 | ⚠️ joint_2 **최대 5.33°**, 평균 2.76° — 아래 참조 |
| `auto_center` | ✅ 경계상자 중심 자동 계산 (105.0, 143.4 mm) |
| `min_fraction` 가드 | ✅ 1.01 로 올리니 fraction 1.00 을 차단 |
| 기동 순서 뒤집기 | ✅ move_group 먼저 띄워도 시뮬 기동 후 재시작 없이 복구 |

Cartesian Path 계산 시간은 웨이포인트 5~100 개에서 **12~50ms** 수준이다 — 스트로크가
늘어도 계산 자체는 병목이 아니다.

---

## 9. 알려진 문제 · 미해결

### joint_1 · joint_6 의 한계가 비대칭이다 — 지금은 우회 중

`hcr_robot.xacro` 에서 이 둘의 한계는 `[0, 6.283185]` 다. 나머지 네 관절은 전부 ±대칭인데
이 둘만 하한이 0 이고, 하필 `hcr_home` 이 그 둘을 **정확히 0** — 하한 위에 둔다.

mock_hardware 는 정확히 `0.0` 을 돌려주므로 아무 일도 없다. 그러나 MuJoCo 는 실제 물리를
풀어서 `-1.06e-14` 같은 노이즈를 돌려주고, 그 순간 시작 상태가 한계 밖이 되어 플래닝이
통째로 실패한다.

**시도했으나 안 되는 것들 (반복하지 말 것)**

| 시도 | 결과 |
|---|---|
| `ompl.fix_start_state: true` | ❌ 그 파라미터는 **continuous · planar · floating** 관절의 회전값 정규화만 한다(플러그인 설명 원문). `joint_1` 은 한계가 있는 revolute 라 대상이 아니다 |
| `request_adapters` 에서 `CheckStartStateBounds` 제거 | ❌ 실패가 한 단계 뒤로 밀릴 뿐. OMPL 상태공간도 같은 URDF 한계로 만들어져 `START_STATE_INVALID` |

현재는 `config/joint_limits_mujoco.yaml` 로 `robot_description_planning.joint_limits.
<joint>.min_position` 을 덮어써서 우회한다.

> ⚠️ **이건 플래닝만 푼다.** MJCF 도 같은 URDF 에서 변환되어 `range="0 6.28318"` 을 그대로
> 갖는다. 제대로 된 해결은 `hcr_robot.xacro` 수정 + MJCF 재변환이고, **시뮬레이션팀과 함께
> 정할 사안이다.**

#### 실측 — 눌림이 실제로 일어난다 (2026-08-14)

`paper.scale: 0.001`(그림 84 × 93 mm)로 그리면서
`/joint_trajectory_controller/controller_state` 를 샘플링한 결과다.

| 관절 | 지령 범위 | 실제 범위 | 오차 | 판정 |
|---|---|---|---|---|
| **joint_1** | −7.07° ~ +7.61° | **−0.03°** ~ +6.82° | −8.35° | **한계에 눌림** |
| **joint_6** | −0.15° ~ +0.17° | **−0.00°** ~ +0.17° | −0.15° | **한계에 눌림** |
| joint_2 | −8.74° ~ +6.94° | −9.14° ~ +3.63° | **+14.96°** | 한계 무관 — 추종 지연 |
| joint_3 | 80.0° ~ 96.5° | 80.2° ~ 96.6° | −4.12° | 추종 지연 |
| joint_4 | −99.7° ~ −91.1° | −99.8° ~ −91.1° | −1.52° | 정상 |
| joint_5 | −91.3° ~ −76.6° | −91.3° ~ −76.6° | −0.92° | 정상 |

MoveIt 이 joint_1 을 **−7.07° 까지 지시했는데 MuJoCo 는 −0.03° 에서 멈췄다.**
펜 끝에서의 편차로 환산하면:

```
7.04° × 반경 0.461 m = 56.7 mm      (그림 전체 폭 84.4 mm 의 67 %)
```

즉 **MuJoCo 가 그린 것은 고양이 모양이 아니라 한쪽이 뭉개진 형태다.**

> ⚠️ **`fraction: 1.00` 이어도 안심할 수 없다.** 위 실행에서 다섯 스트로크 모두
> `fraction 1.00` 이었고 실행도 전부 `SUCCEEDED` 였다. `joint_limits_mujoco.yaml` 이
> MoveIt 쪽 한계를 넓혀 두었기 때문에 **MoveIt 은 MuJoCo 가 갈 수 없는 영역까지 계획하고도
> 성공으로 보고한다.** 로그만 봐서는 알 수 없고, 위처럼 `controller_state` 를 봐야 한다.

> ⚠️ **`joint_6` 도 같은 문제다.** 지금까지 joint_1 만 언급했지만 두 관절의 한계가 같으므로
> 증상도 같다. 이번엔 필요 범위가 작아 편차가 작았을 뿐이다.

작은 그림(`scale: 0.0004`, 34 × 37 mm)에서도 필요 범위가 ±2.8° 였으므로 **이미 눌리고
있었다.** 그때 측정된 joint_1 오차 3.8~4.0° 가 그 흔적이다 — 당시엔 동적 추종 지연으로
해석했으나, 상당 부분이 눌림이었다.

### 펜이 아직 MoveIt 의 TCP 가 아니다

`move_group` 은 펜이 없는 `hcr_robot.urdf.xacro` 를, MuJoCo 는 펜이 붙은
`hcr_robot_pen.xacro` 를 쓴다. 따라서 펜은 **MoveIt 충돌 검사에 들어가지 않고**,
`pen_tip` 프레임도 MoveIt 쪽에 없다. F4.1 을 제대로 풀려면 이 결정을 다시 봐야 한다
(`pen.tip_offset_z` 는 그때를 위한 자리다).

### 실행 감시를 풀어 둔 상태다

`config/moveit_controllers_mujoco.yaml` 에서 `allowed_start_tolerance: 0.0`,
`execution_duration_monitoring: false` 다. MuJoCo 의 궤적 추종이 아직 성립하지 않아
(관성 큰 관절이 목표를 못 따라감 — `src/simulation/README.md` "구현 상태") 실패 원인이
"연동"인지 "추종"인지 섞이는 것을 막기 위한 **임시 설정**이다. 추종이 개선되면 원래
값(`0.01` / `true`)으로 되돌려 실제로 만족하는지 확인할 것.

### MuJoCo 가 계획한 궤적을 따라가지 못한다

연결은 성립하는데(8절) **보이는 것이 계획과 다르다.** 측정값:

| 구간 | 최대 오차 | 평균 | joint_2 최대 |
|---|---|---|---|
| 정지 (자세 유지) | 0.17° | — | **0.16°** |
| 홈 이동 (관절공간) | 2.57° | 1.26° | 0.39° |
| **고양이 외곽선 (Cartesian)** | **5.33°** | 2.76° | **5.33°** |

정지 시 0.16° → 그리는 중 5.33°. 중력 처짐이 아니라 **동적 추종 지연**이고, 하필
**그리는 동작에서 가장 심하다** — `src/simulation/README.md` "구현 상태"의 서술과 일치한다.

**그림을 키우면 더 나빠진다.** `scale` 을 0.0004 → 0.001 로 올리자 joint_2 오차가
**5.33° → 14.96°** 로 세 배 가까이 커졌다 (지령 +6.94° 인데 실제는 +3.63° 에서 멈춤).
joint_2 는 가동범위가 ±90° 라 한계와 무관한 순수 추종 문제다.

> ⚠️ 위 표의 joint_1 수치는 **상당 부분이 추종 지연이 아니라 관절 한계 눌림이었다** —
> 아래 "joint_1 · joint_6 의 한계" 절의 실측 참조. 순수한 추종 문제로 볼 것은
> **joint_2 · joint_3** 이다.

> ⚠️ **렌더링 부하 탓이 아니다.** 세 조건에서 재 봤는데 추종 오차는 5.3~5.9° 로
> 같은 자리에 있다 — 헤드리스 5.330° · 소프트웨어 렌더링 5.392° · GPU 5.912°.
> 렌더링은 **별개의 실재하는 문제**이지 추종 실패의 원인이 아니다.
> 측정은 `/joint_trajectory_controller/controller_state` 의 `error.positions` 를
> 실행 중 샘플링한 것이다.

### MuJoCo 뷰어는 제어 루프를 갉아먹는다 — GPU 로도 해결 안 된다

`headless:=false` 로 뷰어를 띄우면 500Hz 제어 루프가 주기를 놓친다. **뷰어를 띄운다는
사실 자체가 원인이고, GPU 를 붙여도 횟수는 그대로다.**

| | 30초당 오버런 중앙값 | Total time 중앙값 | loop 최대 |
|---|---|---|---|
| 헤드리스 | **0.0 회** | — | — |
| GUI + 소프트웨어 렌더링 | 29.0 회 | 2834 us | 20.24 ms |
| GUI + GPU | **29.0 회** | **2407 us** | **10.51 ms** |

GPU 가 바꾸는 것은 **횟수가 아니라 심각도**다 — 최악 루프 시간이 절반(20.24 → 10.51 ms),
Update 중앙값이 2522 → 2051 us 로 예산(2000 us) 경계까지 내려온다.

따라서 **확인을 SVG(7절)로 한다면 헤드리스가 맞다.** 뷰어는 눈으로 볼 때만 켠다.

이 PC 에는 `src/simulation/compose.override.yml` 을 만들어 `/dev/dri` 를 넘겨 두었다
(없으면 `failed to load driver: iris` 로 llvmpipe 소프트웨어 렌더링으로 떨어진다).
`.gitignore` 대상인 PC 로컬 자산이고, **새 컨테이너를 만들지 않는다** — 기존
`simulation` 서비스에 `devices`·`group_add` 만 병합되는 오버레이다
(확인: `./run_container.sh config` → 서비스 1 개).

펜 끝에서 몇 mm 로 나타나는지는 아직 못 쟀다 — TF 폴링으로 시도했으나 표본이
부족했다. 제대로 재려면 7절의 `trace_cli` 경로가 서야 하고, 그러려면 펜이 MoveIt 의
TCP 가 되어야 한다.

### F3.1(순서 최적화)

`optimizeStrokeOrder()` — **`include/drawing_cat/optimizer.hpp`** 에 있다. 이 헤더는
**ROS·MoveIt 에 의존하지 않는다**(의도된 것). 순수 기하 계산만 담아서 로봇 없이 시험이
돌아간다:

```bash
colcon test --packages-select drawing_cat      # test/test_optimizer.cpp, 5 케이스
colcon test-result --verbose                   # 실패 내용
```

> 이 시험은 "얼마나 빨라지나"가 아니라 **"의도대로 동작하나"** 를 본다 — NN 순서, 방향
> 반전, 폐곡선 회전, 회전·반전 뒤에도 좌표를 잃거나 만들지 않는지, 이탈점 규칙이
> `draw_cat.cpp` 의 `end_x/end_y` 와 같은지. 시간 측정은 로봇이 있어야 하므로
> [`docs/Trajectory Optimization.md`](../../../docs/Trajectory%20Optimization.md) 로 간다.

**Nearest Neighbor** 로 매번 "펜에서 가장 가까운 아직 안 그린 스트로크"를 고른다. 줄이는 것은 **펜업 이동 거리** — 펜을 든 채 다음
스트로크로 날아가는 구간이고, 그림에 흔적을 남기지 않으므로 짧을수록 순수하게 이득이다.

진입점을 고르는 방법이 스트로크 종류마다 다르다:

| | 진입 선택지 | 처리 |
|---|---|---|
| 열린 스트로크 (눈·입 같은 선) | 양 끝점 **2 개** | 가까운 쪽으로 들어가려면 좌표를 뒤집는다 (방향 반전) |
| 닫힌 스트로크 (윤곽선) | **모든 정점** | 가장 가까운 정점이 맨 앞에 오도록 배열을 회전 (`rotate_closed_start`) |

닫힌 곡선에 선택지가 많은 이유는, **폐곡선에는 "첫 점"이라는 게 원래 없기 때문**이다.
어느 정점에서 출발하든 한 바퀴 돌아 제자리로 오므로 결과가 같다. `flat[0]` 은 비전의
컨투어 추적기가 우연히 시작한 점일 뿐이라 우리에게 유리할 이유가 없다.

> ⚠️ **vision 토픽 경로는 전부 `closed=true`** 라(`draw_cat.cpp` 의 `s.closed = true`),
> 실전에서 절감폭의 대부분은 **회전**에서 나온다. 방향 반전은 `params`·`csv` 로 열린
> 스트로크가 들어올 때만 동작한다.

### ⚠️ 실측 결과 — 거리는 줄었지만 시간은 거의 안 줄었다

2026-08-14 에 A/B/C/D 네 조건으로 쟀다 (상세: [`docs/Trajectory Optimization.md`](../../../docs/Trajectory%20Optimization.md)).

| 조건 | n | 펜업 거리 | 합계 시간 | 기준 대비 |
|---|---|---|---|---|
| 최적화 끔 | 1 | 136.8 종이mm | 3.65 s | — |
| + NN 순서 | 1 | 101.2 (−26.1 %) | 3.66 s | **0 — 노이즈 이하** |
| + 폐곡선 회전 | 3 | 80.5 (−41.2 %) | 3.590 s | −1.6 % |
| + 이동 속도 0.5 | 3 | 80.5 | 3.413 s | **−6.5 %** |

> ⚠️ **`travel_velocity_scaling` 은 지금 축척에서 아무 효과가 없다.** 이동 하나가 실제
> 4.5 ~ 24 mm 라 최고 속도에 도달할 일이 없다 — 0.1 → 0.5 로 올려도 펜업 시간이
> 0.51 → 0.51 로 그대로였다. **진짜 손잡이는 `travel_acceleration_scaling`** 이고,
> 시뮬에서 1.0 이 최선이었다 (합계 3.340 s = 베이스라인 대비 **−8.5 %**). 상세는 §4.3.
>
> 🔴 **그래서 기본값을 가속 1.0 으로 올렸다.** 실물로 옮기기 전에 하드웨어 확인이 필요하다 —
> 걱정은 속도가 아니라 **도착 시 급감속**이고, 펜이 테이프 고정이라 컴플라이언스가 없다.
> 다만 이 값은 펜업 구간에만 걸려서 급감속 순간 펜은 종이에서 8 mm 떠 있고, 되돌리는 것도
> 파라미터 한 줄이다. 물어볼 것은
> [`Pipeline Integration Status.md`](../../../docs/Pipeline%20Integration%20Status.md) §5 "하드웨어".

**펜업 시간은 펜업 거리에 비례하지 않는다.** 이동 하나가 실제로는 8 ~ 14 mm 라
가감속 램프와 궤적당 고정 오버헤드가 지배한다 — 펜업 시간의 약 71 % 가 거리와 무관하다.
그래서 **`travel_velocity_scaling` 상향 하나가 순서 최적화 전체보다 크게 기여했다.**

베이스라인에서 펜업은 전체의 **15.8 %** 였고(= 이 작업의 상한), 그중 38 % 를 회수했다.
나머지 84.2 % 는 그리기 시간이라 순서 최적화가 손댈 수 없다.

효과를 재는 법과 실측 결과는 **[`docs/Trajectory Optimization.md`](../../../docs/Trajectory%20Optimization.md)** 에 따로 모았다.

> ⚠️ **`ros2 run` 으로 A/B 를 돌리면 안 된다.** 런치 파일이 `robot_description`·SRDF·
> kinematics 를 넘겨주는데 `ros2 run` 에는 그게 없어서 `No kinematics plugins defined`
> 가 뜨고 fraction 이 전부 0 이 된다. 파라미터 파일을 복사해 고친 뒤 `params_file:=` 로 넘긴다.

> ⚠️ 시작점은 `(center_x_mm, center_y_mm)` 이다 — `flangePose()` 에서 `mx_mm == center_x_mm`
> 이면 home 플랜지 위치로 환원되기 때문에, **종이 좌표계에서 home 이 놓인 자리**가 곧
> 그 값이다. 로봇 실제 자세를 몰라도 mm 공간에서 순서를 정할 수 있는 근거다
> (mm ↔ world 가 아핀 변환이라 거리의 대소가 보존된다).
>
> 단 이 등식은 `paper.use_current_pose_as_origin: true` 일 때만 성립한다. `false` 면
> **첫 스트로크 선택 하나만** 틀어진다 (이후 홉은 스트로크 끝점끼리라 무관).

### 펜업 이동을 그리기에서 분리

예전에는 스트로크마다 `computeCartesianPath` 를 **한 번** 부르면서 "날아가기 + 그리기"가
한 호출에 묶여 있었다. MoveIt 은 `waypoints[0]` 으로 가는 구간도 알아서 만들어 주는데,
같은 호출이라 **날아가는 구간까지 그리기와 똑같은 규칙**을 받았다 — 속도 10 %, 공간상
직선 강제, 2 mm 마다 IK.

붓글씨로 치면 획을 긋는 손은 느리고 정확해야 하지만 다음 획으로 손을 옮기는 동작은
아무렇게나 빨라도 된다. 지금은 호출을 둘로 쪼갰다:

| | 배율 | 보간 | 근거 |
|---|---|---|---|
| 펜업 이동 | `travel_velocity_scaling` (0.5) | `travel_eef_step` (10 mm) | 종이에 안 닿아 그림에 영향 없음 |
| 그리기 | `velocity_scaling` (0.1) | `eef_step` (2 mm) | 정확도가 필요 |

`travel_mode: "joint"` 로 두면 관절공간 계획을 쓴다 — 직선 제약이 없어 더 짧게 갈 수도
있지만, **OMPL RRT 는 샘플링 기반이라 짧은 경로를 보장하지 않는다.** 돌아가는 경로가
나오면 배율을 올린 이득이 상쇄되므로 기본값은 `"cartesian"`(직선)이다.

> 펜업 이동이 실패해도 그리기가 깨지지 않는다 — `waypoints[0]` 이 접근점 그대로라
> 그리기 호출이 알아서 데려간다 (성공했으면 길이 0 구간이 되는 것뿐이다).

### F3.1 에서 아직 안 한 것

- **NN 은 근사다.** 스트로크가 적으면(≤ 8 개) 전수탐색이, ~15 개까지는 Held-Karp DP 가
  **정확한 최적해**를 즉시 준다. 2-opt 후처리도 싸다. 지금 개수에서는 NN 이 이미
  최적해를 낼 가능성이 높아 **개선 0 % 가 나와도 알고리즘이 틀린 게 아니다.**
- **스트로크 내부는 안 건드린다.** 점 개수를 줄이거나(다글라스-포이커) 곡선으로 잇는
  것은 범위 밖이고, 그림 모양이 바뀌므로 비전과 합의가 필요하다.

---

## 10. `ROS_DOMAIN_ID` — 조용히 실패하는 곳

네 모듈 compose.yml 이 전부 `${ROS_DOMAIN_ID:-0}` 이고 `.env` 파일이 없다. 그런데
`run_container.sh` 는 `HOST_UID` · `HOST_GID` · `VIDEO_GID` · `RENDER_GID` ·
`XAUTHORITY` 를 전부 export 하면서 **`ROS_DOMAIN_ID` 만 빠뜨렸다.**

- 두 컨테이너가 다른 도메인이면 → **에러 없이** 그냥 서로 안 보인다
- 도메인 0 이면 → 같은 LAN 의 남의 ROS 그래프에 합류한다. 이름이 겹치면
  `/joint_states` 가 조용히 빈 배열이 된다 (`mujoco_sim.launch.py` 독스트링 경고)

그래서 **컨테이너를 띄우기 전에 반드시 export** 해야 한다. 도메인을 바꿀 때는
`down → export → up` 순서다 (`up` 이후 export 는 이미 뜬 컨테이너에 반영되지 않는다).

```bash
./run_container.sh down
export ROS_DOMAIN_ID=42
./run_container.sh up
```

확인:

```bash
docker exec moveit2_dev    bash -c 'echo $ROS_DOMAIN_ID'
docker exec simulation_dev bash -c 'echo $ROS_DOMAIN_ID'
```

> 제안: 다른 변수와 같은 방식으로 `run_container.sh` 의 export 블록에 한 줄 넣으면
> 이 실패 자체가 사라진다 — `export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"`.
> 네 모듈 모두에 넣어야 하므로 팀 합의가 필요하다.

### 실제로 당한 사례 (2026-08-14) — 증상이 컨트롤러 문제로 보인다

컨테이너를 `down` 으로 지운 뒤 **`export` 없이** `./run_container.sh shell` 로 다시 만들었다.
컨테이너는 기본값 **0** 으로 생성됐고, 마침 같은 PC 에 도메인 0 으로 도는
다른 컨테이너(`demo.launch.py` = mock hardware)가 있었다.

```
moveit2_dev        ROS_DOMAIN_ID=0     ← 새로 만들어진 것
simulation_dev     ROS_DOMAIN_ID=42
markch_moveit2_dev ROS_DOMAIN_ID=0     ← mock hardware
```

결과는 **에러 없이 엉뚱한 곳에 붙는 것**이었다:

```
Planning request complete!          ← 계획은 성공 (mock 쪽 /joint_states 로)
Execute request accepted
Execute request aborted             ← 1.5 ms 만에 즉시 거부
'hcr_home' 실행 실패
```

`moveit_controllers_mujoco.yaml` 은 `joint_trajectory_controller` 를 기대하는데 발견된 것은
mock 의 `hcr_arm_controller` 였다. **계획이 성공하기 때문에 도메인 문제로 의심하기 어렵다.**

**5 초 만에 판별하는 법** — 셸 2 에서 move_group 을 띄우기 **전에**:

```bash
ros2 control list_controllers
```

| 나오는 것 | 뜻 |
|---|---|
| `joint_trajectory_controller` | ✅ MuJoCo 에 제대로 붙었다 |
| `hcr_arm_controller` | ❌ mock hardware 에 붙었다 — 도메인 확인할 것 |
| 서비스 응답 없음 | 시뮬레이션이 안 떠 있다 |

컨테이너를 지운 뒤 다시 만들 때가 유일한 위험 구간이다. **이미 떠 있는 컨테이너에 셸을
더 여는 것은 안전하다** — `docker exec` 는 컨테이너의 환경변수를 물려받는다.

---

## 11. 트러블슈팅

| 증상 | 원인 | 확인 |
|---|---|---|
| `Failed to fetch current robot state`, 기준 위치 `(0,0,0)`, fraction 전부 `0.00` | ① 시뮬이 안 떠 있음 ② 노드가 별도 스레드에서 spin 되지 않음 | 먼저 `ros2 topic hz /joint_states` — 발행자가 없으면 ① |
| `Start state out of bounds` | joint_1 한계 문제 (9절) | `joint_limits_mujoco.yaml` 이 로드됐는지: `ros2 param get /move_group robot_description_planning.joint_limits.joint_1.min_position` |
| 컨트롤러를 못 찾음 | 컨트롤러 이름 불일치 | `ros2 control list_controllers` — `joint_trajectory_controller` 여야 한다 |
| RViz 에 자취가 안 보임 | ① RViz 가 `draw_cat` 보다 늦게 뜸 ② 그림이 너무 작음 | `ros2 topic info /pen_trace` 로 Subscription count 확인 |
| 노드가 서로 안 보임 | `ROS_DOMAIN_ID` 불일치 (10절) | 양쪽 컨테이너에서 `echo $ROS_DOMAIN_ID` |
| `/tf` 가 튐 | `robot_state_publisher` 중복 | `ps -eo pid,etime,args \| grep robot_state_publisher` |
| `unknown goal response, ignoring...` → `execute() failed` | **`move_group` 이 2 개.** 액션 클라이언트가 둘이라 goal 응답이 엉뚱한 쪽으로 간다. 이전 launch 를 안 끄고 새로 띄우면 이렇게 된다 | `ros2 node list \| grep -c "^/move_group$"` → **1 이어야 한다.** `ros2 node list` 가 "nodes in the graph that share an exact name" 경고를 내면 확정이다 |
| 계획은 되는데 **`Execute request aborted`** (1~2 ms 만에 즉시) | **엉뚱한 컨트롤러 매니저에 붙었다.** 도메인이 어긋나 다른 컨테이너의 mock hardware 를 발견한 경우다. 계획은 `/joint_states` 만 있으면 되므로 성공해서 더 헷갈린다 | `ros2 control list_controllers` → **`joint_trajectory_controller`** 여야 한다. `hcr_arm_controller` 가 나오면 mock 쪽에 붙은 것 (10절) |
| RViz 에 자취가 아예 안 보임 | 로봇 메시에 가려짐 | `MotionPlanning` 디스플레이를 끈다 (2절) |
| 자취는 멀쩡한데 실제가 이상함 | **마커는 계획이지 실제가 아니다** | MuJoCo 창을 보거나 `controller_state` 로 지령 대 실제를 잰다 (9절) |
