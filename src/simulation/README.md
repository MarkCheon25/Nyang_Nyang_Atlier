# simulation

**MuJoCo 접촉 검증 스켈레톤이 서 있는 상태.** 확인 하네스는 완성됐고, MJCF 변환까지
실제로 통과했다. 시뮬 실구동(컨트롤러 스폰 → 궤적 실행)은 아직이다.

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
필요하다 — 스니펫은 `src/moveit2/README.md` §2와 같되 서비스 이름을 `simulation` 으로 바꾼다.

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

### 시뮬 기동

**MJCF 변환을 먼저 해야 한다** (launch 는 변환하지 않는다):

```bash
S=$(ros2 pkg prefix sim_bringup --share)
xacro $S/urdf/hcr_robot_pen.xacro > /tmp/hcr_pen.urdf
/opt/ros/jazzy/share/mujoco_ros2_control/scripts/robot_description_to_mjcf.sh \
  -u /tmp/hcr_pen.urdf -m $S/mjcf/mujoco_inputs.xml --scene $S/mjcf/scene.xml \
  -o ~/data/mjcf -s -c --no-fuse

ros2 launch sim_bringup mujoco_sim.launch.py            # headless:=true 로 창 없이
```

> ⚠️ **`--no-fuse` 와 `-m` 은 선택이 아니다.** 없으면 각각 펜이 병합돼 사라지고
> actuator 가 0 개로 나온다 (아래 "함정 3건" 참조).

컨트롤러 스폰·궤적 실행은 아직 확인하지 않았다 — "구현 상태" 참조.

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
| 컨트롤러 스폰 · 궤적 실행 | ⬜ 미확인 |
| 궤적 기록 노드 (MuJoCo → trace.csv) | ⬜ 미착수 |

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
- **접촉·강성 파라미터 튜닝 — 우선순위 높음** — `solref`/`solimp`/`friction` 과
  `mujoco_inputs.xml` 의 `kp`·`dampratio` 가 전부 출발점 값이다. **4 mm 침투에 408 N**
  이 나오는 현재 설정으로는 필압 실험 자체가 성립하지 않는다. 실물 연필 필압
  1~5 N 대를 재현하는 것이 다음 목표
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
