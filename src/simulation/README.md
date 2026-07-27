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

### 시뮬 기동 (미완)

```bash
ros2 launch sim_bringup mujoco_sim.launch.py
```

아래 "구현 상태" 참조 — 아직 실동작을 확인하지 않았다.

---

## 구현 상태

| 항목 | 상태 |
|---|---|
| 접촉 궤적 타입·통계 (`sim_core`) | ✅ 구현됨 |
| 확인 하네스 (`trace_cli` · SVG·CSV·요약) | ✅ 구현됨 — 시나리오 3종 관통 |
| vision 계획 지도 읽기 (`map.csv`) | ✅ 구현됨 |
| 펜 래퍼 URDF (`hcr_robot_pen.xacro`) | ✅ 파싱 통과 — 링크 10, MuJoCo 플러그인 단독 |
| **URDF → MJCF 변환** | ✅ **통과** — MuJoCo 가 로드함 (body 7 · joint 6 · geom 8) |
| MJCF 씬 (책상·종이·지그) | 🟨 작성됨, 변환 결과와 합쳐 로드하는 것은 미확인 |
| 컨트롤러 스폰 · 궤적 실행 | ⬜ 미확인 |
| **필압 측정 경로** | ⬜ **미정 — 아래 미결** |
| 궤적 기록 노드 (MuJoCo → trace.csv) | ⬜ 미착수 |

**검증된 것 (2026-07-28)**
- `ros-jazzy-mujoco-ros2-control` 0.0.3 · `mujoco-vendor` 3.4.0 apt 설치 확인
- HCR-5 URDF **8개 링크 전부 inertial 보유** — MuJoCo 변환의 주요 장애물 없음
- 펜 래퍼 xacro 파싱: 링크 10개(world·base·link1~6·pen_link·pen_tip), ros2_control 플러그인 **MuJoCo 단독**
- STL → OBJ 변환 + MJCF 생성 후 `mujoco.MjModel.from_xml_path` 로드 성공
- 하네스 3시나리오: `ideal` 조각 3/계획 3 · `weak-force` **조각 10 → 끊김 7회 검출** · `drift` 바운딩박스 이동 확인

**변환에서 잡힌 함정 3건 (다음 세션의 출발점)**

1. **원본 `hcr_robot.xacro` 는 mock 을 무조건 호출한다** (8행 `<xacro:hcr_robot_ros2_control/>`).
   include 하면 mock 과 MuJoCo 하드웨어가 한 URDF 에 공존한다 → **플랫
   `hcr_robot_urdf.urdf` 를 대신 include**해 우회했다. 근본 해결은 원본이 플러그인을
   인자로 받게 하는 것 (업무목록).
2. **`--fuse` 기본값이 펜을 `link6_1` 에 병합한다** — 변환 결과에 `pen_link` body 가 없다
   (body 7 = base + link1~6). 펜 끝에 센서를 붙이려면 `--no-fuse` 를 검토해야 한다.
3. **actuator 가 0 개로 나온다.** ros2_control 이 조인트를 제어하려면 MJCF actuator 가
   있어야 하므로, 변환기의 `-m/--mujoco_inputs` 로 actuator 정의를 넘겨야 한다.
   `-` `--scene` 옵션도 있어 **씬과 defaults/sensor 를 분리해 넘기는 구조**다 —
   지금 `mjcf/scene.xml` 한 장에 든 내용을 그 둘로 쪼개는 것이 다음 작업이다.

## 미결

- **필압 측정 경로** — R2 검증의 핵심인데 아직 열리지 않았다. 후보 셋: ① 변환된 MJCF
  후처리로 `pen_tip` 에 site + `<force>` 센서 ② ros2_control `effort` state_interface 로
  관절 반력 역산(래퍼에 인터페이스는 이미 열어 둠) ③ `mujoco_ros2_control_plugins` 로
  접촉 정보를 토픽으로. 씬이 실제로 뜬 다음 정할 일
- **접촉 파라미터 튜닝** — `mjcf/scene.xml` 의 `solref`/`solimp`/`friction` 은 출발점일
  뿐 실물 필압과 맞춘 값이 아니다
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
