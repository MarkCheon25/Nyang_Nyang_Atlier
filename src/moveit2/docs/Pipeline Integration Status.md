# 비전 → MoveIt2 → MuJoCo 연동 현황

**2026-08-14 갱신 · moveit2 파트**

세 모듈을 잇는 그리기 파이프라인이 끝에서 끝까지 동작한다. 아래는 실측으로 확인한 것,
아직 못 하는 것, 그리고 **다른 팀에 물어봐야 하는 것**이다.

> 이 문서는 팀 공통 문서다. moveit2 내부 상세는
> [`src/moveit2/ws_moveit2/src/drawing_cat/README.md`](../ws_moveit2/src/drawing_cat/README.md),
> 변경 이력은 같은 폴더의 `CHANGELOG.md` 를 본다.

---

## 1. 요약 — 지금 어디까지 됐나

| 영역 | 상태 | 내용 |
|---|---|---|
| 컨테이너 간 통신 | ✅ | `/joint_states` **500 Hz** 수신(σ=0.34 ms). 액션 client·server 가 컨테이너를 가로질러 1:1 연결 |
| 비전 좌표 수용 | ✅ | **2026-08-16 실물 연동 성공.** 비전 마스크 → 컨투어 → 토픽 → 로봇까지 관통. 축척·좌표 방향·타입 해시·프레임 경계 전부 실물 검증 ([상세](Vision%20Integration%20Result.md)) |
| 그리기 실행 | ✅ | 전 스트로크 `fraction 1.00`, 실행 결과 전부 `SUCCEEDED` |
| 계획 대비 검증 | ✅ | `map.csv` → `trace_cli` → `drawn.svg`. 파일 기반 검증 경로 유지 |
| 순서 최적화 (F3.1) | ⚠️ | 구현·실측 완료. 그런데 **시간을 줄인 것은 순서가 아니라 가속도였다** — NN 순서 기여는 0(노이즈 이하), 최선 조합이 **−8.5 %** ([상세](Trajectory%20Optimization.md)) |
| **관절 한계 눌림** | ❌ | **실측 확인.** MoveIt 이 joint_1 을 −7.07° 지시했으나 MuJoCo 는 −0.03° 에서 멈춤 → 펜 끝 **56.7 mm** 편차 |
| **궤적 추종** | ❌ | joint_2 오차 **최대 14.96°**. 그림을 키우면 악화 (5.33° → 14.96°) |
| **펜 끝(TCP)** | ⚠️ | 아직 플랜지 기준. 펜이 충돌 검사에 없고 `pen_tip` 프레임도 없다 |

---

## 2. 구조

컨테이너는 **2 개**다. 둘 다 `network_mode: host` 라 같은 ROS 그래프에 있고,
`ROS_DOMAIN_ID` 만 같으면 붙는다.

```
[vision]                [moveit2_dev]                    [simulation_dev]
SAM3·컨투어 ──────────▶ move_group  (계획·실행)  ──────▶ ros2_control_node (MuJoCo 물리)
          /vision/strokes  draw_cat    (픽셀→mm)          joint_trajectory_controller
          (픽셀 u,v)       rviz2       (펜 자취)   ◀────── joint_state_broadcaster
                              │                            robot_state_publisher
                              │ map.csv (mm)
                              ▼
                          trace_cli ──▶ drawn.svg (계획 회색 + 실제 색, A4 실제 비율)
```

### 좌표 하나가 로봇 움직임이 되기까지

```
비전 픽셀 (u,v) → [letterbox 변환] → 펜 끝 목표점(mm) → [MoveIt: RRT / Cartesian 보간 + IK]
→ 관절각 궤적 → [시간 배분] → ══ DDS 액션 ══ → [컨트롤러: 500Hz 스플라인 보간]
→ MuJoCo position actuator → [물리 엔진: 관성·중력·접촉] → 실제 각도 → /joint_states
```

> ⚠️ **"최적 궤적"이 아니다.** 홈 이동은 OMPL **RRT**(샘플링 기반 — 실행 가능한 경로 하나를
> 찾을 뿐), 그리기는 `computeCartesianPath`(**플래너가 아니라** 직선 보간 + IK)다.
> **그리는 선의 모양은 비전이 준 좌표 그대로다.**
>
> 2026-08-14 에 **F3.1(순서 최적화)** 가 들어갔지만, 그것이 정하는 것은 **스트로크를 어떤
> 순서로 · 어느 점부터 그릴지**이지 선 모양이 아니다. Nearest Neighbor 라 **근사**이기도 하다.

> ⚠️ **MuJoCo 는 궤적을 "재생"하지 않는다 — 이것이 추종 오차의 원인이다.** 컨트롤러가
> 500 Hz 로 푼 위치 지령을 목표로 물리 엔진이 토크를 만들 뿐, 그 각도가 되는 것은 보장되지
> 않는다. joint_2 오차 14.96° 는 **물리 엔진**에서, joint_1 눌림은 **MJCF 관절 한계**에서
> 생긴다 — 발생 지점이 다르다.

> **기동 순서가 왜 반대인가**
> "MoveIt2 가 머리"는 **명령 방향**이고, 기동 순서는 **상태 방향**을 따른다. 플래너는
> "지금 로봇이 어디 있는지"에서 출발하고 그 출처가 MuJoCo 다. 실물도 같다 — 로봇
> 전원을 먼저 올리고 MoveIt 을 띄운다.
>
> 다만 강제가 아니라 관례다(실측). move_group 을 먼저 띄워도 시뮬 기동 후 재시작 없이
> 복구된다. 지켜야 할 것은 "계획·실행을 시도하는 시점에 시뮬이 떠 있을 것" 하나뿐이다.

---

## 3. 좌표를 받는 세 갈래

`strokes.source` 파라미터로 고른다. 어느 쪽으로 들어와도 **결국 mm 로 수렴**하므로
시뮬레이션 검증 경로가 끊기지 않는다.

| source | 전달 | 단위 | 쓰임 | 상태 |
|---|---|---|---|---|
| `params` | YAML 직접 기입 | mm | 개발·회귀 시험용 | ✅ 검증 |
| `csv` | 파일 `map.csv` | mm | 비전 `stroke_map_cli` 산출물, 오프라인 재현 | ✅ 검증 |
| `topic` | `/vision/strokes` | **픽셀** | 실전 연결 (SAM3) | ⚠️ 가짜 발행자로만 |

### 픽셀 → mm 변환은 임의로 정하지 않는다

메시지가 `image_width`·`image_height` 를 실어 보내고, 비전의 `ScaleToPaper` 는 축척
기준을 컨투어 bbox 가 아니라 **이미지 전체 크기**로 잡는다. 그래서 같은 letterbox
계산을 재현하면 **`map.csv` 에 들어갔을 값과 동일한 mm 좌표**가 나온다.

```
# A4 210×297, 여백 15 → 작화영역 180×267
scale    = min(180 / image_width, 267 / image_height)     [mm/px]
offset_x = 15 + (180 - image_width  * scale) * 0.5
offset_y = 15 + (267 - image_height * scale) * 0.5

# 실측: 이미지 1280×960 →
scale    = min(0.140625, 0.278125) = 0.140625     로그값 0.14062 ✓
offset_x = 15 + (180 - 180)/2      = 15.0 mm
offset_y = 15 + (267 - 135)/2      = 81.0 mm
```

용지 세 값은 파라미터라, 비전이 다른 규격을 쓰면 YAML 만 고치면 된다.

### ⚠️ 메시지만으로는 어느 이미지의 부위인지 알 수 없다

`Header` 도 프레임 번호도 없다. 받는 쪽이 경계를 판정한다 —
① `instance_label` 이 중복되면 새 이미지 시작 ② 마지막 수신 후 일정 시간 조용하면
프레임 확정. 프레임이 여러 개 쌓이면 **가장 최신 것**만 쓰고 경고를 남긴다.
부위 개수에 의존하지 않으므로 비전 쪽 `expected_parts` 값과 무관하다.

### ⚠️ 구독 QoS — 인터페이스 문서와 다른 부분

비전 인터페이스 문서에는 *"QoS 를 동일하게(RELIABLE + TRANSIENT_LOCAL) 맞춰야 한다"*
고 되어 있으나, **durability 는 "구독자가 덜 요구하는" 방향이 호환**이다. 실측:

```
발행자 TRANSIENT_LOCAL, 구독 전에 5 개 발행
  VOLATILE        구독자 → 밀린 이력 0 개, 발행자가 보는 구독자 수 1,
                          이후 새로 발행한 3 개는 정상 수신
  TRANSIENT_LOCAL 구독자 → 밀린 5 개 전부 수신
```

기본값을 `VOLATILE` 로 두어 이전 이미지가 섞일 여지를 없앴다. 비전이 한 번 쏘고
끝나는 운용이면 `strokes.use_latched: true` 로 바꾼다.

---

## 4. 측정값

추정이 아니라 실행 중 샘플링한 값이다.

### 통신 · 실행

| 항목 | 값 | 측정 |
|---|---|---|
| `/joint_states` 수신율 | **500 Hz** | moveit2 컨테이너에서 `ros2 topic hz`, σ=0.34 ms |
| Cartesian 경로 계산 | 12–50 ms | 웨이포인트 5–100 개 구간 |
| 스트로크 성공률 | 1.00 | 비전 5 부위 전부, 실행 `SUCCEEDED` |
| 계획 대비 그린 길이 | 98.7 % | `trace_cli` ideal, 조각 수 5 → 5 |

### ⚠️ 관절 한계 눌림 — 실측 확인 (2026-08-14)

`paper.scale: 0.001`(그림 84 × 93 mm)로 그리면서 지령값과 실제값을 함께 샘플링했다.

| 관절 | 지령 범위 | 실제 범위 | 오차 | 판정 |
|---|---|---|---|---|
| **joint_1** | −7.07° ~ +7.61° | **−0.03°** ~ +6.82° | −8.35° | **한계에 눌림** |
| **joint_6** | −0.15° ~ +0.17° | **−0.00°** ~ +0.17° | −0.15° | **한계에 눌림** |
| joint_2 | −8.74° ~ +6.94° | −9.14° ~ +3.63° | **+14.96°** | 한계 무관 — 추종 지연 |
| joint_3 | 80.0° ~ 96.5° | 80.2° ~ 96.6° | −4.12° | 추종 지연 |
| joint_4 | −99.7° ~ −91.1° | −99.8° ~ −91.1° | −1.52° | 정상 |
| joint_5 | −91.3° ~ −76.6° | −91.3° ~ −76.6° | −0.92° | 정상 |

MoveIt 이 joint_1 을 **−7.07° 까지 지시했는데 MuJoCo 는 −0.03° 에서 멈췄다.**
펜 끝 편차로 환산하면:

```
7.04° × 반경 0.461 m = 56.7 mm      (그림 전체 폭 84.4 mm 의 67 %)
```

**MuJoCo 가 그린 것은 고양이 모양이 아니라 한쪽이 뭉개진 형태다.**

> ⚠️ **`fraction: 1.00` 이어도 안심할 수 없다.** 위 실행에서 다섯 스트로크 모두
> `fraction 1.00`, 실행도 전부 `SUCCEEDED` 였다. moveit2 쪽에서 한계를 넓혀 우회해 두었기
> 때문에 **MoveIt 은 MuJoCo 가 갈 수 없는 영역까지 계획하고도 성공으로 보고한다.**

> ⚠️ **`joint_6` 도 같은 문제다.** 두 관절의 한계가 같으므로 증상도 같다.

작은 그림(34 × 37 mm)에서도 필요 범위가 ±2.8° 였으므로 **이미 눌리고 있었다.**

### 궤적 추종 오차 — 별개 문제

측정: `/joint_trajectory_controller/controller_state` 의 `error.positions` 샘플링.

| 구간 | 최대 | 평균 |
|---|---|---|
| 정지 (자세 유지) | 0.17° | — |
| 홈 이동 (관절공간) | 2.57° | 1.26° |
| 외곽선 34 × 37 mm | 5.33° | 2.76° |
| **외곽선 84 × 93 mm** | **14.96°** | — |

정지 시 0.16° → 그리는 중 크게 벌어진다. **중력 처짐이 아니라 동적 추종 지연**이고,
**그림을 키울수록 악화된다.** joint_2 는 가동범위가 ±90° 라 한계와 무관한 순수 추종
문제다 — 지령 +6.94° 인데 실제는 +3.63° 에서 멈췄다.

> 위 관절 한계 눌림과 **원인이 다르다.** 순수한 추종 문제로 볼 것은 joint_2 · joint_3 이고,
> joint_1 · joint_6 의 큰 오차는 한계 눌림이다.

### MuJoCo 뷰어와 제어 루프 — GPU 로도 해결 안 된다

| 조건 | 30초당 오버런 | Total time 중앙값 | loop 최대 |
|---|---|---|---|
| 헤드리스 | 0.0 회 | — | — |
| 뷰어 + 소프트웨어 렌더링 | 29.0 회 | 2834 µs | 20.24 ms |
| 뷰어 + GPU | 29.0 회 | 2407 µs | 10.51 ms |

오버런의 원인은 소프트웨어 렌더링이 아니라 **뷰어를 띄운다는 사실 자체**다. GPU 가
바꾸는 것은 횟수가 아니라 **심각도**(최악 루프 시간 절반). 추종 오차는 세 조건 모두
5.3–5.9° 로 같은 자리 — **렌더링 부하는 추종 실패의 원인이 아니다.** 확인을 SVG 로
한다면 헤드리스가 맞다.

---

## 5. 다른 팀에 필요한 결정

moveit2 쪽에서 혼자 못 푸는 것들. 위험한 순서.

### 시뮬레이션팀 — 가장 시급

**`joint_1` · `joint_6` 의 한계를 고치고 MJCF 를 재변환할 수 있을까**

두 관절의 URDF 한계가 `[0, 2π]` 인데 나머지 넷은 전부 ±대칭이고, 하필 `hcr_home` 이
그 하한 위에 정확히 놓여 있다.

> ## 🔴 2026-08-16 — **mock hardware 에서도 재현됐다**
>
> 지금까지 이 문제는 MuJoCo 에서만 관찰돼서 "시뮬레이터 쪽 문제 아니냐" 는 여지가
> 있었다. **`demo.launch.py`(mock components)로 돌려도 똑같이 막힌다.**
>
> ```
> 그림 크기 15 cm → 8 cm 로 줄여도  fraction 0.33 으로 동일
> joint_limits 에 위치 한계 ±2π 추가 →  fraction 1.00, 5/5 실행
> ```
>
> **크기를 줄여도 소용없는 이유:** `hcr_home` 의 `joint_1 = 0` 이 **하한 위**라
> 장벽이 멀리가 아니라 **출발점 그 자리**에 있다. 그래서 축척과 무관하게 윤곽선의
> 같은 지점에서 걸린다.
>
> **→ 시뮬레이터와 무관한 로봇 기술(URDF) 문제라는 것이 확정됐다.**
> 상세: [`Vision Integration Result.md`](Vision%20Integration%20Result.md) §3

**이건 예측이 아니라 실측이다 (2026-08-14):**

```
joint_1   MoveIt 지령  -7.07°  →  MuJoCo 실제  -0.03°  (0 에서 멈춤)
          펜 끝 편차   56.7 mm  =  그림 전체 폭 84.4 mm 의 67 %
```

그림 크기와 무관하게 항상 일어난다 — 34 × 37 mm 로 작게 그려도 필요 범위가 ±2.8° 라
이미 눌리고 있었다. **MuJoCo 가 그리는 것은 고양이 모양이 아니다.**

moveit2 쪽은 `robot_description_planning.joint_limits.<joint>.min_position` 덮어쓰기로
플래닝만 우회해 두었다. 그래서 **`fraction 1.00`, 실행 `SUCCEEDED` 가 나오는데도 실제로는
눌린다** — 로그로는 감지되지 않는다. MJCF 는 같은 URDF 에서 변환되어
`range="0 6.28318"` 을 그대로 갖는다.

필요한 것: `hcr_robot.xacro` 의 두 관절 한계를 ±대칭으로 고치고 MJCF 재변환.
(두 관절을 `continuous` 로 선언하는 것도 방법이지만 변환 영향을 확인해야 한다.)

### 하드웨어 — 실물로 옮기기 전에

**펜업 이동 가속을 관절 한계의 100 % 로 올렸다. 실물에서도 괜찮은지 봐 달라.**

`drawing_cat` 의 `planning.travel_acceleration_scaling` 을 **0.5 → 1.0** 으로 바꿨다
(2026-08-14). 시뮬에서 이것 하나가 전체 실행 시간을 가장 크게 줄였다 — 순서 최적화보다
크다. 근거는 [`Trajectory Optimization.md`](Trajectory%20Optimization.md) §4.3.

| 가속 배율 | 0.1 | 0.3 | 0.5 | 1.0 |
|---|---|---|---|---|
| 펜업 이동 시간 | 0.51 s | 0.29 s | 0.23 s | **0.16 s** |
| 전체 (계획) | 3.59 s | 3.43 s | 3.41 s | **3.34 s** |

**걱정되는 것은 속도가 아니라 도착 시 급감속이다.** 펜업 이동은 실제로 4.5 ~ 24 mm 짜리라
최고 속도에 도달하지도 못한다(그래서 `travel_velocity_scaling` 은 효과가 아예 없었다).
전 구간이 가속 아니면 감속이고, 가속 배율을 올린다는 것은 곧 **감속을 급하게 한다**는 뜻이다.

BRD §2.3 대로 펜이 플랜지에 **테이프 고정**이고 컴플라이언스가 없어서, 그 감속 충격이
펜 고정에 어떻게 오는지는 시뮬로 알 수 없다.

**묻고 싶은 것 두 가지:**

1. 실물 관절의 가속 한계가 URDF/`joint_limits.yaml` 값과 같은가? (같아야 배율 1.0 이
   "한계의 100 %" 라는 계산이 성립한다)
2. 그 감속을 테이프 고정 펜이 견디는가? 못 견디면 몇 % 가 상한인가?

**지금 걸려 있는 안전 여유:**

- 이 값은 **펜업 구간에만** 적용된다. 종이에 닿는 그리기는 `velocity_scaling: 0.1`
  (10 %) 그대로다 — **급감속이 일어나는 순간 펜은 종이에서 8 mm 떠 있다.**
- 되돌리는 데 재빌드가 필요 없다. `draw_cat_params.yaml` 의 한 줄을 낮추면 된다.

> ⚠️ 지금까지의 수치는 전부 **시뮬레이션**이다. 실물에서 이 값을 처음 쓸 때는 0.1 부터
> 올려 가며 확인하는 편이 안전하다.

### 비전팀 — 타입 해시는 해결됐다 ✅

`dy/vision` 브랜치의 정의와 우리 미러를 각각 빌드해 **생성된 타입 기술 파일을 바이트
단위로 비교**했다 (2026-08-15). 완전히 동일하다.

```
RIHS01_d8c0b1c8ffdbe600b7b59775cc2460b82c96be241faf49fea80c7393e77506d7
```

필드가 같고 주석만 다르며, 주석은 해시에 들어가지 않는다. **더 물어볼 것이 없다.**

### 비전팀 — 새로 생긴 것

`dy/vision` 실물 코드를 읽고 나서 남은 질문 6 개(노드 분담 · `vision_interfaces` 소유권 ·
`draw_size` · `instance_label` 유일성 · `epsilon_ratio` · 컨투어 하나만 쓰는 것)는
[`Vision Interface Contract.md`](Vision%20Interface%20Contract.md) §5 에 복사용으로 정리해 두었다.

### 비전팀 — 여유 될 때

**메시지에 `Header`(타임스탬프)를 넣을 수 있을까**

지금은 어느 이미지의 부위인지 구분할 수단이 메시지 안에 없어 받는 쪽이 라벨 중복과
시간 간격으로 추정한다. 동작은 하지만 추정이다. `closed` 필드도 없어 열린 스트로크
(수염 같은 것)는 표현할 수 없다.

### 팀 전체 — 합의 필요

**`ROS_DOMAIN_ID` 를 `run_container.sh` 에서 export 하자**

네 모듈 compose 가 전부 `${ROS_DOMAIN_ID:-0}` 이고 `.env` 도 없는데,
`run_container.sh` 는 `HOST_UID`·`HOST_GID`·`VIDEO_GID`·`RENDER_GID`·`XAUTHORITY` 를
전부 export 하면서 이것만 빠뜨렸다.

```bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"   # 다른 export 옆에 한 줄
```

**2026-08-14 에 실제로 당했다.** 컨테이너를 지운 뒤 `export` 없이 다시 만들었더니 도메인
0 으로 생성됐고, 마침 같은 PC 에 도메인 0 으로 도는 다른 컨테이너(mock hardware)가 있어
**거기에 붙었다.** 증상이 도메인 문제로 보이지 않는 것이 함정이다:

```
Planning request complete!     ← 계획은 성공 (mock 쪽 /joint_states 로)
Execute request aborted        ← 1.5 ms 만에 즉시 거부
```

컨트롤러 이름이 `joint_trajectory_controller` 가 아니라 mock 의 `hcr_arm_controller` 였다.
**5 초 만에 판별하는 법** — move_group 을 띄우기 전에:

```bash
ros2 control list_controllers
#   joint_trajectory_controller → 정상
#   hcr_arm_controller          → mock 에 붙었다. 도메인 확인
```

위험 구간은 **컨테이너를 지운 뒤 다시 만들 때** 하나뿐이다. 이미 떠 있는 컨테이너에
셸을 더 여는 것은 안전하다 (`docker exec` 가 컨테이너 환경변수를 물려받는다).

### moveit2 — 우리 몫

**펜을 MoveIt 의 TCP 로 넣을지 결정**

지금은 플랜지 기준이라 펜이 충돌 검사에 없고 `pen_tip` 프레임도 없다. 실제 접촉
궤적이 없어 `trace_cli` 를 `--dummy` 로만 돌릴 수 있는 것도 이 때문이다. 펜 치수는
전부 실측 전 임시값이며, 교체 지점은 `drawing_cat/README.md` 6 절에 한 곳으로 모아
두었다.

---

## 6. 다시 시도하지 말 것

이름만 보면 정답처럼 보이는데 실측으로 아닌 것이 확인된 것들.

| 시도 | 결과 |
|---|---|
| `ompl.fix_start_state: true` | ❌ 그 파라미터는 continuous·planar·floating 관절의 회전값 정규화만 한다. `joint_1` 은 한계가 있는 revolute 라 대상이 아니다 |
| `CheckStartStateBounds` 어댑터 제거 | ❌ OMPL 상태공간도 같은 URDF 한계로 만들어져 `START_STATE_INVALID` 로 한 단계 뒤에서 죽는다 |
| GPU 패스스루로 오버런 줄이기 | ⚠️ 횟수는 29 회로 동일. 심각도만 절반 |
| `demo.launch.py` 로 MuJoCo 에 붙이기 | ❌ mock hardware 단독 데모용이라 몸통까지 띄운다. 컨트롤러 매니저 2 개, `/joint_states` 오염, `/tf` 이중 발행 등 5 곳이 겹친다 |
| launch 를 `kill` 로 종료 | ⚠️ 자식 노드가 남는다. 실제로 `robot_state_publisher` 4 개, `move_group` 2 개가 쌓여 `unknown goal response` 로 실행이 실패했다. **Ctrl-C 로 끝낼 것** |
| RViz 초록 선으로 왜곡 판정 | ❌ **마커는 "계획"이지 "실제"가 아니다.** 입력 좌표로 그려지므로 MuJoCo 가 못 따라가도 항상 멀쩡해 보인다. 실제는 MuJoCo 창이나 `controller_state` 로 봐야 한다 |
| 컨테이너를 지운 뒤 `export` 없이 다시 만들기 | ❌ 도메인 0 으로 생성되어 **에러 없이 엉뚱한 컨테이너에 붙는다** (아래) |

---

## 7. 직접 돌려보기

셸 4 개. **순서가 중요하다** — `draw_cat` 의 기본 구독 QoS 가 `VOLATILE` 이라 구독이
먼저 떠 있어야 한다.

```bash
# ① simulation 컨테이너 — 몸통
export ROS_DOMAIN_ID=42          # ⚠️ up 하기 전에
ros2 launch sim_bringup mujoco_sim.launch.py

# ② moveit2 컨테이너 — 머리 + RViz
ros2 launch drawing_cat mujoco_moveit.launch.py

# ③ moveit2 컨테이너 — 토픽을 기다린다 (최대 120초)
ros2 launch drawing_cat draw_cat.launch.py \
  params_file:=$(ros2 pkg prefix drawing_cat --share)/config/topic_example.yaml

# ④ "구독 중" 로그가 뜨면 — 비전 없이 시험할 때
ros2 run drawing_cat fake_vision_publisher     # test_tools, 확인 전용
```

> ⚠️ **③ 이 대기에 들어간 뒤 120 초 안에 ④ 를 실행해야 한다.** 넘기면 종료되며 ③ 만
> 다시 실행하면 된다.

> ⚠️ **`ros2 launch` 는 `--ros-args -p` 를 받지 않는다** (그건 `ros2 run` 전용).
> 파라미터를 바꾸려면 `topic_example.yaml` 을 복사해 고친 뒤 `params_file:=` 로 넘긴다.

> ⚠️ **원격에서 세션이 끊기면 그 안의 launch 도 죽는다** (`run_container.sh shell` 은
> `docker exec -it` 이다). 위 절차대로 해도 문제없지만, 원격이 자꾸 끊긴다면 오래 도는
> 것만 `docker exec -d` 로 분리 실행하는 방법이 있다 — moveit2 README 2 절.

**셸 2 에서 move_group 을 띄우기 전에 이 한 줄을 먼저 치면** 도메인 사고를 5 초에
잡는다 (§5 참조):

```bash
ros2 control list_controllers      # joint_trajectory_controller 여야 한다
```

MuJoCo 창을 보려면 셸 ① 에서 `headless:=true` 를 뺀다. 그림이 작으면 움직임이 1° 안팎이라
잘 안 보이므로, 파라미터 파일에서 `paper.scale` 을 0.001 로, `paper.pen_lift` 를 0.05 로
올리면 확실히 보인다.

> ⚠️ RViz 에 자취가 안 보이면 **`MotionPlanning` 디스플레이를 꺼 보라.** 자취는 펜 끝에
> 그려지는데 거기 로봇 팔이 있어 메시에 가린다. 그리고 그 초록 선은 **계획**이지 실제가
> 아니다 — 멀쩡해 보여도 실제 로봇은 다른 곳을 지났을 수 있다.

검증 결과를 눈으로 보려면 `output.map_csv_path` 에 저장된 계획 지도를 시뮬레이션
컨테이너로 옮겨 `trace_cli` 에 넣는다. 계획(회색)과 실제(색)가 겹쳐진 A4 실제 비율
SVG 가 나오고, 인쇄해서 자로 재는 것까지 된다.

---

## 참고

- moveit2 상세: [`src/moveit2/ws_moveit2/src/drawing_cat/README.md`](../ws_moveit2/src/drawing_cat/README.md) (11 개 절)
- **궤적 최적화 측정·비교**: [`Trajectory Optimization.md`](Trajectory%20Optimization.md) — 무엇을 어떻게 재는지와 실측 결과
- 변경 이력: `src/moveit2/ws_moveit2/src/drawing_cat/CHANGELOG.md`
- 확인 도구와 제거 방법: `src/moveit2/ws_moveit2/src/drawing_cat/test_tools/README.md`
- 미러 패키지 주의사항: `src/moveit2/ws_moveit2/src/vision_interfaces/README.md`

> ⚠️ 이 문서의 수치는 **실제 비전 컨테이너가 아니라 스펙대로 발행하는 시험 노드**로
> 확인한 것이다. 비전팀이 타입 해시를 확인해 주면 실물로 다시 재야 한다.
> 다만 관절 한계 눌림·추종 오차는 비전 입력과 무관한 로봇 쪽 문제이므로 그대로 유효하다.
>
> 2026-08-14: 셸 4 개를 손으로 띄워 전 구간을 다시 확인했고, 그 과정에서 관절 한계 눌림을
> 실측했다.
