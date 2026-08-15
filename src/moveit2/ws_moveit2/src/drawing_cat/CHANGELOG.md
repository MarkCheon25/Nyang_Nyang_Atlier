# drawing_cat 작업 기록

시간 역순. 각 항목은 **무엇을 왜 바꿨고, 무엇으로 확인했는지**를 남긴다.

---

## 2026-08-14 ⑦ — F3.1 을 optimizer.hpp 로 분리 · 단위시험을 저장소로

### 한 것

F3.1 계산부(`Stroke` · `dist2d` · `strokeExit` · `totalTravelMm` · `optimizeStrokeOrder`)를
`draw_cat.cpp` 에서 **`include/drawing_cat/optimizer.hpp`** 로 옮겼다. 함수는 `inline`
이고 `namespace drawing_cat` 안이다.

**이 헤더는 ROS·MoveIt 에 의존하지 않는다 — 의도된 분리다.** 순수 기하 계산만 남겨야
로봇 없이 시험이 돈다. 좌표 변환·궤적 실행처럼 ROS 가 필요한 것은 `draw_cat.cpp` 에 남겼다.

그 위에 `test/test_optimizer.cpp` (gtest 5 케이스)를 붙였다. 이전에는 세션 임시 폴더에
있어서 **다음 사람이 코드를 고쳐도 안 돌아가는 상태**였다.

```bash
colcon test --packages-select drawing_cat
```

### 그물이 실제로 잡는지 확인했다

시험이 통과한다는 것만으로는 부족해서, 일부러 고장 내 보고 걸리는지 봤다:

| 고장 | 결과 |
|---|---|
| NN 비교를 반전 (`d < best_dist` → `>`) | 1 error + 1 failure |
| 폐곡선 회전 제거 (모든 정점 → `flat[0]` 고정) | 3 failures |
| 원복 | **0 failures** (헤더가 원본과 바이트 동일 확인) |

### ⚠️ 린터 3 개를 껐다

`colcon test` 를 이 패키지에서 **처음 돌려 봤더니 38 개가 실패했다.** 대상이
`src/draw_cat.cpp` · `launch/*.py` · `test_tools/*.py` 등 **기존 파일들**이다 —
이 저장소가 ROS 2 기본 스타일을 안 따르기 때문이고(4 칸 들여쓰기, Allman 중괄호,
Python 큰따옴표, 한국어 주석), `copyright` · `cpplint` 는 원래부터 같은 이유로 꺼져 있었다.

끄지 않으면 **colcon test 가 항상 빨간불이라 단위시험 실패가 묻힌다.** 그래서
`uncrustify` · `flake8` · `pep257` 도 같은 방식으로 껐다. 되돌리려면 CMakeLists 의
세 줄을 지우고 전체 스타일을 정리하면 된다 (`CLEANUP.md` 2절).

### 같이 만든 것

`CLEANUP.md` — **최종 산출물로 갈 때 빼야 할 것**의 목록. 지금은 아무것도 빼지 않고,
무엇이 검증용이었는지만 기록해 둔다. 나중에는 구분이 안 되기 때문이다.

---

## 2026-08-14 ⑥ — F3.1 순서 최적화 구현 · 펜업 이동 분리

### 한 것

**1) F3.1 — Nearest Neighbor 스트로크 순서 최적화** (`optimizeStrokeOrder`)

vision 규약상 스트로크들 사이에는 순서가 없다. 매번 "펜에서 가장 가까운 아직 안 그린
스트로크"를 골라 **펜업 이동 거리**를 줄인다. 진입점 선택이 종류마다 다르다:

- **열린 스트로크** — 양 끝점 2 개 중 가까운 쪽. 필요하면 좌표를 뒤집는다(방향 반전).
- **닫힌 스트로크** — **모든 정점**이 후보다. 폐곡선에는 "첫 점"이 원래 없고(어느
  정점에서 출발하든 한 바퀴 돌아 제자리), `flat[0]` 은 비전 컨투어 추적기가 우연히
  시작한 점일 뿐이다. 가장 가까운 정점이 앞에 오도록 `std::rotate` 한다.

⚠️ **토픽 경로는 전부 `closed=true` 라, 실전 절감은 대부분 회전에서 나온다.** 방향
반전은 `params`·`csv` 경로에서만 동작한다.

**2) 펜업 이동을 그리기에서 분리**

예전에는 `computeCartesianPath` 한 호출에 "날아가기 + 그리기"가 묶여 있었다. MoveIt 이
`waypoints[0]` 까지 가는 구간도 만들어 주는데, 같은 호출이라 **날아가는 구간까지 속도
10 %, 직선 강제, 2 mm 마다 IK** 를 받았다. 펜이 종이에 안 닿는 구간이라 그럴 이유가 없다.

호출을 둘로 쪼개고 배율을 따로 준다 (`travel_velocity_scaling` 0.5 vs `velocity_scaling` 0.1).

**3) 블록 순서 재배치**

```
기존:  로드 → 필터 → map.csv 저장 → 경계상자(auto_center)
변경:  로드 → 필터 → 경계상자(auto_center) → F3.1 → map.csv 저장
```

- **경계상자가 먼저** 와야 한다 — F3.1 의 시작점으로 쓰는 `center_x_mm/center_y_mm` 이
  `auto_center` 블록에서 덮어써지기 때문.
- **map.csv 는 나중**이라야 한다 — 저장된 `stroke_idx` 가 실제 실행 순서와 같아야
  `trace_cli` 로 볼 때 인덱스가 맞는다.

### 왜 mm 공간에서 순서를 정해도 되는가

`flangePose()` 에서 `mx_mm == center_x_mm` 이면 `p.x = px0 - tip_offset.x` 이고 `px0` 는
현재(home) 자세의 펜 끝이라, 결국 home 플랜지 위치로 환원된다. 즉 **종이 좌표계에서
home 이 놓인 자리가 곧 `(center_x_mm, center_y_mm)`** 이다. 그리고 mm ↔ world 가 아핀
변환이라 **거리의 대소가 보존**되므로, mm 에서 고른 순서가 world 에서도 같은 순서다.
로봇 실제 자세를 몰라도 된다.

⚠️ 이 등식은 `paper.use_current_pose_as_origin: true` 일 때만 성립한다. `false` 면 원점이
파라미터로 고정되어 home 위치를 알 수 없다. 다만 그때 틀어지는 것은 **첫 스트로크 선택
하나뿐**이고(이후 홉은 스트로크 끝점끼리라 무관) 순서 자체가 무효가 되지는 않는다.

### 확인한 것

`draw_cat.cpp` 의 F3.1 블록을 **원본 텍스트 그대로 떼어내** 단위 시험을 붙였다 (복사본이
아니라 원본을 컴파일). 검증 24 개 전부 통과:

| 시험 | 결과 |
|---|---|
| 열린 스트로크 NN 순서 + 방향 반전 | 이동거리 160 → 60 |
| 닫힌 스트로크 시작점 회전 | 진입 141.4 → 0 (`rotate_closed_start: false` 면 141.4 유지) |
| 회전·반전 후 좌표 집합 보존 | 점을 잃거나 만들지 않음 |
| `closed` 이탈점 = 진입점 | main 의 `end_x/end_y` 규칙과 일치 |
| `params` 실제 데이터 5 개 | 펜업 이동 **170.0 → 123.9 종이mm (27.1 % 감소)** |
| 엣지 (0 개 · 1 개 · 2 점 폐곡선) | 크래시 없음 |

빌드: `colcon build --packages-select vision_interfaces drawing_cat` 통과 (새 경고 없음).

### 실측 결과 (같은 날 저녁, 시간 계측 추가 후)

시간 계측(`trajDurationS`·`runTimed`)을 넣고 A/B/C/D 네 조건을 실제로 돌렸다.
상세는 [`docs/Trajectory Optimization.md`](../../../docs/Trajectory%20Optimization.md).

| 조건 | n | 펜업 거리 | 합계 시간 | A 대비 |
|---|---|---|---|---|
| A 베이스라인 | 1 | 136.8 종이mm | 3.65 s | — |
| B + NN 순서 | 1 | 101.2 (−26.1 %) | 3.66 s | **0 — 노이즈 이하** |
| C + 폐곡선 회전 | 3 | 80.5 (−41.2 %) | 3.590 s | −1.6 % |
| D + 이동 속도 0.5 | 3 | 80.5 | 3.413 s | **−6.5 %** |

계획 시간의 재현성은 **±0.01 s (±0.3 %)** 다 (C·D 3 회씩 반복). B−A 의 +0.01 s 가
노이즈와 구분되지 않는다는 근거가 이것이다.

**⚠️ 거리는 시간의 대리 지표가 아니다.** 펜업 거리를 26.1 % 줄였는데 시간은 1.7 %
줄었다. 이동 하나가 실제로는 8 ~ 14 mm 라 **가감속 램프와 궤적당 고정 오버헤드가
지배**하기 때문이다 — 펜업 시간 0.58 s 중 약 0.41 s(71 %)가 거리와 무관하다.

그래서 **`travel_velocity_scaling` 상향 하나(−3.9 %p)가 순서 최적화 전체보다 크게
기여했다.** 베이스라인에서 펜업은 전체의 15.8 % 였고, 그중 38 % 를 회수했다.

### ⚠️ `travel_velocity_scaling` 은 아무 효과가 없다 (속도·가속 분리 실측)

둘을 분리해서 재 보니:

| 속도 | 가속 | 펜업 | 합계 | |
|---|---|---|---|---|
| 0.1 | 0.1 | 0.51 | 3.590 | 기준 |
| **0.5** | **0.1** | **0.51** | 3.580 | **속도만 5 배 → 변화 없음** |
| **0.1** | **0.5** | **0.23** | 3.410 | **가속만 5 배 → 효과 전부** |

**펜업 이동이 실제 4.5 ~ 24 mm 라 최고 속도에 도달할 일이 없다.** 순항 구간 없이
가속하다 바로 감속한다. 그래서 `t = 2√(d/a)` 가 정확히 맞고 (여섯 점이 `1/√a` 에
상수항 없이 맞았다), ① 속도 배율은 무효 ② 거리는 √ 로만 붙어 순서 최적화가 무력하다.

가속 스윕은 **1.0 이 최소**였다 (합계 3.340 s = A 대비 **−8.5 %**, 상한의 53 % 회수).
꺾이는 지점이 없다 — 뱉는 양이 +0.11 s 에서 포화하기 때문이다.

**→ 기본값을 `travel_acceleration_scaling: 0.5 → 1.0` 으로 올렸다** (yaml 과 `draw_cat.cpp`
의 declare 기본값 둘 다 — 다른 예시 yaml 들이 코드 기본값을 물려받기 때문이다).

`travel_*` 를 하나도 안 적은 설정으로 2 회 돌려 확인했다: 펜업 **0.16 s** / 합계
**3.34 s**, 가속을 명시했던 조건과 동일. **펜업 비율 15.8 % → 4.8 %** (원래 0.58 s 중
0.42 s 제거). 남은 상한이 4.8 % 라 **펜업 이동 쪽은 여기서 닫는다.**

🔴 **실물 이관 전 하드웨어 확인이 필요하다.** 걱정은 속도가 아니라 **도착 시 급감속**이다 —
전 구간이 가속 아니면 감속이라 가속 배율을 올리는 것이 곧 감속을 급하게 하는 것이고,
펜이 BRD §2.3 대로 테이프 고정이라 컴플라이언스가 없다. 다만 이 값은 **펜업 구간에만**
걸려서 급감속 순간 펜은 종이에서 8 mm 떠 있고, 그리기는 `velocity_scaling: 0.1` 그대로다.
되돌리는 데 재빌드도 필요 없다. 물어볼 것은
[`Pipeline Integration Status.md`](../../../docs/Pipeline%20Integration%20Status.md) §5 "하드웨어".

### 아직 못 한 것

**가속을 올려 번 시간의 약 39 % 가 그리기 시간 증가로 도로 나간다.** C·D 를 3 회씩
번갈아 돌려 **런 간 변동이 아님을 확인**했다 (C 그리기 3.07~3.09 vs D 3.19~3.19, 범위
안 겹침). 펜업에서 번 0.280 s 중 **0.110 s 를 뱉는다.**

**원인은 미확인.** 다만 두 가지가 좁혀졌다 — ① **가속을 올릴 때만** 생긴다 (속도만 올린
조건은 그리기가 3.07 로 기준과 같다). 즉 도착 시 **감속이 급한 것**이 원인 쪽이다.
② **+0.11 s 에서 포화**한다. 다음 단계는 펜업 이동 뒤 정착 대기 파라미터를 넣고
되돌아오는지 보는 것이다.

그리고 **기본값을 바꿀지가 팀 결정으로 남았다.** 시뮬 최적은 가속 1.0 인데 실물 안전
확인이 안 끝났다 (BRD §2.3: 펜이 테이프 고정, 컴플라이언스 없음).

### 다시 시도하지 말 것

| 시도 | 결과 |
|---|---|
| 닫힌 스트로크를 뒤집어 진입점 바꾸기 | ❌ 폐곡선은 뒤집어도 첫 점이 그대로다. 필요한 건 반전이 아니라 **회전** |
| 순서 최적화를 `auto_center` 앞에 두기 | ❌ 시작점(`center_x_mm`)이 아직 덮어써지기 전이라 엉뚱한 점에서 출발한다 |
| map.csv 를 최적화 앞에서 저장 | ❌ `stroke_idx` 가 실행 순서와 어긋나 `trace_cli` 비교가 깨진다 |

---

## 2026-08-14 ⑤ — 수동 실증 · 관절 한계 눌림 실측

### 한 것

셸 4 개를 손으로 띄워 전 구간을 다시 확인했다. 코드 변경은 없고 문서만 갱신했다.

### 겪은 것 — 도메인 사고

컨테이너를 `down` 으로 지운 뒤 **`export ROS_DOMAIN_ID=42` 없이** 다시 만들어서 도메인
0 으로 생성됐고, 같은 PC 에 도메인 0 으로 도는 mock hardware 컨테이너에 붙었다.

```
Planning request complete!     ← 계획은 성공 (mock 쪽 /joint_states 로)
Execute request aborted        ← 1.5 ms 만에 즉시 거부
```

`ros2 control list_controllers` 가 `joint_trajectory_controller` 가 아니라
`hcr_arm_controller`(mock)를 돌려주는 것으로 확정했다. **계획이 성공하기 때문에 도메인
문제로 의심하기 어렵다** — README 10 절에 판별법과 함께 기록했다.

### 알아낸 것 ① — joint_1 · joint_6 눌림은 실제로 일어난다

`paper.scale: 0.001`(84 × 93 mm)로 그리면서 지령 대 실제를 샘플링했다.

| 관절 | 지령 | 실제 | 판정 |
|---|---|---|---|
| joint_1 | −7.07° ~ +7.61° | **−0.03°** ~ +6.82° | 한계에 눌림 |
| joint_6 | −0.15° ~ +0.17° | **−0.00°** ~ +0.17° | 한계에 눌림 |
| joint_2 | −8.74° ~ +6.94° | −9.14° ~ **+3.63°** | 추종 지연 (오차 14.96°) |

펜 끝 편차 **56.7 mm** — 그림 전체 폭의 67 %. 그런데 `fraction 1.00`, 실행
`SUCCEEDED` 였다. **로그로는 감지되지 않는다.**

지금까지 joint_1 만 언급했으나 **joint_6 도 같은 문제**임이 드러났다. 그리고 이전에
"동적 추종 지연"으로 해석한 joint_1 오차 3.8~4.0° 는 상당 부분이 눌림이었다 —
순수 추종 문제로 볼 것은 joint_2 · joint_3 이다.

### 알아낸 것 ② — 추종 오차는 그림 크기에 비례해 악화된다

joint_2 오차가 그림을 키우자 **5.33° → 14.96°** 로 커졌다.

### 알아낸 것 ③ — RViz 마커는 로봇 메시에 가린다

"마커가 안 그려진다"고 한참 헤맸는데, `MotionPlanning` 디스플레이를 끄니 바로 보였다.
자취는 펜 끝 위치에 그려지는데 거기 로봇 팔이 있다.

그리고 **그 초록 선은 "계획"이지 "실제"가 아니다.** 입력 좌표로 그려지므로 MuJoCo 가
못 따라가도 항상 멀쩡해 보인다 — 왜곡 판정에 쓸 수 없다. 실제로 이번에 자취는 정상인데
MuJoCo 는 56.7 mm 벗어나 있었다.

### 문서 갱신

- README 2 절 — RViz 마커 가림, 계획 vs 실제 구분
- README 8 절 — 수동 실증 결과, 마커 렌더링, 눌림 실측 행 추가
- README 9 절 — 눌림 실측 표, `fraction 1.00` 이어도 안전하지 않다는 경고
- README 10 절 — 도메인 사고 사례와 5 초 판별법
- README 11 절 — `Execute request aborted`, 마커 안 보임 항목 추가
- `docs/Pipeline Integration Status.md` — 위 내용 반영, 시뮬레이션팀 요청을 예측에서
  실측으로 교체

### 미해결로 남은 것

관절 한계·추종 오차 모두 그대로다. 다만 **근거가 예측에서 실측으로 바뀌어** 시뮬레이션팀에
넘길 수 있는 상태가 됐다.

---

## 2026-08-13 ④ — vision 토픽(`/vision/strokes`) 수용

### 배경

vision 담당자가 `/vision/strokes` 토픽 인터페이스를 공유했다. 기존에 쓰던
`map.csv`(mm, 파일)와 **다른 경로**다:

| | 기존 `map.csv` | 새 `/vision/strokes` |
|---|---|---|
| 전달 | 파일 | ROS 2 토픽 |
| 단위 | mm (A4 종이 좌표) | **픽셀** |
| 만드는 쪽 | `vision_core/stroke_map_cli` | `contour_pixel_node` (SAM3) |

vision 은 우리가 건드릴 수 없는 파트이므로 **moveit2 가 맞춘다.** 픽셀→mm 변환이
moveit2 로 오는 것은 F4.1(좌표 변환)이 원래 우리 담당이라 역할상으로도 맞다.

### 바꾼 것

**새 패키지 `ws_moveit2/src/vision_interfaces/`** — vision 정의의 **미러**.
moveit2 워크스페이스가 vision 패키지를 볼 수 없어서 같은 내용으로 다시 선언했다.
구조가 같으면 ROS 2 타입 해시가 같아져 통신이 된다. 어긋나면 **토픽은 보이는데
메시지가 0 개** 들어오므로, 해시 비교 방법을 그 패키지 README 에 적어 두었다.

**`src/draw_cat.cpp`**

- `strokes.source: "topic"` 추가 (기존 `params` · `csv` 는 그대로)
- `StrokeCollector` — 프레임 경계 판정. **메시지에 Header 도 프레임 번호도 없어서**
  받는 쪽이 판정해야 한다. 두 신호를 쓴다:
  - `instance_label` 중복 → 새 이미지 시작 (타이밍 무관, 가장 믿을 만함)
  - 마지막 수신 후 `frame_timeout_ms` 동안 조용하면 프레임 확정
  - 프레임이 여러 개 쌓였으면 **가장 최신 것**을 쓰고 나머지는 버린다(경고 출력)
- `computePaperFit()` — 픽셀→mm letterbox 변환. vision 의
  `vision_core/src/image_to_map.cpp` `ScaleToPaper` 와 **같은 식**이다
- `writeMapCsv()` — 변환 결과를 `map.csv` 로 저장. 토픽으로 받아도 simulation 의
  `trace_cli` 검증 경로가 끊기지 않게 하는 것이 목적

**파라미터 추가** — `strokes.topic` · `strokes.use_latched` ·
`strokes.frame_timeout_ms` · `strokes.wait_timeout_s` ·
`paper.width_mm` · `paper.height_mm` · `paper.margin_mm` · `output.map_csv_path`

### 왜 이렇게 했나 — 걸렸던 두 가지

**① "어느 이미지의 부위인지 모른다"**

메시지에 식별자가 없고 발행자가 `TRANSIENT_LOCAL` 이라, 늦게 구독하면 이전 이미지
부위와 새 이미지 부위가 섞인다.

→ **구독 durability 를 고를 수 있게 했다.** 발행자가 `TRANSIENT_LOCAL` 이어도
구독자는 `VOLATILE` 로 붙을 수 있다 — durability 는 "구독자가 덜 요구하는" 방향이
호환이다. 실측으로 확인했다:

```
발행자 TRANSIENT_LOCAL, 구독 전에 5 개 발행
  VOLATILE        구독자 → 밀린 이력 0 개, 발행자가 보는 구독자 수 1,
                          이후 새로 발행한 3 개는 정상 수신
  TRANSIENT_LOCAL 구독자 → 밀린 5 개 전부 수신
```

기본값은 `use_latched: false`(VOLATILE) — 섞일 여지가 없다. vision 이 한 번 쏘고
끝나는 운용이면 `true` 로 바꾼다. 그때도 프레임이 여러 개면 최신 것만 쓴다.

> vision 담당자 문서의 *"QoS 를 동일하게 맞춰야 한다"* 는 **반대 방향**
> (발행 VOLATILE + 구독 TRANSIENT_LOCAL)에만 해당한다.

**② "물리 스케일이 임의로 정해진다"**

픽셀만 오니 그림 크기를 사람이 정해야 한다고 봤다(`draw_size: 0.15` 같은 값).

→ **임의로 정할 필요가 없다.** vision 의 `ScaleToPaper` 는 축척 기준이 컨투어
bbox 가 아니라 **이미지 전체 크기(`src_size`)** 이고, `Stroke` 메시지가 바로 그
`image_width`/`image_height` 를 실어 보낸다. 용지 규격은 vision `params.hpp` 에
기본값이 있다(210 × 297, 여백 15 → 작화영역 180 × 267). 따라서 같은 계산을
재현하면 **`map.csv` 에 들어갔을 값과 동일한 mm 좌표**가 나온다.

```
scale    = min(180 / image_width, 267 / image_height)     [mm/px]
offset_x = 15 + (180 - image_width  * scale) * 0.5
offset_y = 15 + (267 - image_height * scale) * 0.5
x_mm = offset_x + u * scale        y_mm = offset_y + v * scale
```

용지 세 값은 파라미터로 빼 두었다 — vision 이 다른 값을 쓰면 YAML 만 고치면 된다.

### 확인한 것

스펙대로 발행하는 테스트 노드(`/tmp/fake_vision.py`, 부위 5 개 · 이미지 1280×960)로
전 구간을 돌렸다.

```
'/vision/strokes' 구독 중 (TRANSIENT_LOCAL) — 부위 5 개 수신
letterbox 0.14062 mm/px, 오프셋 (15.0, 81.0) mm   ← 손계산과 일치
  min(180/1280, 267/960) = 0.140625
  15 + (180 - 1280×0.140625)/2 = 15.0
  15 + (267 -  960×0.140625)/2 = 81.0
계획 지도 저장: map.csv
스트로크 0 'cat' 48 점 · 폐곡선 | fraction 1.00
스트로크 1~4 (눈·코·입) | fraction 1.00
===== 5/5 스트로크 실행 =====
```

그 `map.csv` 를 simulation 의 `trace_cli` 에 넣어 파일 검증 경로가 유지되는지 확인:

```
계획 5 개 → 실제 조각 5 개 | 길이 483.6 → 477.3 mm (98.7 %)
바운딩박스 (62.8, 97.5) ~ (147.2, 190.7) mm   ← 변환 결과와 일치
저장: data/out_vision/{drawn.svg, trace.csv, summary.txt}
```

**vision 토픽 → moveit2 변환 → map.csv → simulation 검증**까지 한 줄로 이어진다.

### 테스트 도구 (지울 것)

`test_tools/fake_vision_publisher.py` — vision 스펙대로 `/vision/strokes` 를 발행한다.
`ros2 run drawing_cat fake_vision_publisher` 로 실행되며, `frames` · `image_width` ·
`latched` 등을 파라미터로 바꿔 프레임 경계 판정과 QoS 동작을 직접 시험할 수 있다.

⚠️ **제품 코드가 아니다.** 지울 때는 세 곳: `test_tools/` 폴더, `CMakeLists.txt` 의
`# ── test_tools` 블록, `package.xml` 의 `<!-- test_tools -->` 주석이 붙은 줄.

`wait_timeout_s` 기본값을 30 → 120 초로 올렸다. 셸을 손으로 옮겨 다니며 vision 을
띄우기에 30 초는 빠듯했다 (실제로 테스트 중 두 번 놓쳤다).

### 남는 위험

- **`vision_interfaces` 가 두 곳에 존재한다.** vision 원본이 저장소에 병합되면 이
  미러를 지워야 한다. 그때까지는 정의가 갈라질 수 있다 — 타입 해시로 확인할 것
- vision 쪽 `expected_parts` 기본값(4)과 예시 부위 수(5)가 문서상 안 맞는데, 우리는
  개수에 의존하지 않으므로 영향 없다
- 메시지에 `closed` 필드가 없어 **항상 닫힌 윤곽**으로 처리한다. 열린 스트로크가
  필요해지면 vision 쪽 메시지 변경이 필요하다

---

## 2026-08-13 ③ — GPU override · 추종 오차 측정

### 바꾼 것

`src/simulation/compose.override.yml` 신설 — `/dev/dri` 패스스루. 이 파일이 없어
MuJoCo 뷰어가 llvmpipe(CPU 소프트웨어 렌더링)로 떨어지고 있었다
(`failed to load driver: iris`).

> ⚠️ **새 컨테이너가 생기는 것이 아니다.** 기존 `simulation` 서비스에 `devices` ·
> `group_add` 만 병합되는 오버레이다 (`./run_container.sh config` → 서비스 1 개).

### 확인한 것 — 처음 예상과 달랐다

GPU 가 오버런을 줄일 것으로 봤으나 **횟수는 전혀 안 줄었다.**

| | 30초당 오버런 중앙값 | Total time 중앙값 | loop 최대 |
|---|---|---|---|
| 헤드리스 | 0.0 회 | — | — |
| GUI + 소프트웨어 렌더링 | 29.0 회 | 2834 us | 20.24 ms |
| GUI + GPU | **29.0 회** | **2407 us** | **10.51 ms** |

원인은 소프트웨어 렌더링이 아니라 **뷰어를 띄운다는 사실 자체**다. GPU 가 바꾸는
것은 횟수가 아니라 **심각도**(최악 루프 시간 절반).

추종 오차는 세 조건 모두 같은 자리였다 — 헤드리스 5.330° · 소프트웨어 5.392° ·
GPU 5.912°. **렌더링 부하는 추종 실패의 원인이 아니다.**

### 추종 오차 측정 결과

`/joint_trajectory_controller/controller_state` 의 `error.positions` 샘플링.

| 구간 | 최대 | 평균 | joint_2 최대 |
|---|---|---|---|
| 정지 (자세 유지) | 0.17° | — | 0.16° |
| 홈 이동 (관절공간) | 2.57° | 1.26° | 0.39° |
| **고양이 외곽선 (Cartesian)** | **5.33°** | 2.76° | **5.33°** |

정지 시 0.16° → 그리는 중 5.33°. **중력 처짐이 아니라 동적 추종 지연**이고, 하필
그리는 동작에서 가장 심하다.

---

## 2026-08-13 ② — `map.csv` 수용 · 파라미터화 · `trace_cli` 연결

### 바꾼 것

- `config/draw_cat_params.yaml` 신설 — 하드코딩 값 전부 파라미터로. 재빌드 없이
  YAML 만 고치면 동작이 바뀐다
- `strokes.source: params | csv` — vision 의 `map.csv` 를 읽는 경로 추가.
  파싱 규칙은 `sim_core/src/planned_map.cpp` 와 **일부러 동일**하게 맞췄다
- 폐곡선 처리 — vision 규약상 첫 점을 끝에 중복해 넣지 않으므로 소비자가 닫는다
- `paper.auto_center` — 좌표 범위를 모를 때 경계상자 중심을 자동으로 맞춘다
- `pen.tip_offset_z` — 펜 끝 오프셋 자리 (기본 0.0 = 플랜지 기준)
- `rviz/draw_cat.rviz` — `PenTrace` 마커 디스플레이 포함
- `data/sample_cat_map.csv` — 96 점 폐곡선 샘플

### 고친 버그

`execute()` 반환값을 안 보고 있었다. 홈 이동이 실패해도 "이동 완료"를 찍고 계속
진행해서 이후 기준 자세가 전부 틀어졌다. 이제 홈 이동 실패는 종료, 스트로크 실패는
경고 후 다음으로 넘어가고, 마지막에 `5/5` 형태로 실제 실행 수를 보고한다.
`min_fraction`(기본 0.9) 미만이면 실행하지 않는다.

### 확인한 것

`trace_cli` 로 `ideal` 99.6 %, `weak-force` 에서 선 끊김 1 회 검출. 계획 길이가
**폐곡선의 닫는 구간까지 포함**한 값으로 나와, `closed` 플래그가 CSV 를 건너 그쪽
도구까지 전달되는 것이 교차 검증됐다.

---

## 2026-08-13 ① — MuJoCo 백엔드 연동

### 바꾼 것

- `launch/mujoco_moveit.launch.py` 신설 — `move_group` + `rviz2` 만 띄운다.
  `demo.launch.py` 는 mock_hardware 단독 데모용이라 몸통까지 띄워 시뮬과 5 곳이
  충돌한다
- `config/moveit_controllers_mujoco.yaml` — 컨트롤러 이름을
  `hcr_arm_controller` → `joint_trajectory_controller`. 실행 감시는 임시로 해제
- `config/joint_limits_mujoco.yaml` — joint_1 · joint_6 위치 한계 덮어쓰기
- `CMakeLists.txt` — `moveit2_msgs` → `moveit_msgs` 오타, `find_package(ament_cmake)`
  누락 수정
- `src/draw_cat.cpp` — Jazzy 에서 `Plan::trajectory_` → `trajectory` 로 바뀐 것 반영

`hanwha_robot_arm/` 은 건드리지 않았다 — 공용 자산이고 mock_hardware 데모가 계속
동작해야 한다. MuJoCo 전용 설정은 전부 이 패키지에 두고 절대경로로 주입한다.

### 막혔던 것 — joint_1 한계

`hcr_home` 이 joint_1 · joint_6 을 정확히 0(하한 위)에 두는데, MuJoCo 가 실제 물리를
풀어 `-1.06e-14` 를 돌려주면서 플래닝이 통째로 실패했다. mock_hardware 는 정확히
`0.0` 이라 절대 안 걸린다.

**시도했으나 안 되는 것들** (README 9절에도 기록)

| 시도 | 결과 |
|---|---|
| `ompl.fix_start_state: true` | ❌ continuous · planar · floating 관절만 정규화한다 |
| `CheckStartStateBounds` 제거 | ❌ OMPL 상태공간도 같은 한계로 만들어져 한 단계 뒤에서 실패 |

`robot_description_planning.joint_limits.<joint>.min_position` 덮어쓰기로 우회했다.
**MJCF 는 여전히 `range="0 6.28318"` 이라 플래닝만 풀린 상태다.**

### 확인한 것

컨테이너 간 DDS 로 `/joint_states` 500 Hz(σ=0.34ms) 수신, 액션 client/server 연결,
fraction 1.00 × 5 스트로크, 실행 6/6 SUCCEEDED.
