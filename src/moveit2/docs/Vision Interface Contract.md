# 모듈 경계 — 합의해야 하는 지점 전부

**2026-08-14 작성 · 2026-08-15 `dy/vision` 브랜치 실물 코드 확인 후 대폭 갱신**

moveit2 가 다른 모듈과 **같은 값을 알고 있어야 하는 곳**을 한 곳에 모았다. 어긋나면
대부분 **에러 없이 그림만 틀어지는** 종류라, 코드가 아니라 표로 맞춰 두는 편이 빠르다.

> ## 2026-08-15 갱신 — 추측이 아니라 코드로 확인했다
>
> 비전팀이 `dy/vision` 브랜치를 올려서 **실제 구현을 직접 읽었다.** 이전 판에서
> "문서와 답변이 모순" 이라고 쓴 것은 **틀렸다** — 모순이 아니었다.
> 🔴 로 표시했던 세 개 중 **셋 다 해소**되었고, 대신 **분담 문제**가 새로 드러났다.

## 출처

| | 무엇 |
|---|---|
| **V-CODE** | `origin/dy/vision` 브랜치 — **실제 구현.** 이게 가장 믿을 만하다 |
| V-DOC | `src/vision/README.md` (2026-07-28) — `vision_core` 라이브러리 명세 |
| V-ANS | 2026-08-14 비전팀 답변 |
| M-IMP | 우리 구현 — `drawing_cat/` |

---

## 0. 먼저 — 무엇이 해소되었나

| 이전 판의 주장 | 실제 |
|---|---|
| "축척 규칙이 letterbox vs bbox 로 **모순**" | ❌ 모순 아님. **비전은 축척을 아예 안 한다** — 순수 픽셀만 발행하고, mm 변환은 **소비자 몫**이다 |
| "메시지 타입 해시가 미확인" | ✅ **일치 확인.** 양쪽 정의를 빌드해 생성물을 바이트 비교 |
| "`closed` 정보가 누락되어 열린 스트로크를 억지로 닫는다" | ✅ 문제 없음. `approxPolyDP(..., true)` 라 **항상 닫힌 컨투어**다 |
| "종이 → 로봇 방향이 불명" | ✅ **정확한 식 확보** (아래 §2.2) |

**남은 진짜 문제는 코드가 아니라 분담이다** (§2.0).

---

## 1. 한눈에 보기

| # | 합의 지점 | 상태 |
|---|---|---|
| **0** | **같은 일을 하는 노드가 둘** | 🔴 **분담 결정 필요** — 가장 급함 |
| 1 | 픽셀 → mm 축척 규칙 | ✅ 해소. 소비자가 정한다. 그들 값은 bbox 긴변 → `draw_size` 0.15 m |
| 2 | 종이 mm → 로봇 XY 방향 | ✅ 식 확보. **우리 구현을 고쳐야 한다** |
| 3 | `closed` 전달 | ✅ 불필요. 항상 닫힌 컨투어 |
| 4 | 메시지 타입 해시 | ✅ **일치 확인** (RIHS01_d8c0b1c8…) |
| 5 | 점 개수 손잡이 | 🟡 `epsilon_ratio` (0.01 / cat 0.0025). **우리 실행시간의 95 % 를 좌우** |
| 6 | 목표 그림 크기 | 🟡 그들 기본값 `draw_size: 0.15 m`. 합의된 값인지 확인 필요 |
| 7 | 용지 규격 210×297, 여백 15 | ⚠️ **토픽 경로에서는 무의미** — 종이 개념이 없다 |
| 8 | 좌표 원점 = 종이 좌상단 | ✅ 픽셀 좌상단 그대로 |
| 9 | 스트로크 간 순서 없음 · F3.1 은 moveit2 | ✅ 일치 |
| 10 | 첫 점 중복 없음 | ✅ 일치 |
| 11 | `map.csv` 형식 | ✅ 일치 (파일 경로 한정) |
| 12 | 프레임 경계 = 시간 기준 | ✅ 합의 |
| 13 | 발행 QoS / 대기 시간 | ✅ 비전이 수정 예정 |
| 14 | `ROS_DOMAIN_ID` | 🟡 `run_container.sh` 에 export 없음 |
| **15** | **AC2 "이동시간 20 % 단축" 정의** | 🔴 **해석에 따라 통과/미달** (§2.5) |
| 16 | operator `PlanSummary` 보고 | 🔴 미구현 |
| 17 | 진행 보고·상태머신 (F4.3·F4.4) | 🔴 미구현 |
| 18 | 종이 → 로봇 변환의 주인 = 캘리브레이션 | 🟡 양쪽 다 "현재 자세 = 원점" 으로 임시 대체 중 |
| 19 | 종이 평면 z | 🟡 위와 같음 |
| 20 | N2 = 900 초 예산 | 🟡 실측 필요 |
| 21 | `instance_label` 유일성 | 🟡 미확인 |
| 22 | 픽셀 `int32` | ✅ 애초에 정수다. 문제 없음 |
| **23** | **`vision_interfaces` 경로 소유권** | 🔴 **양쪽이 같은 경로에 올렸다** |
| **24** | **마스크 하나당 가장 큰 컨투어만** | 🟡 부위가 두 덩어리면 하나가 사라진다 |

---

## 2.0 🔴 같은 일을 하는 노드가 둘이다 — **가장 급한 것**

| | **그들** `hello_moveit/src/draw_strokes_node.cpp` (409줄) | **우리** `drawing_cat/src/draw_cat.cpp` (910줄) |
|---|---|---|
| `/vision/strokes` 구독 | ✅ | ✅ |
| `hcr_home` 이동 후 원점 | ✅ | ✅ **같은 방식** |
| 픽셀 → 로봇 | bbox 긴변 → `draw_size` 0.15 m | 이미지 letterbox → A4 작화영역 |
| 좌표 방향 | **스왑 + v 음부호** | 스왑·부호 **없음** ← 고쳐야 함 |
| Cartesian `eef_step` | 0.002 | 0.002 |
| 펜업 높이 | `hover` 0.03 | `pen_lift` 0.008 |
| 속도/가속 | 0.1 / 0.1 | 그리기 0.1 · **펜업 가속 1.0** |
| **F3.1 순서 최적화** | ❌ | ✅ |
| **시간 계측** | ❌ | ✅ |
| **단위시험** | ❌ | ✅ `colcon test` |
| 입력 경로 | 토픽만 | 토픽 · csv · params |
| 패키지 | **`hello_moveit`** (MoveIt 튜토리얼 패키지) | 전용 패키지 |

> **그들 노드가 `hello_moveit` 안에 있다는 것이 성격을 말해 준다.** 전용 패키지를 만들지
> 않았다 — **비전 출력이 제대로 나오는지 확인하려고 만든 임시 하네스**로 읽는 것이 자연스럽다.

**정한 방향 (2026-08-15):**

- **vision 파트는 그대로 간다** — `contour_pixel_node` · SAM3 · 메시지 정의는 손대지 않는다
- **moveit2 파트는 우리 코드를 토대로** 하고, 그들 구현에서 **좌표 변환 규칙만 가져온다**

가져올 것은 아래 §2.1 · §2.2 두 개다. 나머지(F3.1 · 계측 · 단위시험 · 입력 경로 셋)는
우리 쪽에만 있으므로 그대로 유지한다.

### 🔴23. `vision_interfaces` 경로가 겹친다

양쪽 다 `src/moveit2/ws_moveit2/src/vision_interfaces/` 에 올렸다. **내용은 동일**
(해시 일치 확인) 하지만 그들 것에는 **`MaskImage.msg` 가 추가**되어 있다.

병합하면 한쪽이 덮어쓴다. **누가 소유할지 정해야 한다** — 비전팀이 원본을 가지고
moveit2 는 미러로 두는 지금 구조를 유지하되, **`MaskImage.msg` 를 포함한 최신본으로
통일**하는 것이 자연스럽다.

---

## 2.1 ✅ 축척 — 비전은 축척을 하지 않는다

**이전 판의 "letterbox vs bbox 모순" 은 틀렸다.** 실제 구현을 보면 모순이 아니다.

`contour_pixel_node.cpp` (비전 컨테이너) 가 하는 일 전부:

```
MaskImage 수신 → cv::findContours → 가장 큰 컨투어 → approxPolyDP → 픽셀 좌표 발행
```

**mm 변환이 없다.** `image_width`/`image_height` 를 메시지에 실어 보내지만 자기는 안 쓴다
(V-ANS 의 "이미지 크기는 계산에 쓰이고 있지 않다" 가 이 뜻이었다).

그래서 축척은 **소비자가 정한다.** 그들 소비자(`draw_strokes_node`)는:

```cpp
// 모든 부위를 합친 bbox
const double s = draw_size_ / std::max(w_px, h_px);   // draw_size 기본 0.15 m
```

| | 규칙 | 기준 |
|---|---|---|
| `map.csv` 경로 | letterbox (V-DOC) | `vision_core::ScaleToPaper` — A4 작화영역 |
| **토픽 경로** | **소비자 자유** | 비전은 픽셀만 준다 |

> ⚠️ **그래서 우리 `computePaperFit`(A4 letterbox)이 틀린 게 아니다.** 두 소비자가 다른
> 규칙을 고른 것뿐이다. 다만 **같은 로봇으로 같은 그림을 그리는데 크기가 달라지면 곤란**하므로
> 규칙을 통일해야 한다.

> ⚠️ **토픽 경로에는 "종이" 개념이 없다.** `paper.width_mm` · `margin_mm` 같은 우리
> 파라미터는 A4 를 전제하는데, 비전은 종이를 모르고 픽셀만 준다. bbox 방식으로 바꾸면
> 이 파라미터들은 토픽 경로에서 **쓰이지 않게 된다.**

**정할 것:** `draw_size` 를 얼마로 할지. 그들 기본값은 **0.15 m(15 cm)** 다.

---

## 2.2 ✅ 종이 → 로봇 좌표 — 정확한 식을 확보했다

```cpp
// draw_strokes_node.cpp
// 이미지 u(가로) -> 로봇 Y축(좌우)
// 이미지 v(세로) -> 로봇 X축(전후) : 이미지 v는 아래로 증가하므로 - 부호 적용
q.position.x = origin.position.x - (p.v - v_c) * s;
q.position.y = origin.position.y + (p.u - u_c) * s;
q.position.z = origin.position.z + dz;
```

**스왑 + `v` 에만 음부호.** `u` 는 양부호다.

우리 `flangePose()` 는 스왑도 부호반전도 없다:

```cpp
p.x = px0 + (mx_mm - center_x_mm) * scale - tip_offset.x;   // 종이 x → 로봇 +X
p.y = py0 + (my_mm - center_y_mm) * scale - tip_offset.y;   // 종이 y → 로봇 +Y
```

**결과적으로 90° 회전 + 거울상이 된다. 우리 쪽을 고쳐야 한다.**

> 원점 잡는 방식은 양쪽이 **완전히 같다** — `hcr_home` 으로 이동한 뒤 `getCurrentPose()`.
> 우리 `paper.use_current_pose_as_origin: true` 와 동일한 임시 처리이고, 캘리브레이션(F5)
> 이 들어오면 양쪽 다 바뀔 자리다 (§2.7).

> ⚠️ 이 방향이 **실제 설치와 맞는지는 아직 아무도 종이에 그려보지 않았다.** 예제
> `draw_contour.cpp` 의 관찰자 규약을 따른 것이므로, 실물에서 한 번 확인이 필요하다.

---

## 2.3 ✅ `closed` — 문제 없다

```cpp
cv::approxPolyDP(*largest, approx, epsilon, true);   // ← true = 닫힌 곡선
```

마스크 기반이라 **컨투어는 언제나 닫혀 있다.** 우리가 토픽 경로에서 `closed = true` 로
두는 것이 맞다. `Stroke.msg` 에 `closed` 필드가 없어도 정보 손실이 아니다.

> 나중에 centerline(thinning) 방식이 들어오면 열린 폴리라인이 나올 수 있다. 그때 다시 본다.

### 🟡24. 마스크 하나당 **가장 큰 컨투어 하나만** 쓴다

```cpp
auto largest = std::max_element(contours.begin(), contours.end(),
    [](const auto & a, const auto & b) { return cv::contourArea(a) < cv::contourArea(b); });
```

부위가 **두 덩어리로 나뉘면 작은 쪽이 조용히 사라진다.** 수염처럼 좌우로 떨어진 부위나,
마스크가 끊긴 경우가 해당된다. 지금 데이터(고양이 얼굴 5부위)에서는 문제가 없지만
알아 둘 것.

---

## 2.4 ✅ 타입 해시 — 일치 확인

양쪽 정의를 각각 빌드해 생성된 타입 기술 파일을 **바이트 단위로 비교**했다 (2026-08-15).

```
Stroke.json      바이트 단위로 완전 동일
PixelPoint.json  바이트 단위로 완전 동일

RIHS01_d8c0b1c8ffdbe600b7b59775cc2460b82c96be241faf49fea80c7393e77506d7
```

필드가 동일하고 주석만 다르다 — 주석은 해시에 들어가지 않는다. **우리 미러가 맞게
만들어졌다는 것이 실측으로 확인되었다.**

---

## 2.5 🔴 AC2 — 정의에 따라 통과와 미달이 갈린다

**이 항목은 이번 갱신으로 바뀌지 않았다. 여전히 가장 중요한 미결 사항이다.**

`src/moveit2/README.md`:

> F3.1 스트로크 순서 최적화 — **AC2: 무최적화 대비 이동시간 20 % 이상 단축**

`src/operator/README.md` 가 코드로 못박은 판정식:

```cpp
struct PlanSummary { int stroke_count; double draw_len_mm,
                     travel_before_s, travel_after_s; };   // F3.3 → AC2
bool MeetsAc2(const PlanSummary &, double th = 0.20);
```

`(travel_before_s − travel_after_s) / travel_before_s ≥ 0.20` — **펜업 이동 시간** 기준이다.

| `travel_after_s` 를 무엇으로 보나 | 값 | 비율 | 판정 |
|---|---|---|---|
| **NN 순서만** (F3.1 문자 그대로) | 0.57 s | **1.7 %** | ❌ **미달** |
| 순서 + 폐곡선 회전 | 0.51 s | **12.1 %** | ❌ **미달** |
| 순서 + 회전 + 이동 가속 상향 | 0.16 s | **72.4 %** | ✅ 통과 |

(`travel_before_s` = 0.58 s. 근거: [`Trajectory Optimization.md`](Trajectory%20Optimization.md))

**AC2 는 F3.1 항목에 달려 있는데 순서 최적화만으로는 1.7 % 다.** 20 % 를 넘기는 것은
가속 배율이고 그건 순서 최적화가 아니다.

> ⚠️ **구현이 부족해서가 아니다.** 펜업 이동이 실제 4.5 ~ 24 mm 라 순항 구간 없이
> 가감속만 하고, 시간이 거리에 `√` 로만 붙는다. **AC2 를 세울 때 이 물리를 몰랐던 것**이다.

**정할 것:** *무최적화* 가 ① F3.1 만 끈 상태인지 ② moveit2 의 이동 최적화를 전부 끈
상태인지. ②로 보면 72.4 % 로 통과한다. 어느 쪽이든 **데이터는 다 있다.**

---

## 2.6 🔴 operator 가 기대하는 것 — 전부 미구현

| 기대 | 우리 현황 |
|---|---|
| `PlanSummary { stroke_count, draw_len_mm, travel_before_s, travel_after_s }` | ❌ 로그로만 출력 |
| `strokes_done`, `failed_strokes[]`, `aborted_at_stroke` | ❌ 보고 안 함 |
| `Command { Start, EmergencyStop, Pause, Resume, … }` 수신 | ❌ 없음 |
| `Event { PlanReady, StrokeDone, DrawingDone, … }` 발행 | ❌ 없음 |

moveit2 README 의 **F4.3(진행 보고)** · **F4.4(실패 처리)** 가 이것인데 착수 전이다.

> ⚠️ **`travel_before_s` 는 지금 구조로 못 낸다.** 우리는 최적화 전/후의 **거리**(mm)를
> 재는데 AC2 가 요구하는 것은 **초**다. 최적화 전 순서로도 **궤적을 계획**해서 그
> duration 을 재야 한다 (실행은 불필요, 계획만 12 ~ 50 ms).

---

## 2.7 🟡 종이 → 로봇 변환의 주인은 캘리브레이션(F5)

V-DOC:

> - z 없음 — **z 는 moveit2 쪽(F4.1)이 캘리브레이션으로 붙인다**
> - calibration(F5)은 **moveit2 가 이 지도를 로봇 좌표로 옮기는 데만 쓰인다**

**양쪽 구현이 똑같이 임시 처리 중이다** — `hcr_home` 자세의 현재 pose 를 원점으로 삼는다.
그래서 종이가 로봇 앞 **어느 위치·방향·높이**에 놓이는지 아무도 정하지 않았다.

V-DOC 에 남은 열린 질문:

> F5.2용 마커가 **지그에 고정**되어 있는지 확인 필요 (종이 쪽에 있으면 "F5.2는 1회"
> 라는 전제가 무너짐)

---

## 3. 🟡 나머지 미확인

### 🟡5. 점 개수 손잡이는 `epsilon_ratio` 다

V-DOC 의 `resample_step_mm` 이 아니라, 토픽 경로에서 실제로 점 개수를 정하는 것은
`contour_pixel_node` 의 `approxPolyDP` epsilon 이다:

```cpp
double current_epsilon_ratio = default_epsilon_ratio_;   // 0.01
if (label == "cat") current_epsilon_ratio = cat_epsilon_ratio_;   // 0.0025
double epsilon = current_epsilon_ratio * cv::arcLength(*largest, true);
```

**우리 전체 실행 시간의 95 % 가 그리기**이고 그 시간은 점 개수에 직결된다
([`Trajectory Optimization.md`](Trajectory%20Optimization.md)). 값을 우리가 정하자는 게
아니라, **이 손잡이가 시간에 직결된다는 사실을 공유**하는 것이다.

> `approxPolyDP` 는 균일 간격 재샘플링이 아니라 **다각형 근사**다. 직선 구간은 점이
> 성기고 곡률이 큰 곳은 촘촘해진다 — Cartesian waypoint 밀도가 불균일해진다는 뜻이다.

### 🟡21. `instance_label` 유일성

우리는 메시지에 `Header` 가 없어서 **라벨 중복을 "새 이미지 시작" 신호로** 쓴다. 한
이미지 안에 같은 라벨이 두 번 오면 프레임이 쪼개지고 앞부분을 버린다. SAM3 프롬프트가
라벨의 출처이므로 (`metadata.json` 참조) 중복 가능성을 확인해야 한다.

### 🟡14. `ROS_DOMAIN_ID`

`run_container.sh` 가 다른 변수는 다 export 하면서 이것만 빠뜨렸다. 2026-08-14 에 실제로
도메인 0 으로 생성돼 엉뚱한 컨테이너에 붙었다.
[`Pipeline Integration Status.md`](Pipeline%20Integration%20Status.md) §5.

### 🟡20. N2 = 900 초 예산

우리 실측 3.34 초는 **40 × 45 mm** 짜리다. `draw_size` 0.15 m 로 그리면 선 길이가 크게
늘어난다. 실제 크기로 재야 N2 여유를 말할 수 있다.

---

## 4. ✅ 이미 맞는 것

| 항목 | 내용 |
|---|---|
| 메시지 정의 | 타입 해시 일치 확인 |
| 책임 분담 | 스트로크 **간** 순서 없음. F3.1·F3.2 는 moveit2 |
| 폐곡선 | 첫 점을 끝에 중복하지 않음. 닫는 구간은 소비자가 만듦 |
| `closed` | 항상 닫힌 컨투어라 필드가 없어도 무방 |
| 원점 처리 | 양쪽 다 `hcr_home` 현재 자세 기준 (임시) |
| `eef_step` | 양쪽 0.002 |
| `map.csv` | `stroke_idx,point_idx,x_mm,y_mm,closed` — moveit2·simulation 동일 파서 |
| 프레임 경계 | 라벨 중복 + 무음 시간 판정 |
| SRDF 홈 자세 | `hcr_home` (그룹 `hcr_arm`). SRDF 에 정의된 `group_state` 는 이것 하나 |
| 픽셀 정수 | 애초에 `cv::Point` 정수라 손실 없음 |

---

## 5. 남은 질문 — 그대로 복사해서 쓰면 된다

### 비전팀에

> **1. `draw_strokes_node`(hello_moveit)는 확인용 임시 노드로 봐도 될까요?**
> 저희가 `drawing_cat` 패키지로 같은 일을 하고 있어서, moveit2 쪽은 저희 것을 토대로
> 하고 **좌표 변환 규칙(bbox 축척 + 스왑/음부호)만 가져오려** 합니다. 그렇게 진행해도
> 괜찮을까요?
>
> **2. `vision_interfaces` 를 어느 쪽이 소유할까요?**
> 양쪽이 `src/moveit2/ws_moveit2/src/vision_interfaces/` 에 올렸습니다. 내용은 동일한데
> (해시 일치 확인) 그쪽에 `MaskImage.msg` 가 더 있습니다. **그쪽 최신본으로 통일**하고
> 저희는 미러로 따라가는 게 맞을 것 같습니다.
>
> **3. `draw_size` 기본값 0.15 m 가 합의된 값인가요?**
> 아니면 실물 종이 크기가 정해지면 바뀌는 값인가요?
>
> **4. `instance_label` 이 한 이미지 안에서 유일한가요?**
> 저희는 `Header` 가 없어서 **라벨 중복을 "새 이미지 시작" 신호로** 씁니다. 같은 라벨이
> 두 번 오면 프레임이 쪼개지고 앞부분을 버립니다 — 에러 없이 그렇게 됩니다.
>
> **5. `epsilon_ratio` 를 조정할 여지가 있을까요? (판단은 비전팀 몫)**
> 실측 결과 저희 전체 실행 시간의 **95 % 가 그리기**이고 점 개수에 직결됩니다. 선
> 품질과의 균형은 비전팀이 아실 것 같아 **값을 제안하는 게 아니라 이 손잡이가 시간에
> 직결된다는 사실만 공유**드립니다.
>
> **6. 부위가 두 덩어리로 나뉘면 작은 쪽이 사라지는데 괜찮을까요?**
> `contour_pixel_node` 가 마스크당 가장 큰 컨투어 하나만 씁니다. 지금 데이터에서는
> 문제없지만 수염처럼 좌우로 떨어진 부위에서 걸릴 수 있습니다.

### 팀 전체에

> **7. AC2 의 "무최적화 대비 이동시간 20 %" 에서 *무최적화* 의 범위를 정해 주세요.**
> 순서 최적화(F3.1)만으로는 **1.7 %**, 저희 이동 최적화를 전부 포함하면 **72.4 %** 입니다.
> 정의에 따라 통과와 미달이 갈립니다 (§2.5 에 근거 전부 있음).
>
> **8. operator 가 기대하는 `PlanSummary` 를 언제 구현할지 정해 주세요.** (§2.6)
>
> **9. 캘리브레이션 착수 전까지 종이 위치를 어떻게 가정할까요?**
> 지금은 **양쪽 구현 모두** `hcr_home` 자세를 원점으로 삼는 임시 처리입니다. 종이가
> 로봇 앞 어느 위치·방향·높이에 놓이는지 정해진 것이 없습니다 (§2.7).

---

## 참고

- 우리 구현 상세: [`drawing_cat/README.md`](../ws_moveit2/src/drawing_cat/README.md)
- 파이프라인 현황: [`Pipeline Integration Status.md`](Pipeline%20Integration%20Status.md)
- 시간 측정 결과: [`Trajectory Optimization.md`](Trajectory%20Optimization.md)
- 비전 실물 코드: `origin/dy/vision` 브랜치
- 비전 명세: `src/vision/README.md`
- 미러 패키지 주의사항: [`vision_interfaces/README.md`](../ws_moveit2/src/vision_interfaces/README.md)
