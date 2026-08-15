# 비전 ↔ moveit2 실물 연동 — 관통 결과

**2026-08-16 · 비전 마스크에서 로봇 동작까지 끝에서 끝까지 확인**

비전팀 `dy/vision` 코드로 실제 파이프라인을 돌렸다. 무엇이 검증됐고, 무엇이 막혔고,
그 과정에서 **인수 기준(AC2)에 대해 무엇이 확정됐는지**를 남긴다.

> 합의 지점 목록은 [`Vision Interface Contract.md`](Vision%20Interface%20Contract.md),
> 시간 최적화 근거는 [`Trajectory Optimization.md`](Trajectory%20Optimization.md).

---

## 1. 무엇을 돌렸나

```
[mask_replay]  →  /vision/parts  →  [contour_pixel]  →  /vision/strokes  →  [draw_cat]  →  로봇
  마스크 PNG 재생      MaskImage       비전팀 C++ 노드        Stroke          우리 노드
  (비전 컨테이너)                    (비전 컨테이너)                      (moveit2 컨테이너)
```

**SAM3 는 대신 세웠다.** 비전팀이 올린 `sam3_output/` 마스크 5 장을 그대로 발행하는
`test_tools/mask_replay.py` 를 만들어 썼다 — 발행자의 관찰 가능한 계약(토픽·타입·QoS·
버스트 방식·노드 수명)을 `sam3_extract_ros_node.py` 에서 그대로 옮겨 맞췄으므로 하류는
재생인지 진짜인지 구분할 수 없다. **`contour_pixel` 부터는 전부 비전팀 실제 코드다.**

| | 값 |
|---|---|
| 입력 이미지 | `cat_char.png` 480 × 538 px |
| 부위 | 5 개 — cat · eye_left · eye_right · nose · mouth |
| 점 개수 | 107 (cat 54 / mouth 17 / 나머지 각 12) |
| 백엔드 | `hcr_moveit_config/demo.launch.py` (mock hardware) |
| `draw_size_m` | 0.08 → 실제 그림 **67.4 × 80.0 mm** |

> mock 을 쓴 이유 — 지금 검증 대상이 **좌표와 메시지 경로**지 로봇 동역학이 아니다.
> MuJoCo 는 관절 한계 눌림과 추종 오차가 섞여 들어와 원인을 가릴 수 없다.

---

## 2. ✅ 검증된 것

| 항목 | 근거 |
|---|---|
| **컨테이너 간 통신** | 두 컨테이너의 노드 4 개가 맞물려 동작 |
| **메시지 타입 해시** | 실물 DDS 에서 일치. `RIHS01_d8c0b1c8…` — 미러 방식이 통한다는 실증 |
| **프레임 경계 판정** | 부위 5 개가 **32 ms 버스트**로 도착 → 한 프레임으로 묶임. `frame_timeout_ms: 1500` 이 충분히 안전 |
| **bbox 축척** | 444 × 527 px → **긴 변 80.0 mm** 정확. `paper.scale` 이 상쇄되는 설계도 실물 확인 |
| **종이 → 로봇 축 매핑** | RViz 자취가 **고양이 모양 그대로** — 90° 회전·거울상 아님 |
| **F3.1 순서 최적화** | 펜업 이동 **229.4 → 125.7 종이mm (45.2 % 감소)** |
| **전 스트로크 실행** | **5/5**, 모든 `fraction 1.00` (관절 한계 우회 적용 시) |

### 좌표 변환이 이번에 처음 검증됐다

`draw_strokes_node.cpp` 에서 읽은 규칙을 우리 코드에 반영한 것이 이번 실행으로 확인됐다:

```
종이 y (아래로 증가)  →  로봇 −X
종이 x (오른쪽으로)   →  로봇 +Y
```

> ⚠️ **이 오류는 `trace_cli` 로는 원리적으로 못 잡는다.** 그건 종이 좌표계에서 비교하므로
> 로봇 쪽 축이 어떻게 배치되든 항상 멀쩡해 보인다. **RViz 자취(로봇 좌표계)로만 드러난다.**

---

## 3. 🔴 관절 한계 — **mock 에서도 재현됐다**

`cat` 외곽선만 `fraction 0.33` 으로 실행되지 않았다. 원인을 좁힌 과정:

| 시도 | 결과 |
|---|---|
| `draw_size` 0.15 → 0.08 (절반 이하) | ❌ **fraction 이 그대로 0.33** — 크기 문제가 아니다 |
| `joint_limits.yaml` 에 위치 한계 ±2π 추가 | ✅ **fraction 1.00, 5/5** |

### 왜 크기를 줄여도 소용없나

```xml
<!-- hcr_robot.xacro — joint_1 -->
<limit upper="6.283185" lower="0.0" .../>
```

`hcr_home` 의 `joint_1 = 0` 이 **정확히 하한 위**다. 그래서 joint_1 을 음수로 보내야 하는
경로는 **출발하는 순간 막힌다.** 장벽이 "멀리"가 아니라 **출발점 그 자리**에 있으므로,
그림을 줄여도 윤곽선의 같은 상대 위치에서 걸린다 — fraction 이 축척과 무관하게 고정되는
이유가 이것이다.

### 이번 실행의 새로운 가치

**지금까지 이 문제는 MuJoCo 에서만 관찰됐다.** 그래서 "시뮬레이터 쪽 문제 아니냐" 는
여지가 남아 있었는데, **mock hardware 에서 동일하게 재현됐다.**

**→ 시뮬레이터와 무관한 로봇 기술(URDF) 문제라는 것이 확정됐다.**
[`Pipeline Integration Status.md`](Pipeline%20Integration%20Status.md) §5 의 시뮬레이션팀
요청(“두 관절 한계를 ±대칭으로 고치고 MJCF 재변환”)의 근거가 하나 더 늘었다.

> ⚠️ 확인에 쓴 `joint_limits.yaml` 수정은 **다른 팀 영역이라 되돌렸다.** 우리 쪽
> `mujoco_moveit.launch.py` 는 `drawing_cat/config/joint_limits_mujoco.yaml` 로 같은
> 우회를 이미 갖고 있지만, `demo.launch.py`(mock)에는 없다. mock 으로 계속 시험하려면
> 우리 패키지에 mock 용 런치 파일을 만드는 편이 낫다.

---

## 4. 🔴 AC2 — 실제 데이터에서 사실상 불가능하다

**이번 실행의 가장 중요한 발견이다.**

```
펜업 이동   0.14 s    3.0%   (5 회)
그리기      4.57 s   97.0%   (5 회)   ← cat 외곽선 하나가 3.11 s = 전체의 66 %
─────────────────────────
이동+그리기 4.71 s
실측(벽시계) 5.27 ~ 5.31 s   (계획 대비 +12 ~ 13 %)
```

계획 시간은 두 번 돌려 **소수점까지 동일**했다 (0.14 / 4.57 / 4.71).

### 펜업 비율이 계속 떨어져 왔다

| 데이터 | 펜업 비율 |
|---|---|
| 합성 데이터, 최적화 전 | 15.8 % |
| 합성 데이터, 최적화 후 | 4.8 % |
| **실제 비전 데이터, 5/5** | **3.0 %** |

**F3.1 이 펜업 *거리* 를 45.2 % 줄였는데도 전체 시간 기여는 1 % 남짓이다.**

AC2 판정식은 `(travel_before_s − travel_after_s) / travel_before_s ≥ 0.20` 이라 **펜업
시간 기준으로는 통과할 수 있다.** 다만 그것이 **전체 작업 시간에 주는 영향은 1 % 미만**
이라는 사실이 실물로 확인됐다. 팀이 AC2 의 의도를 "전체 시간 단축" 으로 봤다면 그 목표는
순서 최적화로 달성할 수 없다.

### 시간을 지배하는 것은 점 개수다

`cat` 54 점 → **3.11 s**, 나머지 12 ~ 17 점 → 각 0.33 ~ 0.45 s.

점 개수를 정하는 것은 비전의 `epsilon_ratio`(기본 0.01, cat 전용 0.0025)다. **우리 쪽
파라미터가 아니다.** 시간을 더 줄이려면 이 값이나 그리기 속도 배율을 봐야 한다.

---

## 5. 재현 절차

컨테이너 두 개, 셸 5 개. `contour_pixel`(셸 3)과 RViz(셸 2)는 한 번 띄우면 그대로 둔다.

```bash
# 준비 — 호스트
cp src/moveit2/ws_moveit2/src/drawing_cat/test_tools/mask_replay.py \
   <dy/vision worktree>/src/vision/ws_vision/

# 셸 1 — moveit2 컨테이너 : 백엔드
ros2 launch hcr_moveit_config demo.launch.py

# 셸 2 — moveit2 컨테이너 : RViz (PenTrace 디스플레이 포함)
rviz2 -d $(ros2 pkg prefix drawing_cat --share)/rviz/draw_cat.rviz

# 셸 3 — 비전 컨테이너 : 컨투어 추출
ros2 run vision_node contour_pixel

# 셸 4 — moveit2 컨테이너 : 그리기
ros2 launch drawing_cat draw_cat.launch.py params_file:=<설정>

# 셸 5 — 비전 컨테이너 : 마스크 재생 (셸 4 가 "구독 중" 을 찍은 뒤 120 초 안에)
python3 ~/ws_vision/mask_replay.py
```

### ⚠️ 걸렸던 함정 넷 — 다시 겪지 말 것

| 증상 | 원인 |
|---|---|
| yaml 을 고쳤는데 동작이 그대로 | `--symlink-install` 은 **설정 파일만** 즉시 반영한다. **C++ 바이너리는 재빌드해야 한다.** 확인: `strings <바이너리> \| grep scale_mode` |
| `strings` 로 확인했는데 안 잡힘 | `strings` 는 기본이 ASCII 전용이라 **한글 문자열은 안 나온다.** 영문 식별자로 확인할 것 |
| RViz 에 자취가 안 보임 | `demo.launch.py` 의 RViz 에는 **Marker 디스플레이가 없다.** `Add → By topic` 은 `draw_cat` 이 살아 있을 때만 목록에 뜬다 — `By display type` 으로 추가하거나 우리 `draw_cat.rviz` 를 쓸 것 |
| 자취가 보이는데 실행은 4/5 | **마커는 실행 여부와 무관하게 발행된다.** 화면은 "계획" 이지 "실제" 가 아니다 — 판단은 로그의 `?/5 스트로크 실행` 으로 |

---

## 6. 남은 것

| | 무엇 | 비고 |
|---|---|---|
| **①** | 실제 SAM3 로 재확인 | 하류는 전부 검증됐으므로 **`SAM3 → contour_pixel` 한 고리만** 보면 된다 |
| ② | 마스크 품질 변동 | 재생으로 못 덮는 유일한 항목 — 구멍·여러 덩어리일 때 `contour_pixel` 이 "가장 큰 것 하나" 만 쓰는 경로 |
| ③ | mock 용 런치 파일 | 관절 한계 우회를 포함한 것을 `drawing_cat` 에 두면 매번 공유 설정을 건드리지 않아도 된다 |
| ④ | AC2 재논의 | 위 §4 수치를 팀에 가져갈 것 |
| ⑤ | URDF 관절 한계 수정 | 시뮬레이션팀. **mock 재현으로 근거가 강화됐다** |
