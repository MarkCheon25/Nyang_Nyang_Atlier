# `hcr5_viz` — 펜 끝 자취 시각화

TF 에서 **펜 끝(`pen_tip`)** 을 따라가며 지나간 자취를 RViz 에 선으로 그린다.
**로봇을 제어하지 않는 읽기 전용 노드**다.

## 왜 필요한가

지금까지는 데카르트 경로가 잘 나왔는지를 **달성률 숫자**로만 판단했다.
그런데 그리기의 결과물은 결국 "펜이 지나간 선" 그 자체다. 그것이 보이지 않으면:

- 사각형이 진짜 사각형인지 (모서리가 뭉개지지 않았는지) 알 수 없다
- A4 크기로 키웠을 때 **어디서** 일그러지는지 알 수 없다
- 계획한 도형과 실제 자취가 얼마나 벌어지는지 잴 수 없다 ← **실기 선 품질 지표**

## 실행

```bash
# 터미널 1 — MoveIt
ros2 launch hcr5_moveit_config demo.launch.py

# 터미널 2 — 자취 시각화
ros2 run hcr5_viz pen_trail

# 터미널 3 — 도형 그리기
ros2 launch hcr5_examples example.launch.py example:=cartesian_square
```

> **launch 파일이 없는 이유** — 이 노드는 파라미터를 전부 기본값과 함께 선언하고
> TF 만 읽으므로 외부 주입이 필요 없다. launch 를 두면 기본값이 두 곳에 중복되어
> 한쪽만 고쳤을 때 조용히 어긋난다. `hcr5_examples/example.launch.py` 는
> robot_description 등을 **반드시** 주입해야 해서 launch 가 필수인 경우이고,
> 여기는 그렇지 않다.

## RViz 설정

**demo.launch.py 로 띄웠다면 손댈 것이 없다.** `moveit.rviz` 에 MarkerArray 디스플레이가
`펜 자취 (실제)` 라는 이름으로 이미 등록돼 있다 (2026-08-09 추가).

RViz 를 따로 띄웠거나 디스플레이가 없다면:
`Add` → `By topic` → **`/pen_trail/trail`** → `MarkerArray`

| 마커 | 색 | 뜻 |
|---|---|---|
| `pen_trail` (LINE_STRIP) | 빨강 | 펜 끝이 **지나간 자취** |
| `pen_tip` (SPHERE) | 노랑 | **현재** 펜 끝 위치 |

## 자취 지우기

도형을 바꿔가며 실험할 때 이전 선이 겹치지 않게:

```bash
ros2 service call /pen_trail/clear std_srvs/srv/Empty
```

## 파라미터

전부 `--ros-args -p` 로 바꿀 수 있다. **코드를 고칠 필요가 없다.**

| 인자 | 기본 | 뜻 |
|---|---|---|
| `base_frame` | `base_link` | 자취를 그릴 기준 좌표계 |
| `tip_frame` | `pen_tip` | 따라갈 프레임 |
| `min_point_distance` | `0.002` | 이만큼 움직여야 점을 찍는다 [m] |
| `max_points` | `20000` | 최대 점 개수 (넘으면 오래된 것부터 버림) |
| `publish_rate` | `30.0` | 발행 주기 [Hz] |
| `line_width` | `0.002` | 선 굵기 [m] |
| `tip_size` | `0.008` | 현재 위치 구슬 지름 [m] |

```bash
# 더 촘촘하게 (곡선을 볼 때)
ros2 run hcr5_viz pen_trail --ros-args -p min_point_distance:=0.0005

# 플랜지 자취와 비교 — 펜 오프셋 150mm 가 눈에 보인다
ros2 run hcr5_viz pen_trail --ros-args -p tip_frame:=link6_1
```

> `min_point_distance` 를 0 으로 두면 정지 상태에서도 같은 점이 계속 쌓여
> 메모리를 먹는다. 너무 크면 곡선이 각져 보인다. 2mm 가 그리기 스케일에 맞다.

## 검증 결과 (mock, 2026-08-08)

XZ 평면에 한 변 10cm 사각형을 그린 결과:

```
ns=pen_trail (LINE_STRIP)  점 88 개
    x: -0.0159 ~ +0.0841   폭 0.1000 m   ← 정확히 10cm
    y: -0.6053 ~ -0.6052   폭 0.0000 m   ← 평면 유지 (0.1mm 이내)
    z: +0.1189 ~ +0.2189   폭 0.1000 m   ← 정확히 10cm
```

**Y 폭이 0** 이라는 것이 중요하다 — 사각형이 평면을 벗어나지 않았다는 뜻이다.
그리기에서는 "종이 평면을 유지하는가"가 핵심 요구이므로, 이 값이 곧 품질 지표다.

## 파이썬으로 만든 이유

- MoveIt C++ API 가 필요 없다 (TF 와 Marker 만 쓴다)
- `--symlink-install` 덕분에 **고칠 때마다 재빌드가 필요 없다.**
  노드만 다시 띄우면 된다 — 값을 만져가며 실험하기에 유리하다
  (C++ 인 `hcr5_examples` 는 매번 `colcon build` 를 해야 한다)

## 다음에 붙일 것

- **입력 도형 선** — 예제가 요청한 waypoints 를 초록 선으로 발행.
  빨강(실제 자취)과 겹쳐 보면 **편차가 눈에 보인다**
- **편차 수치화** — 두 선 사이 최대·평균 거리 계산 → 실기 선 품질 계측기
- **자취 저장** — CSV/YAML 로 떨궈 오프라인 분석
