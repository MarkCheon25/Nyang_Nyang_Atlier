# test_tools — 연결 확인 전용

⚠️ **제품 코드가 아니다. 실제 vision 이 붙어 검증이 끝나면 지운다.**

여기 있는 것은 vision 컨테이너 없이 `/vision/strokes` 경로가 살아 있는지 혼자
확인하기 위한 도구다. 그리기 로직에는 아무 영향이 없다.

| 파일 | 역할 |
|---|---|
| `fake_vision_publisher.py` | vision 스펙대로 `/vision/strokes` 를 발행한다 |

---

## 쓰는 법

**셸 3 개**가 필요하다. 순서가 중요하다 — `draw_cat` 의 기본 구독 QoS 가
`VOLATILE`(살아 있는 발행분만)이라 **구독이 먼저 떠 있어야** 한다.

```bash
# ── 셸 1: 시뮬레이션 (몸통)
ros2 launch sim_bringup mujoco_sim.launch.py          # simulation 컨테이너

# ── 셸 2: move_group + RViz (머리)
ros2 launch drawing_cat mujoco_moveit.launch.py       # moveit2 컨테이너

# ── 셸 3: 그리기 — 토픽을 기다린다 (최대 30초)
ros2 launch drawing_cat draw_cat.launch.py \
  params_file:=$(ros2 pkg prefix drawing_cat --share)/config/topic_example.yaml

# ── 셸 4: "구독 중" 로그가 뜨면 가짜 vision 을 쏜다
ros2 run drawing_cat fake_vision_publisher
```

셸 3 에 이런 로그가 나오면 성공이다:

```
'/vision/strokes' 구독 중 (VOLATILE: 살아 있는 발행분만) — 최대 30초 대기
부위 5 개 수신 | 이미지 1280×960 px | letterbox 0.14062 mm/px, 오프셋 (15.0, 81.0) mm
계획 지도 저장: /tmp/map_from_vision.csv
스트로크 0 'cat' | 점 48 개 · 폐곡선 | 성공률(fraction): 1.00
...
===== 고양이 그리기 완료 | 5/5 스트로크 실행 =====
```

### 순서를 못 맞추겠으면

`draw_cat` 을 나중에 띄워도 되게 하려면 `strokes.use_latched: true` 로 바꾼다.
발행자의 `TRANSIENT_LOCAL` 이력을 받아온다.

> ⚠️ **`ros2 launch` 는 `--ros-args -p` 를 받지 않는다** — 그건 `ros2 run` 전용이다.
> launch 파일은 선언된 인자(`params_file`)만 받으므로, 값을 바꾸려면 **설정 파일을 따로
> 만들어** 넘겨야 한다. 아래처럼 하면 된다.

```bash
cat > /tmp/latched.yaml <<'EOF'
draw_cat_node:
  ros__parameters:
    strokes:
      source: "topic"
      topic: "/vision/strokes"
      use_latched: true          # ← 여기만 바꾼 것
      frame_timeout_ms: 1500
      wait_timeout_s: 120.0
    paper:
      width_mm: 210.0
      height_mm: 297.0
      margin_mm: 15.0
      auto_center: true
      scale: 0.0004
      pen_lift: 0.008
    planning:
      min_fraction: 0.9
    output:
      map_csv_path: "/tmp/map_from_vision.csv"
EOF
ros2 launch drawing_cat draw_cat.launch.py params_file:=/tmp/latched.yaml
```

**크게 그려서 MuJoCo 창에서 잘 보이게 하려면** 같은 방식으로 `scale: 0.001`,
`pen_lift: 0.05` 로 만든다 (84 × 93 mm, 관절이 5~9° 움직여 눈에 띈다).

---

## 옵션

| 파라미터 | 기본값 | 뜻 |
|---|---|---|
| `topic` | `/vision/strokes` | 발행할 토픽 |
| `image_width` / `image_height` | `1280` / `960` | 원본 이미지 크기 (px) |
| `frames` | `1` | 이미지 몇 장을 발행할지 |
| `part_interval_s` | `0.15` | 부위 사이 간격 |
| `frame_interval_s` | `3.0` | 이미지 사이 간격 |
| `latched` | `true` | `false` 면 발행자를 VOLATILE 로 |
| `startup_delay_s` | `2.0` | 발행 전 대기 (구독자 디스커버리 시간) |

**프레임 경계 판정 시험** — 이미지 3 장을 연달아 보내면, `draw_cat` 이
`instance_label` 중복으로 경계를 잡고 **가장 최신 프레임만** 쓰는지 볼 수 있다.

```bash
ros2 run drawing_cat fake_vision_publisher --ros-args -p frames:=3
```

**이미지 크기가 달라도 mm 결과가 같은지 시험** — letterbox 변환이 이미지 크기에
맞춰 축척을 바꾸므로, 가로세로 비가 같으면 mm 좌표가 같아야 한다.

```bash
ros2 run drawing_cat fake_vision_publisher --ros-args -p image_width:=640 -p image_height:=480
```

---

## ⚠️ 타입 해시 — 실제 vision 과 붙이기 전에 반드시 확인

`ws_moveit2/src/vision_interfaces` 는 vision 정의의 **미러**다. 필드가 하나라도
다르면 타입 해시가 달라지고, 그러면 **토픽은 보이는데 메시지가 0 개** 들어온다.
에러가 안 나서 QoS 문제로 오해하기 쉽다.

```bash
ros2 topic info /vision/strokes --verbose | grep -i "type hash"
```

이 도구로 발행하든 실제 vision 이 발행하든 **같은 값**이 나와야 한다.
**기준 해시값은 [`vision_interfaces/README.md`](../../vision_interfaces/README.md) 에만
적어 둔다** — 여러 곳에 베껴 두면 정의가 바뀌었을 때 어느 값이 맞는지 알 수 없게 된다.
다르면 `vision_interfaces/msg/*.msg` 를 원본과 맞춰 다시 빌드한다.

---

## 지우는 법

실제 vision 연결이 확인되면:

1. 이 폴더(`test_tools/`) 삭제
2. `CMakeLists.txt` 에서 `# ── test_tools` 로 시작하는 블록 삭제
3. `package.xml` 에서 `<!-- test_tools -->` 주석이 붙은 `<exec_depend>rclpy</exec_depend>` 삭제
4. 이 README 와 상위 `README.md` 의 test_tools 언급 정리

`vision_interfaces` 미러는 **별개다** — vision 원본이 저장소에 병합되어 moveit2
컨테이너에서 보이게 될 때 지운다. 그 전까지는 필요하다.
