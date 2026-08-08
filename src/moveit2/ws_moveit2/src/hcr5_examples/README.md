# `hcr5_examples` — HCR-5 MoveIt2 예제

관절 목표 → 자세 목표(펜 끝 IK) → 데카르트 직선 경로 순으로, **그리기 파이프라인에 필요한 동작을 하나씩** 확인한다.
각 예제는 절대 좌표를 박지 않고 **현재 자세 기준 상대 이동**이라, 어느 자세에서 실행해도 대체로 도달 가능하다.

## 실행

전제 — 다른 터미널에서 MoveIt 이 떠 있어야 한다:

```bash
ros2 launch hcr5_moveit_config demo.launch.py
#   RViz 없이 돌리려면:  ros2 launch hcr5_moveit_config demo.launch.py use_rviz:=false
```

예제 실행:

```bash
ros2 launch hcr5_examples example.launch.py example:=joint_goal
ros2 launch hcr5_examples example.launch.py example:=pose_goal
ros2 launch hcr5_examples example.launch.py example:=cartesian_square
```

> `ros2 run` 으로 직접 띄우면 안 된다. `MoveGroupInterface` 는 robot_description · SRDF ·
> kinematics · joint_limits 를 **자기 노드의 파라미터로** 요구하는데, 그걸 넣어주는 것이
> 이 launch 파일이다.

## 예제

### 1. `joint_goal` — 관절 목표

현재 관절값에서 `joint_1` 만 +0.3 rad(≈17°) 돌린다. IK 를 쓰지 않으므로 **"계획 → 실행" 사슬만 순수하게** 확인한다.

```
planning frame : base_link
end effector   : pen_tip          ← 펜 끝이 기준점임을 확인
현재 관절값 [0.000 0.000 1.567 -1.590 -1.470 0.000]
계획 성공 — 실행한다
실행 결과: 성공
```

### 2. `pose_goal` — 자세 목표 (IK)

현재 **펜 끝** 자세를 읽어 Z 로 10cm 내린 곳을 목표로 준다.
**IK 가 플랜지가 아니라 펜 끝 기준으로 풀린다**는 것을 증명한다.

```
현재 펜 끝 [-0.016 -0.605 0.419]  (frame: base_link)
목표 펜 끝 [-0.016 -0.605 0.319]
도달 펜 끝 [-0.016 -0.605 0.319]  (목표와 Z 오차 -0.0001 m)
```

### 3. `cartesian_square` — 데카르트 직선 경로 ★

**이 프로젝트의 핵심 동작이다.** 그리기 = 펜 끝이 정해진 선을 따라가는 것이고, 그 선을 만드는 도구가
`computeCartesianPath` 다. 현재 자세에서 XY 평면에 한 변 10cm 사각형을 그린다(자세는 유지).

```
시작점 [-0.016 -0.605 0.319]
데카르트 경로 달성률: 100.0%  (구간 72 개)
총 소요시간 7.00 초 — 실행한다
실행 결과: 성공
시작점 복귀 오차: 0.0000 m
```

**반드시 볼 것 — 달성률(fraction)**

| 값 | 의미 |
|---|---|
| `1.00` | 전 구간 IK 가 풀렸다 — 정상 |
| `< 1.0` | 중간에 IK 가 끊겼거나 특이점·충돌을 만났다 |

이 값이 실기에서 **"선이 왜 일그러지는가"의 1차 지표**가 된다. 낮게 나오면 시작 자세를 바꾸거나,
IK 솔버 교체(KDL → TRAC-IK) · 자세 제약 추가 · 작업영역 재배치를 검토한다.

**시간 매개변수화를 다시 하는 이유** — `computeCartesianPath` 산출물은 시간 정보가 부실하다.
그래서 TOTG 로 다시 시간을 입힌다. 이때 `joint_limits.yaml` 의 속도·가속도 한계가 실제로 쓰인다
(가속도 한계가 없으면 여기서 실패한다 — 상세본 §7.1).

## 조정할 만한 값

`src/03_cartesian_square.cpp` 상단:

| 상수 | 기본 | 뜻 |
|---|---|---|
| `SIDE` | `0.10` | 사각형 한 변 (m) |
| `EEF_STEP` | `0.005` | 보간 간격 (m). 촘촘할수록 선이 매끄럽지만 점이 많아진다 |
| `VEL_SCALE` · `ACC_SCALE` | `0.1` | 속도·가속도 스케일. 실기 연결 전까지 낮게 유지 |

## 다음 단계와의 연결

3번 예제가 **2단계 "역방향 검증"(선 → 궤적)의 최소 버전**이다. 2단계에서는 여기에
**펜 끝 자취를 선으로 그리는 시각화**(`hcr_viz`)를 붙여, 입력한 도형과 실제 지나간 선을
겹쳐 보고 편차를 수치로 낸다.

- 계획·구축 경위: [`src/moveit2/docs/HCR5_ros2_control_구축_상세.md`](../../../docs/HCR5_ros2_control_구축_상세.md)
- 요약: [`src/moveit2/docs/HCR5_ros2_control_요약.md`](../../../docs/HCR5_ros2_control_요약.md)
