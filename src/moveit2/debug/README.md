# `debug/` — 코드 해설본

작성한 코드를 **한 줄씩 뜯어 설명한 사본**을 모아둔 곳이다.
"이 줄이 왜 있는가 / 빼면 어떻게 되는가"를 적었다.

## ⚠️ 읽기 전에

- **여기 있는 파일은 빌드·실행되지 않는다.** 워크스페이스(`ws_moveit2/`) 바깥이라
  colcon 이 건드리지 않는다. 순수 학습·참조용이다.
- **원본을 고치면 여기도 같이 고쳐야 한다.** 자동 동기화가 없다.
  값이 어긋나면 **원본이 정답**이다.

## 목록

| 해설본 | 원본 | 무엇을 배우나 |
|---|---|---|
| `01_joint_goal.annotated.cpp` | `hcr5_examples/src/01_joint_goal.cpp` | MoveGroupInterface 기본 골격. **왜 스핀 스레드가 필요한가**, 왜 파라미터 자동선언이 필수인가 |
| `02_pose_goal.annotated.cpp` | `hcr5_examples/src/02_pose_goal.cpp` | IK 를 쓰는 자세 목표. **관절 목표와 무엇이 다른가**, 왜 orientation 을 복사하나 |
| `03_cartesian_square.annotated.cpp` | `hcr5_examples/src/03_cartesian_square.cpp` | **그리기의 핵심.** 데카르트 경로, 달성률(fraction)의 의미, TOTG 로 시간 입히기 |
| `example.launch.annotated.py` | `hcr5_examples/launch/example.launch.py` | **왜 `ros2 run` 으로는 안 뜨는가.** MoveIt 파라미터 4종의 역할 |
| `urdf_xacro.annotated.md` | `hcr5_description/urdf/*.xacro` | URDF 구조, **펜 TCP 를 붙인 방법과 축 방향을 정한 근거**, 관절 한계 교체 |

## 읽는 순서 (권장)

```
① urdf_xacro.annotated.md          로봇이 어떻게 정의되어 있는지 먼저
      ↓
② example.launch.annotated.py      실행 환경이 어떻게 갖춰지는지
      ↓
③ 01_joint_goal                    가장 단순한 형태 (IK 없음)
      ↓
④ 02_pose_goal                     IK 추가 (펜 끝 기준)
      ↓
⑤ 03_cartesian_square              직선 경로 — 여기가 그리기의 원형
```

①②를 건너뛰고 ③부터 읽으면 "이 파라미터가 어디서 왔지?" 에서 막힌다.

## 각 해설본의 공통 구조

- 상단 — 이 파일이 **무엇을 증명하는가**, 전체 흐름
- 본문 — 코드 블록 단위로 【번호】를 붙이고 그 아래 설명
- `★` 표시 — **특히 중요하거나 자주 틀리는 곳**
- 하단 **【부록】진단표** — 증상 → 의심 지점 대응표

## 진단표 통합

세 예제의 부록을 합치면 이렇게 된다.

| 증상 | 의심 지점 |
|---|---|
| `Failed to fetch parameter 'robot_description'` | `ros2 run` 으로 띄웠다. launch 를 써야 한다 |
| 관절값을 못 받음 / 무한 대기 | 스핀 스레드 누락, 또는 `demo.launch.py` 미실행 |
| `end effector` 가 `link6_1` | SRDF 그룹이 chain 이 아니다 (`Add Kin. Chain` 으로 재설정) |
| `No acceleration limit was defined` | `joint_limits.yaml` 가속도 한계 |
| `Action client not connected to action server` | `ros2_controllers.yaml` 인터페이스가 빈 `[]` |
| pose 목표가 항상 실패 | kinematics 파라미터 누락 |
| pose 목표가 가끔 실패 | KDL timeout(0.005s). 재실행하거나 TRAC-IK 검토 |
| 데카르트 달성률 < 100% | 시작 자세 / 도형 크기 / 특이점 / IK 솔버 |
| 실행 성공인데 RViz 가 안 움직임 | **주황색은 Query Goal State 다.** 실제 로봇은 Scene Robot |

## 함께 볼 것

- 구축 경위·설계 근거 전체: [`../docs/HCR5_ros2_control_구축_상세.md`](../docs/HCR5_ros2_control_구축_상세.md)
- 1페이지 요약: [`../docs/HCR5_ros2_control_요약.md`](../docs/HCR5_ros2_control_요약.md)
- 예제 실행법: [`../ws_moveit2/src/hcr5_examples/README.md`](../ws_moveit2/src/hcr5_examples/README.md)
- 설정 검증: `python3 ../tools/check_moveit_config.py`
