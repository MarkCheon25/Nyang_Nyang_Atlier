# HCR-5 MoveIt2 + ros2_control 구축 — 요약

**2026-08-08 · 브랜치 `ysh_moveit_ros2_control` · ysh (moveit2 모듈)**
상세본: [`HCR5_ros2_control_구축_상세.md`](./HCR5_ros2_control_구축_상세.md)

---

## 무엇을 했나

물려받은 `hanwha_robot_arm` 자산을 고치는 대신 **`hcr5_description` · `hcr5_moveit_config` 두 패키지를 새로 구성**했다.
CAD 산출물(메쉬·링크 치수)만 가져오고 나머지는 전부 새로 만들었다.

**핵심 3가지가 달라졌다:**

| | 기존 | 새로 |
|---|---|---|
| 체인 끝 | `link6_1` (플랜지) | **`pen_tip` (펜 끝)** — 그리기의 IK 기준점 |
| 엔드이펙터 | 정의 없음 | **`tool0` · `pen_tip` 툴 체인 신설** |
| 관절/속도 한계 | CAD 기본값 (속도 실기의 **32배**) | **실기 공식값** (180°/s) |

## 현재 상태

```
colcon build · xacro 파싱 · check_urdf        ✅
체인:  base_link → … → link6_1 → tool0 → pen_tip
MoveIt 계획(OMPL) · 시간매개변수화(TOTG)      ✅
joint_state_broadcaster                        ✅ active
hcr_arm_controller (JTC)                       ✅ active
하드웨어 hcr5_system                            ✅ position 6개 [claimed]

RViz Plan&Execute 10회 연속                    ⬜ 육안 확인 남음
펜 방향 RViz 육안 확인                          ⬜ (FK 계산으로는 확인됨)
```

**→ 1단계 거의 완료. 위 ⬜ 2건이 마지막 판정 조건.**

## 짚고 갈 전제 하나

**"MQTT를 ROS2로 바꾼다"는 성립하지 않는다.** 둘은 대체 관계가 아니라 층위가 다르다 —
MQTT는 로봇으로 가는 **통신 채널**, ROS2는 PC 쪽 **프레임워크**다.
그리고 Rodi 1.003.005 에서는 **MQTT가 유일하게 열린 제어 통로**다(TCP 7000은 Rodi 2.x 요구).

→ 최종 구조는 **MQTT를 ros2_control 아래에 감추는** 것이다:
```
MoveIt2 → JointTrajectoryController → [하드웨어 플러그인] → MQTT → HCR-5
                                        ↑ sim/실기 교체 지점
```

## 주요 결정

1. **ros2_control 유지** (액션서버 직결 아님) — sim 팀도 같은 URDF를 쓰므로 **플러그인만 갈아끼우면 sim/실기가 같은 코드로 검증**된다
2. **ros2_control 블록을 `description` 에 배치** (MoveIt 관례와 반대) — sim 팀이 moveit_config 없이 description만 쓰기 때문
3. **실기 연결은 인자 한 줄로** — `hardware_plugin:=<드라이버>/HcrSystem`. `pen_length` 도 인자라 펜홀더 설계 확정 시 숫자만 교체

## 검증에서 나온 수확

- **`joint_1`·`joint_6` 의 음수 방향이 막혀 있었다** — 11° 이동이 **348° 대회전**으로 계획되는 상태였다. 그리기엔 치명적
- **CAD 치수가 실기 제원과 일치** — FK 로 계산한 작업반경이 **915mm** 로 공식값과 정확히 같다. 물려받은 링크 치수를 신뢰해도 된다
  (단 **관절 부호·영점 규약**은 별개로 실기 대조 필요)
- **펜 방향 FK 검증** — `link6_1 → pen_tip = [-0.15, 0, 0]`, 손목이 밀려나는 방향의 연장선. 팔 바깥이 맞다

## 막혔던 곳 2건

| 증상 | 원인 | 해결 |
|---|---|---|
| `No acceleration limit was defined for joint_1` — 계획은 되는데 실행 안 됨 | **URDF `<limit>` 에는 가속도 항목이 없다**(effort·velocity만). Setup Assistant가 `0`으로 생성 | `joint_limits.yaml` 에 `max_acceleration: 3.5` |
| `Action client not connected to action server` | `ros2_controllers.yaml` 의 `command_interfaces`/`state_interfaces` 가 **빈 `[]`** → 컨트롤러가 configure 실패 | `[position]` / `[position, velocity]` |

> ⚠️ **Setup Assistant를 재생성하면 위 2건이 되돌아간다.** 펜홀더 설계가 확정되면 툴 링크에 메쉬가 생겨
> 자기충돌 매트릭스를 다시 만들어야 하므로, **재생성은 예정된 이벤트다.**
>
> 대비: ① 두 파일 상단에 이유를 주석으로 박음 ② **검증 스크립트**를 둠 —
> ```bash
> python3 src/moveit2/tools/check_moveit_config.py     # 0=통과 / 1=문제
> ```
> moveit_config 패키지 **바깥**에 있어 재생성에도 살아남고, 문제가 있으면 **고칠 값까지 출력**한다.
> **재생성 직후에는 반드시 한 번 돌릴 것.**

## 다음

```
1단계 마무리 (RViz 육안 확인 2건)
   ↓
2단계  궤적 시각화(관찰) + 역방향 검증(선→궤적)   ← 그리기 가능성을 sim 에서 판정
   ║ 병렬: 가짜 HCR-5 스텁 · 상태 브릿지
[인천] movej 프로토콜 캡처
   ↓
3단계 명령 브릿지 → 4단계 실기 → 5단계 vision 캘리브레이션
```

**2단계에 "역방향(선 → 궤적)"을 추가했다.** 당초 계획은 관찰 방향만 있어서, "선대로 그릴 수 있는가"를
4단계 실기 앞에서 처음 시험하게 된다. IK 불연속·특이점·작업영역 문제를 거기서 만나면 3·4단계 설계가 통째로 바뀐다.

---

## 🔥 팀 요청 (시간 급함 — 주말 인천)

**1. movej/movel 기능테스트하실 때 `capture.py` 를 같이 돌려주세요.**
그 조작이 우리 3단계 블로커와 정확히 같은 대상이라, 캡처만 하면 프로토콜이 잡힙니다. 읽기 전용이라 로봇에 아무것도 쓰지 않습니다.

**2. ★ movej 를 연속 2~3개 실행했을 때 중간에 멈췄다 가는지, 끊김 없이 이어지는지(blending) 봐주세요.**
"매번 정지"면 그리기 선에 마디가 지고, 서보 스트리밍 경로를 따로 찾아야 합니다. **3·4단계 설계가 여기서 갈립니다.**

**3. 가만히 있는 상태로 30초만 따로 캡처해 주세요.**
`motion/tool/position` 실측값이 남으면, 실기 없이도 URDF 영점·부호 규약 검증을 미리 할 수 있습니다.

절차·조작 순서는 상세본 §10 에 있습니다.

**4. 안전설정은 중복 조사 불필요** — `limitCheck`, 충돌 감지·복구는 2026-07-29 에 이미 PC 제어로 실증됨
([`hcr_comm/README.md`](../../drivers/hcr_comm/README.md) §6).

**5. vision 팀** — A4 위치·그림을 **어떤 단위·어떤 원점**으로 넘길지 지금 합의 필요. 좌표 변환이 moveit2 몫이라 나중에 어긋나면 양쪽 재작업입니다.
