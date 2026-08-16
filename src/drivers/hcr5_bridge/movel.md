# movel — 직교 좌표 직선 이동

> 2026-08-16 세션 260816-레제 착수 문서. **설계 근거**를 담는다 — 사용법은 [`README.md`](README.md) §1.1·§3.5 가 원본이다.
>
> ✅ **2026-08-16 (260816-컨스터블) 구현·실기 실증 완료.** 이 문서는 이제 *"왜 이렇게 만들었나"* 의 원본이고,
> 설계가 옳았는지의 답은 §1.4 에 있다. 착수 시점의 🔴 블로커 2개(§1.4 스키마 · §2.3 회전규약)는 **둘 다 닫혔다.**

---

## 1. 주요기능

### 1.1 무엇을 하는가

```
ros2 run hcr5_bridge movel <좌표계> <x> <y> <z> [옵션]
```

**점 A 에서 점 B 로, 플랜지(또는 TCP)가 직선을 그리며 한 번에 간다.**
관절 하나하나는 곡선으로 움직여도 된다 — 구속하는 것은 **엔드이펙터의 궤적**이다.

| 좌표계 | 기준 원점 | 목표 계산 | 예 |
|---|---|---|---|
| `base` | 실기 base **절대** (mm) | `T = (x, y, z)` | `movel base 490 -170.5 441.5` |
| `world` | base 축 방향 **상대** Δ | `T = cur + (x, y, z)` | `movel world 0 0 20` — 수직 20mm ↑ |
| `tool` | 현재 툴 축 기준 **상대** Δ | `T = cur + R·(x, y, z)` | `movel tool 0 0 -10` — 펜 방향 10mm |

- **단위는 mm.** 관절값이 아니라 직교 좌표다.
- **자세(rx, ry, rz)는 인자에 없다** — 현재 자세를 그대로 유지한다. 근거는 §2.3.
- 좌표계 토큰은 **기준 원점**을 고르는 것이지 제어점(flange/tcp/pen)을 고르는 것이 아니다.
  제어점은 별도 옵션으로 둔다.

### 1.2 경로 — `program/plan` 의 `linear` 노드

보간을 **컨트롤러가 소유한다.** PC 는 목표를 주고 궤적은 만들지 않는다.

```
1. 목표 위치 계산 (좌표계별 — 위 표)
2. program/clear
3. program/plan  — MOVE 노드 1개
     move.selected: "linear",  radius: 0
     waypoint.endPoint.fixed.{tcp, flange, joint}
4. program/play
5. program/end · event/motion 으로 도착 판정
```

**노드 하나 = 구간 하나**이므로 점 A→B 는 쭉 한 번에 간다. 분할하지 않는다.

| 실측 근거 | 값 | 출처 |
|---|---|---|
| `linear` 직선 이탈 | **0.045mm / 179.3mm 구간** — 로봇 반복정밀도(±0.1mm)보다 좋다 | `hcr5_comm/README.md` §6.2 |
| `radius:0` 웨이포인트 통과 오차 | 0.34mm (정확 통과) | 〃 |

⚠️ **`radius` > 0 을 쓰지 않는다.** radius=50 이면 궤적이 그 웨이포인트를 **22.3mm 떨어져** 지나간다.

### 1.3 배타 — ros2_control 스택과 동시에 못 쓴다

`program/play` 는 컨트롤러가 자기 궤적을 실행하는 것이고, `bringup.launch.py` 는
`HcrSystemInterface` 가 명령 인터페이스를 claim 해 주기적으로 관절 지령을 쓴다.
**둘이 동시에 돌면 명령이 부딪힌다** (README §1.2 — 두 경로는 배타).

| | JTC 경로 (교체 전 movel·movej) | **program/plan 경로 (이 문서의 movel)** |
|---|---|---|
| 진입 | `bringup.launch.py` | **스택 불필요** |
| 보간 소유 | JTC (관절공간) | **컨트롤러 (직교공간)** |
| 플랜지 경로 | 곡선 (호) | **직선** |
| `/joint_states` | 발행됨 | 없음 |
| 현재 자세 조회 | ROS `/joint_states` | **MQTT `get/command/pos`** |
| MoveIt·충돌검사 | 가능 | 없음 |

→ movel 은 `servo` 와 같이 **rclcpp 없이도 도는 실행기**가 된다. 스택이 죽어 있어도 돈다.

> 📌 **2026-08-16 오후(260816-호크니), `movej` 도 같은 문으로 왔다** — [`movej.md`](movej.md).
> 위 표의 첫째 칸에 남아 있던 실행기가 이제 없다. JTC 경로는 **MoveIt 이 쓰는 문**으로만 남는다.

### 1.4 구현 상태 — ✅ 완료 (2026-08-16, 260816-컨스터블)

| 항목 | 상태 |
|---|---|
| `src/movel.cpp` JTC 경로 → `program/plan` 경로 | ✅ **전면 교체.** rclcpp 의존 제거(§1.3) |
| 좌표계 인자 (`base`·`world`·`tool`) | ✅ 셋 다 구현 |
| `program/plan` linear 발행 | ✅ `buildPlan()` — 캡처한 정확형 |
| `tool` 좌표계의 회전행렬 R | ✅ `include/hcr5_bridge/pose_convention.hpp` — §2.3 이 닫혔다 |

**🟢 `program/plan` 스키마 확보 완료.** 아래 두 경로 중 **②(펜던트 캡처)** 를 골랐다(Mark 판단).
`capture.py` 를 켜고 펜던트에서 linear 프로그램을 만들어 **적용**했다 — ROOT·INITIALIZE·MOVE 노드
형식과 `coordinates`·`variables`·`thread`·`subprogram` 필수 필드를 전부 얻었다.
정확형은 [`../hcr5_comm/README.md`](../hcr5_comm/README.md) §6 으로 승격했다.
⚠️ **캡처 원자료(128MB)는 커밋하지 않았다** — 업무목록 **L18**.

> 예상대로 **등록만 하면 MQTT 에 안 실린다.** 적용해야 실린다 (`hcr5_comm/README.md` §10).
>
> 그리고 그 역도 확인됐다 — **`program/plan` 을 발행해도 펜던트 파일 목록(`HTW Storage`)에는 안 뜬다.**
> 두 층이 다르다. 컨트롤러 `mongoLog` 대조 결과 우리 발행도 펜던트 적용과 **똑같이**
> `programEvent : program_plan` 을 남기고 `status/program` 이 `PROGRAM_STATE_INIT` 으로 전이했다.
> 다른 것은 펜던트 쪽에만 붙는 `clickApplyProgram`("send program file to server") 한 단계뿐이고,
> `.file` 을 만드는 것이 그 단계다. **적재 여부를 파일 목록으로 판정하면 안 된다.**

#### 실기 실증 (2026-08-16, 서보 ON · e-stop 대기 · 축온 48~51°C)

| # | 명령 | 거리 | 도착 오차 |
|---|---|---|---|
| 1·2 | `movel world 0 0 ±20` | 20mm | 0.001mm (왕복 누적 드리프트 0.002mm) |
| 3·4 | `movel world ∓60 0 ±80` | 100mm 대각선 | 0.001mm |

**직선성** — 100mm 대각선을 `motion/flange/position` 으로 추적(65샘플 / 2.203초):

| 항목 | 값 |
|---|---|
| 직선 대비 수직편차 (정속구간, 양끝 3mm 제외) | **최대 0.0184mm · RMS 0.0074mm** |
| 〃 (전 구간) | 최대 0.1063mm — 가속 시작 0.8mm 지점 1샘플 |
| 자세 유지 | rx 0.035° · ry 0.0001° · rz 0.0005° |
| 정속 속도 | 50.0mm/s (지령과 일치) |

→ **§3.3 이 지적한 *"플랜지 경로가 직선이 아니다"* 는 해소됐다.** §1.2 의 컨트롤러 실측
(0.045mm / 179.3mm)과 같은 수준이고, 로봇 반복정밀도(±0.1mm) 안이다.

> 속도 프로파일에 100~140mm/s 스파이크가 보이는데 **가속이 아니라 샘플 유실**이다 —
> 29.1Hz 스트림을 다 못 받아 dt 가 과소평가된다. 50 과 104 가 번갈아 나오는 패턴이 증거고,
> 구간 누적거리는 일정하다. 궤적 분석 때 이 함정을 먼저 보라.

#### 부수로 확정된 것

- **`program/play` 는 인자가 필요 없다.** `hcr5_comm/README.md` §4.1 이 적어 둔 `{selectedIndex:[a,b]}`
  는 미검증이었고(펜던트 캡처에도 play 가 없었다), `{}` 로 **4회 연속 정상 실행**했다.
- **IK→FK 왕복 오차 0.000000mm** — waypoint 의 `joint` 표현을 믿어도 된다.
- **`arrPose` 는 행렬이 아니라 평면 6배열** `[x,y,z,rx,ry,rz]` 다. R 을 여기서 못 얻는다(§2.3 이 실측으로 간 이유).

---

## 2. 사용자 의도

### 2.1 무엇을 원했나 (2026-08-16, Mark)

> "점A에서 점B로 가는데, **엔드이펙터 혹은 플랜지 기준으로는 직선인 것을 의도**하고 세션을 진행하고 있거든.
> 로봇 관절 하나하나는 곡선으로 움직임을 받을수도 있겠지."

> "**점A에서 점B로는 쭉 한 번에 가야지.**"

이 두 문장이 설계를 결정했다. **구속 대상은 엔드이펙터의 궤적이고, 관절 궤적은 자유다.**
그리고 중간 정지·분할 없이 한 구간으로 간다.

### 2.2 좌표계 = 기준 원점

`<좌표계>` 토큰은 **기준 원점**을 고른다. 산업로봇 펜던트의 "base 좌표계 / 툴 좌표계" 관례와 같고,
HCR-5 직교 조그의 `jog/start {standard:"base"}` 와도 같은 축이다.
두산 `movel` 의 `ref`(reference frame)·`mode`(absolute/relative) 인자에 대응한다 (§3).

**제어점**(무엇을 그 좌표에 맞추나 — flange / tcp / pen)은 **다른 축**이라 섞지 않는다.
좌표계 토큰에 합성하지 않고 별도 옵션으로 둔다.

### 2.3 x, y, z 만 준다 — 자세는 안 건드린다

> "x, y, z만 두면 그 점까지 로봇암이 이동하기만 하면 되."

이 결정이 **미해결 문제 하나를 정면으로 우회했다.** 착수 시점의 문제 서술은 이랬다:

> 🔴 **실기의 rx / ry / rz 회전 표현 규약이 문서 어디에도 정의돼 있지 않다.**
> 오일러각인지 회전벡터인지, 오일러라면 축 순서가 무엇인지 미확인이다. 알려진 것은
> 홈 자세에서 `rx=-180, ry=0, rz=0` 이라는 점 하나뿐이다 (`hcr5_comm/README.md` §7.2).

자세를 **바꾸려면** 회전을 합성해야 하고 그러려면 규약이 필요하다. 그러나 자세를 **유지**하면
`get/command/pos` 가 준 orientation 을 그대로 되돌려 주기만 하면 되므로 규약을 몰라도 된다.
→ 이 우회는 유효했고, `base`·`world` 는 규약과 무관하게 성립한다.

⚠️ 단 `tool` 좌표계는 예외였다 — 툴 축 방향으로 Δ 를 밀려면 **회전행렬 R** 이 필요하다.
자세를 바꾸지는 않으므로 합성은 불필요하지만 R 자체는 있어야 한다. 후보 둘을 두었다:

| 안 | 방법 | 조건 |
|---|---|---|
| (a) tf2 | `base_link` → `link6_1` 변환에서 R. URDF↔실기가 RMS 0.0060mm 정합(T16)이라 규약 없이 정확 | **스택이 떠 있어야 한다** — §1.3 배타와 충돌 |
| (b) 규약 실측 | FK RPC(`robot/convertPose`)로 알려진 관절값의 rx·ry·rz 를 받아 대조 | 읽기 전용·안전. 확정하면 **문서 자산이 된다** |

`program/plan` 경로에서는 스택을 안 띄우므로 (a)를 쓸 수 없었고, **(b)로 닫혔다.**

#### ✅ 확정 — ZYX 오일러 (2026-08-16, 260816-컨스터블)

```
R = Rz(rz) · Ry(ry) · Rx(rx)        ← ZYX 오일러 (= XYZ 고정축, roll-pitch-yaw)
```

**로봇을 움직이지 않고** FK RPC 만으로 갈랐다. 원리는 두 항등식이다:

| 무엇을 돌리나 | 플랜지가 도는 축 | 항등식 |
|---|---|---|
| J6(wrist3) 만 θ | 자기 z축 (툴축) | `R(θ) = R(0) · Rz(θ)` |
| J1(base) 만 θ | 월드 z축 | `R(θ) = Rz(θ) · R(0)` |

후보 7종(ZYX·XYZ·ZYZ·ZYZ역순·회전벡터·ZXY·YXZ)에 두 항등식의 잔차를 재어
**ZYX 만 6.9e-16**, 나머지 6종은 전부 **1.28 이상**이었다.

> ⚠️ **홈 자세만으로는 못 가린다.** 홈은 `ry=0` 이라 Rx 와 Ry 가 교환돼 **ZYX 와 ZXY 가 동점**
> (둘 다 잔차 0)이 된다. `ry≠0` 인 자세(`[30,−70,−60,−80,55,25]`)를 표본에 넣고서야 유일해졌다.
> 규약을 재확인할 일이 생기면 이 함정부터 보라.

**원본은 코드 자산**이다 — [`include/hcr5_bridge/pose_convention.hpp`](include/hcr5_bridge/pose_convention.hpp)
머리주석이 규약·근거·실측 방법을 함께 든다(`joint_convention.hpp` 가 관절 규약을 소유하는 것과 같은 자리).

이로써 **`tool` 좌표계가 열렸고**, 자세를 **바꾸는** 이동도 규약 측면에서는 막힌 것이 없다
(다만 인자 설계는 §2.3 결정대로 아직 `x y z` 만 받는다).

여전히 **미실측**인 것: `program/plan` 의 `attr.coordinate:"base"|"tcp"` 가 컨트롤러 쪽 좌표계
해석을 제공하는지. 현행 구현은 **PC 에서 좌표계를 풀어** base 절대 목표를 넘기므로 이것에 의존하지 않는다.

### 2.4 실기 운전 경험 — 안전 게이트의 눈금

> "지금 현재까지 로봇을 여러번 돌려보았지만 **움직임은 속도가 매우 느리고, 관절한계도 크게 문제가 없었어.**"

→ 게이트를 과하게 조이지 않는다. 다만 **직교 이동거리 상한**은 새로 필요하다:
상대이동(`world`·`tool`)에서 `10` 을 `100` 으로 오타 내면 그대로 10배를 가는데,
기존 게이트(관절한계·각속도)는 **직교 거리를 안 본다**.

📌 **게이트 ⑤ 서보 상태 추가 (2026-08-16 오후).** `movej` 실측 중에 드러난 구멍을 `movel` 에도 같이 메웠다.
`program/play` 는 **서보가 꺼져 있어도 `code:0` 으로 정상 ack 한다** — `clear`·`plan`·`play` 세 ack 가
모두 성공인데 로봇은 1mm 도 안 움직이고, 유일한 신호는 도착 타임아웃이다. movel 은 서보가 켜진
세션에서만 돌려봐서 이 구멍이 안 드러났을 뿐 같은 결함이었다. 근거와 구현은
`include/hcr5_bridge/servo_gate.hpp` · [`movej.md`](movej.md) §2.3.

### 2.5 이 컨테이너의 자리

> "이번에 이용할 이미지는 Nyang 어쩌고 새로 만든 이미지를 써주면 좋겠어.
> **이 이미지가 로봇암 움직임 제어의 기초를 담당할 예정**이야."

`nyang_nyang_atlier:jazzy` (2026-08-16 05:09 빌드) 가 로봇암 움직임 제어의 기준 환경이다.
`moveit2_dev` 는 팀 공용으로 그대로 두고 병존한다 (README §2.1).

---

## 3. 두산 로봇암 조사결과

**질문** — 두산로보틱스의 `movel` 은 우리가 구현하려던 JTC 방식을 쓰는가?
**답 — 아니다. 컨트롤러가 직선 보간을 소유한다.**

### 3.1 확인된 사실

두산 ROS 2 패키지 `doosan-robot2` 의 `movel` 은 **`motion/move_line` 서비스**(`dsr_msgs2/srv/MoveLine`)를
호출한다. 요청 필드는 다음과 같다:

```
pos, vel, acc, time, radius, ref, mode, blend_type, sync_type
```

**`radius` 와 `blend_type` 의 존재가 결정적이다.** 블렌딩 반경은 **보간하는 쪽만** 가질 수 있는
파라미터다. JTC 로 관절점 열을 보내는 구조라면 radius 는 의미가 없다.
→ 두산은 목표를 컨트롤러에 넘기고 궤적 생성을 위임한다. **JTC 궤적을 만들어 보내지 않는다.**

> 두산도 `ros2_control`/JTC 를 제공하지만 그것은 MoveIt 경로용이고 `movel` 서비스와는 다른 문이다.

### 3.2 우리 HCR-5 의 대응물

**`program/plan` 의 `move.selected:"linear"` 가 정확히 같은 자리다.**

| 두산 `MoveLine` | HCR-5 `program/plan` linear | 비고 |
|---|---|---|
| `pos` | `waypoint.endPoint.fixed.{tcp,flange,joint}` | HCR-5 는 **세 표현을 동시에** 담는다 |
| `vel` · `acc` | `move.linear.{velocity, acceleration}` | 기본 500mm/s · 1000mm/s² |
| `radius` | `move.linear.radius` | ⚠️ 스트로크에는 0 (§1.2) |
| `blend_type` | `move.linear.continues` | `true` 면 속도가 0을 안 찍는다 (19~20mm/s 유지 실측) |
| `ref` (reference frame) | `attr.coordinate: "base"｜"tcp"` | **미실측** — §2.3 |
| `mode` (absolute/relative) | 대응 미확인 | 〃 |
| `sync_type` | `program/play` 후 `program/end` 대기 | ack ≠ 도착 |

두 컨트롤러가 같은 개념을 같은 이름으로 갖고 있다 — **우연이 아니라 산업로봇 모션 명령의 공통 형태**다.
`movel` 이라는 이름이 직교공간 직선 보간을 뜻한다는 것도 ABB `MoveL`·KUKA `LIN` 과 같다.

### 3.3 그래서 무엇이 바뀌었나

이 조사 전의 설계는 **JTC 에 시작·끝 두 점만 실어 보내는 것**이었다. 그 경우 JTC 가 관절공간에서
보간하고 플랜지 경로는 그 관절 궤적의 FK — **일반적으로 호(곡선)** 가 된다. 이동이 클수록,
시작·끝 자세 차가 클수록 크게 휜다. **교체 전** `src/movel.cpp` 머리주석과 README §3.5 가
*"직선이 아니다"* 라고 못 박은 것이 이 뜻이다 (2026-08-16 교체로 해소 — §1.4 실증표).

**`movel` 이라는 이름과 실제 동작이 어긋나 있었고, 두산 대조가 그것을 드러냈다.**

검토한 대안 셋 중 `program/plan` linear 를 골랐다:

| | 방식 | 직선성 | 판단 |
|---|---|---|---|
| A | 직선을 N점 분할 → 각 점 IK → JTC 궤적 하나 | 점 간격이 결정 | 스택은 그대로지만 IK RPC 왕복 130ms × N |
| B | MoveIt `computeCartesianPath` | 같음 (KDL IK) | `move_group` 필요 · KDL 해 ≠ 실기 해 |
| **C** | **`program/plan` linear** | **실측 이탈 0.045mm/179mm** | ✅ **채택** — 두산과 같은 층, 가장 정확, 한 구간에 한 번에 |

---

## 참조

| 무엇 | 어디 |
|---|---|
| 사용법·운전 절차·안전 | [`README.md`](README.md) — **원본** |
| `program/plan` 스키마·블렌딩 실측 | [`../hcr5_comm/README.md`](../hcr5_comm/README.md) §6 — **원본** |
| 관절 규약 변환·기구학 정렬 | 〃 §7 · `include/hcr5_bridge/joint_convention.hpp` |
| 두산 ROS 2 `movel` | [doosan-robot2](https://github.com/DoosanRobotics/doosan-robot2) · [예제](https://github.com/DoosanRobotics/doosan-robot2/blob/master/dsr_example2/py/dsr_example2_py/dsr_service_motion_basic.py) · [Jazzy 문서](https://doosanrobotics.github.io/doosan-robotics-ros-manual/) |
