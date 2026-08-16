# HCR-5 호스트 통신·제어 (`src/drivers/hcr5_comm`)

> **한 줄 요약** — PC를 컨트롤러에 **랜선 직결**하면, 한화 HCR-5(2018 1세대, **Rodi 1.003.005**)를
> **벤더 업그레이드 없이** 제어할 수 있다. 컨트롤러가 내부적으로 쓰는 **MQTT 버스가 그대로 외부에 열려 있고**,
> 그 위의 명령 프로토콜을 역설계해 **PC에서 로봇을 실제로 움직이는 데 성공했다** (2026-07-29).
> 2026-08-05 에 **절대 관절이동·프로그램 실행(블렌딩)·기구학 RPC** 까지 확보해, 소묘 실행에 필요한
> 명령 집합이 전부 갖춰졌다.
>
> 이전 결론([`hanwha_robot_arm/docs/실기_연결_현황.md`](../../../hanwha_robot_arm/docs/실기_연결_현황.md), 07-28)의
> "Rodi 업그레이드가 유일한 관문"은 **RoboDK/커뮤니티 ROS2 플러그인 경로에만** 해당한다. 이 문서가 그 경로 밖의
> **네이티브 통로**를 다룬다.

*조사·실증: 2026-07-29 (세션 260729-꽃무릇) · 2026-08-05 (세션 260805-봉선화). 전 과정 랜선 직결 실측.*
*ROS2 쪽 구현은 [`src/drivers/hcr5_bridge/README.md`](../hcr5_bridge/README.md).*

---

## 0. 결론

| 질문 | 답 |
|---|---|
| 회사 제공 앱/통신방식이 있나? | 컨트롤러 = **Windows PC**. 내부가 **MQTT(Mosquitto) + Express REST + MongoDB** 마이크로서비스로 돌고, 티치펜던트 UI가 이 버스로 로봇을 몬다. **이 버스가 외부에 그대로 열려 있다.** |
| CAN으로 되나? | ❌ 외부 CAN 통로 없음. CAN/EtherCAT은 컨트롤러↔관절 **내부** 버스. 외부 제어는 **이더넷(MQTT)** 이 답. |
| Rodi 1.x에서 PC 제어 가능한가? | ✅ **실증됨.** 서보 ON, 관절 조그(07-29), **절대 관절이동·프로그램 실행**(08-05)까지 PC 명령으로 성공. |
| 소묘 스트로크를 낼 수 있나? | ✅ **실증됨.** `program/plan` 의 `continues:true` 연쇄로 웨이포인트 전환에서 **TCP 속도가 0을 안 찍는다**(§6.2). |
| 명령 분해능이 선 품질 병목인가? | ❌ **아니다 — 이전 결론 반전.** 0.01°는 펜던트 UI 표시 정밀도였고, 전정밀도로 보내면 컨트롤러가 그대로 반영한다(§5.3). |

---

## 1. 물리 연결 & 네트워크

- PC 유선 NIC(`enp3s0`)를 컨트롤러 이더넷 포트에 직결.
- **컨트롤러 IP: `192.168.0.20/24`** (MAC `00:90:fb:5e:5a:01`). 공장 서브넷 192.168.0.x.
- PC를 같은 서브넷 고정 IP로: 예 `192.168.0.100/24`.
  ```bash
  nmcli connection add type ethernet ifname enp3s0 con-name hcr5 \
    ipv4.method manual ipv4.addresses 192.168.0.100/24 ipv6.method disabled autoconnect no
  nmcli connection up hcr5
  ping -c2 192.168.0.20
  ```
- **함정(08-05)**: **컨트롤러를 재부팅하면 이더넷 링크가 끊겨 이 프로필이 내려간다**(`autoconnect no`).
  통신이 죽었으면 케이블부터 의심하기 전에 `nmcli connection up hcr5` 를 다시 하라.
- 발견 방법(로봇 IP를 모를 때): `tools/discover/arpsweep.sh` 로 서브넷 ARP 스윕 → 살아있는 호스트 확인.
- ping RTT ~0.3ms, **TTL 128 → Windows 스택**.

## 2. 컨트롤러 아키텍처 (열린 포트, 전수 스캔)

| 포트 | 서비스 | 역할 |
|---|---|---|
| **1883** | **MQTT broker (Mosquitto 1.4.7, 무인증)** | ★ 로봇 실시간 메시지 버스 (상태 발행 + 명령 구독) |
| **9001** | **MQTT over WebSocket** | 펜던트 UI(브라우저/Electron)가 붙는 통로 |
| **4000 · 8000** | Node.js **Express REST API** (CORS `*`) | 라우트는 비공개(POST RPC). 미규명 — **설정 화면 일부가 이쪽으로 간다**(§8 함정 2) |
| 27017 | MongoDB | 프로그램·설정·로그 저장소 (인증 걸림) |
| 502 | Modbus TCP | I/O 통합용 (모션 지령 채널 아님) |
| 80 | Microsoft IIS/7.5 | 기본 페이지(내용 없음) |
| 135/139/445/49152~ | Windows SMB/RPC | OS 내부 |

발행 서비스명(`heartbeat` 토픽): **`hcr-control-svc`**. 브로커에 클라이언트 ~10개, 구독 ~211개.
관리자 계정 기본값 **`Admin`/`170502`, `user`/`hcr5`** — 안전 설정 화면은 로그인이 필요하다.

## 3. MQTT 상태 버스 (읽기 — 완전 수동, 안전)

`#` 구독 시 로봇이 자기 상태 전체를 브로드캐스트한다(**실측 발행률 29.1Hz**). 봉투:
```json
{"type":"pub","uuid":"<uuidv1>","data":{"thng_id":"1","data":{ ...실내용... }}}
```

| 토픽 | 내용 |
|---|---|
| `motion/joint/position` | `{base,shoulder,elbow,wrist1,wrist2,wrist3}` 도 단위 |
| `motion/tool/position` · `motion/flange/position` | `{x,y,z,rx,ry,rz}` |
| `status/robot` | `ROBOT_STATE_IDLE` 등 |
| `status/operation` | `operationStatus`(SERVO_ON/OFF), `robotState`, `controllerStatus`, `directTeach`, `limitCheck`, `collisionMitigation` |
| `status/safety` | `reduced`, `normalVelocity`, `reducedVelocity` |
| `status/program` | `status`(`PROGRAM_STATE_INIT`｜`STOPPED`), `repeat`, `start`, `repeatFlag`, `velocity` |
| `status/variables` · `status/variables/all` · `refresh/global/variables` | 전역변수 — `{"g_var_1":{"type":"number","value":100}}` |
| `monitor/robot` | `power`, `voltage`(48V), `current`, 6축 `temp` ← **서보 과열 감시용**(§8 함정 3) |
| `monitor/io/{configurable,digital,analog,tool}` | I/O 8ch씩 |
| `motion/joint/{range,speed}` · `heartbeat` · `modbus/device/error` | — |

관찰: `python3 tools/mqtt_sub.py 192.168.0.20 1883 12 '#'`

## 4. 명령 프로토콜 (쓰기) ★

**펜던트 조작을 관찰해 역설계**했다(07-29 56,723건 + 08-05 추가 캡처, joint 1·6 및 6축 교차검증).

**봉투:** `{"type":<type>, "uuid":<uuidv1>, "data":{"thng_id":1, ...인자}}`
**RPC 규칙:** `type:"pubWithAck"` 로 명령 토픽에 publish → **`uuid`와 같은 이름의 토픽**으로
`{"code":0,"data":{},"msg":"success"}` 응답이 온다. `type:"pub"` 는 단방향.
응답 알맹이는 **한 겹 안쪽**(`data.data`)에 들어 있다. **ack 왕복 실측 115~137ms.**

> ⚠️ **ack ≠ 도착.** ack 는 명령 접수일 뿐이다. 이동 완료는 로봇이 `event/motion` 으로 따로 알린다.

### 4.1 명령 표 (07-29 · 08-05 통합)

| 동작 | 토픽 | type | data 인자 | 확보 |
|---|---|---|---|---|
| 서보 ON/OFF | `set/operation` | pubWithAck | `{operationStatus:"SERVO_ON"｜"SERVO_OFF"}` | 07-29 |
| 매뉴얼/자동 모드 | `robot/mode` | pub | `{isManual:true｜false}` | 07-29 |
| 소프트 관절제한 | `set/limitCheck` | pubWithAck | `{mode:true｜false}` | 07-29 |
| **충돌 PAUSED 해제** | `event/collision/clear` | pubWithAck | `{}` | 07-29 |
| 에러 해제 | `error/reset` | — | `{}` | 08-05 |
| 컨트롤러 리셋 | `controller/system/reset` | pubWithAck | `{}` | 08-05 |
| **관절 조그 시작** | `jogJoint/start` | pubWithAck | `{joint:1~6, direction:"positive"｜"negative", speed:"45.00"}` | 07-29 |
| **관절 조그 정지** | `jogJoint/stop` | pub | `{}` | 07-29 |
| **직교 조그 시작** | `jog/start` | pubWithAck | `{axis:"x"｜"y"｜"z", direction, standard:"base", speed:"62.50", distance:-1}` | 08-05 |
| **직교 조그 정지** | `jog/stop` | pub | `{}` | 08-05 |
| **절대 관절이동** ★ | `move/joint/here` | pubWithAck | `{jointAngle:[j1..j6]}` 도 단위 | 08-05 |
| **이동 중단** | `move/stop` | pub | `{}` | 08-05 |
| 홈 복귀 | `move/joint/home` | pubWithAck | `{velocity:"22.50"}` — **관절값 없이 속도만** 싣는다 | 08-05 |
| 핸드가이드 on/off | `directTeaching/start`｜`/stop` | pub | `{}` | 07-29 |
| 현재 자세 조회 | `get/command/pos` | pubWithAck | `{}` → `{tcp:{position,orientation}, flange:{…}, joint:[6]}` | 08-05 |
| 전역 속도 조회 | `get/velocity` | pubWithAck | `{}` → `{velocity:1}` | 08-05 |
| **전역 속도 설정** | `set/velocity` | pubWithAck | `{velocity:"0.51"}` — 0~1 **배율**(§6.2) | 08-05 |
| **순기구학(FK)** | `robot/convertPose` | pubWithAck | `{info:{joint:[6]}, poseType:"tcp"}` → `{position,orientation,arrPose}` | 08-05 |
| **역기구학(IK)** | `robot/convertJointAngle` | pubWithAck | `{info:{position,orientation,joint:시드}, poseType:"tcp"}` → `{joint:[6]}` | 08-05 |
| 툴(TCP) 설정 적용 | `robot/setup/tcp` | pubWithAck | `{devc_thng_id, devc_name, info:{TCP_POSITION_XYZ, TCP_ROTATION_XYZ, TCP_GRAVITY_XYZ, TCP_USE_GRAVITY_YN, TOOL_PAYLOAD, TOOL_BOUNDARY:{INFO:{CONE,CYLINDER}}}}` | 08-05 |
| **프로그램 적재** ★ | `program/plan` | pubWithAck | 프로그램 트리 전체 — **§6** | 08-05 |
| **프로그램 실행** | `program/play` | pubWithAck | **`{}` 로 충분하다**(08-16 실증, 4회). `{selectedIndex:[a,b]}` 는 부분 실행용 **선택지**이고 필수가 아니다 — 종전 기재는 미검증이었다 | 08-05 · **08-16** |
| 프로그램 반복 | `program/set/repeat` | pubWithAck | `{programRepeatFlag:true｜false}` | 08-05 |
| 프로그램 정지 | `program/stop` | — | `{}` — 실행 중 유효성·지연은 **미실측**(§11) | 08-05 |
| 프로그램 비우기 | `program/clear` | pubWithAck | `{}` | 08-05 |

- `joint` 인덱스 1=base … 6=wrist3(§7). 각도·속도 단위 = 도. 조그 상태머신: `IDLE →(start)→ MOVING →(stop)→ STOPPING → STOPPED`.
- **`mongoLog` 토픽**이 내부 함수 호출을 중계한다(`operation.js`의 `procedure`, `operationName`). 미포착 명령을 알아내는 **지도**.

### 4.2 로봇 → PC 이벤트 (읽기 전용)

| 토픽 | 내용 | 쓰임 |
|---|---|---|
| **`event/motion`** | `{"event":"moveHere"}` | **이동 도착 완료 신호.** ack 와 별개다 — 도착 판정은 이것으로만 한다 |
| `program/index` | `{index:{0:a,1:b}}` | 실행 중인 노드 인덱스 → 진행률 추적 |
| `program/end` | — | 프로그램 종료 |
| `event/collision` | `EVENT_COLLISION_DETECTED`, code 204000 | 충돌 트립(§8) |
| `event/collision/mitigation/complete` | params = 충돌 시점 관절 float | 완화 완료 |
| `event/button` | 202100 = **E-STOP 눌림** · 202101 = 해제 | 물리 e-stop 을 소프트웨어가 관측 가능 |
| `event/exit/recording` | 의미 미확정 | — |

### 4.3 에러·경고 코드

| 토픽 | 코드 | 의미 |
|---|---|---|
| `error/network` | 200000 / sub **280002** `EVENT_ERROR_DRIVE_ERROR` | params=`[드라이브번호, 에러코드]`. 08-05 드라이브 0x40 장애(§8 함정 3) |
| `error/event` | 203000 / sub **203102** `EVENT_SAFETY_LIMIT_ACCELERATION_EXCEED` | 가속도 안전한계 초과 |
| `error/command` | **150033** `CTRLPORT_ERROR_ROBOT_MOTION_IS_NOT_RUNNABLE` | 메시지가 *"there might be singular points"* 로 **추정형이다 — 오진 주의**(§8 함정 1) |
| `warn/event` | 201104 `COMMUNICATION_SDO_READ` · 201100 `RECEIVE_ERROR_CHANGED` · 201102 `PARSE_WORKING_COUNT_CHANGED` | EtherCAT 필드버스 이상 징후 |

## 5. 제어 실증

### 5.1 2026-07-29 — 쓰기 성립·조그

1. **무동작 쓰기 검증** — PC에서 `set/limitCheck {mode:true}` 발행 → 응답 `{code:0,"success"}`,
   `status/operation.limitCheck` **off→on** 확인.
2. **실제 이동** — PC에서 `jogJoint/start`(base, 저속) → `jogJoint/stop` 시퀀스로 **base 축이 물리적으로 회전**.
   이동각 ≈ `speed(°/s) × 시간(s)`.

→ **Rodi 1.x HCR-5의 PC 제어 성립. 벤더 업그레이드 불필요.**

### 5.2 2026-08-05 — 절대 관절이동(movej) 거동

- **한 번 발행하면 목표까지 자율 주행한다.** 실측: 추가 명령 없이 **6.3초간 61° 이동 후 자력 도착**.
  → 조그와 달리 **시간 기반 정지가 성립하지 않는다.** 정지 수단은 `move/stop` 뿐이다.
- **도달오차 0.0000°.**
- **ack 는 접수, 도착은 `event/motion {"event":"moveHere"}`.** ack 왕복 115~137ms.
- 상태 발행률 29.1Hz — ROS2 `/joint_states` 주기의 상한이다.

### 5.3 분해능 — 이전 결론 반전 ★

기존 문서([`실기_연결_현황.md`](../../../hanwha_robot_arm/docs/실기_연결_현황.md) §4.3-1)는 **0.01°(reach 915mm 에서 ≈0.16mm)
분해능을 선 품질의 병목**으로 지목했다. **그것은 펜던트 UI 표시 정밀도(`%7.2f`)였다.**

| 보낸 값 | TCP 오차 |
|---|---|
| 소수 2자리 반올림(펜던트가 보내는 형태) | **0.065mm** |
| 전정밀도(double) 그대로 | **0.001mm** |

**컨트롤러는 전정밀도를 그대로 반영한다. 분해능은 병목이 아니다.**
따라서 `mqtt_cmd.py movej` 는 입력값을 반올림하지 않고 그대로 싣는다. 브릿지도 같아야 한다.

## 6. `program/plan` — 소묘 실행의 본 경로 ★

단발 자세 이동은 `move/joint/here` 로 충분하다. **연속 스트로크(선)** 는 `program/plan` 으로만 낼 수 있다 —
보간을 컨트롤러가 소유하고, 웨이포인트 전환에서 **속도가 0으로 떨어지지 않는 블렌딩**을 제공하기 때문이다.

### 6.1 스키마

> **2026-08-16 정확형으로 승격.** 종전 이 절은 `attr` 안쪽만 그린 **요약**이었고 바깥 봉투가
> 비어 있어 실제로 발행할 수가 없었다(구 업무목록 L13). 펜던트에서 linear 프로그램을 만들어
> **적용**하는 동안 `capture.py` 로 떠서 전체 형태를 얻었고, `hcr5_bridge/src/movel.cpp`
> `buildPlan()` 이 이 형태로 발행해 **실기 4회 이동으로 실증**했다.

**바깥 봉투 — 이 8개 키가 최상위다** (`data` 안에 `thng_id` 와 나란히 들어간다)

```jsonc
{
  "name":       "hcr5_bridge_movel",   // 프로그램 이름. mongoLog 에 이 이름이 찍힌다
  "variables":  {},                    // 전역변수. 안 쓰면 빈 객체
  "coordinates": {                     // 좌표계 정의. 둘 다 필요하다
    "base": { "name": "Base", "position": {x,y,z}, "orientation": {x,y,z} },
    "tcp":  { "name": "TCP",  "position": {x,y,z}, "orientation": {x,y,z} }
  },
  "velocity":   100,                   // **전역 속도 배율(%)** — 노드 velocity 에 곱한다
  "repeat":     false,
  "program":    { ROOT },              // 본체 — 아래
  "thread":     { ROOT, "uuid": 0, "child": [] },   // 안 쓰면 빈 ROOT. uuid 가 **정수 0**
  "subprogram": []
}
```

⚠️ **`velocity`(전역 %) 와 노드의 `move.linear.velocity`(mm/s) 는 다른 것이다.**
전역이 100 이어야 노드 값이 그대로 나간다. 펜던트에서 속도를 낮춰 두면 여기가 100 이 아니다.

**노드 트리**

```
ROOT
├─ INITIALIZE        ← 있어야 한다
└─ MOVE …            ← 웨이포인트 구간 하나 = 노드 하나
```

모든 노드가 **같은 6개 키**를 갖는다. `attr` 만 타입별로 다르다:

```jsonc
{ "uuid": "<uuid v1>", "name": "MOVEL", "type": "MOVE",
  "child": [], "time": 0, "attr": { … } }
```

| 타입 | `name` | `attr` |
|---|---|---|
| `ROOT` | **`null`** | `{}` (빈 객체) |
| `INITIALIZE` | `"Initialize"` | `{"skip": false, "always": false}` |
| `MOVE` | 자유 | 아래 |

```jsonc
"attr": {
  "skip":   false,
  "repeat": 1,
  "frame":      "flange",            // 기준 프레임
  "coordinate": "base" | "tcp",
  "options":  { "vision": {"position": "continuously", "type": "general"} },
  "move":     { … },                 // 6.1.1
  "waypoint": { … }                  // 6.1.2
}
```

**6.1.1 `move` — 어떻게 갈 것인가**

```jsonc
"move": {
  "selected": "joint" | "linear" | "arc" | "circle",
  "joint":  { velocity, acceleration, continues, startVelocity, endVelocity, repeat },
  "arc":    { …joint 과 같은 키… },
  "circle": { …joint 과 같은 키… },
  "linear": { …위 키 + radius }      // radius = 블렌딩 반경
}
```

- 네 종류의 파라미터 객체가 **모두 실린다.** `selected` 가 그중 하나를 고른다.
- **기본값**: `joint` velocity 50 / acceleration 100, `linear` velocity 500 / acceleration 1000 / radius 0.
- **단위**: `linear` 는 mm/s·mm/s², **`joint` 는 도/s·도/s²** — 2026-08-16 실기 **182점**으로 확정
  (**6축 전부**, d 30~360°, v 10~90°/s, a 25~400°/s². 원본 = `hcr5_bridge/movej.md` §2.3).

  **지령 속도는 오차 0.13% 로 그대로 지켜진다** (회귀계수 K=0.9987±0.0019). rad/s 면 20 rad/s=1146°/s 라
  0.05초, 정격 대비 %(180°/s) 면 36°/s 라 1.9초여야 했는데 실측은 3.48초 — 자릿수로 배제된다.

  **⚠️ 가감속은 사다리꼴이 아니다.** 램프에 `v/a` 의 **1.54배**가 걸린다:

  ```
  t = 0.9987·(d/v) + 1.5392·(v/a) + 0.2034초    RMS 잔차 0.0365초 · 최대 0.1355초 (182점)
       ±0.0019        ±0.0053        ±0.0055
  ```

  | 모형 | RMS 잔차 | 최대 잔차 |
  |---|---|---|
  | 순수 사다리꼴 `d/v + v/a` | 0.68초 | — |
  | **`d/v + 1.54·v/a + c`** ← 채택 | **0.037초** | **0.136초** |
  | 위에 저크항(`a/j`)·속도 1차항을 더한 4모수 | 0.037초 | — (**나아지지 않는다**) |

  실무적 함의 셋:
  - **지령 가속도의 실효값은 `0.65·a`** 다. `acceleration:100` 은 65°/s² 처럼 거동한다.
  - **가감속 소요 각도는 `0.77·v²/a`** — 사다리꼴로 계산한 `v²/a` 보다 23% 작다. §8 함정 1 의
    가감속 여유 계산에 이 값을 쓰면 덜 보수적이다 (막고 싶으면 `v²/a` 를 그대로 쓰는 쪽이 안전).
  - ✅ **`c` 는 상수다.** play ack 왕복 + 실행 시작·정지 판정 지연이고, 속도·거리·가속도와도
    **시각과도** 무관하다. 다만 **소수 셋째 자리를 믿지 마라** — 회차 하나하나의 산포가 0.035초라
    묶음평균이 0.19~0.22 사이에서 흔들린다.
    🔴 한때 *"세션 중 0.189 → 0.226 으로 드리프트한다"* 고 적어 두었으나 **기각됐다**: 같은 지령
    80회를 29분(중간 12.7분 정지)에 걸쳐 돌려 시간축만 흔들었더니 기울기가 −0.010±0.005 /
    +0.003±0.005 초/10분 으로 **부호부터 갈렸다.** 묶음평균 넷에서 추세를 읽은 오독이었다.

  🔴 **π/2=1.5708 은 아니다 — 6.0σ 로 배제된다.** 한때 코사인형 램프를 시사한다고 적어 두었으나,
  `a` 만 25~400 으로 흔들어 `v/a` 지렛대를 16배 벌리자 갈렸다 (그 30점 단독으로도 B=1.5409±0.0085).
  저크 제한항(`a/j`)·속도 1차항·`√(v/a)` 를 각각 넣어 4모수로 풀어도 자유도 보정 RMS 가 3모수와
  같다 — **3모수가 맞다.**

  **✅ 여섯 축이 모두 같은 모형을 따른다.** 6축에 똑같은 지령(d=60°·v∈{30,60}·a=100)을 준 균형설계
  24점에서 축 평균 잔차의 산포가 0.0116초로, 우연만으로 기대되는 0.0157초보다 **작다**.
  관성이 10배 넘게 차이 나는 J1(팔 전체 회전)과 J6(툴축 회전)이 같은 식 위에 있다.

  **✅ 중력 방향은 무관하다.** J2(어깨, 중력 토크 최대)를 같은 지령으로 올렸다 내렸다 한 잔차 차이가
  **0.0000초**였다. 들어올리는 쪽이 느릴 것이라는 직관이 틀렸다 — 컨트롤러가 프로파일을 먼저 지킨다.

  **다축 동시 이동은 최대 각변위 축(선행축)이 시간을 지배한다** — 벡터 노름이 아니다.
  J5 90°+J6 90° 는 선행축 예측 3.64 / 노름 예측 4.89 에 대해 실측 **3.68초**.

  ⚠️ 이 측정은 **전역 배율이 1** 인 것을 `get/velocity` 로 확인하고 잰 값이다(§6.2). 재현 시 먼저 볼 것.
  ⚠️ 오전 2표본으로 낸 `k=1.017` 은 **폐기됐다** — 사다리꼴을 강제한 탓에 램프 부족분이 주행항으로
  밀린 허상이다. 미지수 2개를 표본 2개로 풀면 항상 정확히 맞으니 잔차가 검증을 못 한다.
- ⚠️ **노드마다 값이 다르다.** §8 함정 1 의 사고 프로그램은 `linear` 를 velocity 500 / **acceleration 100** 으로
  쓰고 있었다. 가감속 여유 계산은 기본값이 아니라 **그 노드의 실제 값**으로 하라.
- `radius` 는 **`linear` 에만 있다.**

**6.1.2 `waypoint` — 어디로 갈 것인가**

```jsonc
"waypoint": {
  "middlePoint": { "selected": "fixed" | "relative" | "variable",
                   "fixed": { "tcp":    {position, orientation},
                              "flange": {position, orientation},
                              "joint":  [j1..j6] } },
  "endPoint":    { … 동일 구조 … }
}
```

**웨이포인트 하나가 tcp·flange·joint 세 표현을 동시에 담는다.** 표현이 셋이므로 JSON 이 빠르게 커진다 —
스트로크 수백 개면 수 MB 급이 된다(§11 미해결 1).

- **세 표현은 서로 정합해야 한다.** `joint` 는 `robot/convertJointAngle`(IK)로 풀어 넣는다 —
  IK→FK 왕복 오차 **0.000000mm** 실측(2026-08-16)이라 믿고 써도 된다.
- **`middlePoint` 는 `linear` 에서 안 쓰이지만 채워 둔다.** arc·circle 용 칸인데,
  캡처가 현재 자세로 채워 보내고 있어 그대로 따랐다. **비웠을 때 파서가 어떻게 되는지는 미확인.**

**6.1.3 발행 순서 — `clear` → `plan` → `play` → `program/end`**

```
program/clear   {}      pubWithAck   ← 이전 프로그램을 비운다
program/plan   {위 봉투} pubWithAck   ← 적재. MOVE 노드 1개면 약 4.3KB
program/play    {}      pubWithAck   ← ⚠️ 여기서 로봇이 움직인다. 서보 ON 필요
program/end                          ← 도착. ack 가 아니라 이걸로 판정한다
```

> 🔴 **`program/play` 는 서보가 꺼져 있어도 `code:0` 으로 정상 ack 한다** (2026-08-16 실측).
> `clear`·`plan`·`play` 세 ack 가 전부 성공인데 로봇은 1mm 도 안 움직였고, 자세를 되읽으니
> 소수점 끝자리까지 발행 전과 같았다. **ack 로는 이 상황을 알 수 없다** — 유일한 신호는 도착 타임아웃이다.
> 발행 전에 `status/operation` 의 `operationStatus` 를 확인하라(약 20Hz 로 올라온다).
> `hcr5_bridge` 는 이것을 게이트 ⑤ 로 넣었다 — `include/hcr5_bridge/servo_gate.hpp`.

⚠️ **`program/play` 에 인자를 안 줘도 된다.** §4.1 이 적어 둔 `{selectedIndex:[a,b]}` 는
08-05 캡처 기반의 **미검증** 기재였는데, `{}` 로 **4회 연속 정상 실행**했다(2026-08-16).
`selectedIndex` 는 부분 실행용 선택지이지 필수 인자가 아니다.

> 🔴 **적재 여부를 펜던트 파일 목록으로 판정하면 안 된다.** `program/plan` 으로 올린 프로그램은
> 펜던트 `HTW Storage` 의 `.file` 목록에 **안 뜬다** — 두 층이 다르다.
> 컨트롤러 `mongoLog` 대조 결과 우리 발행도 펜던트 적용과 **똑같이** `programEvent : program_plan`
> 을 남기고 `status/program` 이 `PROGRAM_STATE_INIT` 으로 전이한다. 차이는 펜던트 쪽에만 붙는
> `clickApplyProgram`("send program file to server") 한 단계뿐이고, `.file` 을 만드는 것이 그 단계다.
> **적재 확인은 ack + `status/program` 으로 한다.**

### 6.2 블렌딩 실증 — 스트로크가 실행 가능하다

같은 프로그램 안에 대조군이 잡혔다. **웨이포인트 전환 시 TCP 최저속도**:

| 전환 조건 | TCP 최저속도 | 해석 |
|---|---|---|
| `continues:false`, `endVelocity:0` | **0.02 mm/s** | 완전 정지 |
| `continues:false`, `endVelocity:0` | **0.27 mm/s** | 완전 정지 |
| `continues:true`, endVel 50 → startVel 50, `radius:50` | **18.93 mm/s** | 연속 통과 |
| `continues:true`, endVel 50 → startVel 50 | **20.08 mm/s** | 연속 통과 |

**→ `continues:true` 로 이으면 속도가 0을 찍지 않는다(19~20mm/s 유지). 소묘 스트로크 실행 가능이 확인됐다.**

**궤적 정확도**

| 항목 | 실측 |
|---|---|
| `radius:0` 웨이포인트 통과 오차 | **0.34mm** (정확 통과) |
| `radius:50` 웨이포인트 통과 오차 | **22.3mm 떨어져 지나갔다** — 모서리를 깎는다 |
| `linear` 직선성 | 179.3mm 구간에서 직선 이탈 **최대 0.045mm** — 로봇 반복정밀도(±0.1mm)보다 좋다 |

**전역 velocity 는 배율이다** — `set/velocity`(0~1) 값이 노드 velocity 에 곱해진다: 50mm/s × 0.51 = 실측 20~31mm/s.

### 6.3 스트로크 체인 구성법 (그대로 쓸 것)

```
시작 노드:  {startVelocity:0, endVelocity:V, continues:true,  radius:0}
중간 노드:  {startVelocity:V, endVelocity:V, continues:true,  radius:0}
종료 노드:  {startVelocity:V, endVelocity:0, continues:false, radius:0}
```

> ⚠️ **`radius`>0 을 스트로크 내부에 쓰지 마라.** radius=50 이면 궤적이 그 웨이포인트를 **22.3mm 떨어져**
> 지나간다 — 선이 뭉개진다. 선 모양을 지키는 조합은 **`radius:0` + `continues:true`** 뿐이다.

> ⚠️ **속도 프로파일이 물리적으로 가능해야 한다.** 안 그러면 "특이점" 으로 **오진**된다(§8 함정 1).

## 7. 기구학 정렬 — 빠뜨리면 조용히 틀린다 ⚠️

**이 절을 브릿지가 빼먹으면 에러가 나지 않는다. 로봇이 조용히 엉뚱한 자세로 간다 — 가장 위험한 실패 방식이다.**

### 7.1 관절 매핑 (조그 실측 확정)

6축을 하나씩 순차 조그해 `motion/joint/position` 의 어느 키가 변하는지로 확정했다.

- `joint` 인덱스 **1~6 = base · shoulder · elbow · wrist1 · wrist2 · wrist3**. URDF `joint_1~6` 과 같은 순서.
- **축간 간섭 없음.** `positive` 조그 시 **6축 모두 값이 증가**한다(부호 방향 일치).
- **영점 오프셋 없음** — 펜던트 표시값 = 버스 값.

### 7.2 URDF 규약 변환

실기와 URDF 는 **영점 규약이 다르다.** FK RPC(`robot/convertPose`)로 6축을 20°씩 전 회전 스윕해
(**108 샘플, 원 적합 RMS 0.000mm**) 실기 기구학을 추출하고 URDF 와 대조한 결과, **정렬돼 있지 않았다**.

```
q_URDF[i](도) = SIGN[i] * q_real[i](도) + DELTA[i]
  SIGN  = (+1, +1, −1, +1, +1, +1)      ← J3 만 부호 반전
  DELTA = ( 90,  90,   0,  90,   0,  0)
```

- 실기 zero = 팔이 **수평으로 뻗은** 자세, URDF zero = 팔이 **수직으로 선** 자세
- 실기 홈 `[0, −90, −90, −90, 90, 0]` = URDF `[90, 0, 90, 0, 90, 0]`
- 그 자세의 flange = **(490.0, −170.5, 441.5) mm, rx = −180°**

### 7.3 URDF origin 교정

| 항목 | 기존 | 교정 | 비고 |
|---|---|---|---|
| `joint_6.x` | −0.089596 | **−0.132498** | **−42.9mm — 지배적 오류** (이것만 고쳐도 RMS 43.4→1.6mm) |
| `joint_2.x` | −0.0785 | −0.059138 | |
| `joint_3.x` | 0.000652 | −0.019348 | |
| `joint_3.y` | 0.001 | 0 | |
| `joint_3.z` | 0.424 | 0.425001 | |
| `joint_4.z` | 0.338567 | 0.338499 | |
| `joint_5.z` | 0.089596 | 0.089504 | |

**교정 결과: 109개 자세에서 위치오차 RMS 43.4mm → 0.0060mm** (최대 0.0130mm).
limit 도 CAD 더미값에서 매뉴얼 공식값으로 교체했다. mesh 는 CAD 기준 그대로라 joint_6 을 옮긴 만큼 시각화가 어긋난다(별도 과제).
근거 주석은 `hanwha_robot_arm/HCR_5/hcr_robot_description/urdf/hcr_robot.xacro` 의 **관절 정의부 앞** 블록.

### 7.4 TCP 오프셋

현재 등록된 `tool1` = `{X:−52.25, Y:−13.49, Z:116.12}` → **오프셋 실측 128.05mm**(회전 0).
펜 장착 시 `robot/setup/tcp` 로 갱신하거나, 전 구간을 flange 기준으로 제어하고 **펜 오프셋을 PC 가 소유**하는
우회가 가능하다.

## 8. 안전 ⚠️

- **PC MQTT 명령은 펜던트 인에이블 스위치(데드맨)를 거치지 않는다.** 첫 이동·실험 시 **e-stop에 손, 로봇 반경 정리** 필수.
  **물리 e-stop 만이 최후 수단이다.**
- `jogJoint/start` 는 **개루프 속도 조그**(정지 명령까지 계속 이동). `tools/mqtt_cmd.py` 의 `jog` 는
  `start → sleep → stop` 을 `finally` 로 감싸 **정지를 항상 보장**하고, 시작 응답 대기를
  **회전시간 예산 이내(최대 2초)** 로 묶어 과주행을 막는다.
- **`move/joint/here` 는 목표까지 자율 주행한다.** 시간 기반 정지가 성립하지 않는다 — `movej` 의 **3중 가드**(§9)를
  우회하지 말고, 직접 발행할 일이 있으면 `move/stop` 을 손에 쥐고 하라.
- **ack ≠ 도착.** `event/motion` 을 못 받았는데 도착했다고 가정하지 마라.
- `set/limitCheck {mode:true}` 로 **소프트 관절제한을 켜 두는 것**을 권장(공식 가동범위 밖 자기정지).
- `status/safety.reducedVelocity` = 감속 모드 속도 상한. 실험은 감속·저속으로.
- **서보를 켠 채 오래 두지 마라** — 홀딩 토크로 축 온도가 오른다(함정 3).

### 충돌 감지·복구 (2026-07-29 실증)

모터 전류 기반 충돌감지가 **실제로 트립**한다(외부 센서 아님). 저속 조그 중 손으로 저항해 유발 → 검증됨.
전이(전부 MQTT로 관찰 가능):

```
MOVING ─(충돌)→ event/collision (EVENT_COLLISION_DETECTED, 204000)
       → MOVING → PAUSING → PAUSED   (isCollision=true, directTeach 자동 enable)
       → event/collision/mitigation/complete
       → PAUSED → STOPPED             (isCollision=true 로 래치 유지)
       ─(복구)→ event/collision/clear {thng_id:1}   → isCollision=false, 정상 복귀
```

- **소프트 관절제한(limitCheck)은 래치 안 됨** — 한계에서 얌전히 정지, 리셋 불필요.
- **충돌은 래치됨** — `PAUSED` 로 멈추고 `event/collision/clear` 로 명시적 해제 필요.
- **핵심:** 감지·정지·해제·재개 전 과정을 **PC/ROS2로 처리 가능**(펜던트 불필요). 물리 e-stop만 설계상 수동.
- 자율 운전 시: `event/collision`·`isCollision` 을 감시하고, 백오프 후 `event/collision/clear` 로 자동 복구하거나 사람 개입으로 정지하는 정책을 ROS2 층에 둔다.

### 함정 (2026-08-05 실측 — 재발 방지)

**1. 150033 "특이점" 은 오진하기 쉽다.**
메시지가 *"there might be singular points"* 로 **추정형**이다. 08-05 의 실제 원인은 **불가능한 속도 프로파일**이었다:
`acceleration:100`mm/s² 로 `velocity:500`mm/s 에 도달하려면 **1250mm** 가 필요한데 구간은 **112~261mm** 였고,
`startVelocity:500` 은 직전 구간이 낼 수 없는 속도였다.
**velocity 를 50 으로 낮추자 무에러 완주.** 기구학적으로도 전 웨이포인트가 J5=90° 라 손목 특이점(J5=0)에서 가장 먼 자세였다.
→ 이 에러가 나면 **자세를 의심하기 전에 속도·가속도·구간거리를 먼저 계산하라.**

**2. 설정 화면은 MQTT 를 안 쓸 때가 있다.**
툴·속도 설정 조작이 캡처에 **전혀 안 잡힌 세션**이 있었다(`mongoLog` 0건). REST(4000/8000)로 가는 것으로 보이고
라우트 추측 탐색은 전부 404 였다. 단 **적용까지** 하면 `robot/setup/tcp` 가 MQTT 로 실린다 — **등록만 하고 적용을 안 하면 안 실린다.**

**3. 드라이브 0x40 장애.**
**아무 명령도 없는 상태에서** 6축 드라이브가 **0.8초 만에 전부 트립**(`error/network` 280002 `EVENT_ERROR_DRIVE_ERROR`).
드라이브 1부터 순차 전파(EtherCAT 데이지체인). 이후 `SDO_READ` 경고 **122건**, Working Counter 이상 **3160회**로
필드버스 두절. **소프트 리셋 실패 → 컨트롤러 재부팅으로 복구.**
배경: 축 온도가 세션 시작 **37°C → 장애 직전 58~61°C**(서보를 1시간 반 켜둔 채 방치).
**단정은 못 한다** — 가장 뜨거운 축은 wrist3 인데 먼저 죽은 건 base 였다. 그래도 **서보를 켠 채 오래 두지 말 것.**
**08-05 세션은 서보를 켜둔 채 끝났다.** 실기를 만지기 전에 `mqtt_cmd.py pos` 로 통신을 보고
`monitor/robot` 의 `temp` 를 먼저 확인하라. 60°C 를 넘었거나 `-1`(통신 두절)이면 재발이다.

**4. 컨트롤러 재부팅 → 이더넷 프로필이 내려간다.** `nmcli connection up hcr5` 재실행(§1).

**5. 관리자 계정.** 안전 설정 화면은 로그인 필요 — 기본값 `Admin`/`170502`, `user`/`hcr5`.

## 9. 도구 (`tools/`)

| 파일 | 용도 | 쓰기? |
|---|---|---|
| `mqtt_sub.py` | 임의 토픽 구독 관찰 (`host port sec topic`) — **신규 토픽만** 찍는다 | 읽기 |
| `mqtt_full.py` | 핵심 토픽 전체 페이로드/스키마 덤프 | 읽기 |
| `mqtt_baseline.py` | 순수 구독 — 착수 게이트 계측(축온 · `controllerStatus`) | 읽기 |
| `mqtt_trace.py` | **전량 타임스탬프 JSONL** — 발행 간격·선점·이벤트 순서 (`host sec out.jsonl`) | 읽기 |
| `capture.py` | 명령 캡처 — baseline(상태) 대비 **새 명령 토픽만** 분리 기록 (`host port outdir`) | 읽기 |
| `mqtt_logger.py` | **세션 통째로 돌려 두는 장시간 로거** — 회전(6h·겹침 1h)·5초 재접속·이상징후 분리·요약 | 읽기 |
| `mqtt_cmd.py` | **명령 송신** — 아래 서브커맨드 | **쓰기** |
| `ros_trace.py` | **ROS 쪽** — 추종오차·값 갱신빈도·velocity 품질 (`sec out.json`) ⚠️ 컨테이너에서 실행 | 읽기 |
| `discover/arpsweep.sh` | 서브넷 ARP 스윕(호스트 발견) | 읽기 |
| `discover/portscan.sh` | TCP 포트 스캔 | 읽기 |

`ros_trace.py` 를 뺀 전부가 순수 Python 표준 라이브러리다(브로커 라이브러리 불필요). 대상 IP는 각 스크립트 상단/인자로 조정.

**집계와 시각은 다른 도구다** — `mqtt_sub.py` 는 신규 토픽만 찍어 **0건이 '안 왔다'인지 '안 듣고 있었다'인지 구분되지 않는다**.
발행 간격·순서처럼 **시각이 있어야 재는 것**은 `mqtt_trace.py` 로 뜬다. 플러그인은 성공 발행을 로그로 남기지 않으므로
(실패만 찍는다) 재발행 간격은 ROS 로그가 아니라 **버스에서** 떠야 한다.

⚠️ **`ros_trace.py` 는 컨테이너 안에서 돈다**(rclpy·control_msgs). 그런데 `compose.yml` 은 `ws_moveit2` 와
`hanwha_robot_arm` 만 마운트하고 **`src/drivers/` 는 마운트하지 않아 이 파일이 컨테이너에 안 보인다.** 넣어서 쓴다:
```bash
docker cp tools/ros_trace.py markch_moveit2_dev:/tmp/ros_trace.py
docker exec markch_moveit2_dev bash -lc 'source install/setup.bash && python3 /tmp/ros_trace.py 45'
```
마운트 범위 확장은 **T15 곁가지**로 열려 있다 — 닫히면 이 복사 단계가 없어진다.

`mqtt_trace.py`·`ros_trace.py` 는 **T15 C 구간(실기 쓰기) 계측기**다. 왜 이 값들을 재는지는
[`guideline/중간결과물.md`](guideline/중간결과물.md) §3-C.

### `mqtt_logger.py` — 세션 통째로 거는 로거 (2026-08-16 신설)

`mqtt_trace.py` 가 **짧은 창의 계측기**라면 이쪽은 **작업 세션 내내 걸어 두는 기록기**다.
`#` 구독이라 **펜던트 조작이든 PC 명령이든 같은 파일에 들어간다** — 명령 출처와 무관한 수집이 성립한다.

```bash
python3 tools/mqtt_logger.py            # ~/hcr5_logs 에 쌓고, Ctrl-C 로 종료+요약
```

출력 3종 — 원본(`hcr5_<시각>.csv`, 회전) · 이상징후·명령만(`events_<시각>.csv`) · 요약(`summary_<시각>.md`).
원본은 수신 **원문 JSON 을 `payload` 열에 그대로** 넣는다(봉투를 안 푼다). 압축하지 않는다.

- **회전·겹침** — 로거 시작 시각을 원점으로 6시간마다 자르고 앞뒤 30분씩 겹쳐 쓴다(파일 하나 7시간, 겹침 1시간).
  겹침 구간은 같은 레코드가 두 파일에 들어간다 — `t_iso` 로 중복 제거하면 원본이 복원된다.
  **파일 이름은 그 파일에 처음 기록한 실제 시각**이다
- **재접속** — 5초 고정 주기로 **무한** 재시도. 로봇보다 먼저 띄워도 되고, 컨트롤러가 재부팅돼도 붙는다.
  실패 로그는 첫 1건만 남기고 이후는 센다(전원이 몇 시간 꺼져 있어도 로그를 안 덮는다)
- **공백이 증거다** — 끊김은 `_logger/disconnect`·`_logger/reconnect` 로 파일 안에 남는다.
  **이 로그의 공백을 "조작이 없었다"로 읽으면 안 된다** (컨트롤러가 죽으면 브로커도 같이 죽는다)
- **관절값 튐 진단** — 표본 **간격**과 **가속도(2차 차분)** 를 같이 본다. `dt>0.2s` 면 `_logger/gap`(결측),
  간격이 정상인데 `|Δv/Δt|>4000°/s²` 면 `_logger/joint_jump`, 속도가 `180°/s`(공식 최대) 초과면 `..._hard`.
  **결측과 튐을 섞으면 오진한다** — 결측 구간의 차분은 근거가 못 된다
  > ⚠️ **변화량(Δ) 임계는 폐기했다.** 초판은 `|Δ|>1.0°`(08-11 캡처 최대 0.273° 의 3.7배)를 썼는데,
  > 08-16 실기 조그가 40~63°/s 로 돌면서 34ms 에 1.4~2.1° 가 정상으로 나와 **오탐 51건**이 됐다.
  > Δ 임계는 결국 "몇 °/s 넘으면 튐"이라는 뜻이고, 정상 속도와 이상 속도는 그렇게 안 갈린다.
  > **튐은 빠른 게 아니라 속도가 불연속으로 꺾이는 것**이라 가속도로 바꿨다.
  > 🔻 **한계 둘.** ① 34ms 간격에서 4000°/s² 는 **단발 불연속 약 4.6° 이상**만 잡는다(2°는 못 잡는다).
  > ② 임계를 **63°/s 까지만 돌린 실측**에서 뽑았다 — 정격(180°/s)에 가깝게 돌리면 정상 가감속도
  > 같이 커져 오탐이 돌아올 수 있다. **속도를 크게 올리면 임계를 다시 재라** (`--acc-limit`)
- 판정하지 않는다. `error/command` 150033 의 *"there might be singular points"* 가 추정형이라(§8 함정 1)
  원문과 정황만 모으고 해석은 사람이 한다

⚠️ 이 로거는 **`monitor/robot` 축온을 화면에 경고하지 않는다**(조용히 기록만 한다). 착수 게이트로 축온을
보려면 `mqtt_baseline.py` 를, 운전 중 감시가 필요하면 감시견을 따로 띄운다.

검증 근거·재현 도구: [`measurements/260816_로거검증/`](measurements/260816_로거검증/)

### `mqtt_cmd.py` 서브커맨드

| 서브커맨드 | 토픽 | 비고 | 쓰기? |
|---|---|---|---|
| `servo on｜off` | `set/operation` | | **쓰기** |
| `mode manual｜auto` | `robot/mode` | | **쓰기** |
| `limitcheck on｜off` | `set/limitCheck` | | **쓰기** |
| `jog <1-6> <positive｜negative> [speed] [ms]` | `jogJoint/start` → `stop` | `finally` 로 정지 보장 | **쓰기** |
| `stopjog` | `jogJoint/stop` | 수동 즉시정지 | **쓰기** |
| **`movej <j1..j6> [--max-delta D] [--timeout S]`** | `move/joint/here` | **3중 가드**(아래) | **쓰기** |
| `movestop` | `move/stop` | movej 중단 | **쓰기** |
| `clearcollision` | `event/collision/clear` | 충돌 PAUSED 해제 | **쓰기** |
| `directteach on｜off` | `directTeaching/start｜stop` | 핸드가이드 | **쓰기** |
| `send <topic> <json> [ack]` | 임의 | **가드 있음**(아래) | **쓰기** |
| `pos` | `get/command/pos` | 관절각·TCP·flange | 읽기 |
| `fk <j1..j6>` | `robot/convertPose` | 관절각 → TCP 포즈 | 읽기 |
| `ik <x y z rx ry rz>` | `robot/convertJointAngle` | TCP 포즈 → 관절각(시드는 현재 자세 자동) | 읽기 |

**`movej` 3중 가드** — `move/joint/here` 는 자동 정지가 없으므로:
① **발행 전** `get/command/pos` 로 현재 자세를 읽어 **최대 이동량이 `--max-delta`(기본 5°, 허용 0<D≤30) 이내**인지 검증한다.
넘으면 발행하지 않고 죽는다. 현재 자세를 못 읽어도 죽는다.
② 목표가 **공식 관절한계**(±360°, J3 ±165° — User Manual v2.0 Appendix F) 밖이면 죽는다.
③ 도착 신호 `event/motion{"event":"moveHere"}` 를 `--timeout`(기본 30s)까지 기다리고, **못 받으면 `finally` 에서 `move/stop`** 을 쏜다.

**`send` 가드** — `jogJoint/start` 와 `move/joint/here` 는 **거부한다**. 위 가드를 우회하는 경로이기 때문이다.
인자 오타도 조용히 삼키지 않는다(`on｜off` 등은 정확히 그 문자열, 아니면 종료코드 2. 축약형 `pos`/`neg` 는 허용).

**명령 예시**
```bash
python3 tools/mqtt_cmd.py servo on                 # 서보 ON
python3 tools/mqtt_cmd.py limitcheck on            # 소프트 제한 ON
python3 tools/mqtt_cmd.py pos                      # 읽기 전용 — 연결 확인에 먼저 쓴다
python3 tools/mqtt_cmd.py jog 1 positive 10 300    # base 축 저속 0.3초(≈3°)
python3 tools/mqtt_cmd.py stopjog                  # 즉시 정지
python3 tools/mqtt_cmd.py fk 0 -90 -90 -90 90 0    # 실기 홈의 TCP 포즈
python3 tools/mqtt_cmd.py movej 0 -90 -90 -90 90 2 # 절대 관절이동(가드 통과 시에만 발행)
python3 tools/mqtt_cmd.py movestop                 # movej 중단
```

## 10. 웹 조사 교차검증 (2026-07-29)

- **RoboDK 경로** = 컨트롤러가 여는 **TCP 7000** 평문 라인 프로토콜(`VERSION`/`CJNT`/`MOVJ j1..j6`/`RSTATE`).
  Rodi-X 플러그인(asar)이 `net.createServer`로 연다. **Rodi ≥ 2.001.003.012 요구 → 우리 1.003.005엔 차단.**
  ("6667" 설은 오류, 실제 7000.)
- 두 경로(RoboDK / 우리 MQTT)는 **같은 하부 제어 레이어**(`rodix_api`의 `commandModel.move_joint([j1..j6])`)를 구동.
  모션 원자 = **6관절 도단위 배열** → 우리 `motion/joint/position` 과 대칭이라, **절대이동 명령도 6관절 target을 실을 가능성 높음.**
  → **08-05 에 적중 확인.** `move/joint/here {jointAngle:[j1..j6]}` 가 정확히 그 형태였다.
- 한화 공개 매뉴얼엔 MQTT/REST 스펙 **전무**(내부 미공개). **경험적 캡처가 유일한 규명 경로였다.**

## 11. 다음 단계

**해소됨 (08-05)**
- [x] 절대 관절이동(`move/joint/here`)·프로그램 실행(`program/plan`·`program/play`)·웨이포인트 스키마 역설계 → §4·§6
- [x] 명령 분해능·지연 측정 → **분해능은 병목이 아니다**(§5.3), ack 왕복 115~137ms, 상태 29.1Hz
- [x] 실기 ↔ URDF 기구학 정렬 → §7 (RMS 0.0060mm)

**남은 것 (우선순위 순)**
- [ ] **`program/plan` 크기 한계 실측** — 스트로크 수백 개 × 웨이포인트당 tcp+flange+joint 3표현이면 수 MB 급 JSON.
      Mosquitto 1.4.7·컨트롤러 파서가 받아줄지 미실측이고 **완주(AC1)의 잠재 차단 요인**이다. 캡처가 아니라 **부하 실험**으로만 확정된다.
      한계가 있으면 `program/play {selectedIndex:[a,b]}` 부분 실행으로 **배치 분할**이 가능하다.
- [ ] **즉시정지 지연 실측** — `program/stop` 은 확보됐으나 실행 중 유효성·지연(BRD N4 "즉시")이 미실측.
- [ ] **펜 장착 TCP 결정** — `robot/setup/tcp` 로 설정 vs 전 구간 flange 제어 + 오프셋을 PC 가 소유.
- [ ] **필압 ↔ 충돌감지 간섭 실험** — 펜이 종이를 누르는 반력이 전류 기반 충돌감지를 트립시키면 PAUSED 래치로 작화가 반복 중단된다.
- [ ] 미규명 스키마 — Script 노드 컨테이너 형식 · `arc`/`circle` 의 `middlePoint` · waypoint `relative`/`variable` · `:4000`/`:8000` REST 라우트.
- [ ] **ROS2 궤적 실행 층** — `FollowJointTrajectory` 액션 서버 → `program/plan` 변환 → `program/play`.
      설계는 [`hcr5_bridge/README.md`](../hcr5_bridge/README.md) §5. **접점 ①**.
- [ ] **ros2_control 하드웨어 인터페이스** — **접점 ②**. 진행 중이다 → **§12**.

## 12. ros2_control 하드웨어 인터페이스 (T15) — 현황

같은 폴더의 [`guideline/`](guideline/) 가 이 작업의 **문서 3종**을 든다.
§4 의 명령 프로토콜을 ros2_control **하드웨어 컴포넌트**로 감싸 MoveIt2 가 실기를 직접 잡게 하는 경로다.
§11 의 액션서버 경로(**접점 ①**)와 **다른 접점**이고, 둘은 배타가 아니다 — 어느 쪽으로 수렴하는지는
`guideline/최종결과물.md` '차이' 절이 판정한다.

**구현 실물은 같은 폴더의 [`hcr5_bridge/`](../hcr5_bridge/) 다.** §4 의 명령 프로토콜을 ros2_control
하드웨어 컴포넌트로 감싼 플러그인 `hcr5_bridge/HcrSystemInterface` 와 MQTT 클라이언트·관절규약이
한 패키지에 들어 있다. 개요·설치·실행이 전부
[`hcr5_bridge/README.md`](../hcr5_bridge/README.md) 한 곳에 있다 — 기동·운전은 §3.
(종전 `운전절차.md` · `RUNTIME.md` 는 2026-08-16 에 그 README 로 흡수됐다.)

> **2026-08-15 에 두 단계로 옮겼다.**
> ① **르누아르** — `src/moveit2/ws_moveit2/src/hcr_bridge/` → `src/drivers/hcr_comm/hcr5_bridge/`.
> 종전 배치의 근거는 *"`compose.yml` 이 `src/drivers/` 를 마운트하지 않아 컨테이너에서 안 보인다"* 였는데,
> `compose.yml:72` 가 `../../src/drivers` 를 컨테이너 `ws_moveit2/src/drivers` 로 마운트하면서
> **그 제약이 사라졌다**. hw interface 계층과 MoveIt2(계획·검증)의 기능 경계를 가르는 것이 목적이었다.
> ② **앵그르** — `hcr_comm/` 이 **`hcr5_mqtt`(이 문서)·`hcr5_bridge`·`hcr5_measurements`·`ros2_control_hw_interface`**
> 넷으로 분해되며 `hcr5_bridge` 가 `drivers/` 바로 아래로 올라갔다 (Mark 지시).
> **같은 근거로 두 번 옮긴 것이 아니라 상위 지시가 바뀐 것이다.**
> 개명(`hcr_bridge`→`hcr5_bridge`)은 HCR-5 모델을 이름에 명시하고 동료 ysh 의 `hcr5_description`
> 계열과 맞추기 위한 것이다(Mark 판정). **CMake 타깃명·클래스명 `HcrSystemInterface` 는 유지**했다.
>
> **2026-08-16 — 세 번째 단계.** ② 의 4형제 평면 배치가 **`hcr5_bridge` 와 나머지 셋** 으로 다시 갈렸다 (Mark 지시).
> 가른 축은 **빌드되느냐**다 — `package.xml` 을 가진 ROS 패키지는 `hcr5_bridge` 하나뿐이고,
> 나머지 셋은 전부 그것을 **둘러싼 문서·도구·증거**였다. 그래서 셋을 `hcr5_comm/`(이 문서, `hcr5_mqtt` 에서 개명) 아래로 모았다:
> `tools/`(도구) · `measurements/`(증거) · `guideline/`(판정 — 종전 `ros2_control_hw_interface/`).
> `hcr5_mqtt` → `hcr5_comm` 개명은 **`measurements/` 가 MQTT 전용이 아니기 때문**이다 —
> 버스를 만지는 스크립트 20개 중 **16개가 ROS 를 쓴다**(실측). `hcr5_mqtt` 아래에 두면 이름이 내용보다 좁아진다.

### 문서 3종 진척

| 문서 | 역할 | 남은 것 |
|---|---|---|
| `중간결과물.md` | 계약·실측 | **'완료 · 실기 미검증' 칸** — B·C 실기 대면으로만 닫힌다 |
| `검증.md` | 기준↔실측 판정 | **집계의 원본은 [`검증.md`](guideline/검증.md) 자신이다** — 통과/실패/미검증 수치를 여기 옮겨 적지 않는다. 미검증 행은 전부 **서보 ON 창(검C2)** 대기 |
| `최종결과물.md` | 결론·진입점 | 마무리 세션 1개 |

> 종전에 여기 든 수치("15행 중 3 통과 / 12 미검증")가 `검증.md` 집계와 어긋나 있었다.
> **같은 사실을 두 곳에 두면 조용히 어긋난다** — `hcr5_bridge/README.md` §5 의 체크박스를
> 걷어낸 것과 같은 이유다(260815-피오렌티노).

### 실기 없이 닫히는 것은 다 닫혔다 (2026-08-11 실측, 04pc)

| 확인 | 결과 |
|---|---|
| 빌드 (V-1) | ✅ 경고 **0건**. ⚠️ `hcr5_bridge` 만 지으면 기동이 안 된다 — `hcr_robot_description`·`hcr_moveit_config` 도 같이 짓는다 |
| mock 회귀 (V-10) | ✅ `100 Hz`·`is_async False`·`/joint_states` **99.99~100.01Hz**·`hcr_home` 편차 **0**. **`<param name="allow_motion">` 을 `mock_components/GenericSystem` 이 거부하지 않는다** |
| 실기 없이 실기 모드 로드 (V-11) | ✅ 3경로 **3003 / 0 / 3022ms** 사유별 ERROR — OS 타임아웃(수십 초)에 안 잡힌다 |
| CM abort 완화책 (V-12) | ✅ **완화 가능.** `hardware_components_initial_state.shutdown_on_initial_state_failure:false` 또는 `.unconfigured:[<컴포넌트>]` — **둘 다 같은 네임스페이스 하위다** |
| 실기 도달 | ✅ §1 절차 재확인 — `192.168.0.100/24` 에서 `ping` **3/3**, RTT **0.252~0.551ms**, TTL **128** |
| **B 실기 읽기 · C 실기 쓰기** | ⬜ **미착수** — `on_activate`·`read`·`write` 는 실기에서 **한 번도 안 돌았다** |

**지금 구조적으로 막는 것은 없다.** 남은 선행은 착수 게이트뿐이다 — 축온 **≤50°C**(§8 함정 3) ·
`controllerStatus` = `FIELD_BUS_SW_STATE_CONNECTED` · `move_group` 중복 **0**. 게이트 계측은
`tools/mqtt_baseline.py`(§9, 순수 구독).

> ⚠️ **실기 없이 실기 모드를 띄우면 `controller_manager` 가 프로세스째 abort 한다.** 플러그인은 계약대로
> ERROR 를 반환하고 abort 는 프레임워크 거동이다 — 데모 기동 절차에서는 위 완화책 파라미터를 쓴다.
>
> ⚠️ **`host`·`port` 는 xacro 로 관통돼 있지 않다.** URDF `<hardware>` 에 노출된 `<param>` 은
> `allow_motion` 하나뿐이고 나머지 6종은 코드 기본값을 쓴다(`192.168.0.20:1883` 등). 다른 주소로 시험하려면
> 전개된 URDF 에 `<param>` 을 직접 넣는다 — V-11 이 그 방식으로 측정됐다.
