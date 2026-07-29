# HCR-5 호스트 통신·제어 (`src/hcr_comm`)

> **한 줄 요약** — PC를 컨트롤러에 **랜선 직결**하면, 한화 HCR-5(2018 1세대, **Rodi 1.003.005**)를
> **벤더 업그레이드 없이** 제어할 수 있다. 컨트롤러가 내부적으로 쓰는 **MQTT 버스가 그대로 외부에 열려 있고**,
> 그 위의 명령 프로토콜을 역설계해 **PC에서 로봇을 실제로 움직이는 데 성공했다** (2026-07-29).
>
> 이전 결론([`hanwha_robot_arm/docs/실기_연결_현황.md`](../../hanwha_robot_arm/docs/실기_연결_현황.md), 07-28)의
> "Rodi 업그레이드가 유일한 관문"은 **RoboDK/커뮤니티 ROS2 플러그인 경로에만** 해당한다. 이 문서가 그 경로 밖의
> **네이티브 통로**를 다룬다.

*조사·실증: 2026-07-29 (세션 260729-꽃무릇). 전 과정 랜선 직결 실측.*

---

## 0. 결론

| 질문 | 답 |
|---|---|
| 회사 제공 앱/통신방식이 있나? | 컨트롤러 = **Windows PC**. 내부가 **MQTT(Mosquitto) + Express REST + MongoDB** 마이크로서비스로 돌고, 티치펜던트 UI가 이 버스로 로봇을 몬다. **이 버스가 외부에 그대로 열려 있다.** |
| CAN으로 되나? | ❌ 외부 CAN 통로 없음. CAN/EtherCAT은 컨트롤러↔관절 **내부** 버스. 외부 제어는 **이더넷(MQTT)** 이 답. |
| Rodi 1.x에서 PC 제어 가능한가? | ✅ **실증됨.** 서보 ON, 관절 조그를 PC 명령으로 성공. |

---

## 1. 물리 연결 & 네트워크

- PC 유선 NIC(`enp3s0`)를 컨트롤러 이더넷 포트에 직결.
- **컨트롤러 IP: `192.168.0.20/24`** (MAC `00:90:fb:5e:5a:01`). 공장 서브넷 192.168.0.x.
- PC를 같은 서브넷 고정 IP로: 예 `192.168.0.100/24`.
  ```bash
  nmcli connection add type ethernet ifname enp3s0 con-name hcr5 \
    ipv4.method manual ipv4.addresses 192.168.0.100/24 ipv6.method disabled autoconnect no
  nmcli connection up hcr5
  ```
- 발견 방법(로봇 IP를 모를 때): `tools/discover/arpsweep.sh` 로 서브넷 ARP 스윕 → 살아있는 호스트 확인.
- ping RTT ~0.3ms, **TTL 128 → Windows 스택**.

## 2. 컨트롤러 아키텍처 (열린 포트, 전수 스캔)

| 포트 | 서비스 | 역할 |
|---|---|---|
| **1883** | **MQTT broker (Mosquitto 1.4.7, 무인증)** | ★ 로봇 실시간 메시지 버스 (상태 발행 + 명령 구독) |
| **9001** | **MQTT over WebSocket** | 펜던트 UI(브라우저/Electron)가 붙는 통로 |
| **4000 · 8000** | Node.js **Express REST API** (CORS `*`) | 라우트는 비공개(POST RPC). 미규명 |
| 27017 | MongoDB | 프로그램·설정·로그 저장소 (인증 걸림) |
| 502 | Modbus TCP | I/O 통합용 (모션 지령 채널 아님) |
| 80 | Microsoft IIS/7.5 | 기본 페이지(내용 없음) |
| 135/139/445/49152~ | Windows SMB/RPC | OS 내부 |

발행 서비스명(`heartbeat` 토픽): **`hcr-control-svc`**. 브로커에 클라이언트 ~10개, 구독 ~211개.

## 3. MQTT 상태 버스 (읽기 — 완전 수동, 안전)

`#` 구독 시 로봇이 자기 상태 전체를 **~30Hz** 로 브로드캐스트한다. 봉투:
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
| `monitor/robot` | `power`, `voltage`(48V), `current`, 6축 `temp` |
| `monitor/io/{configurable,digital,analog,tool}` | I/O 8ch씩 |
| `motion/joint/{range,speed}` · `heartbeat` · `modbus/device/error` | — |

관찰: `python3 tools/mqtt_sub.py 192.168.0.20 1883 12 '#'`

## 4. 명령 프로토콜 (쓰기) ★

**펜던트 조작을 관찰해 역설계**했다(56,723건, joint 1·6 교차검증).

**봉투:** `{"type":<type>, "uuid":<uuidv1>, "data":{"thng_id":1, ...인자}}`
**RPC 규칙:** `type:"pubWithAck"` 로 명령 토픽에 publish → **`uuid`와 같은 이름의 토픽**으로
`{"code":0,"data":{},"msg":"success"}` 응답이 온다. `type:"pub"` 는 단방향.

| 동작 | 토픽 | type | data 인자 |
|---|---|---|---|
| 서보 ON/OFF | `set/operation` | pubWithAck | `{operationStatus:"SERVO_ON"｜"SERVO_OFF"}` |
| 매뉴얼/자동 모드 | `robot/mode` | pub | `{isManual:true｜false}` |
| 소프트 관절제한 | `set/limitCheck` | pubWithAck | `{mode:true｜false}` |
| **관절 조그 시작** | `jogJoint/start` | pubWithAck | `{joint:1~6, direction:"positive"｜"negative", speed:"45.00"}` |
| **관절 조그 정지** | `jogJoint/stop` | pub | `{}` |
| 핸드가이드 on/off | `directTeaching/start`｜`/stop` | pub | `{}` |
| **충돌 PAUSED 해제** | `event/collision/clear` | pubWithAck | `{}` |

읽기 전용 이벤트(로봇→): `event/collision`(`EVENT_COLLISION_DETECTED`, code 204000), `event/collision/mitigation/complete`(params=충돌 시점 관절 float).

- `joint` 인덱스 1=base … 6=wrist3. 각도·속도 단위 = 도. 상태머신: `IDLE →(start)→ MOVING →(stop)→ STOPPING → STOPPED`.
- **`mongoLog` 토픽**이 내부 함수 호출을 중계한다(`operation.js`의 `procedure`, `operationName`). 미포착 명령(절대이동 등)을 알아내는 **지도**.

## 5. 제어 실증 (2026-07-29)

1. **무동작 쓰기 검증** — PC에서 `set/limitCheck {mode:true}` 발행 → 응답 `{code:0,"success"}`,
   `status/operation.limitCheck` **off→on** 확인.
2. **실제 이동** — PC에서 `jogJoint/start`(base, 저속) → `jogJoint/stop` 시퀀스로 **base 축이 물리적으로 회전**.
   이동각 ≈ `speed(°/s) × 시간(s)`.

→ **Rodi 1.x HCR-5의 PC 제어 성립. 벤더 업그레이드 불필요.**

## 6. 안전 ⚠️

- **PC MQTT 명령은 펜던트 인에이블 스위치(데드맨)를 거치지 않는다.** 첫 이동·실험 시 **e-stop에 손, 로봇 반경 정리** 필수.
- `jogJoint/start` 는 **개루프 속도 조그**(정지 명령까지 계속 이동). `tools/mqtt_cmd.py` 의 `jog` 는
  `start → sleep → stop` 을 `finally` 로 감싸 **정지를 항상 보장**하고, 시작 응답 대기를 2초로 제한해 과주행을 막는다.
- `set/limitCheck {mode:true}` 로 **소프트 관절제한을 켜 두는 것**을 권장(공식 가동범위 밖 자기정지).
- `status/safety.reducedVelocity` = 감속 모드 속도 상한. 실험은 감속·저속으로.

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

## 7. 도구 (`tools/`)

| 파일 | 용도 | 쓰기? |
|---|---|---|
| `mqtt_sub.py` | 임의 토픽 구독 관찰 (`host port sec topic`) | 읽기 |
| `mqtt_full.py` | 핵심 토픽 전체 페이로드/스키마 덤프 | 읽기 |
| `capture.py` | 명령 캡처 — baseline(상태) 대비 **새 명령 토픽만** 분리 기록 (`host port outdir`) | 읽기 |
| `mqtt_cmd.py` | **명령 송신** (`servo｜mode｜limitcheck｜jog｜stopjog｜send`) | **쓰기** |
| `discover/arpsweep.sh` | 서브넷 ARP 스윕(호스트 발견) | 읽기 |
| `discover/portscan.sh` | TCP 포트 스캔 | 읽기 |

순수 Python 표준 라이브러리만 사용(브로커 라이브러리 불필요). 대상 IP는 각 스크립트 상단/인자로 조정.

**명령 예시**
```bash
python3 tools/mqtt_cmd.py servo on                 # 서보 ON
python3 tools/mqtt_cmd.py limitcheck on            # 소프트 제한 ON
python3 tools/mqtt_cmd.py jog 1 positive 10 300    # base 축 저속 0.3초(≈3°)
python3 tools/mqtt_cmd.py stopjog                  # 즉시 정지
```

## 8. 웹 조사 교차검증 (2026-07-29)

- **RoboDK 경로** = 컨트롤러가 여는 **TCP 7000** 평문 라인 프로토콜(`VERSION`/`CJNT`/`MOVJ j1..j6`/`RSTATE`).
  Rodi-X 플러그인(asar)이 `net.createServer`로 연다. **Rodi ≥ 2.001.003.012 요구 → 우리 1.003.005엔 차단.**
  ("6667" 설은 오류, 실제 7000.)
- 두 경로(RoboDK / 우리 MQTT)는 **같은 하부 제어 레이어**(`rodix_api`의 `commandModel.move_joint([j1..j6])`)를 구동.
  모션 원자 = **6관절 도단위 배열** → 우리 `motion/joint/position` 과 대칭이라, **절대이동 명령도 6관절 target을 실을 가능성 높음.**
- 한화 공개 매뉴얼엔 MQTT/REST 스펙 **전무**(내부 미공개). **경험적 캡처가 유일한 규명 경로였다.**

## 9. 다음 단계

- [ ] **절대 관절이동(movej)·직선이동(movel)·프로그램 실행·웨이포인트 티치** 명령 미포착 — 펜던트에서 해당 동작을
      실행하며 `capture.py` 로 관찰해 동일하게 역설계 (소묘 파이프라인에 필수).
- [ ] `:4000/:8000` Express REST 라우트 규명(펜던트 UI가 치는 엔드포인트).
- [ ] 명령 분해능·지연 측정(선 품질 병목 여부, 0.01°/왕복 지연).
- [ ] 소묘 파이프라인(`src/moveit2` 계획 → 이 명령 채널로 실행) 연결 설계.
