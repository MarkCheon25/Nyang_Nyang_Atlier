# 실측 원자료 — ros2_control 하드웨어 인터페이스 (2026-08-11, 04pc)

T15 의 B(실기 읽기)·C(실기 쓰기) 구간에서 **실기 HCR-5 를 상대로 실제로 뜬 원자료**다.
[`../../guideline/중간결과물.md`](../../guideline/중간결과물.md) §3-B·§3-C·§3-D 의
모든 수치와 [`검증.md`](../../guideline/검증.md) 의 실측 칸이 여기서 나왔다.

> **왜 리포에 넣었나 (Rokey6 지시, 08-12)** — 원본은 세션 스크래치와 컨테이너 `/tmp` 에만 있었다.
> 문서는 *"세션이 끝나면 사라진다"* 고 적었지만 실제로는 남아 있었고, **검증 세션이 그것으로 재산출을 했다.**
> 재부팅 한 번이면 날아갈 자리였다 — **여기 있는 것이 이제 유일본이다.**
> 서보 ON 창은 축온 예산(T20)이 걸려 마음대로 다시 못 뜬다. **데이터가 로봇 시간보다 비싸다.**

- 27MB → **1.7MB** — 500KB 넘는 것만 `gzip -9`. 원본 바이트는 그대로다(`gzip -t` 검증 완료)
- **이 디렉터리는 읽기 전용으로 다룬다.** 새 계측은 새 날짜 디렉터리를 만든다

## 뜬 사람·때 (전부 04pc)

| 세션 | 구간 | 이 디렉터리의 산물 |
|---|---|---|
| 260811-진공펌프 | 실기 불필요 4행 | `logs/v10_mock.log` · `scripts/{mqtt_stub.py,v11_base.urdf}` |
| 260811-버스바 | **B 실기 읽기** (서보 OFF) | `logs/{B_launch.log,cm_stats.yaml}` |
| 260811-오일리스부싱 | **C 착수분** — 결함 ① 수정 · V-8 (서보 ON 17.3s) | `bus/A_bus.jsonl.gz` · `ros/{A_ros.json.gz,A_ros.txt,c_dry.json}` · `logs/{A_bus_sum.txt,c1_bus.log,c_launch.log}` |
| 260811-단자대 | **C 본체** — 실기 쓰기 (서보 ON 3창 ~62s) | `bus/trace_{activate,C,T19}.jsonl.gz` · `ros/ros_trace_{C,T19}.json.gz` · `logs/{launch_C.log,c_run.log}` |
| 260811-유성기어 | **검증 — 재산출** | `verify/*.py` |

## 파일 지도 — 어느 판정이 이 파일에 걸려 있나

| 파일 | 무엇 | 근거가 되는 곳 |
|---|---|---|
| `bus/trace_C.jsonl.gz` | C 본체 창 89.7s · 20456줄 · 86토픽 (서보 ON→OFF 포함) | **V-7** 도달오차 · **V-14** 22건 연쇄 선점 · 재발행 간격·ack 왕복 · stop-and-go 42% |
| `bus/trace_T19.jsonl.gz` | base 역방향(10°→5°) 창 50.0s · 11406줄 | **T19** `motion/joint/speed` 부호 없음(0/488) · **V-13** 버스 차분 |
| `bus/trace_activate.jsonl.gz` | `allow_motion:=true` 활성화 창 75.0s · 서보 OFF | **발견 ⑦** 궤적 없이 나간 2건 · **발견 ⑧** `150026 …SERVO_OFF` |
| `bus/A_bus.jsonl.gz` | V-8 강한 형태(서보 ON) 45.0s · 10237줄·22토픽 | **V-8** `move/joint/here` **0건** · 궤적구간 p-p 0.000163° · 감청 유효성 |
| `ros/ros_trace_C.json.gz` · `_T19` | `/joint_states` + `/hcr_arm_controller/controller_state` 전 표본 | **V-15** 추종오차 0.8846→0.0005° · **V-13** velocity · **V-6** 값 갱신 빈도 |
| `ros/A_ros.json.gz` · `A_ros.txt` | V-8 의 ROS 측 | **V-8** 추종오차가 정확히 **5.0000°** — 게이트가 플러그인 안에 있다는 직접 증거 |
| `ros/c_dry.json` | V-8 약한 형태(서보 OFF) ROS 측 | **V-8** 약한 형태 |
| `logs/cm_stats.yaml` | `/controller_manager/statistics/full` 스냅샷 | **B3** read/write 실효 **33.33Hz** · 컨트롤러 100.01Hz (**결함 ②** 의 근거) |
| `logs/B_launch.log` | B 전 구간 런치 로그 361줄 | **V-2·V-3·V-4** · **V-9** 두절 **1028ms** · **결함 ①③** 의 로그 원문 |
| `logs/v10_mock.log` | mock 회귀 런치 로그 | **V-10** — `GenericSystem` 이 `<param name="allow_motion">` 을 거부하지 않는다 |
| `scripts/` | 구현 세션이 쓴 실행·분석 스크립트 + V-11 픽스처 | 출처 보존용. `tools/{mqtt_trace,ros_trace}.py` 의 전신이다 |
| `verify/` | **검증 세션의 재산출 스크립트** | 아래 |

## 읽는 법

```bash
zcat bus/trace_C.jsonl.gz | head -1          # 한 줄 = 메시지 1건
# {"t": 시작 후 경과초, "topic": "motion/joint/position", "d": {…파싱된 페이로드…}}
```

- **버스 JSONL** — `d.data.data` 가 알맹이다(봉투 두 겹). 관절 키는 `base shoulder elbow wrist1 wrist2 wrist3`,
  단위 **도**. `move/joint/here` 는 `type:"pubWithAck"` 이고 **ack 는 그 `uuid` 를 토픽명으로** 되돌아온다
  (`d.data.code == 0` 이면 성공) — 왕복시간은 이 두 시각의 차다.
- **ROS JSON** — `{"summary":…, "joint_states":[[t, position[6], velocity[6]],…],
  "controller_state":[[t, reference[6], feedback[6], error[6]],…]}`. 단위 **라디안**, 관절 순서 `joint_1`~`joint_6`.
- 규약 변환은 `q_URDF(rad) = deg2rad(SIGN·q_실기(deg) + DELTA)`, `SIGN=(+,+,−,+,+,+)`,
  `DELTA=(90,90,0,90,0,0)` — 원본은 [`joint_convention.hpp`](../../../hcr5_bridge/include/hcr5_bridge/joint_convention.hpp).

## 재산출 — `verify/`

검증 세션(260811-유성기어)이 **구현 세션의 분석기를 쓰지 않고 새로 짠** 것이다. 경로만 고치면 그대로 돈다
(현재 스크립트 안의 경로는 소멸한 세션 스크래치를 가리킨다 — **이 디렉터리로 고쳐서 쓴다**).

| 스크립트 | 재산출하는 것 |
|---|---|
| `verify_bus.py` | 발행 수·ack 왕복·재발행 간격·버스 발행률·창 전체 p-p·축온·부하 |
| `verify_seg.py` | 목표 연쇄 분해(V-14)·도달오차(V-7)·stop-and-go·구간 절단 p-p(V-8) |
| `verify_t19.py` | speed 부호(T19)·\|speed\|↔\|차분\| 상관 4가지 방법·C 도달오차 |
| `verify_ros.py` | 추종오차(V-15)·velocity 품질(V-13)·값 갱신 빈도(V-6) |
| `recheck.py` | 규약 변환 정답지 12칸 재계산 + 문서 산술 정합 |

**재산출 결과 — 문서 수치가 자릿수까지 재현됐다**(ack 왕복 35.8/61.9/63.6/168.4ms · 도달값 5.002967/10.002617° ·
추종오차 0.8846→0.0005° · velocity −3.1565~+0.0150 °/s · speed 음수 0/488 · read 33.3312Hz(SD 0.1252) ·
정답지 홈 12칸 **비트 단위 일치**). 상세와 **어긋난 6건**(데드밴드 임계 유도 오류 등)은
루트 `실기PC_작업기록.md` 260811-유성기어 절 — 그쪽이 원본이다.

⚠️ **창 정의에 민감한 양이 있다** — stop-and-go 비율(41.5~45.9%)·버스 값변화(14.7~16.4Hz)·ROS 값갱신
(10.8~15.5Hz)·상관 r(−0.05~0.23)은 구간을 어떻게 자르느냐로 움직인다. 재산출값이 문서와 소수점에서
다르면 **먼저 창 정의를 맞춰 보고**, 그래도 다르면 그때 어긋난 것이다.
