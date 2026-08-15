# P2 — pose goal 경로 원자료 (260812-플랜지베어링)

> ⚠️ **이 폴더만 T15 검증이 아니다.** 형제 폴더(`c1/`·`c2/`·루트 `logs/`)는 `검증.md` 15행 재현이고,
> 여기는 **T21 좌표계 제어**의 P2 다. 자리를 여기로 잡은 이유는 **원본 스크립트 `verify/c2_moveaction.sh` 가
> 여기 있고 그것을 복사해 만들었기 때문**이다(지시서 §5 화이트리스트가 지정).

- 지시서 = 루트 `briefs/P2_pose_goal_경로.md` (git 밖) · 설계 근거 = `briefs/좌표계_제어_작업계획.md` §5-P2
- 실측·발견의 **원본 서술** = 루트 `실기PC_작업기록.md` **260812-플랜지베어링** 절 (git 밖)
- **실기 무접촉 · 서보 무관 · `hcr5` 링크 미기동.** 전 구간 **mock** 이다 — 드라이브 단절(T-DRIVE)과 무관하게 돌았다

---

## ★ 먼저 — 이 폴더의 수치를 결론으로 쓰지 마라

**펜홀더는 아직 실측된 적이 없다** (Rokey6 2026-08-12). 펜과 홀더를 물리적으로 붙여만 놓았고,
**로봇 컨트롤러에 TCP 등록도 안 돼 있다.** `nyang_pen.xacro` 의 치수는 **자리표시 값**이다:

| | 지금 URDF 값 | 성격 |
|---|---|---|
| `holder_length` | 0.05 m | **자리표시** |
| `pen_length` | 0.15 m | **자리표시** |
| 질량 합 | 65 g | **자리표시** |

충돌 형상이 도달 범위를 **크게** 바꾼다는 것이 이 세션에 실측됐다(아래 기준선 표).
따라서 여기 수치는 **로봇에 대한 사실이 아니라, 치수를 넣으면 다시 뽑을 대조군**이다.

**이 폴더의 값어치는 숫자가 아니라 아래 절차다** — 치수만 나오면 바로 구현으로 넘어가도록 짜 놓은 틀.

---

## 틀 — 치수를 재고 나면 이대로 돌린다

```bash
# ── 0. 스택 (mock). demo.launch.py 는 동시에 하나만 — 띄우기 전 pgrep -x move_group | wc -l 이 0 인지 본다
docker start markch_moveit2_dev
docker exec markch_moveit2_dev bash -lc \
  "cd ~/ws_moveit2 && source install/setup.bash && export ROS_DOMAIN_ID=0 && \
   ros2 launch hcr_moveit_config demo.launch.py"

# ── 1. 잰 값을 넣는다. **여기가 단일 출처다** — 다른 파일은 손대지 않는다
#      src/drivers/nyang_pen/urdf/nyang_pen.xacro  매크로 기본값 (holder_length · pen_length · 반지름 · 질량)
# ── 2. 재빌드
docker exec markch_moveit2_dev bash -lc \
  "cd ~/ws_moveit2 && colcon build --packages-select nyang_pen && source install/setup.bash"
#      ⚠️ 재빌드 후 스택을 다시 띄워야 URDF 가 갱신된다

# ── 3. pen_tip 이 잰 값과 맞는지 확인 (홈에서)
docker cp ../verify/p2_tf_read.py markch_moveit2_dev:/tmp/ && \
docker exec markch_moveit2_dev bash -lc \
  "cd ~/ws_moveit2 && source install/setup.bash && python3 /tmp/p2_tf_read.py pen_tip link6_1"

# ── 4. ★ 시드를 홈으로 맞춘다. 안 맞추면 아래 값이 기준선과 비교가 안 된다 (함정 ① 참조)
../verify/c2_moveaction.sh 1.570796 0 1.570796 0 1.570796 0 0.1 false

# ── 5. 도달 범위·IK 실패율 재측정 — 기준선과 **같은 명령**이라 그대로 대조된다
docker cp ../verify/p2_ik_probe.py ../verify/p2_ws_map.py markch_moveit2_dev:/tmp/
docker exec markch_moveit2_dev bash -lc \
  "cd ~/ws_moveit2 && source install/setup.bash && export ROS_DOMAIN_ID=0 && \
   python3 /tmp/p2_ik_probe.py pen_tip 0.005 && python3 /tmp/p2_ws_map.py pen_tip 0.005"

# ── 6. 좌표로 겨눈다. **스크립트 수정 없음** — link_name 이 인자다
../verify/pose_goal.sh <X> <Y> <Z> <QX> <QY> <QZ> <QW> pen_tip 0.1 false
#      펜이 바닥을 향하는 자세 = quat(xyzw) (1, 0, 0, 0)
#      ⚠️ 실기로 갈 때 스케일은 0.014 (원본 c2_moveaction.sh 기본값). 0.1 은 mock 용이다
```

**7. 펜던트(컨트롤러 `tool1`) TCP 등록** — 같은 값을 복사해 넣는다. URDF 가 단일 출처이고 펜던트 값은 파생이다.
실기 접근이 필요해 **P3 이후**다.

> 위 3·5 는 `453b6c7` 이전까지 **스크립트를 고쳐야** 돌았다(`ik_link_name` 하드코딩). 인자로 뺐다 — 틀이 여기서 끊겼었다.

---

## 기준선 — `453b6c7` · 시드 **홈** · 공구치수 **자리표시 값**

### 무엇이 펜 때문에 바뀌었나

같은 링크(`link6_1`)·같은 평면·같은 스크립트에서 **펜 유무만** 다르다:

| | 전체 도달률 | z=0.20 평면에서 **y 전폭이 뚫린 x 구간** |
|---|---|---|
| `10d8615` (펜 없음) | 81.0% | 0.20~0.75 → **550 × 1000 mm** |
| **`453b6c7` (펜 있음)** | **80.0%** | 0.45~0.75 → **300 × 1000 mm** |

**전체 도달률은 1%p 밖에 안 움직였는데 쓸 수 있는 직사각형은 절반이 됐다.**
펜이 베이스 **정면 중심선(y=0)** 에 구멍을 뚫어 그 x 행들이 통째로 "전폭 아님" 이 되기 때문이다:

```
  x=0.20 |##########.##########|   ← 가운데 '.' = y 정확히 0.00
  x=0.25 |##########.##########|      x=0.40 까지 이어진다
  …
  x=0.45 |#####################|   ← 여기부터 전폭
```

원인은 펜·홀더가 충돌 검사에 들어온 것이다 — P1 이 `nyang_pen_* ↔ link2_1` 충돌을 15개 자세에서 실측하고
**일부러 살려 뒀다**(플랜지↔홀더만 껐다). **덩치가 큰 공구일수록 이 구멍이 커진다 — 치수 실측이 중요한 이유다.**

### 20점 프로브 (`p2_ik_probe.py`)

| 링크 | IK 실패율 | 성공 최대 어깨거리 | 실패 최소 |
|---|---|---|---|
| `link6_1` | **20.0%** (`10d8615` 과 동일) | 0.8933 m | 0.8981 m |
| `pen_tip` | **55.0%** | 0.7520 m | 0.4934 m |

⚠️ **20점 표본은 플랜지용으로 고른 공중의 점**이라(z≈0.5) `pen_tip` 55.0% 는 소묘의 질문이 아니다.
펜 끝의 질문은 아래 평면 지도가 답한다.

### 평면 도달지도 (`p2_ws_map.py`, 1344점)

`pen_tip` 기준에서는 **z 가 곧 종이 높이**다 — 환산이 없다.

| z | 도달률 | y 전폭 뚫린 x |
|---|---|---|
| 0.00 | 83.6% | 0.45~0.75 |
| 0.05 | 82.4% | 0.25~0.70 |
| 0.10 | 81.0% | 0.25~0.70 |
| 0.20 | 74.7% | 0.20~0.65 |
| **총계** | **80.4%** | |

**`pen_tip` z ↔ `link6_1` z+0.20 은 같은 평면이다** — 격자째 대조해 336칸 중 불일치 **0 / 2 / 2 칸**.
즉 공구 길이만큼 더한 환산은 정확하다. (잔여 2칸은 아래 시드 흔들림 폭 안이다)

---

## ✅ B2 — RViz 마커 드래그 (Rokey6 손, 2회 실행 모두 통과)

에이전트가 못 하는 GUI 조작이라 Rokey6 가 직접 끌었다. **실물은 안 움직였고 그것이 정상이다** —
mock(`GenericSystem`)이고 `hcr_bridge` 를 띄우지 않았다. B2 가 증명할 것은 실기 동작이 아니라 **경로**다.

```
MoveGroupMoveAction: Received request          ← 마커가 /move_action 을 두드렸다
Combined planning and execution request
hcr_arm_controller: Received new action goal   ← ★ ros2_control JTC 까지 갔다
Completed trajectory execution with status SUCCEEDED
```

- 관절 6축 전부 변함 (최대 `joint_3` **−47.21°**) · `pen_tip` **250.9 mm** 이동 · 실행 2.202 s / 3.446 s
- **`/move_action` 서버는 `/move_group` 하나뿐이고 `/rviz2` 가 그 클라이언트다** —
  **마커와 `pose_goal.sh` 가 같은 문으로 들어간다.** 한쪽이 되면 다른 쪽도 된다

---

## 함정 — 다음 세션이 밟기 쉬운 것

**① `pen_tip` 실패율은 로봇 현재 자세(IK 시드)에 따라 달라진다.** 같은 표본·같은 timeout 인데
홈에서 **55.0%**, 다른 자세에서 **50.0%** 였다. **`link6_1` 은 같은 조건에서 20.0% 로 불변**이다 —
`10d8615` 에 적은 *"시드 무관"* 은 **플랜지에서만 성립하고 공구에는 일반화되지 않는다.**
KDL 이 현재 자세에서 출발하는 국소 해법이라 공구가 붙어 목표가 경계로 몰리면 출발점이 결과를 뒤집는다(가설).
→ **비교하려면 시드를 홈으로 맞춘다.** 위 절차 4단계.

**② `/compute_ik` 에는 "위치만" 모드가 없다.** `pose_stamped` 가 위치·자세 한 덩어리고 자세를 빼는 플래그가 없다.
단위 쿼터니언을 넣는 것은 자세 무시가 아니라 **"엉뚱한 자세로 풀어라"** 라서 실패율이 오히려 오른다.
자세 구속 없는 실패율은 **`pose_goal.sh`(계획 단계)** 로 재야 한다 — 거기서는 정말로 뺀다.
(`10d8615` 의 15.0% 가 그 경로에서 나온 값이라 유효하다)

**③ `absolute_*_axis_tolerance` 는 회전각 허용치가 아니다** — 축별 `t` 는 총 최대 **√3·t** 를 허용한다.
각도로 묶으려면 `parameterization=1`(ROTATION_VECTOR).

**④ 목표=현재인 시험은 회귀 시험이 아니다** — MoveIt 이 계획에 들어가지도 않고 `val:1` 을 준다(P1 E3 위양성).

---

## 파일 지도

| 파일 | 무엇 | 근거가 되는 곳 |
|---|---|---|
| **`logs/p2_ik_probe_home.log`** | ★ **재측정** 20점 프로브 2링크 (시드 홈) | `pen_tip` 55.0% · `link6_1` 20.0% |
| **`logs/p2_wsmap_pentip_home.log`** | ★ **펜 끝 기준 평면 지도** 1344점 (ASCII 전문) | z별 도달률 · 전폭 구간 |
| **`logs/p2_wsmap_flange_home.log`** | ★ 플랜지 **대조군** (펜 붙은 상태) | 550mm → 300mm · 등가 검사 |
| **`logs/p2_seed_dependence.log`** | ★ **시드 의존성** 실측 (함정 ①) | 홈 55.0% vs 다른 자세 50.0% |
| **`logs/p2_b2_marker.log`** | ★ **B2** 전후 관절·TF + move_group 로그 발췌 | B2 통과 근거 전부 |
| `logs/p2_b3_sweep.log` · `p2_b3_{ori,noori}.json` | B3 스윕 20점 × 2모드 (`10d8615`, **펜 없음**) | 옛 20.0% / 15.0% |
| `logs/p2_ik_timeout.log` | `/compute_ik` timeout 3종(5·50·500ms) | **timeout 은 병목이 아니다** |
| `logs/p2_ws_map.log` | 옛 플랜지 지도 (`10d8615`, **펜 없음**) | 81.0% · 550×1000mm |
| `logs/p2_b1_exec.log` · `p2_b1b_exec.log` | B1 실행 (계획 궤적 15점 포함) | 도달오차 0.0586 / 0.0765 mm |
| `logs/p2_b4_joint.log` | B4 회귀 — 원본 `c2_moveaction.sh` 그대로 | 최대 관절오차 9.02e-05 rad |
| `logs/p2_movegroup_d0.log` · `d2.log.gz` | 스택 런치 로그 | 실패 사유(OMPL 타임아웃·자기충돌) |

## 분석기 — `../verify/`

| 파일 | 몫 |
|---|---|
| **`pose_goal.sh`** | ★ **산출물.** 좌표로 목표를 준다. `c2_moveaction.sh` 복사본이고 `goal_constraints` 안만 다르다 |
| `p2_ik_probe.py` | `/compute_ik` 직접 호출 — IK 실패와 경로계획 실패를 가른다. **링크·timeout 인자** |
| `p2_ws_map.py` | 평면 도달지도. **링크 인자** — `pen_tip` 이면 z 가 종이 높이 그 자체 |
| `p2_sweep.py` | B3 스윕 — **`pose_goal.sh` 를 그대로 호출한다**(검사 대상을 재구현하지 않는다) |
| `p2_tf_read.py` | base→링크 변환 **전정밀도** 판독. `tf2_echo` 는 소수 3자리라 mm 이하를 못 잰다 |

⚠️ `p2_*.py` 는 컨테이너 **안**에서 돈다(`docker cp` 로 넣는다). `pose_goal.sh`·`p2_sweep.py` 는 **호스트**에서 돈다.

## 남은 것

- ⬜ **펜홀더 치수 실측** + **강성 판정**(rigid/spring) — **Rokey6.** 이게 들어와야 위 기준선이 의미를 갖는다
- ⬜ **펜던트 TCP 등록** — URDF 값을 복사한다. 실기 접근 필요 → P3 이후
- ⬜ **P3(실기)** — 드라이브 복구가 선행(T-DRIVE)
- ⬜ `link6_1` mesh 42.9mm 정합 — T16 이동이 `link6_1.stl` 에 반영 안 됐다(P1 발견, 별도 과제)
