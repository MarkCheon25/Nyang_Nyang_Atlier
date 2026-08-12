# P2 — pose goal 경로 원자료 (260812-플랜지베어링)

> ⚠️ **이 폴더만 T15 검증이 아니다.** 형제 폴더(`c1/`·`c2/`·루트 `logs/`)는 `검증.md` 15행 재현이고,
> 여기는 **T21 좌표계 제어**의 P2 다. 자리를 여기로 잡은 이유는 **원본 스크립트 `verify/c2_moveaction.sh` 가
> 여기 있고 그것을 복사해 만들었기 때문**이다(지시서 §5 화이트리스트가 지정).

- 지시서 = 루트 `briefs/P2_pose_goal_경로.md` (git 밖) · 설계 근거 = `briefs/좌표계_제어_작업계획.md` §5-P2
- 실측·발견의 **원본 서술** = 루트 `실기PC_작업기록.md` **260812-플랜지베어링** 절 (git 밖)
- **실기 무접촉 · 서보 무관 · `hcr5` 링크 미기동.** 전 구간 **mock** 이다 — 드라이브 단절(T-DRIVE)과 무관하게 돌았다
- 코드 = `markch/hcr5_ros2` **`f079978`** (`src/` 무변경 — 이 커밋은 산출물 추가뿐)

## 한 장 요약

| 무엇 | 값 |
|---|---|
| **IK 실패율** (구면격자 20점) | **자세구속 O 20.0% · X 15.0%** |
| **실패의 정체** | **전부 도달 경계.** IK 품질 실패 **0건** |
| **도달 경계** | IK 성공 최대 어깨거리 **0.8933 m** / 실패 최소 **0.8981 m** (**4.8mm 폭**) |
| **`kinematics_solver_timeout`** | **병목 아님** — 5ms→500ms(100배)에도 실패 4건·같은 점 |
| **도달 오차** | **0.0586 / 0.0765 mm** (허용반경 0.1mm) |
| **소묘 평면 도달지도** | 1344점 중 **81.0%** · **z=0.20 평면 550×1000mm 100% 도달** |
| B4 회귀 | 원본 joint goal 통과 (최대 관절오차 **9.02e-05 rad**) |
| B2 | ⬜ **미결** — 마커 드래그는 GUI 조작이라 에이전트가 못 한다 |

⚠️ **어깨 원점 `(0, 0, 0.148778)` 에서 재야 경계가 한 값으로 모인다.** base 원점 기준으로는 안 모인다.
⚠️ **도달지도는 `link6_1`(플랜지) 기준이다.** `pen_tip` 은 −X 방향 200mm 아래 → **종이 높이 +200mm 가 flange 목표**.

## 파일 지도

| 파일 | 무엇 | 근거가 되는 곳 |
|---|---|---|
| `logs/p2_b3_sweep.log` | **B3 스윕 20점 × 2모드** stdout | **IK 실패율 20.0% / 15.0%** · 실패 4좌표 |
| `logs/p2_b3_{ori,noori}.json` | 같은 스윕의 점별 원자료(error_code·계획시간) | 위 표의 재산출 근거 |
| `logs/p2_ik_timeout.log` | **`/compute_ik` timeout 3종**(5ms·50ms·500ms) | **C 판정 — timeout 은 병목이 아니다** |
| `logs/p2_ws_map.log` | **소묘 평면 도달지도 1344점** (ASCII 4장 전문) | z별 도달률 · 100% 도달 구간 |
| `logs/p2_b1_exec.log` | B1 실행 결과(**계획 궤적 15점 전체 포함**) | 도달오차 **0.0586mm** · **계획끝점 = 실행값** |
| `logs/p2_b1b_exec.log` | B1b(기본 DOMAIN=0) 실행 결과 | 도달오차 **0.0765mm** |
| `logs/p2_b4_joint.log` | B4 회귀 — 원본 `c2_moveaction.sh` 그대로 | 최대 관절오차 **9.02e-05 rad** |
| `logs/p2_movegroup_d2.log.gz` | DOMAIN=2 스택 런치 로그(B1·B3 구간) | **실패 사유** — OMPL 5초 타임아웃 · `base_link↔link3_1` 자기충돌 |
| `logs/p2_movegroup_d0.log` | DOMAIN=0 스택 런치 로그(B1b·B4·C 재측정 구간) | 〃 |

> `p2_movegroup_d2.log.gz` 는 309,487 B → **25,067 B**(12.3배). c1 선례대로 **압축 전후 산출 일치**를 확인했다
> (`FAILURE` 행수 13 / 13).

## 분석기 — `../verify/`

| 파일 | 몫 |
|---|---|
| **`pose_goal.sh`** | ★ **이 세션의 산출물.** 좌표로 목표를 준다. `c2_moveaction.sh` 복사본이고 `goal_constraints` 안만 다르다 |
| `p2_ik_probe.py` | `/compute_ik` 직접 호출 — **IK 실패와 경로계획 실패를 가른다.** timeout 인자 |
| `p2_ws_map.py` | 소묘 평면 도달지도(수평면 격자 IK 스윕) |
| `p2_sweep.py` | B3 스윕 — **`pose_goal.sh` 를 그대로 호출한다**(검사 대상을 재구현하지 않는다) |
| `p2_tf_read.py` | base→링크 변환 **전정밀도** 판독. `tf2_echo` 는 소수 3자리라 mm 이하 오차를 못 잰다 |

⚠️ `p2_*.py` 3종은 컨테이너 **안**에서 돈다(`src/drivers/` 가 마운트 밖이라 **`docker cp` 로 넣어야 한다**).
`pose_goal.sh`·`p2_sweep.py` 는 **호스트**에서 돈다(`docker exec` 로 들어간다).

## 재현

```bash
docker start markch_moveit2_dev
docker exec markch_moveit2_dev bash -lc \
  "cd ~/ws_moveit2 && source install/setup.bash && export ROS_DOMAIN_ID=0 && \
   ros2 launch hcr_moveit_config demo.launch.py"          # 인자 없이 = mock

# 단발 pose goal (플랜지 기준). 목표 좌표는 TF 에서 읽어 근처를 준다 — 문서에서 베끼지 않는다
docker cp ../verify/p2_tf_read.py markch_moveit2_dev:/tmp/ && \
docker exec markch_moveit2_dev bash -lc \
  "cd ~/ws_moveit2 && source install/setup.bash && python3 /tmp/tf_read.py link6_1 base_link"
../verify/pose_goal.sh <X> <Y> <Z> <QX> <QY> <QZ> <QW> link6_1 0.1 false

# IK 실패율 · timeout 비교
docker cp ../verify/p2_ik_probe.py markch_moveit2_dev:/tmp/ik_probe.py
docker exec markch_moveit2_dev bash -lc \
  "cd ~/ws_moveit2 && source install/setup.bash && python3 /tmp/ik_probe.py 0.005"
```

⚠️ **`demo.launch.py` 는 동시에 하나만.** 띄우기 전 `pgrep -x move_group | wc -l` 이 0 인지 본다.
⚠️ **`hcr_robot_description` 재빌드 주의** — P1 이 `hcr_robot.xacro` 에 `nyang_pen` include 를 넣는 중이면
마운트가 붙기 전까지 재빌드+relaunch 조합이 깨진다(이 세션은 재빌드하지 않았다).

## 남은 것

- ⬜ **B2** — RViz 인터랙티브 마커를 끌어 Plan & Execute. **GUI 조작이라 사람 손이 필요하다.**
  구조 근거는 실측됐다: `/move_action` **액션 서버 1개**(`/move_group`) · **`/rviz2` 가 그 클라이언트**
- ⬜ **P3(실기)** — 드라이브 복구가 선행(T-DRIVE). 착수 게이트는 기존과 같다
- P1 이 `pen_tip` 을 세우면 `pose_goal.sh` 의 `link_name` 인자만 바꿔 그대로 겨눈다 — **스크립트 수정 없음**
