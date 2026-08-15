# 최종 산출물로 갈 때 빼야 할 것

**2026-08-14 작성 · 이 문서 자체도 정리가 끝나면 지운다**

개발·검증 과정에서 들어온 것들의 목록이다. 제품에 남으면 안 되는 것, 남길지 정해야
하는 것, 그냥 둬도 되는 것을 나눠 둔다.

> ⚠️ **지금 무엇을 빼는 문서가 아니다.** 검증이 아직 진행 중이라 아래 것들은 전부
> **그대로 쓰고 있다.** 최종 산출물을 낼 때 이 목록을 열어 하나씩 처리하면 된다.
> 2 절의 권고도 지금 실행하라는 뜻이 아니라, **그때 판단할 재료**를 미리 적어 둔 것이다.

> 이런 것을 왜 미리 적어두나 — **나중에는 무엇이 검증용이었는지 구분이 안 된다.**
> `test_tools/` 는 처음부터 CMakeLists 에 지우는 방법까지 같이 적어 뒀는데, 그 방식을
> 나머지에도 적용한 것이다.

---

## 1. 반드시 빼는 것 — 제품 코드가 아니다

### `test_tools/` — 가짜 비전 발행자

비전 컨테이너 없이 `/vision/strokes` 연결을 확인하려고 만든 것. **비전이 실제로 붙어
검증이 끝나면 지운다.**

지우는 곳 세 군데 (전부 표시해 뒀다):

| 파일 | 무엇 |
|---|---|
| `test_tools/` | 폴더째 |
| `CMakeLists.txt` | `# ── test_tools ──` 부터 `# ── test_tools 끝 ──` 까지 |
| `package.xml` | `<exec_depend>rclpy</exec_depend>` (그 위 주석과 함께) |

자세한 것은 `test_tools/README.md`.

### `data/` — 시험용 좌표 데이터

실제로는 비전의 `stroke_map_cli` 가 만든 `map.csv` 를 쓴다. 아래 넷은 비전 없이
돌려보려고 만든 **가짜 고양이**다.

| 파일 | 무엇 | 쓰인 곳 |
|---|---|---|
| `data/sample_cat_map.csv` | 윤곽선 1 개. 수식으로 그린 것 | `config/csv_example.yaml` 의 기본 `csv_path` |
| `data/gen_sample_map.py` | 위 파일 생성기 | — |
| `data/experiment_map.csv` | **부위 5 개, 전부 폐곡선.** A/B 실험 입력 | `docs/Trajectory Optimization.md` §5 |
| `data/gen_experiment_map.py` | 위 파일 생성기 | — |

지우는 곳: 폴더째 + `CMakeLists.txt` 의 `install(DIRECTORY ...)` 목록에서 `data` 한 줄.

> ⚠️ **`config/csv_example.yaml` 의 `csv_path` 가 `sample_cat_map.csv` 를 가리킨다.**
> data/ 를 지우면 이 기본값도 같이 고쳐야 한다 (비전이 만든 파일 경로로).

### `test/` + gtest 블록 — **지우지 않는 편이 낫다**

F3.1 단위시험(`test/test_optimizer.cpp`). `colcon test` 로만 돌고 **제품 바이너리에는
들어가지 않는다.** 유지 비용이 0 이라 남기는 쪽을 권한다.

그래도 지운다면 세 곳:

| 파일 | 무엇 |
|---|---|
| `test/` | 폴더째 |
| `CMakeLists.txt` | `# ── F3.1 단위시험 ──` 부터 `# ── F3.1 단위시험 끝 ──` 까지 |
| `package.xml` | `<test_depend>ament_cmake_gtest</test_depend>` (그 위 주석과 함께) |

> `include/drawing_cat/optimizer.hpp` 는 **제품 코드다.** 지우면 안 된다.

---

## 2. 나중에 결정할 것 — **지금은 전부 그대로 둔다**

측정 과정에서 들어왔는데 제품에 남길 가치가 애매한 것들. 아래 🔴🟡 는 **그때 참고할
의견**이고, 지금 실행하는 항목이 아니다.

### 🔴 `travel_mode: "joint"` 분기 — 빼는 쪽을 권한다

`draw_cat.cpp` 의 관절공간 이동 분기. **한 번도 측정하지 않았고 기본값도 아니다**
(`"cartesian"`). 검증 안 된 경로가 코드에 남아 있는 것이라 위험 쪽에 가깝다.

빼면 `travel_mode` 파라미터와 그 검증 블록(`travel_mode != "cartesian" && ...`)까지
같이 사라져 코드가 눈에 띄게 줄어든다.

### 🟡 `travel_velocity_scaling` — 남기되 경고를 유지

**실측으로 효과가 0 이 확인됐다** (0.1 → 0.5 로 올려도 펜업 0.51 → 0.51). 지금 축척에서
이동이 실제 4.5 ~ 24 mm 라 최고 속도에 도달할 일이 없기 때문이다.

지우지 않은 이유: 그림을 훨씬 크게 그리면 순항 구간이 생겨 의미가 살아난다. 다만
**이름만 보면 효과가 있을 것 같은 함정 파라미터**라, 남긴다면 지금 달아 둔 경고 주석을
반드시 같이 남겨야 한다.

### 🟡 `optimize_order` · `rotate_closed_start` — 남기는 쪽을 권한다

원래 A/B 측정용 토글이었다. 제품에서 끌 일은 없지만, **문제가 생겼을 때 최적화를 꺼서
원인을 가르는 수단**이 된다. 유지 비용이 거의 없다.

### 🟡 린터 3 개를 꺼 두었다 — 스타일을 맞출지 결정

`CMakeLists.txt` 에서 `uncrustify` · `flake8` · `pep257` 을 껐다 (2026-08-14).

**이 패키지는 처음부터 이 린터들을 통과한 적이 없다.** 처음으로 `colcon test` 를 돌려
보니 38 개가 실패했고, 대상이 `src/draw_cat.cpp` · `launch/*.py` · `test_tools/*.py` 등
**기존 파일들**이었다. 이 저장소가 ROS 2 기본 스타일을 안 따르기 때문이다 (C++ 4 칸
들여쓰기 + Allman 중괄호, Python 큰따옴표, 한국어 주석). `copyright` · `cpplint` 는
원래부터 같은 이유로 꺼져 있었다.

껐어야 했던 이유는 **끄지 않으면 `colcon test` 가 항상 빨간불이라 단위시험의 실패가
묻히기 때문**이다. 스타일을 ROS 2 기본에 맞추기로 한다면 그 세 줄을 지우고 전체를
정리하면 된다 — 그때는 이 항목도 함께 지운다.

### 🟡 시간 분해 로그의 해설 문구

계측 자체는 남기는 게 좋지만(아래 3 절), 마지막 줄은 실험 해설이지 제품 로그가 아니다:

```
⚠️ 펜업이 전체의 4.8% 다 — **순서 최적화로 줄일 수 있는 시간의 상한**이 이 값이다.
```

숫자만 남기고 이 한 줄은 빼는 편이 맞다.

---

## 3. 그냥 두는 것

| 무엇 | 왜 |
|---|---|
| 시간 계측 (`trajDurationS`·`runTimed`·시간 분해 블록) | 제품에서도 **회귀 감지**에 쓴다. 어느 날 느려지면 바로 보인다. 비용은 로그 몇 줄 |
| `config/topic_example.yaml` | 실전(비전 토픽) 경로 설정이다. 예시가 아니라 제품 |
| `config/csv_example.yaml` | csv 경로도 실제 운용 경로다. 단 `csv_path` 기본값만 위 ⚠️ 대로 고친다 |
| `docs/Trajectory Optimization.md` | 코드가 아니라 **지금 기본값이 왜 그 값인지의 근거**다. 지우면 누군가 가속 1.0 을 이유 없이 되돌린다 |

---

## 4. 저장소 밖 — 저절로 사라지거나, 지워도 되는 것

| 무엇 | 어디 | 조치 |
|---|---|---|
| `test_f31.cpp` · `f31_extracted.inc` | 세션 스크래치패드 | ✅ **`test/test_optimizer.cpp` 로 옮겼다.** 스크래치패드 쪽은 버려도 된다 |
| `predict.cpp` · `hops.cpp` | 세션 스크래치패드 | 실험 중 홉 거리를 뽑아 본 일회용 도구. 사라져도 무방 (수치는 `docs/Trajectory Optimization.md` §4.1 표에 남아 있다) |
| `exp_?.yaml` · `sw_*.yaml` · `def_check.yaml` · `*.log` | moveit2 컨테이너 `/tmp` | 컨테이너 재생성 시 사라진다 |
| `drawing_cat/log/` (88K) · `drawing_cat/src/log/` (44K) | 패키지 안 | **colcon 을 패키지 디렉터리에서 돌려 생긴 잔재.** `.gitignore` 의 `log/` 에 걸려 커밋되지는 않는다. 지워도 무방 |

---

## 5. 지울 때 순서

1. 비전 실물 연결 검증 → `test_tools/` 제거 (3 군데)
2. 비전이 만든 `map.csv` 경로 확정 → `config/csv_example.yaml` 의 `csv_path` 교체
3. 그 다음 `data/` 제거 (폴더 + CMakeLists 한 줄)
4. `travel_mode` 결정 → `"joint"` 분기 제거
5. 마지막에 이 문서 삭제

> ⚠️ **2 번을 3 번보다 먼저 해야 한다.** 순서를 바꾸면 `csv_example.yaml` 이 없는 파일을
> 가리키는 상태로 남는다.
