# vision

**A(이미지 → 지도)의 스켈레톤이 서 있는 상태.** 인터페이스·빌드·확인 경로는 완성됐고, 실제
이미지 처리 알고리즘(F2.1~F2.3)은 아직 비어 있다. B·C·D는 설계만 있고 코드는 없다.

## 다섯 가지 기능

vision 모듈은 서로 독립된 다섯 기능을 담당한다. B·C·D는 모두 같은 eye-in-hand 카메라(엔드이펙터 장착)를 쓰고 목적만 다르다. E는 2026-07-27에 운영 레인에서 넘어온 것으로, **흐름상으로는 A보다 먼저** 온다.

### A. 이미지 → 지도 (F2 이미지처리) — `MVP 코어`

| | 내용 |
|---|---|
| 입력 | 이미지 한 장 — 흰 배경 고양이 라인아트/실루엣 (F1이 그대로 전달) |
| 출력 | **지도** — 정렬되지 않은 스트로크들의 집합 |

지도를 이루는 스트로크 하나:
- 종이 평면 좌표(x, y mm)로 스케일링된 점들의 나열(F2.4) — 스트로크 **내부**는 끝에서 끝까지 그리는 순서가 있음
- 좌표 규약: **종이 좌상단이 (0,0)**, +x 오른쪽, +y 아래쪽 — 이미지 픽셀 좌표계와 같은 방향이라 F2.4가 방향 반전 없이 스케일만 적용하면 됨
- z 없음 — z는 moveit2 쪽(F4.1)이 캘리브레이션으로 붙인다
- 곡선이어도 조밀하게 샘플링된 점으로 통일 — 원본 컨투어를 곡선으로 매끄럽게 피팅한 뒤 균일 간격으로 재추출(내부 처리 과정), 점과 점 사이는 직선으로 잇는다고 간주

**스트로크끼리의 순서·펜업/다운 결정은 vision의 일이 아니다.** 원래 F3(스트로크 계획: 순서 최적화·펜업다운 계획)로 한데 묶여 있었으나, 순서 최적화(F3.1)는 로봇의 실제 이동거리를 알아야 하는 일이라 **moveit2 쪽 책임**으로 확정됐다. 순서를 모르면 펜업 구간(어디서 들고 어디로 이동할지)도 정할 수 없으므로 펜업/다운 결정(F3.2)도 함께 moveit2로 넘어간다. 그래서 지도는 "경로"가 아니라 "정렬 안 된 스트로크 집합"이고, 스트로크 수는 이 집합의 크기 그대로라 별도 필드가 필요 없다.

**calibration과 무관**: 지도는 순수 종이 평면 mm 좌표라 calibration 없이 계산된다. calibration(F5)은 moveit2가 이 지도를 로봇 좌표로 옮기는 데만 쓰인다 — vision과 calibration은 독립적으로 진행되다가 moveit2 내부(F4.1)에서 만난다.

### B. 카메라 이미지 → 마커 포즈 (F5.1·F5.2 캘리브레이션 지원) — `이후`

카메라는 eye-in-hand로 엔드이펙터(6축 암 끝)에 장착됨 — BRD F5·§2.3, SA §4에 반영 완료.

| | 내용 |
|---|---|
| 입력 | 카메라 이미지 한 장 + 마커 사양·카메라 내부파라미터(고정 설정, 매번 오는 값 아님) |
| 출력 | 마커 검출 성공 여부 + (성공 시) 마커의 **카메라 기준** 위치·방향 |

이 함수는 로봇 자세를 모른다 — 이미지 캡처 시점의 로봇 자세와 짝짓는 일은 vision 바깥의 캘리브레이션 로직이 담당(hand-eye solve용으로 여러 자세를 모을 때 필요). 같은 함수가 F5.1(hand-eye 캘리브레이션 — 고정 마커를 로봇이 여러 자세에서 봄)과 F5.2(종이/지그 위치 인식 — 마커를 한두 자세에서 봄)에 재사용된다.

### C. 완성작 촬영 (BRD F8.2 — AC3·F6.3 지원) — `이후`

| | 내용 |
|---|---|
| 트리거 | moveit2의 "작업 완료" 메시지 |
| 입력 | 그 시점에 촬영한 이미지 |
| 출력 | 촬영된 이미지 그대로(가공 없음) — AC3(CLIP 판정)·F6.3(이력 저장)이 사용 |

### D. 중간 진행 모니터링 (BRD F8.1) — `이후`

| | 내용 |
|---|---|
| 트리거 | 전체 스트로크의 약 50% 지점 — **작업당 1회만**. 도달 판정은 **⑥ 작업 관리**가 한다(moveit2는 스트로크 완료만 알린다) |
| 입력 | 그 시점에 촬영한 이미지 + 그 시점까지 그려졌어야 할 부분(계획의 절반) |
| 출력 | 계획 대비 실제가 얼마나 다른지에 대한 정보(유사도·차이) |

**판단(계속/중단)은 vision이 내리지 않는다** — 사람이 이 정보를 보고 운영 UI에서 결정. Phase 1이 "사람이 지켜보는 환경"이라는 BRD 전제와 일치.

**C·D 모두 촬영 자세 이동이 선행된다 (BRD F8.3)** — 펜과 카메라가 같은 플랜지에 있어 작화 자세에서는 종이를 정면으로 볼 수 없다. pen-up → 촬영 자세 이동 → 촬영 → 복귀 순서이며, 이 이동은 로봇 동작이라 moveit2가 수행한다(vision은 촬영된 이미지만 받는다).

### E. 입력 적합성 검사 (F1.2) — `MVP 코어`

| | 내용 |
|---|---|
| 트리거 | operator가 이미지 업로드를 받은 직후 |
| 입력 | 이미지 한 장 (아직 검사되지 않은 것) |
| 출력 | 적합 여부 + **부적합 시 사유** (BRD F1.2 — "사유와 함께 거부") |

**2026-07-27 결정 — 검사 주체는 vision이다.** `Process Flow.md` §2.1은 이 판정을 운영 레인에 그렸으나, "흰 배경 라인아트냐"는 픽셀을 봐야 판정된다. 판정을 픽셀 아는 쪽에 두는 편이 자연스럽고 N1(제품 코드 C++)과도 맞는다 — 업로드 검사에 모듈 경계를 한 번 건너는 비용은 감수한다.

**A와는 별개 함수다.** A는 여전히 "통과된 이미지가 들어온다"를 계약으로 두고, 검사 결과에 따라 A를 부를지는 operator(⑥ 작업 관리)가 정한다. 부적합이면 로봇은 한 번도 움직이지 않는다 (PF §3.1 — 잘못된 입력을 가장 값싸게 막는 자리).

판정 기준의 구체값(흰 배경 비율·색상 수·선 두께 분포 등)은 구현 단계 정의 대상이다.

---

## A의 인터페이스 — 확정

의미(위 표)는 07-26에, 형식(아래)은 07-27에 확정됐다. **이 절이 형식의 원본이다** —
헤더 주석과 이 문서가 어긋나면 헤더가 맞다고 보고 이 문서를 고친다.

### 경계 타입 — `atlier/vision/stroke_map.hpp`

**OpenCV·ROS 어디에도 의존하지 않는다.** 지도를 받기만 하는 쪽(moveit2)이 이 헤더 하나만
include 하면 되게 하기 위함이다. 의존을 늘리지 말 것.

```cpp
namespace atlier::vision {

struct Point2 { double x_mm{}, y_mm{}; };

struct Stroke {
  std::vector<Point2> points;    // 그리는 순서 있음 (끝 → 끝)
  bool closed{false};            // 첫 점과 끝 점이 이어짐. 중복 점은 넣지 않는다
};

struct StrokeMap {                 // = "지도"
  std::vector<Stroke> strokes;     // ⚠️ 순서 없음 — F3.1 은 moveit2 책임
  double paper_w_mm{210.0};
  double paper_h_mm{297.0};
};

// 통계 유틸 — 총 선길이는 N2(15분) 예산 추정과 AC2 측정의 입력이라 받는 쪽에서도 쓴다
std::size_t TotalPointCount(const StrokeMap &);
double      TotalDrawLengthMm(const StrokeMap &);
bool        BoundingBox(const StrokeMap &, Point2 & min_out, Point2 & max_out);
}
```

여백은 지도에 담지 않는다 — 좌표가 이미 종이 좌상단 원점이라 소비자가 여백을 알 필요가 없다.

### 파라미터 — `atlier/vision/params.hpp`

| 필드 | 기본값 | 비고 |
|---|---|---|
| `paper_w_mm` / `paper_h_mm` | 210 / 297 | A4 세로. 가로로 쓰려면 값을 맞바꾼다 |
| `margin_mm` | 15 | 사방 여백 → 작화영역 180 × 267 |
| **`resample_step_mm`** | **1.0** | **그대로 moveit2 데카르트 waypoint 밀도가 된다.** 작을수록 선은 매끄럽지만 점 수가 늘어 N2를 압박한다 — 실측 후 조정 |
| `min_stroke_len_mm` | 2.0 | 이보다 짧으면 노이즈로 폐기 |
| `blur_ksize` | 3 | 홀수. 0이면 블러 생략 |
| `invert` | false | 흰 배경·검은 선이 기본 |
| `use_thinning` | true | `ximgproc::thinning` centerline. Canny는 선의 **이중 윤곽**을 낸다 (SA §5.3) |

### API — `atlier/vision/image_to_map.hpp` (OpenCV 의존)

```cpp
StrokeMap BuildStrokeMap(const cv::Mat &, const Params &);
StrokeMap BuildStrokeMapFromFile(const std::string &, const Params &);

// 단계별 — 중간 결과를 눈으로 보기 위해 공개한다
using PolylinePx = std::vector<cv::Point2d>;
cv::Mat                 Preprocess(const cv::Mat &, const Params &);                      // F2.1
std::vector<ContourPx>  ExtractContours(const cv::Mat & binary, const Params &);          // F2.2
std::vector<PolylinePx> ToPolylines(const std::vector<ContourPx> &, const Params &);      // F2.3
StrokeMap               ScaleToPaper(const std::vector<PolylinePx> &, cv::Size, const Params &); // F2.4

StrokeMap MakeDummyMap(const Params &);   // 하네스 검증용 — 이미지를 보지 않는다
```

픽셀 단계는 `cv::Point2d`를 쓰고 mm 로 넘어가는 곳은 `ScaleToPaper` 한 곳뿐이다 —
`x_mm` 필드에 픽셀값이 담기는 사고를 타입으로 막는다.

### 규약

- **배치**: 종횡비 유지 → 작화영역에 맞춰 축소 → 중앙 정렬(letterbox)
- **실패**: 빈 이미지·성립하지 않는 파라미터는 `std::invalid_argument`, 파일 읽기 실패는
  `std::runtime_error`. **스트로크 0개는 정상 반환**이다
- **F1.2 적합성 검사는 A가 하지 않는다** — "통과된 이미지가 들어온다"가 A의 계약이다.
  단 **검사 자체는 vision 이 맡기로 확정**됐다(2026-07-27) — A와 별개 함수로 둔다 (위 E 참조)
- **경계 전달 형식(토픽/서비스/메시지 타입)은 아직 정하지 않았다** — 코어를 ROS 무관
  라이브러리로 분리해 둔 이유가 이 결정을 moveit2 착수 시점까지 늦출 수 있어서다

---

## 구성

```
vision/
├── README.md            # (이 파일) 인터페이스 원본 + 세팅 절차
├── Dockerfile           # ROS 2 Jazzy + OpenCV 4.6 contrib — moveit2 와 별도 이미지
├── compose.yml          # 실행 정의 (GPU 설정 없음)
├── compose.override.yml # ⚠️ PC별 GPU 설정 — 필요한 PC만 만든다 (git 제외, §2)
├── entrypoint.sh
├── run_container.sh     # 헬퍼 (build/up/shell/down/logs/config)
├── data/                # ⛔ git 제외 — 입력 샘플과 CLI 산출물
└── ws_vision/src/
    ├── vision_core/     # ROS 무관 C++ 라이브러리 + 확인용 CLI
    └── vision_node/     # rclcpp 래퍼 (경계 인터페이스 미정)
```

**2층으로 나눈 이유** — 코어가 ROS를 모르면 ROS 없이 단위 테스트가 되고, 경계 전달 형식을
나중에 정할 수 있으며, moveit2 는 타입 헤더 하나만 include 하면 된다. `vision_core` 에
`rclcpp` 를 붙이지 말 것.

**moveit2 와 컨테이너를 분리한 이유** — RViz2·MoveIt2 가 필요 없어 이미지가 훨씬 가볍고,
공용 moveit2 이미지를 건드리지 않으므로 동료 PC에 재빌드를 강요하지 않는다. 두 환경 통합은
추후 과제.

## 1. 이미지 빌드 (최초 1회)

```bash
cd src/vision
./run_container.sh build      # 5~15분 (ros-base + OpenCV)
```

전제 조건(Docker·Compose v2·docker 그룹 등)은 `src/moveit2/README.md` §0과 같다.

## 2. GPU 설정 — 대개 필요 없다

**vision 은 CPU 만으로 동작한다.** 결과 확인이 파일 출력이라 GPU 없이 지장이 없고,
`cv::imshow` 로 창을 띄울 때만 렌더 노드 접근이 필요하다. 그때만 `compose.override.yml` 을
만든다 — 스니펫은 `src/moveit2/README.md` §2 의 (A)/(B)/(C)와 같되 서비스 이름을
`moveit2` → `vision` 으로 바꾼다.

## 3. 빌드와 실행

```bash
./run_container.sh shell           # 컨테이너 진입
colcon build --symlink-install     # 워크스페이스 루트가 기본 작업 디렉터리
source install/setup.bash
```

### 결과 확인 — `stroke_map_cli`

**이 도구가 있는 이유는 결과를 눈으로 보기 위해서다.** 숫자는 stdout, 그림은 SVG·PNG,
좌표는 CSV 로 떨어진다. `data/` 가 호스트에 마운트되어 있어 컨테이너 밖에서 바로 열면 된다.

```bash
# 알고리즘 없이 하네스만 검증 (이미지를 읽지 않는다)
ros2 run vision_core stroke_map_cli --dummy --out-dir ~/data/out --show-points

# 실제 이미지
ros2 run vision_core stroke_map_cli --in ~/data/cat.png --out-dir ~/data/out --dump-stages
```

| 출력 | 내용 |
|---|---|
| `map.svg` | **주력.** mm 를 그대로 쓰므로 브라우저에서 A4 실제 비율로 보이고, **인쇄하면 자로 실측**할 수 있다. 스트로크마다 색이 달라 몇 조각인지 세어진다 |
| `04_paper.png` | 종이 배치를 300DPI(2480×3508)로 렌더 — 뷰어에서 바로 확인용 |
| `map.csv` | `stroke_idx,point_idx,x_mm,y_mm,closed` — 좌표 수치 직접 확인 |
| `01~03_*.png` | `--dump-stages` 일 때만. F2.1 이진화 · F2.2 윤곽 · F2.3 재샘플링(점을 찍어 **간격을 눈으로 확인**) |

`--show-points` 는 SVG 에 점을 찍는다(간격 확인용, 파일이 커진다). 전체 옵션은 `--help`.

### ROS 노드

```bash
ros2 run vision_node vision_node --ros-args \
  --params-file install/vision_node/share/vision_node/config/vision_params.yaml \
  -p image_path:=/home/rosuser/data/cat.png -p svg_path:=/home/rosuser/data/out_node.svg
```

> ⚠️ **인자 순서 주의** — ROS 2 는 뒤에 오는 것이 이긴다. `--params-file` 을 `-p` 뒤에 두면
> YAML 의 빈 `image_path` 가 개별 지정을 덮어써 노드가 그냥 대기한다.

경계 인터페이스가 미정이라 이 노드는 아직 토픽·서비스를 열지 않는다. 파라미터·빌드·링크
경로가 살아 있는지 확인하는 용도다.

---

## 구현 상태

| 단계 | 상태 |
|---|---|
| F2.1 `Preprocess` | ⬜ TODO — 그레이스케일·이진화·노이즈 제거 |
| F2.2 `ExtractContours` | ⬜ TODO — `thinning` 호출부만 있음. **함정**: `findContours` 는 폐곡선 경계를 주므로 1픽셀 centerline 에 그대로 쓰면 선의 양쪽을 돌아 같은 선을 두 번 긋게 된다. 그래프 기반 선 추적이 필요 |
| F2.3 `ToPolylines` | ⬜ TODO — **함정**: `resample_step_mm` 은 mm 인데 이 단계 좌표는 픽셀이다. 먼저 mm 로 옮기고 재샘플링하는 편이 자연스러울 수 있으며, 그 경우 F2.3·F2.4 경계가 지금 선언과 달라진다 |
| F2.4 `ScaleToPaper` | ✅ 구현됨 — letterbox 중앙정렬·여백·closed 판정 |
| 하네스 (SVG·PNG·CSV·CLI) | ✅ 구현됨 |
| B · C · D | ⬜ 설계만 |
| E (F1.2 적합성 검사) | ⬜ 설계만 — 주체는 확정(2026-07-27), 함수·판정 기준은 미착수 |

검증된 것 (2026-07-27): 더미 지도 관통 — 스트로크 4개 / 점 1825개 / 총 선길이 1820.75mm
(테두리 894 + 대각선 644.0 + 원 282.7 검산 일치), 바운딩박스 (15,15)~(195,282) 로 여백
15mm 정확. SVG 210×297mm, PNG 2480×3508(=300DPI A4), CSV 1825행. ROS 노드 경로까지 동작.

## 미결

- ~~F1.2 적합성 검사의 주체~~ → **확정 (2026-07-27) — vision 이 한다** (위 E 참조).
  남은 것은 **판정 기준의 구체값과 함수 시그니처** — A 와 같은 2층 구조를 따를지,
  `Params` 를 공유할지는 착수 시 정한다
- F2.3·F2.4 의 경계 (위 구현 상태 표의 함정) — 재샘플링을 픽셀에서 할지 mm 에서 할지
- F5.2용 마커가 지그에 고정되어 있는지 확인 필요 (종이 쪽에 있으면 "F5.2는 1회"라는 전제가 무너짐)
- D의 "그 시점까지 그려졌어야 할 부분"은 moveit2가 정한 순서·진행률과 vision의 스트로크
  데이터를 합쳐야 만들어짐 — 이 조합을 누가 만드는지 미정. 단 **⑥ 작업 관리는 후보에서
  빠진다(2026-07-27)**: ⑥이 들고 있는 계획 정보는 스트로크 **개수**·선길이·이동시간뿐이고
  좌표는 없어(`atlier::job::PlanSummary`) 조합할 재료가 없다. 상태 소유자가 대용량
  데이터까지 들면 경계가 뚱뚱해져서 의도적으로 뺀 것이다. 남은 후보는 vision 또는
  moveit2이며, 순서를 정한 주체이자 지도를 받은 쪽이라 **moveit2가 유력**

## 설계 참조

| 문서 | 경로 |
|---|---|
| 시스템 구조 | `docs/System Architecture.md` — §2 전체 구조도, §4 구성 블록 설명, §5 기술 스택 |
| 요구사항 | `docs/Business Requirements.md` |
| 처리 흐름 | `docs/Process Flow.md` |
| 경계 상대편 | `src/moveit2/README.md` — 지도를 받아 F3.1·F3.2·F4 를 수행 |
| 경계 상대편 | `src/operator/README.md` — E(F1.2) 판정을 요청하고, C·D 정보를 화면·이력으로 받는다 |
