// vision 이 만든 계획 지도를 **파일로** 읽는다.
//
// ⚠️ **vision_core 에 링크하지 않는다 — 의도된 것이다.**
// 두 모듈은 별도 브랜치·별도 컨테이너에서 자라고 있고, 시뮬레이션이 계획 지도에
// 필요로 하는 것은 "좌표 몇 개"뿐이다. 코드로 붙이면 이 모듈이 OpenCV 까지 끌고
// 오게 되고 브랜치도 얽힌다. **`stroke_map_cli` 가 뱉는 `map.csv` 를 경유**하면
// 결합이 파일 형식 하나로 줄어든다.
//
// 읽는 형식 (vision 의 `map.csv`):
//     stroke_idx,point_idx,x_mm,y_mm,closed
//
// 형식이 바뀌면 여기가 깨진다 — 그 대가로 얻는 것이 모듈 독립성이다.
#ifndef ATLIER_SIM_PLANNED_MAP_HPP
#define ATLIER_SIM_PLANNED_MAP_HPP

#include <string>
#include <vector>

#include "atlier/sim/contact_trace.hpp"

namespace atlier::sim
{

/// 계획된 스트로크 하나 (vision `atlier::vision::Stroke` 의 파일 표현).
struct PlannedStroke
{
  std::vector<Point2> points;
  bool closed{false};
};

/// 계획 지도 — 그리려고 했던 것.
///
/// ⚠️ 스트로크들 **사이에는 순서가 없다**(vision 규약). 실제 그리는 순서는
/// moveit2 의 F3.1 이 정하므로, 이 지도만으로는 "어느 것을 먼저 그었는지" 알 수 없다.
/// 겹쳐 그려 모양을 비교하는 데는 지장이 없다.
struct PlannedMap
{
  std::vector<PlannedStroke> strokes;
  double paper_w_mm{210.0};
  double paper_h_mm{297.0};
};

/// `map.csv` 를 읽는다. 파일이 없거나 형식이 어긋나면 std::runtime_error.
/// 헤더 줄(`stroke_idx,...`)은 있으면 건너뛴다.
PlannedMap LoadPlannedMapCsv(const std::string & path);

/// 계획된 총 선 길이(mm) — vision 의 `TotalDrawLengthMm` 과 같은 계산.
/// 실제로 그려진 `DrawnLengthMm` 과 짝지으면 "얼마나 덜 그렸는가"가 나온다.
double PlannedLengthMm(const PlannedMap & map);

}  // namespace atlier::sim

#endif  // ATLIER_SIM_PLANNED_MAP_HPP
