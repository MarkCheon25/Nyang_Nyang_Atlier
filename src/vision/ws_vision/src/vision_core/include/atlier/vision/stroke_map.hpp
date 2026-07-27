// vision → moveit2 경계 타입.
//
// 이 헤더는 OpenCV·ROS 어느 쪽에도 의존하지 않는다 — moveit2 가 지도를 받으려고
// 이 헤더 하나만 include 하면 되게 하기 위함이다. 의존을 늘리지 말 것.
#ifndef ATLIER_VISION_STROKE_MAP_HPP
#define ATLIER_VISION_STROKE_MAP_HPP

#include <cstddef>
#include <vector>

namespace atlier::vision
{

/// 종이 평면 위의 점 하나.
///
/// 좌표 규약: **원점은 종이 좌상단**, +x 오른쪽, +y 아래. 이미지 픽셀 좌표계와
/// 방향이 같아 F2.4 가 방향 반전 없이 스케일만 적용하면 된다.
/// z 는 없다 — 펜 높이는 moveit2 가 캘리브레이션 결과로 붙인다(F4.1).
struct Point2
{
  double x_mm{0.0};
  double y_mm{0.0};
};

/// 펜을 종이에서 떼지 않고 긋는 연속 선 하나 (BRD 부록 A "스트로크").
struct Stroke
{
  /// 그리는 순서대로 나열된 점들 — 스트로크 **내부**에는 끝에서 끝까지 순서가 있다.
  /// 점과 점 사이는 직선으로 잇는다고 간주한다. 곡선도 조밀하게 재샘플링된
  /// 점열로 통일된다 (Params::resample_step_mm).
  std::vector<Point2> points;

  /// 마지막 점이 첫 점으로 이어지는 폐곡선인지. 윤곽선은 대개 true 다.
  /// true 여도 첫 점을 points 끝에 중복해 넣지 않는다 — 잇는 것은 소비자 몫이다.
  bool closed{false};
};

/// vision 이 moveit2 에 넘기는 **지도**.
///
/// ⚠️ **스트로크들 사이에는 순서가 없다.** 그리는 순서 최적화(F3.1)와 펜업/다운
/// 결정(F3.2)은 로봇의 실제 이동거리를 알아야 하는 일이라 moveit2 책임이다.
/// 스트로크 개수는 이 집합의 크기 그 자체이므로 별도 필드를 두지 않는다.
struct StrokeMap
{
  std::vector<Stroke> strokes;

  /// 이 지도가 어느 종이를 기준으로 계산됐는지. 좌표가 이미 종이 원점 기준이라
  /// 소비자는 여백을 알 필요가 없고, 용지 크기만 있으면 범위 검증이 된다.
  double paper_w_mm{210.0};
  double paper_h_mm{297.0};
};

// ── 통계 유틸 ──────────────────────────────────────────────────────────────
// 지도를 받은 쪽에서도 쓴다. 총 선 길이는 N2(15분) 예산 추정과 AC2 측정의
// 입력이 되므로 moveit2 쪽에서도 필요하다.

/// 모든 스트로크의 점 개수 합.
std::size_t TotalPointCount(const StrokeMap & map);

/// 펜이 종이에 닿은 채 이동하는 총 거리(mm). closed 스트로크는 마지막 점에서
/// 첫 점으로 돌아오는 구간까지 포함한다. 스트로크 **사이**의 펜업 이동은
/// 순서가 정해져야 알 수 있으므로 여기 포함되지 않는다.
double TotalDrawLengthMm(const StrokeMap & map);

/// 지도의 바운딩 박스. 점이 하나도 없으면 false 를 반환하고 출력은 건드리지 않는다.
bool BoundingBox(const StrokeMap & map, Point2 & min_out, Point2 & max_out);

}  // namespace atlier::vision

#endif  // ATLIER_VISION_STROKE_MAP_HPP
