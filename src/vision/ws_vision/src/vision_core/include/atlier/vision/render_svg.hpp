// 지도 → SVG.
//
// SVG 의 좌표계는 mm 를 그대로 쓸 수 있고 원점도 좌상단·+y 아래라, 지도 좌표를
// **변환 없이** 그대로 옮긴다. 변환 버그가 낄 자리가 없다는 것이 이 형식을 고른
// 이유다. 브라우저에서 열면 A4 실제 비율로 보이고, 인쇄하면 실측 확인이 된다.
//
// 이 헤더는 OpenCV·ROS 에 의존하지 않는다.
#ifndef ATLIER_VISION_RENDER_SVG_HPP
#define ATLIER_VISION_RENDER_SVG_HPP

#include <string>

#include "atlier/vision/stroke_map.hpp"

namespace atlier::vision
{

struct SvgStyle
{
  /// 선 굵기(mm). 연필선 굵기와 무관한 표시용 값이다.
  double stroke_width_mm{0.35};

  /// 스트로크마다 색을 바꾼다 — 지도가 몇 조각으로 쪼개졌는지 한눈에 보인다.
  bool color_per_stroke{true};

  /// 각 점에 작은 원을 찍는다. **재샘플링 간격을 눈으로 확인하는 용도**라
  /// 점이 많으면 파일이 커진다.
  bool show_points{false};
  double point_radius_mm{0.25};

  /// 작화영역 테두리를 점선으로 그린다. 음수면 그리지 않는다.
  /// Params::margin_mm 을 그대로 넘기면 여백이 맞는지 보인다.
  double margin_mm{-1.0};
};

/// 지도를 SVG 문서 문자열로 만든다.
std::string RenderSvg(const StrokeMap & map, const SvgStyle & style = SvgStyle{});

/// RenderSvg 결과를 파일로 쓴다.
/// @return 쓰기 성공 여부
bool SaveSvg(const StrokeMap & map, const std::string & path, const SvgStyle & style = SvgStyle{});

}  // namespace atlier::vision

#endif  // ATLIER_VISION_RENDER_SVG_HPP
