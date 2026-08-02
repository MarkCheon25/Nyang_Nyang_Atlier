// **계획과 실제를 한 장에 겹쳐 그린다.** 이 모듈의 확인 경로다.
//
// vision 의 SVG 가 "지도가 종이에 어떻게 앉는가"를 보여줬다면, 여기 SVG 는
// "그 지도대로 **실제로 그어졌는가**"를 보여준다. mm 를 그대로 쓰므로 브라우저에서
// A4 실제 비율로 보이고, 인쇄하면 자로 실측할 수 있다.
#ifndef ATLIER_SIM_TRACE_SVG_HPP
#define ATLIER_SIM_TRACE_SVG_HPP

#include <string>

#include "atlier/sim/contact_trace.hpp"
#include "atlier/sim/planned_map.hpp"

namespace atlier::sim
{

struct SvgStyle
{
  bool show_planned{true};       ///< 계획 지도를 옅은 회색으로 깔아 준다
  bool show_drawn{true};         ///< 실제로 그어진 선

  /// 조각마다 색을 바꾼다. **선 끊김을 눈으로 잡는 스위치** — 한 스트로크가
  /// 여러 색으로 쪼개져 보이면 필압이 모자라 펜이 떴다는 뜻이다 (R2).
  bool color_by_segment{true};

  /// 필압을 선 굵기에 반영한다. 약하게 눌린 구간이 가늘게 보인다.
  bool width_by_force{false};

  /// 펜이 떠서 이동한 구간(pen-up)을 점선으로 그린다. N2 예산에서 공중 이동이
  /// 차지하는 몫이 눈에 보인다.
  bool show_travel{false};

  double planned_width_mm{0.35};
  double drawn_width_mm{0.5};
  double force_ref_n{0.5};       ///< width_by_force 일 때 기준 필압
  double margin_mm{15.0};        ///< 참고선 — vision Params::margin_mm 과 맞춘다
  bool show_margin{true};
};

/// 계획(선택)과 실제를 겹쳐 SVG 문자열로 만든다. `planned` 가 nullptr 이면 실제만.
std::string RenderSvg(const DrawnTrace & trace, const PlannedMap * planned, const SvgStyle & style);

/// 위 결과를 파일로. 실패 시 false.
bool SaveSvg(
  const DrawnTrace & trace, const PlannedMap * planned,
  const std::string & path, const SvgStyle & style);

}  // namespace atlier::sim

#endif  // ATLIER_SIM_TRACE_SVG_HPP
