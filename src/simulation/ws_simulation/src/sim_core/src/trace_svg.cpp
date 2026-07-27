#include "atlier/sim/trace_svg.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

#include "atlier/sim/trace_io.hpp"

namespace atlier::sim
{
namespace
{

std::string Num(double value, int precision = 3)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(precision);
  out << value;
  return out.str();
}

/// 조각마다 다른 색. 한 스트로크가 여러 색으로 쪼개져 보이면 선이 끊긴 것이다.
const char * SegmentColor(std::size_t index)
{
  static const char * kPalette[] = {
    "#1a73e8", "#e8710a", "#188038", "#d93025",
    "#9334e6", "#00897b", "#8d6e63", "#7cb342",
  };
  return kPalette[index % (sizeof(kPalette) / sizeof(kPalette[0]))];
}

/// 연속 접촉 구간의 [시작, 끝] 인덱스 목록.
std::vector<std::pair<std::size_t, std::size_t>> ContactSegments(const DrawnTrace & trace)
{
  std::vector<std::pair<std::size_t, std::size_t>> segments;
  bool inside = false;
  std::size_t start = 0;
  for (std::size_t index = 0; index < trace.samples.size(); ++index) {
    const bool contact = trace.samples[index].in_contact;
    if (contact && !inside) {
      start = index;
      inside = true;
    } else if (!contact && inside) {
      segments.emplace_back(start, index - 1);
      inside = false;
    }
  }
  if (inside && !trace.samples.empty()) {
    segments.emplace_back(start, trace.samples.size() - 1);
  }
  return segments;
}

}  // namespace

std::string RenderSvg(const DrawnTrace & trace, const PlannedMap * planned, const SvgStyle & style)
{
  const double width = trace.paper_w_mm;
  const double height = trace.paper_h_mm;

  std::ostringstream out;
  // mm 를 그대로 쓴다 — 브라우저에서 A4 실제 비율로 보이고 인쇄하면 자로 실측된다.
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
      << "width=\"" << Num(width, 1) << "mm\" height=\"" << Num(height, 1) << "mm\" "
      << "viewBox=\"0 0 " << Num(width, 1) << " " << Num(height, 1) << "\">\n";
  out << "  <rect width=\"" << Num(width, 1) << "\" height=\"" << Num(height, 1)
      << "\" fill=\"white\" stroke=\"#cccccc\" stroke-width=\"0.2\"/>\n";

  if (style.show_margin && style.margin_mm > 0.0) {
    out << "  <rect x=\"" << Num(style.margin_mm, 1) << "\" y=\"" << Num(style.margin_mm, 1)
        << "\" width=\"" << Num(width - 2 * style.margin_mm, 1)
        << "\" height=\"" << Num(height - 2 * style.margin_mm, 1)
        << "\" fill=\"none\" stroke=\"#eeeeee\" stroke-width=\"0.2\" stroke-dasharray=\"2 2\"/>\n";
  }

  // ── 계획 (바닥에 옅게 깔아 실제와 겹쳐 보이게) ─────────────────────────────
  if (style.show_planned && planned != nullptr) {
    out << "  <g fill=\"none\" stroke=\"#b0b0b0\" stroke-width=\"" << Num(style.planned_width_mm)
        << "\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n";
    for (const PlannedStroke & stroke : planned->strokes) {
      if (stroke.points.size() < 2) {
        continue;
      }
      out << "    <polyline points=\"";
      for (const Point2 & point : stroke.points) {
        out << Num(point.x_mm) << "," << Num(point.y_mm) << " ";
      }
      if (stroke.closed) {
        out << Num(stroke.points.front().x_mm) << "," << Num(stroke.points.front().y_mm);
      }
      out << "\"/>\n";
    }
    out << "  </g>\n";
  }

  if (!style.show_drawn) {
    out << "</svg>\n";
    return out.str();
  }

  const std::vector<std::pair<std::size_t, std::size_t>> segments = ContactSegments(trace);

  // ── 공중 이동 (pen-up) ─────────────────────────────────────────────────────
  if (style.show_travel && segments.size() >= 2) {
    out << "  <g fill=\"none\" stroke=\"#dddddd\" stroke-width=\"0.2\" stroke-dasharray=\"1 1\">\n";
    for (std::size_t index = 1; index < segments.size(); ++index) {
      const ContactSample & from = trace.samples[segments[index - 1].second];
      const ContactSample & to = trace.samples[segments[index].first];
      out << "    <line x1=\"" << Num(from.x_mm) << "\" y1=\"" << Num(from.y_mm)
          << "\" x2=\"" << Num(to.x_mm) << "\" y2=\"" << Num(to.y_mm) << "\"/>\n";
    }
    out << "  </g>\n";
  }

  // ── 실제로 그어진 선 ───────────────────────────────────────────────────────
  out << "  <g fill=\"none\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n";
  for (std::size_t segment_index = 0; segment_index < segments.size(); ++segment_index) {
    const auto [first, last] = segments[segment_index];
    const char * color = style.color_by_segment ? SegmentColor(segment_index) : "#202124";

    if (!style.width_by_force) {
      out << "    <polyline stroke=\"" << color << "\" stroke-width=\""
          << Num(style.drawn_width_mm) << "\" points=\"";
      for (std::size_t index = first; index <= last; ++index) {
        out << Num(trace.samples[index].x_mm) << "," << Num(trace.samples[index].y_mm) << " ";
      }
      out << "\"/>\n";
      continue;
    }

    // 필압을 굵기에 반영하려면 구간마다 따로 그려야 한다 — 파일이 커지는 대신
    // 약하게 눌린 곳이 가늘게 보인다.
    for (std::size_t index = first + 1; index <= last; ++index) {
      const ContactSample & previous = trace.samples[index - 1];
      const ContactSample & current = trace.samples[index];
      const double reference = style.force_ref_n > 0.0 ? style.force_ref_n : 1.0;
      const double scale = std::max(0.15, std::min(2.0, current.force_n / reference));
      out << "    <line stroke=\"" << color << "\" stroke-width=\""
          << Num(style.drawn_width_mm * scale) << "\" x1=\"" << Num(previous.x_mm)
          << "\" y1=\"" << Num(previous.y_mm) << "\" x2=\"" << Num(current.x_mm)
          << "\" y2=\"" << Num(current.y_mm) << "\"/>\n";
    }
  }
  out << "  </g>\n";
  out << "</svg>\n";
  return out.str();
}

bool SaveSvg(
  const DrawnTrace & trace, const PlannedMap * planned,
  const std::string & path, const SvgStyle & style)
{
  return SaveText(path, RenderSvg(trace, planned, style));
}

}  // namespace atlier::sim
