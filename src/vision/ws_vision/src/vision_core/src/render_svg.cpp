#include "atlier/vision/render_svg.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>

namespace atlier::vision
{
namespace
{

/// 구분이 잘 되는 색 순환 — 지도가 몇 조각으로 쪼개졌는지 눈으로 세기 위한 용도다.
const char * const kPalette[] = {
  "#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4",
  "#008080", "#9a6324", "#800000", "#808000", "#000075",
};
constexpr std::size_t kPaletteSize = sizeof(kPalette) / sizeof(kPalette[0]);

/// mm 값을 SVG 좌표 문자열로. 소수 3자리(=1µm)면 충분하고, 꼬리 0 을 덜어내면
/// 점이 수천 개일 때 파일 크기가 눈에 띄게 줄어든다.
std::string Num(double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  std::string text(buffer);
  if (text.find('.') != std::string::npos) {
    text.erase(text.find_last_not_of('0') + 1);
    if (!text.empty() && text.back() == '.') {
      text.pop_back();
    }
  }
  return text;
}

}  // namespace

std::string RenderSvg(const StrokeMap & map, const SvgStyle & style)
{
  std::ostringstream out;

  // width/height 를 mm 로 주고 viewBox 를 같은 수치로 두면, 지도 좌표를 그대로
  // 써도 브라우저·인쇄에서 A4 실제 크기로 나온다.
  out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
      << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\"\n"
      << "     width=\"" << Num(map.paper_w_mm) << "mm\""
      << " height=\"" << Num(map.paper_h_mm) << "mm\"\n"
      << "     viewBox=\"0 0 " << Num(map.paper_w_mm) << " " << Num(map.paper_h_mm) << "\">\n"
      << "  <title>Nyang_Nyang_Atlier stroke map</title>\n"
      << "  <rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n";

  if (style.margin_mm >= 0.0) {
    const double width = map.paper_w_mm - 2.0 * style.margin_mm;
    const double height = map.paper_h_mm - 2.0 * style.margin_mm;
    if (width > 0.0 && height > 0.0) {
      out << "  <!-- 작화영역 (여백 " << Num(style.margin_mm) << "mm) -->\n"
          << "  <rect x=\"" << Num(style.margin_mm) << "\" y=\"" << Num(style.margin_mm)
          << "\" width=\"" << Num(width) << "\" height=\"" << Num(height) << "\""
          << " fill=\"none\" stroke=\"#cccccc\" stroke-width=\"0.2\""
          << " stroke-dasharray=\"2 2\"/>\n";
    }
  }

  out << "  <g fill=\"none\" stroke-width=\"" << Num(style.stroke_width_mm) << "\""
      << " stroke-linecap=\"round\" stroke-linejoin=\"round\">\n";

  for (std::size_t index = 0; index < map.strokes.size(); ++index) {
    const Stroke & stroke = map.strokes[index];
    if (stroke.points.size() < 2) {
      continue;
    }
    const char * color = style.color_per_stroke ? kPalette[index % kPaletteSize] : "#000000";

    // closed 는 polygon 이 알아서 첫 점으로 닫아 준다 — 규약대로 첫 점을 끝에
    // 중복해 넣지 않아도 되는 이유다.
    const char * tag = stroke.closed ? "polygon" : "polyline";
    out << "    <" << tag << " stroke=\"" << color << "\" points=\"";
    for (std::size_t i = 0; i < stroke.points.size(); ++i) {
      if (i != 0) {
        out << ' ';
      }
      out << Num(stroke.points[i].x_mm) << ',' << Num(stroke.points[i].y_mm);
    }
    out << "\"/>\n";
  }
  out << "  </g>\n";

  if (style.show_points) {
    out << "  <g fill=\"#333333\" stroke=\"none\">\n";
    for (const auto & stroke : map.strokes) {
      for (const auto & point : stroke.points) {
        out << "    <circle cx=\"" << Num(point.x_mm) << "\" cy=\"" << Num(point.y_mm)
            << "\" r=\"" << Num(style.point_radius_mm) << "\"/>\n";
      }
    }
    out << "  </g>\n";
  }

  out << "</svg>\n";
  return out.str();
}

bool SaveSvg(const StrokeMap & map, const std::string & path, const SvgStyle & style)
{
  std::ofstream file(path);
  if (!file) {
    return false;
  }
  file << RenderSvg(map, style);
  return file.good();
}

}  // namespace atlier::vision
