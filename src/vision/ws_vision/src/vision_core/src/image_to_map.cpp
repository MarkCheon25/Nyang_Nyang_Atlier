#include "atlier/vision/image_to_map.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace atlier::vision
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double Distance(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.x_mm - b.x_mm, a.y_mm - b.y_mm);
}

/// 선분 a→b 를 step 간격으로 샘플링해 뒤에 이어 붙인다.
/// include_end 가 false 면 b 를 넣지 않는다 — 변을 연달아 이을 때 중복을 피하기 위함.
void AppendSegment(Stroke & stroke, const Point2 & a, const Point2 & b, double step, bool include_end)
{
  const double length = Distance(a, b);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / step)));
  for (int i = 0; i < steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    stroke.points.push_back(Point2{a.x_mm + (b.x_mm - a.x_mm) * t, a.y_mm + (b.y_mm - a.y_mm) * t});
  }
  if (include_end) {
    stroke.points.push_back(b);
  }
}

}  // namespace

// ── F2.1 전처리 ─────────────────────────────────────────────────────────────
cv::Mat Preprocess(const cv::Mat & src, const Params & params)
{
  if (src.empty()) {
    throw std::invalid_argument("Preprocess: 입력 이미지가 비어 있다");
  }

  // TODO(F2.1): 그레이스케일 변환 → (blur_ksize 블러) → 이진화 → 노이즈 제거.
  //   기준은 Otsu 이진화지만(SA §5.1), 라인아트는 배경이 이미 흰색이라 고정
  //   임계값으로 충분할 수 있다. invert 파라미터로 반전 입력도 받는다.
  //   출력 규약: CV_8UC1, **선이 255(흰색)** — 이후 단계가 이것을 전제한다.
  (void)params;
  return cv::Mat();
}

// ── F2.2 윤곽 추출 ──────────────────────────────────────────────────────────
std::vector<ContourPx> ExtractContours(const cv::Mat & binary, const Params & params)
{
  if (binary.empty()) {
    // F2.1 이 아직 TODO 라 여기로 빈 영상이 온다. 채워지면 이 가드는 사라진다.
    return {};
  }

  cv::Mat work = binary;
  if (params.use_thinning) {
    // Canny 는 선의 **이중 윤곽**을 낸다 — centerline 을 먼저 뽑는다 (SA §5.3).
    cv::ximgproc::thinning(binary, work, cv::ximgproc::THINNING_ZHANGSUEN);
  }

  // TODO(F2.2): centerline 에서 선을 추출한다.
  //   ⚠️ 함정 — findContours 는 **폐곡선 경계**를 주므로 thinning 결과(1픽셀 선)에
  //   그대로 쓰면 선의 양쪽을 돌아 나오는 왕복 경로가 된다. 같은 선을 두 번 긋게
  //   되므로 그래프 기반 선 추적(분기점·끝점 검출 후 경로 분할)이 필요하다.
  //   방식 확정 전까지 비워 둔다.
  return {};
}

// ── F2.3 스트로크 변환 ──────────────────────────────────────────────────────
std::vector<PolylinePx> ToPolylines(const std::vector<ContourPx> & contours, const Params & params)
{
  // TODO(F2.3): 곡선 피팅 후 균일 간격 재추출 + min_stroke_len_mm 미만 폐기.
  //   ⚠️ 단위 문제 — resample_step_mm·min_stroke_len_mm 은 mm 인데 여기 좌표는
  //   픽셀이다. 픽셀 단계에서 재샘플링하려면 F2.4 의 스케일을 미리 알아야 하므로,
  //   순서를 바꿔 (먼저 mm 로 옮기고 → 재샘플링) 푸는 편이 자연스러울 수 있다.
  //   그 경우 F2.3 과 F2.4 의 경계가 지금 선언과 달라진다 — 구현 시 확정할 것.
  (void)contours;
  (void)params;
  return {};
}

// ── F2.4 종이 좌표 스케일링 ─────────────────────────────────────────────────
StrokeMap ScaleToPaper(
  const std::vector<PolylinePx> & polylines, cv::Size src_size, const Params & params)
{
  if (src_size.width <= 0 || src_size.height <= 0) {
    throw std::invalid_argument("ScaleToPaper: 원본 이미지 크기가 유효하지 않다");
  }
  const double drawable_w = DrawableWidthMm(params);
  const double drawable_h = DrawableHeightMm(params);
  if (drawable_w <= 0.0 || drawable_h <= 0.0) {
    throw std::invalid_argument("ScaleToPaper: 여백이 용지보다 크다");
  }

  // 종횡비를 유지한 채 작화영역에 맞춰 축소하고 중앙 정렬한다 (letterbox).
  const double scale =
    std::min(drawable_w / src_size.width, drawable_h / src_size.height);
  const double offset_x = params.margin_mm + (drawable_w - src_size.width * scale) * 0.5;
  const double offset_y = params.margin_mm + (drawable_h - src_size.height * scale) * 0.5;

  StrokeMap map;
  map.paper_w_mm = params.paper_w_mm;
  map.paper_h_mm = params.paper_h_mm;
  map.strokes.reserve(polylines.size());

  for (const auto & polyline : polylines) {
    if (polyline.size() < 2) {
      continue;
    }
    Stroke stroke;
    stroke.points.reserve(polyline.size());
    for (const auto & pixel : polyline) {
      // 픽셀 좌표계와 종이 좌표계는 방향이 같다(+x 오른쪽, +y 아래) — 반전 없이
      // 스케일과 오프셋만 적용한다.
      stroke.points.push_back(
        Point2{offset_x + pixel.x * scale, offset_y + pixel.y * scale});
    }

    // 첫 점과 끝 점이 재샘플링 간격 이내면 폐곡선으로 본다. 규약상 첫 점을
    // 끝에 중복해 넣지 않으므로, 사실상 같은 점이면 끝 점을 덜어낸다.
    if (stroke.points.size() >= 3) {
      const double gap = Distance(stroke.points.front(), stroke.points.back());
      if (gap <= params.resample_step_mm) {
        stroke.closed = true;
        if (gap <= 1e-9) {
          stroke.points.pop_back();
        }
      }
    }
    map.strokes.push_back(std::move(stroke));
  }
  return map;
}

// ── 통합 진입점 ─────────────────────────────────────────────────────────────
StrokeMap BuildStrokeMap(const cv::Mat & image, const Params & params)
{
  if (image.empty()) {
    throw std::invalid_argument("BuildStrokeMap: 입력 이미지가 비어 있다");
  }
  const cv::Mat binary = Preprocess(image, params);
  const std::vector<ContourPx> contours = ExtractContours(binary, params);
  const std::vector<PolylinePx> polylines = ToPolylines(contours, params);
  return ScaleToPaper(polylines, image.size(), params);
}

StrokeMap BuildStrokeMapFromFile(const std::string & path, const Params & params)
{
  const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
  if (image.empty()) {
    throw std::runtime_error("이미지를 읽지 못했다: " + path);
  }
  return BuildStrokeMap(image, params);
}

// ── 하네스 ──────────────────────────────────────────────────────────────────
StrokeMap MakeDummyMap(const Params & params)
{
  const double drawable_w = DrawableWidthMm(params);
  const double drawable_h = DrawableHeightMm(params);
  if (drawable_w <= 0.0 || drawable_h <= 0.0) {
    throw std::invalid_argument("MakeDummyMap: 여백이 용지보다 크다");
  }

  const double step = std::max(params.resample_step_mm, 0.01);
  const double x0 = params.margin_mm;
  const double y0 = params.margin_mm;
  const double x1 = x0 + drawable_w;
  const double y1 = y0 + drawable_h;

  StrokeMap map;
  map.paper_w_mm = params.paper_w_mm;
  map.paper_h_mm = params.paper_h_mm;

  // ① 작화영역 테두리 — 인쇄해서 자로 재면 여백이 실제로 margin_mm 인지 확인된다.
  {
    Stroke border;
    AppendSegment(border, Point2{x0, y0}, Point2{x1, y0}, step, false);
    AppendSegment(border, Point2{x1, y0}, Point2{x1, y1}, step, false);
    AppendSegment(border, Point2{x1, y1}, Point2{x0, y1}, step, false);
    AppendSegment(border, Point2{x0, y1}, Point2{x0, y0}, step, false);
    border.closed = true;
    map.strokes.push_back(std::move(border));
  }

  // ② 대각선 2개 — 중앙 정렬이 맞는지 교차점으로 확인한다.
  {
    Stroke diagonal;
    AppendSegment(diagonal, Point2{x0, y0}, Point2{x1, y1}, step, true);
    map.strokes.push_back(std::move(diagonal));
  }
  {
    Stroke diagonal;
    AppendSegment(diagonal, Point2{x1, y0}, Point2{x0, y1}, step, true);
    map.strokes.push_back(std::move(diagonal));
  }

  // ③ 중앙 원 — 곡선이 step 간격으로 샘플링되는지 눈으로 본다.
  {
    Stroke circle;
    const double center_x = x0 + drawable_w * 0.5;
    const double center_y = y0 + drawable_h * 0.5;
    const double radius = std::min(drawable_w, drawable_h) * 0.25;
    const int steps = std::max(8, static_cast<int>(std::ceil(2.0 * kPi * radius / step)));
    circle.points.reserve(static_cast<std::size_t>(steps));
    for (int i = 0; i < steps; ++i) {
      const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(steps);
      circle.points.push_back(
        Point2{center_x + radius * std::cos(theta), center_y + radius * std::sin(theta)});
    }
    circle.closed = true;
    map.strokes.push_back(std::move(circle));
  }

  return map;
}

}  // namespace atlier::vision
