#include "atlier/vision/debug_dump.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <system_error>

namespace atlier::vision
{
namespace
{

std::string JoinPath(const std::string & dir, const std::string & name)
{
  return (std::filesystem::path(dir) / name).string();
}

/// 배경으로 쓸 원본을 3채널 밝은 회색조로 눕힌다 — 그 위에 그린 선이 잘 보인다.
cv::Mat MakeBackdrop(const cv::Mat & src)
{
  cv::Mat gray;
  if (src.channels() == 1) {
    gray = src.clone();
  } else {
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  }
  cv::Mat faded;
  cv::addWeighted(gray, 0.35, cv::Mat::ones(gray.size(), CV_8UC1) * 255, 0.65, 0.0, faded);
  cv::Mat canvas;
  cv::cvtColor(faded, canvas, cv::COLOR_GRAY2BGR);
  return canvas;
}

/// 구분이 잘 되는 BGR 색 순환 (render_svg.cpp 의 팔레트와 같은 의도).
cv::Scalar PaletteColor(std::size_t index)
{
  static const cv::Scalar kPalette[] = {
    cv::Scalar(75, 25, 230),   cv::Scalar(75, 180, 60),   cv::Scalar(216, 99, 67),
    cv::Scalar(49, 130, 245),  cv::Scalar(180, 30, 145),  cv::Scalar(128, 128, 0),
    cv::Scalar(36, 99, 154),   cv::Scalar(0, 0, 128),     cv::Scalar(0, 128, 128),
    cv::Scalar(117, 0, 0),
  };
  constexpr std::size_t kCount = sizeof(kPalette) / sizeof(kPalette[0]);
  return kPalette[index % kCount];
}

}  // namespace

bool EnsureOutDir(const DumpOptions & opts)
{
  std::error_code ec;
  std::filesystem::create_directories(opts.out_dir, ec);
  return !ec;
}

bool DumpPreprocess(const cv::Mat & binary, const DumpOptions & opts)
{
  if (binary.empty()) {
    return false;
  }
  return cv::imwrite(JoinPath(opts.out_dir, "01_preprocess.png"), binary);
}

bool DumpContours(
  const cv::Mat & src, const std::vector<ContourPx> & contours, const DumpOptions & opts)
{
  if (src.empty()) {
    return false;
  }
  cv::Mat canvas = MakeBackdrop(src);
  for (std::size_t i = 0; i < contours.size(); ++i) {
    cv::drawContours(
      canvas, contours, static_cast<int>(i), PaletteColor(i), 1, cv::LINE_AA);
  }
  return cv::imwrite(JoinPath(opts.out_dir, "02_contours.png"), canvas);
}

bool DumpPolylines(
  const cv::Mat & src, const std::vector<PolylinePx> & polylines, const DumpOptions & opts)
{
  if (src.empty()) {
    return false;
  }
  cv::Mat canvas = MakeBackdrop(src);
  for (std::size_t i = 0; i < polylines.size(); ++i) {
    const cv::Scalar color = PaletteColor(i);
    const PolylinePx & polyline = polylines[i];
    for (std::size_t k = 1; k < polyline.size(); ++k) {
      cv::line(canvas, polyline[k - 1], polyline[k], color, 1, cv::LINE_AA);
    }
    // 점을 하나씩 찍는다 — 재샘플링 간격이 고른지 보는 것이 이 그림의 목적이다.
    for (const auto & point : polyline) {
      cv::circle(canvas, point, 1, cv::Scalar(40, 40, 40), cv::FILLED, cv::LINE_AA);
    }
  }
  return cv::imwrite(JoinPath(opts.out_dir, "03_polylines.png"), canvas);
}

bool DumpPaper(const StrokeMap & map, const Params & params, const DumpOptions & opts)
{
  const int dpi = std::max(1, opts.render_dpi);
  const double px_per_mm = static_cast<double>(dpi) / 25.4;

  const int width = static_cast<int>(std::lround(map.paper_w_mm * px_per_mm));
  const int height = static_cast<int>(std::lround(map.paper_h_mm * px_per_mm));
  if (width <= 0 || height <= 0) {
    return false;
  }

  cv::Mat canvas(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

  // 작화영역 테두리 — 여백이 맞는지 이 사각형으로 확인한다.
  if (params.margin_mm > 0.0) {
    const cv::Point top_left(
      static_cast<int>(std::lround(params.margin_mm * px_per_mm)),
      static_cast<int>(std::lround(params.margin_mm * px_per_mm)));
    const cv::Point bottom_right(
      static_cast<int>(std::lround((map.paper_w_mm - params.margin_mm) * px_per_mm)),
      static_cast<int>(std::lround((map.paper_h_mm - params.margin_mm) * px_per_mm)));
    cv::rectangle(canvas, top_left, bottom_right, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
  }

  const int thickness = std::max(1, static_cast<int>(std::lround(0.35 * px_per_mm)));
  for (std::size_t i = 0; i < map.strokes.size(); ++i) {
    const Stroke & stroke = map.strokes[i];
    if (stroke.points.size() < 2) {
      continue;
    }
    std::vector<cv::Point> pixels;
    pixels.reserve(stroke.points.size());
    for (const auto & point : stroke.points) {
      pixels.emplace_back(
        static_cast<int>(std::lround(point.x_mm * px_per_mm)),
        static_cast<int>(std::lround(point.y_mm * px_per_mm)));
    }
    cv::polylines(canvas, pixels, stroke.closed, PaletteColor(i), thickness, cv::LINE_AA);
  }

  return cv::imwrite(JoinPath(opts.out_dir, "04_paper.png"), canvas);
}

bool SaveCsv(const StrokeMap & map, const std::string & path)
{
  std::ofstream file(path);
  if (!file) {
    return false;
  }
  file << "stroke_idx,point_idx,x_mm,y_mm,closed\n";
  file.setf(std::ios::fixed);
  file.precision(3);
  for (std::size_t s = 0; s < map.strokes.size(); ++s) {
    const Stroke & stroke = map.strokes[s];
    for (std::size_t p = 0; p < stroke.points.size(); ++p) {
      file << s << ',' << p << ',' << stroke.points[p].x_mm << ',' << stroke.points[p].y_mm
           << ',' << (stroke.closed ? 1 : 0) << '\n';
    }
  }
  return file.good();
}

}  // namespace atlier::vision
