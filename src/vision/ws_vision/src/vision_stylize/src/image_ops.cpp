#include "vision_stylize/image_ops.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace vision_stylize
{

namespace
{

cv::Mat decode(const std::vector<unsigned char> & bytes, const char * what)
{
  cv::Mat raw(1, static_cast<int>(bytes.size()), CV_8UC1,
    const_cast<unsigned char *>(bytes.data()));
  cv::Mat image = cv::imdecode(raw, cv::IMREAD_COLOR);
  if (image.empty()) {
    throw PreflightError(std::string("이미지를 디코드할 수 없다: ") + what);
  }
  return image;
}

std::vector<unsigned char> encode_png(const cv::Mat & image)
{
  std::vector<unsigned char> out;
  if (!cv::imencode(".png", image, out)) {
    throw PreflightError("PNG 인코딩에 실패했다");
  }
  return out;
}

/// 가장자리에서 안쪽으로 들어오며 "바깥 테두리 색과 같은 줄"을 걷어낸다.
/// 판정 기준은 그 줄의 모든 화소가 기준색에서 tolerance 안에 있는가.
cv::Rect trim_uniform_border(const cv::Mat & image, int tolerance)
{
  if (image.empty()) {
    return cv::Rect(0, 0, 0, 0);
  }

  const cv::Vec3b reference = image.at<cv::Vec3b>(0, 0);
  auto is_border_color = [&](const cv::Vec3b & px) {
      return std::abs(px[0] - reference[0]) <= tolerance &&
             std::abs(px[1] - reference[1]) <= tolerance &&
             std::abs(px[2] - reference[2]) <= tolerance;
    };
  auto row_uniform = [&](int y) {
      for (int x = 0; x < image.cols; ++x) {
        if (!is_border_color(image.at<cv::Vec3b>(y, x))) {return false;}
      }
      return true;
    };
  auto col_uniform = [&](int x) {
      for (int y = 0; y < image.rows; ++y) {
        if (!is_border_color(image.at<cv::Vec3b>(y, x))) {return false;}
      }
      return true;
    };

  int top = 0, bottom = image.rows - 1, left = 0, right = image.cols - 1;
  while (top < bottom && row_uniform(top)) {++top;}
  while (bottom > top && row_uniform(bottom)) {--bottom;}
  while (left < right && col_uniform(left)) {++left;}
  while (right > left && col_uniform(right)) {--right;}

  // 전부 같은 색인 그림이면 원본을 그대로 둔다 (잘라낼 것이 없다).
  if (right <= left || bottom <= top) {
    return cv::Rect(0, 0, image.cols, image.rows);
  }
  return cv::Rect(left, top, right - left + 1, bottom - top + 1);
}

/// 목표 종횡비에 맞춰 캔버스를 조정한다. 배율 조정은 하지 않는다.
cv::Mat apply_fit(const cv::Mat & image, double target_aspect, FitMode mode)
{
  const double current = static_cast<double>(image.cols) / static_cast<double>(image.rows);
  // 이미 충분히 가까우면 손대지 않는다.
  if (std::abs(current - target_aspect) < 0.01) {
    return image;
  }

  if (mode == FitMode::kStretch) {
    cv::Mat out;
    const int width = image.cols;
    const int height = std::max(1, static_cast<int>(std::lround(width / target_aspect)));
    cv::resize(image, out, cv::Size(width, height), 0, 0, cv::INTER_AREA);
    return out;
  }

  if (mode == FitMode::kContain) {
    // 넉넉한 캔버스를 만들고 가운데 얹는다. 남는 곳은 흰색 — 고양이가 잘리지 않는다.
    int width = image.cols;
    int height = image.rows;
    if (current > target_aspect) {
      height = std::max(height, static_cast<int>(std::lround(width / target_aspect)));
    } else {
      width = std::max(width, static_cast<int>(std::lround(height * target_aspect)));
    }
    cv::Mat canvas(height, width, image.type(), cv::Scalar(255, 255, 255));
    const int x = (width - image.cols) / 2;
    const int y = (height - image.rows) / 2;
    image.copyTo(canvas(cv::Rect(x, y, image.cols, image.rows)));
    return canvas;
  }

  // kCover — 넘치는 쪽을 가운데 기준으로 잘라낸다.
  int width = image.cols;
  int height = image.rows;
  if (current > target_aspect) {
    width = std::max(1, static_cast<int>(std::lround(height * target_aspect)));
  } else {
    height = std::max(1, static_cast<int>(std::lround(width / target_aspect)));
  }
  const int x = (image.cols - width) / 2;
  const int y = (image.rows - height) / 2;
  return image(cv::Rect(x, y, width, height)).clone();
}

}  // namespace

FitMode parse_fit_mode(const std::string & name)
{
  if (name == "contain") {return FitMode::kContain;}
  if (name == "cover") {return FitMode::kCover;}
  if (name == "stretch") {return FitMode::kStretch;}
  throw PreflightError("--fit 은 contain · cover · stretch 중 하나여야 한다: " + name);
}

const char * fit_mode_name(FitMode mode)
{
  switch (mode) {
    case FitMode::kContain: return "contain";
    case FitMode::kCover:   return "cover";
    case FitMode::kStretch: return "stretch";
  }
  return "none";
}

ImageBlob preprocess(
  const ImageBlob & src, const PreprocessOptions & options, PreprocessReport * report)
{
  cv::Mat image = decode(src.bytes, src.filename.c_str());

  PreprocessReport local;
  local.src_width = image.cols;
  local.src_height = image.rows;
  local.grayscale = options.grayscale;
  local.fit = "none";

  // ① 가장자리 여백
  if (options.trim) {
    const cv::Rect keep = trim_uniform_border(image, options.trim_tolerance);
    local.trim_left = keep.x;
    local.trim_top = keep.y;
    local.trim_right = image.cols - (keep.x + keep.width);
    local.trim_bottom = image.rows - (keep.y + keep.height);
    if (keep.width > 0 && keep.height > 0) {
      image = image(keep).clone();
    }
  }

  // ② 목표 종횡비 (--size WxH 가 있을 때만)
  if (options.target_aspect && *options.target_aspect > 0.0) {
    image = apply_fit(image, *options.target_aspect, options.fit);
    local.fit = fit_mode_name(options.fit);
  }

  // ③ 색 지우기 — 지시를 어길 재료를 미리 없앤다. 토큰과는 무관하다.
  if (options.grayscale) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray, image, cv::COLOR_GRAY2BGR);
  }

  // ④ 크기 축소 — 입력 토큰을 줄이는 유일한 손잡이. 키우지는 않는다.
  if (options.max_edge > 0) {
    const int longest = std::max(image.cols, image.rows);
    if (longest > options.max_edge) {
      const double scale = static_cast<double>(options.max_edge) / static_cast<double>(longest);
      cv::Mat resized;
      cv::resize(
        image, resized,
        cv::Size(
          std::max(1, static_cast<int>(std::lround(image.cols * scale))),
          std::max(1, static_cast<int>(std::lround(image.rows * scale)))),
        0, 0, cv::INTER_AREA);
      image = resized;
    }
  }

  local.out_width = image.cols;
  local.out_height = image.rows;
  if (report) {
    *report = local;
  }

  ImageBlob out;
  out.bytes = encode_png(image);
  out.format = ImageFormat::kPng;
  out.filename = "input_sent.png";
  return out;
}

std::vector<unsigned char> binarize(const std::vector<unsigned char> & png_bytes)
{
  cv::Mat image = decode(png_bytes, "API 응답 PNG");

  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  cv::Mat bw;
  cv::threshold(gray, bw, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
  return encode_png(bw);
}

}  // namespace vision_stylize
