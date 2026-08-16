#include "vision_stylize/types.hpp"

#include <sstream>

namespace vision_stylize
{

const char * mime_type(ImageFormat format)
{
  switch (format) {
    case ImageFormat::kPng:  return "image/png";
    case ImageFormat::kJpeg: return "image/jpeg";
    case ImageFormat::kWebp: return "image/webp";
  }
  return "application/octet-stream";
}

const char * extension(ImageFormat format)
{
  switch (format) {
    case ImageFormat::kPng:  return ".png";
    case ImageFormat::kJpeg: return ".jpg";
    case ImageFormat::kWebp: return ".webp";
  }
  return ".bin";
}

std::string PreprocessReport::summary() const
{
  std::ostringstream os;
  os << src_width << "x" << src_height << " -> " << out_width << "x" << out_height;

  const int trimmed = trim_left + trim_top + trim_right + trim_bottom;
  if (trimmed > 0) {
    os << " · 여백 잘라냄 L" << trim_left << " T" << trim_top
       << " R" << trim_right << " B" << trim_bottom;
  } else {
    os << " · 여백 없음";
  }

  os << (grayscale ? " · 흑백" : " · 색 유지");
  os << " · fit=" << fit;
  return os.str();
}

bool is_retryable(long http_status, bool transport_error)
{
  if (transport_error) {
    return true;
  }
  if (http_status == 429) {
    return true;
  }
  return http_status >= 500 && http_status <= 599;
}

ApiError::ApiError(long http_status, const std::string & body, bool transport_error)
: std::runtime_error(
    transport_error
    ? ("전송 실패: " + body)
    : ("HTTP " + std::to_string(http_status) + " — " + body)),
  http_status_(http_status),
  body_(body),
  transport_error_(transport_error),
  retryable_(is_retryable(http_status, transport_error))
{
}

}  // namespace vision_stylize
