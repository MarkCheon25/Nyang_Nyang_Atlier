#include "vision_stylize/cost_guard.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>

namespace vision_stylize
{

namespace
{

// ── 토큰 계수 — 전부 ⬜ 미확인 ─────────────────────────────────────────────
// 첫 실호출로 확인해 고친다 (llm_readme §1 미결 "입력 토큰 계수 미확인").
// 여기 값이 내는 금액은 자릿수 감각이지 청구서가 아니다.

/// 입력 이미지를 격자로 자를 때의 한 칸 크기(픽셀). 계열 공통이라고 알려진 값.
constexpr int kInputPatchPixels = 32;

/// 1024×1024 기준 출력 토큰. 다른 크기는 픽셀 수에 비례해 늘린다.
constexpr long kOutputTokensLow1024 = 272;
constexpr long kOutputTokensMedium1024 = 1056;
constexpr long kOutputTokensHigh1024 = 4160;
constexpr double kReferencePixels = 1024.0 * 1024.0;

/// size=auto 일 때 무엇으로 가정할지. 문서가 정사각형이 가장 빠르다고 말한다.
constexpr int kAssumedAutoEdge = 1024;

/// 대략 4바이트에 1토큰. 프롬프트는 전체 비용에서 아주 작은 몫이라 이 정도로 충분하다.
constexpr double kBytesPerTextToken = 4.0;

// ── 출력 크기 규칙 (llm_readme §1 확정사실) ────────────────────────────────
constexpr int kSizeMultiple = 16;
constexpr int kMaxEdge = 3840;
constexpr double kMaxAspect = 3.0;
constexpr long kMinPixels = 655360;
constexpr long kMaxPixels = 8294400;

/// PNG IHDR 에서 가로세로를 읽는다. 픽셀을 푸는 것이 아니라 머리말만 본다.
bool png_dimensions(const std::vector<unsigned char> & bytes, int * width, int * height)
{
  // 8바이트 서명 + 4바이트 길이 + "IHDR" + 가로(4) + 세로(4)
  if (bytes.size() < 24) {return false;}
  if (bytes[12] != 'I' || bytes[13] != 'H' || bytes[14] != 'D' || bytes[15] != 'R') {
    return false;
  }
  auto read_be32 = [&bytes](std::size_t at) {
      return (static_cast<std::uint32_t>(bytes[at]) << 24) |
             (static_cast<std::uint32_t>(bytes[at + 1]) << 16) |
             (static_cast<std::uint32_t>(bytes[at + 2]) << 8) |
             static_cast<std::uint32_t>(bytes[at + 3]);
    };
  *width = static_cast<int>(read_be32(16));
  *height = static_cast<int>(read_be32(20));
  return *width > 0 && *height > 0;
}

/// JPEG 의 SOF 마커에서 가로세로를 읽는다.
bool jpeg_dimensions(const std::vector<unsigned char> & bytes, int * width, int * height)
{
  std::size_t at = 2;   // SOI(FFD8) 다음부터
  while (at + 9 < bytes.size()) {
    if (bytes[at] != 0xFF) {++at; continue;}

    const unsigned char marker = bytes[at + 1];
    // SOF0~SOF3 · SOF5~SOF7 · SOF9~SOF11 · SOF13~SOF15 가 크기를 들고 있다.
    const bool is_sof =
      (marker >= 0xC0 && marker <= 0xCF) &&
      marker != 0xC4 && marker != 0xC8 && marker != 0xCC;
    if (is_sof) {
      *height = (bytes[at + 5] << 8) | bytes[at + 6];
      *width = (bytes[at + 7] << 8) | bytes[at + 8];
      return *width > 0 && *height > 0;
    }
    if (marker == 0xD8 || marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)) {
      at += 2;   // 길이 없는 마커
      continue;
    }
    const std::size_t length = (bytes[at + 2] << 8) | bytes[at + 3];
    if (length < 2) {return false;}
    at += 2 + length;
  }
  return false;
}

/// 형식별 머리말에서 크기를 읽는다. 못 읽으면 false.
bool image_dimensions(const ImageBlob & blob, int * width, int * height)
{
  switch (blob.format) {
    case ImageFormat::kPng:  return png_dimensions(blob.bytes, width, height);
    case ImageFormat::kJpeg: return jpeg_dimensions(blob.bytes, width, height);
    case ImageFormat::kWebp: return false;   // 쓸 일이 없어 구현하지 않았다
  }
  return false;
}

long input_tokens_for(int width, int height)
{
  const long across = (width + kInputPatchPixels - 1) / kInputPatchPixels;
  const long down = (height + kInputPatchPixels - 1) / kInputPatchPixels;
  return across * down;
}

long output_tokens_for(const std::string & quality, const OutputSize & size)
{
  long base = kOutputTokensMedium1024;
  if (quality == "low") {
    base = kOutputTokensLow1024;
  } else if (quality == "high") {
    base = kOutputTokensHigh1024;
  } else if (quality == "medium" || quality == "auto" || quality.empty()) {
    base = kOutputTokensMedium1024;
  }

  const double pixels = static_cast<double>(size.width) * static_cast<double>(size.height);
  return static_cast<long>(std::lround(base * (pixels / kReferencePixels)));
}

/// FNV-1a 64비트. 오프셋을 바꿔 두 번 돌리고 이어 붙여 128비트처럼 쓴다.
std::uint64_t fnv1a(const unsigned char * data, std::size_t size, std::uint64_t offset)
{
  std::uint64_t hash = offset;
  for (std::size_t i = 0; i < size; ++i) {
    hash ^= data[i];
    hash *= 1099511628211ULL;
  }
  return hash;
}

void feed(std::uint64_t * a, std::uint64_t * b, const unsigned char * data, std::size_t size)
{
  *a = fnv1a(data, size, *a);
  *b = fnv1a(data, size, *b);
}

void feed(std::uint64_t * a, std::uint64_t * b, const std::string & text)
{
  feed(a, b, reinterpret_cast<const unsigned char *>(text.data()), text.size());
}

}  // namespace

std::optional<OutputSize> parse_size(const std::string & size)
{
  if (size.empty() || size == "auto") {
    return std::nullopt;
  }

  const std::size_t x = size.find('x');
  if (x == std::string::npos || x == 0 || x + 1 >= size.size()) {
    throw PreflightError("--size 는 auto 또는 <W>x<H> 형식이어야 한다: " + size);
  }

  OutputSize parsed;
  try {
    parsed.width = std::stoi(size.substr(0, x));
    parsed.height = std::stoi(size.substr(x + 1));
  } catch (const std::exception &) {
    throw PreflightError("--size 의 숫자를 읽을 수 없다: " + size);
  }
  if (parsed.width <= 0 || parsed.height <= 0) {
    throw PreflightError("--size 의 값이 0 이하다: " + size);
  }
  return parsed;
}

void validate_output_size(const OutputSize & size)
{
  if (size.width % kSizeMultiple != 0 || size.height % kSizeMultiple != 0) {
    throw PreflightError(
      "출력 크기의 양변은 16의 배수여야 한다: " +
      std::to_string(size.width) + "x" + std::to_string(size.height));
  }
  if (std::max(size.width, size.height) > kMaxEdge) {
    throw PreflightError(
      "출력 최대변은 3840 이하여야 한다: " +
      std::to_string(size.width) + "x" + std::to_string(size.height));
  }

  const double longer = std::max(size.width, size.height);
  const double shorter = std::min(size.width, size.height);
  if (longer / shorter > kMaxAspect) {
    throw PreflightError(
      "출력 종횡비는 3:1 이하여야 한다: " +
      std::to_string(size.width) + "x" + std::to_string(size.height));
  }

  const long pixels = static_cast<long>(size.width) * static_cast<long>(size.height);
  if (pixels < kMinPixels || pixels > kMaxPixels) {
    throw PreflightError(
      "출력 총 픽셀은 655,360 ~ 8,294,400 이어야 한다: " + std::to_string(pixels) +
      " (" + std::to_string(size.width) + "x" + std::to_string(size.height) + ")");
  }
}

std::string CostEstimate::summary() const
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(4);
  os << "$" << usd
     << "  (입력 이미지 " << input_image_tokens << "토큰"
     << " · 출력 이미지 " << output_image_tokens << "토큰"
     << " · 텍스트 " << text_input_tokens << "토큰";
  if (output_size_assumed) {
    os << " · 출력 크기는 " << assumed_output.width << "x" << assumed_output.height << " 로 가정";
  }
  os << ")";
  return os.str();
}

CostEstimate estimate_cost(const StylizeRequest & request, const Pricing & pricing)
{
  CostEstimate estimate;

  // ── 입력 이미지 — 참조 그림도 넣는다. 한 장이 늘면 토큰이 는다.
  auto add_input = [&estimate](const ImageBlob & blob) {
      int width = 0;
      int height = 0;
      if (image_dimensions(blob, &width, &height)) {
        estimate.input_image_tokens += input_tokens_for(width, height);
      } else {
        // 크기를 못 읽으면 기본 가정으로 대신한다 — 0 으로 두면 비용을 낮게 속인다.
        estimate.input_image_tokens += input_tokens_for(kAssumedAutoEdge, kAssumedAutoEdge);
      }
    };
  add_input(request.photo);
  if (request.reference) {
    add_input(*request.reference);
  }

  // ── 출력 이미지
  const auto parsed = parse_size(request.params.size);
  if (parsed) {
    estimate.assumed_output = *parsed;
  } else {
    estimate.assumed_output = OutputSize{kAssumedAutoEdge, kAssumedAutoEdge};
    estimate.output_size_assumed = true;
  }
  estimate.output_image_tokens =
    output_tokens_for(request.params.quality, estimate.assumed_output);

  // ── 텍스트
  estimate.text_input_tokens =
    static_cast<long>(std::ceil(request.prompt.size() / kBytesPerTextToken));

  estimate.usd =
    estimate.input_image_tokens / 1e6 * pricing.image_input_per_1m +
    estimate.output_image_tokens / 1e6 * pricing.image_output_per_1m +
    estimate.text_input_tokens / 1e6 * pricing.text_input_per_1m;

  return estimate;
}

std::string request_hash(const StylizeRequest & request)
{
  std::uint64_t a = 14695981039346656037ULL;
  std::uint64_t b = 1099511628211ULL;

  feed(&a, &b, request.photo.bytes.data(), request.photo.bytes.size());
  if (request.reference) {
    feed(&a, &b, request.reference->bytes.data(), request.reference->bytes.size());
  } else {
    feed(&a, &b, std::string("<no-reference>"));
  }
  feed(&a, &b, request.prompt);
  feed(&a, &b, request.params.model);
  feed(&a, &b, request.params.size);
  feed(&a, &b, request.params.quality);
  feed(&a, &b, request.params.background);
  feed(&a, &b, request.params.output_format);

  std::ostringstream os;
  os << std::hex << std::setfill('0') << std::setw(16) << a << std::setw(16) << b;
  return os.str().substr(0, 12);   // 파일명에 넣을 12자리
}

}  // namespace vision_stylize
