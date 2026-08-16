#include "vision_stylize/image_io.hpp"

#include <cstring>
#include <fstream>

namespace vision_stylize
{

namespace
{

bool starts_with(const unsigned char * data, std::size_t size,
  const unsigned char * pattern, std::size_t pattern_size)
{
  return size >= pattern_size && std::memcmp(data, pattern, pattern_size) == 0;
}

}  // namespace

std::optional<ImageFormat> sniff_format(const unsigned char * data, std::size_t size)
{
  static const unsigned char kPng[] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  static const unsigned char kJpeg[] = {0xFF, 0xD8, 0xFF};
  static const unsigned char kRiff[] = {'R', 'I', 'F', 'F'};
  static const unsigned char kWebp[] = {'W', 'E', 'B', 'P'};

  if (starts_with(data, size, kPng, sizeof(kPng))) {
    return ImageFormat::kPng;
  }
  if (starts_with(data, size, kJpeg, sizeof(kJpeg))) {
    return ImageFormat::kJpeg;
  }
  // WebP 은 "RIFF" + 4바이트 길이 + "WEBP"
  if (starts_with(data, size, kRiff, sizeof(kRiff)) && size >= 12 &&
    std::memcmp(data + 8, kWebp, sizeof(kWebp)) == 0)
  {
    return ImageFormat::kWebp;
  }
  return std::nullopt;
}

ImageBlob read_image(const std::filesystem::path & path)
{
  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) {
    throw PreflightError("파일이 없다: " + path.string());
  }

  const auto size = std::filesystem::file_size(path, ec);
  if (ec) {
    throw PreflightError("파일 크기를 읽을 수 없다: " + path.string());
  }
  if (size == 0) {
    throw PreflightError("빈 파일이다: " + path.string());
  }
  if (size >= kMaxInputBytes) {
    throw PreflightError(
      "입력 이미지가 50MB 이상이다 (" + std::to_string(size) + " 바이트): " + path.string());
  }

  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw PreflightError("파일을 열 수 없다: " + path.string());
  }

  ImageBlob blob;
  blob.bytes.resize(static_cast<std::size_t>(size));
  in.read(reinterpret_cast<char *>(blob.bytes.data()), static_cast<std::streamsize>(size));
  if (!in) {
    throw PreflightError("파일을 끝까지 읽지 못했다: " + path.string());
  }

  // 확장자가 아니라 내용으로 판단한다.
  const auto format = sniff_format(blob.bytes.data(), blob.bytes.size());
  if (!format) {
    throw PreflightError(
      "PNG · JPEG · WebP 중 어느 것도 아니다 (확장자가 아니라 파일 내용으로 판정): " +
      path.string());
  }

  blob.format = *format;
  blob.filename = path.filename().string();
  return blob;
}

void write_bytes(const std::filesystem::path & path, const std::vector<unsigned char> & bytes)
{
  if (path.has_parent_path()) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
  }

  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    throw PreflightError("파일을 쓸 수 없다: " + path.string());
  }
  out.write(
    reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  if (!out) {
    throw PreflightError("파일을 끝까지 쓰지 못했다: " + path.string());
  }
}

void write_text(const std::filesystem::path & path, const std::string & text)
{
  if (path.has_parent_path()) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
  }

  std::ofstream out(path, std::ios::trunc);
  if (!out) {
    throw PreflightError("파일을 쓸 수 없다: " + path.string());
  }
  out << text;
  if (!out) {
    throw PreflightError("파일을 끝까지 쓰지 못했다: " + path.string());
  }
}

}  // namespace vision_stylize
