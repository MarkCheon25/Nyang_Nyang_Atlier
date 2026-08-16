// image_io.hpp — 이미지 파일 읽기·쓰기
//
// 파일을 바이트로 읽고, 정말 PNG/JPEG/WebP 인지 확인하고, 50MB 를 넘지 않는지 본다.
// **확장자를 믿지 않고 파일 앞머리 몇 바이트를 직접 본다.** 저장할 때도 받은 바이트를
// 그대로 쓴다 — 여기서는 이미지를 픽셀로 풀지 않는다.

#ifndef VISION_STYLIZE__IMAGE_IO_HPP_
#define VISION_STYLIZE__IMAGE_IO_HPP_

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "vision_stylize/types.hpp"

namespace vision_stylize
{

/// OpenAI 이미지 API 의 입력 상한 (2026-08-16 문서 확인)
constexpr std::size_t kMaxInputBytes = 50u * 1024u * 1024u;

/// 앞머리 바이트로 형식을 알아낸다. 모르는 형식이면 비어 있는 값을 돌려준다.
std::optional<ImageFormat> sniff_format(const unsigned char * data, std::size_t size);

/// 파일을 읽어 ImageBlob 으로. 형식·크기 검사에 걸리면 PreflightError 를 던진다.
ImageBlob read_image(const std::filesystem::path & path);

/// 바이트를 그대로 파일에 쓴다. 상위 디렉터리는 미리 만들어 둔다.
void write_bytes(const std::filesystem::path & path, const std::vector<unsigned char> & bytes);

/// 텍스트 파일 쓰기 (요약 파일용)
void write_text(const std::filesystem::path & path, const std::string & text);

}  // namespace vision_stylize

#endif  // VISION_STYLIZE__IMAGE_IO_HPP_
