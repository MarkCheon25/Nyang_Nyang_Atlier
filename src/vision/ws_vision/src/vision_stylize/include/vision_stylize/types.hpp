// types.hpp — 데이터 구조와 오류 구분
//
// 프로그램 안에서 오가는 값들의 모양만 적는다. 여기가 얇아야 나머지 파일들이
// 서로를 모른 채 있을 수 있다.
//
// 실패는 두 갈래다:
//   보내기 전 실패 (PreflightError) — 파일 없음 · 형식 아님 · 크기 규칙 위반
//   보낸  뒤 실패 (ApiError)        — 서버가 거절 · 전송 오류
// 뒤쪽은 다시 "다시 보내면 될 것"과 "다시 보내봐야 돈만 나가는 것"으로 갈리는데,
// 그 판정을 하는 함수는 is_retryable() 하나뿐이다. 여러 곳에서 상태 코드를 보고
// 나누기 시작하면 규칙이 흩어지고 그중 하나만 고쳐지는 사고가 난다.

#ifndef VISION_STYLIZE__TYPES_HPP_
#define VISION_STYLIZE__TYPES_HPP_

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace vision_stylize
{

// ── 이미지 ─────────────────────────────────────────────────────────────────

enum class ImageFormat
{
  kPng,
  kJpeg,
  kWebp
};

/// multipart 의 Content-Type 에 실을 문자열
const char * mime_type(ImageFormat format);

/// 파일로 저장할 때 붙일 확장자 (점 포함)
const char * extension(ImageFormat format);

/// 이미지 한 장 — 바이트 + 형식 + 보낼 때 쓸 파일명.
/// 픽셀로 풀지 않은 원본 바이트다. 디코드는 image_ops 가 필요할 때만 한다.
struct ImageBlob
{
  std::vector<unsigned char> bytes;
  ImageFormat format = ImageFormat::kPng;
  std::string filename = "image.png";
};

// ── 요청 파라미터 ───────────────────────────────────────────────────────────

/// API 에 그대로 실려 나가는 설정. 빈 문자열이면 그 항목을 보내지 않는다
/// (= 서버 기본값에 맡긴다).
struct StylizeParams
{
  std::string model = "gpt-image-2";
  std::string size = "auto";       ///< "auto" 또는 "<W>x<H>"
  std::string quality = "low";     ///< low · medium · high · auto
  std::string background;          ///< 비우면 안 보냄
  std::string output_format;       ///< 비우면 안 보냄 (기본 PNG)
};

/// 전처리가 무슨 일을 했는지. 결과가 이상할 때 "무엇을 보냈길래" 를 되짚는 근거라
/// 요약 파일에 그대로 들어간다.
struct PreprocessReport
{
  int src_width = 0;
  int src_height = 0;
  int trim_left = 0;
  int trim_top = 0;
  int trim_right = 0;
  int trim_bottom = 0;
  bool grayscale = false;
  int out_width = 0;
  int out_height = 0;
  std::string fit = "none";        ///< contain · cover · stretch · none

  /// 사람이 읽을 한 덩어리 문장 (요약 파일용)
  std::string summary() const;
};

/// 요청 한 건 — 사진 + 참조 그림 + 프롬프트 + 설정.
/// 참조 그림은 선택이다. 참조가 안 통하면 인자만 빠지고 코드는 그대로다.
struct StylizeRequest
{
  ImageBlob photo;
  std::optional<ImageBlob> reference;
  std::string prompt;
  StylizeParams params;
};

/// 결과 한 건
struct StylizeResult
{
  std::vector<unsigned char> png;
  std::string request_hash;
  double estimated_cost_usd = 0.0;
};

// ── 오류 ────────────────────────────────────────────────────────────────────

/// 보내기 전에 걸린 것. 돈이 나가지 않았다.
class PreflightError : public std::runtime_error
{
public:
  explicit PreflightError(const std::string & what)
  : std::runtime_error(what) {}
};

/// 다시 보내면 될 실패인가. **이 판정은 여기 한 곳에서만 한다.**
///   transport_error : 연결 자체가 안 됨 (타임아웃 · DNS · 끊김)
///   429             : 속도 제한
///   5xx             : 서버 쪽 문제
/// 그 밖의 4xx 는 요청이 틀린 것이므로 다시 보내면 같은 실패를 돈 내고 반복한다.
bool is_retryable(long http_status, bool transport_error);

/// 보낸 뒤에 걸린 것. 돈이 나갔을 수 있다.
class ApiError : public std::runtime_error
{
public:
  ApiError(long http_status, const std::string & body, bool transport_error);

  long http_status() const noexcept {return http_status_;}
  const std::string & body() const noexcept {return body_;}
  bool transport_error() const noexcept {return transport_error_;}
  bool retryable() const noexcept {return retryable_;}

private:
  long http_status_;
  std::string body_;
  bool transport_error_;
  bool retryable_;
};

}  // namespace vision_stylize

#endif  // VISION_STYLIZE__TYPES_HPP_
