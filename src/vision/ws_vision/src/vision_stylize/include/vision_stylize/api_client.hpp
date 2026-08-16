// api_client.hpp — OpenAI 통신
//
// 요청을 만들어 보내고 답을 받는다. 사진과 참조 그림을 multipart 로 싣고, 돌아온
// base64 문자열을 이미지 바이트로 되돌린다. 재시도 판단은 스스로 하지 않고
// types.hpp 의 is_retryable() 을 따른다.
//
//  · **image[] 에 대상 사진이 먼저, 참조 그림이 뒤.** 순서가 "무엇이 편집 대상인가" 를 뜻한다
//  · **API 키는 헤더로만 실린다.** 명령행 인자로 키를 받는 경로를 두지 않는다 —
//    ps 와 셸 히스토리에 그대로 남는다
//  · **주소를 생성자에서 받는다.** 시험할 때 가짜 서버를 물릴 수 있어야 한다

#ifndef VISION_STYLIZE__API_CLIENT_HPP_
#define VISION_STYLIZE__API_CLIENT_HPP_

#include <string>
#include <vector>

#include "vision_stylize/types.hpp"

namespace vision_stylize
{

/// base64 문자열을 바이트로. 이것 하나 때문에 의존을 늘리지 않는다.
std::vector<unsigned char> base64_decode(const std::string & text);

class ApiClient
{
public:
  /// base_url 은 스킴과 호스트까지. 경로(/v1/images/edits)는 이 클래스가 붙인다.
  explicit ApiClient(
    std::string api_key,
    std::string base_url = "https://api.openai.com");

  /// 한 번 보내고 받는다. 실패하면 ApiError 를 던진다 (재시도 없음).
  std::vector<unsigned char> edit_image(const StylizeRequest & request) const;

  /// 다시 보내면 될 실패(429 · 5xx · 전송 오류)만 지수 백오프로 재시도한다.
  /// 그 밖의 4xx 는 즉시 던진다 — 다시 보내봐야 같은 실패에 돈만 나간다.
  std::vector<unsigned char> edit_image_with_retry(
    const StylizeRequest & request, int max_attempts = 3) const;

  void set_timeout_seconds(long seconds) {timeout_seconds_ = seconds;}
  long timeout_seconds() const {return timeout_seconds_;}

private:
  std::string api_key_;
  std::string base_url_;
  long timeout_seconds_ = 300;
};

}  // namespace vision_stylize

#endif  // VISION_STYLIZE__API_CLIENT_HPP_
