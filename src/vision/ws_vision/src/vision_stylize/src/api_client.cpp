#include "vision_stylize/api_client.hpp"

#include <chrono>
#include <thread>
#include <utility>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

namespace vision_stylize
{

namespace
{

std::size_t append_to_string(void * data, std::size_t size, std::size_t nmemb, void * userdata)
{
  auto * out = static_cast<std::string *>(userdata);
  const std::size_t total = size * nmemb;
  out->append(static_cast<char *>(data), total);
  return total;
}

/// libcurl 핸들을 예외 경로에서도 반드시 정리한다.
struct CurlHandle
{
  CURL * handle = nullptr;
  curl_mime * mime = nullptr;
  curl_slist * headers = nullptr;

  ~CurlHandle()
  {
    if (mime) {curl_mime_free(mime);}
    if (headers) {curl_slist_free_all(headers);}
    if (handle) {curl_easy_cleanup(handle);}
  }
};

void add_image_part(
  curl_mime * mime, const char * field_name, const ImageBlob & blob)
{
  curl_mimepart * part = curl_mime_addpart(mime);
  curl_mime_name(part, field_name);
  curl_mime_filename(part, blob.filename.c_str());
  curl_mime_type(part, mime_type(blob.format));
  curl_mime_data(
    part, reinterpret_cast<const char *>(blob.bytes.data()), blob.bytes.size());
}

void add_text_part(curl_mime * mime, const char * field_name, const std::string & value)
{
  if (value.empty()) {
    return;   // 빈 값은 보내지 않는다 — 서버 기본값에 맡긴다
  }
  curl_mimepart * part = curl_mime_addpart(mime);
  curl_mime_name(part, field_name);
  curl_mime_data(part, value.c_str(), CURL_ZERO_TERMINATED);
}

/// 서버가 돌려준 오류 본문에서 사람이 읽을 문장만 뽑는다. 실패하면 원문 그대로.
std::string extract_error_message(const std::string & body)
{
  try {
    const auto json = nlohmann::json::parse(body);
    if (json.contains("error")) {
      const auto & error = json.at("error");
      if (error.contains("message") && error.at("message").is_string()) {
        return error.at("message").get<std::string>();
      }
    }
  } catch (const nlohmann::json::exception &) {
    // 아래로 떨어져 원문을 그대로 쓴다
  }
  return body.substr(0, 500);
}

}  // namespace

std::vector<unsigned char> base64_decode(const std::string & text)
{
  static constexpr char kPad = '=';

  auto value_of = [](unsigned char c) -> int {
      if (c >= 'A' && c <= 'Z') {return c - 'A';}
      if (c >= 'a' && c <= 'z') {return c - 'a' + 26;}
      if (c >= '0' && c <= '9') {return c - '0' + 52;}
      if (c == '+') {return 62;}
      if (c == '/') {return 63;}
      return -1;   // 공백·개행·패딩·그 밖
    };

  std::vector<unsigned char> out;
  out.reserve(text.size() * 3 / 4 + 3);

  int buffer = 0;
  int bits = 0;
  for (const char raw : text) {
    if (raw == kPad) {
      break;
    }
    const int value = value_of(static_cast<unsigned char>(raw));
    if (value < 0) {
      continue;   // 개행·공백은 건너뛴다
    }
    buffer = (buffer << 6) | value;
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<unsigned char>((buffer >> bits) & 0xFF));
    }
  }
  return out;
}

ApiClient::ApiClient(std::string api_key, std::string base_url)
: api_key_(std::move(api_key)), base_url_(std::move(base_url))
{
  if (api_key_.empty()) {
    throw PreflightError(
      "API 키가 비어 있다. 환경변수 OPENAI_API_KEY 를 설정할 것 "
      "(명령행 인자로 받는 경로는 두지 않는다 — ps 와 셸 히스토리에 남는다)");
  }
  while (!base_url_.empty() && base_url_.back() == '/') {
    base_url_.pop_back();
  }
}

std::vector<unsigned char> ApiClient::edit_image(const StylizeRequest & request) const
{
  CurlHandle curl;
  curl.handle = curl_easy_init();
  if (!curl.handle) {
    throw ApiError(0, "libcurl 초기화 실패", true);
  }

  curl.mime = curl_mime_init(curl.handle);

  // image[] 에 대상 사진이 먼저, 참조 그림이 뒤. 순서가 "무엇이 편집 대상인가" 를 뜻한다.
  // 참조가 없으면 단수 필드명 image 를 쓴다 (문서상 단일 이미지의 기본 형태).
  if (request.reference) {
    add_image_part(curl.mime, "image[]", request.photo);
    add_image_part(curl.mime, "image[]", *request.reference);
  } else {
    add_image_part(curl.mime, "image", request.photo);
  }

  add_text_part(curl.mime, "model", request.params.model);
  add_text_part(curl.mime, "prompt", request.prompt);
  add_text_part(curl.mime, "size", request.params.size);
  add_text_part(curl.mime, "quality", request.params.quality);
  add_text_part(curl.mime, "background", request.params.background);
  add_text_part(curl.mime, "output_format", request.params.output_format);

  // 키는 헤더로만 실린다.
  const std::string auth = "Authorization: Bearer " + api_key_;
  curl.headers = curl_slist_append(curl.headers, auth.c_str());
  curl.headers = curl_slist_append(curl.headers, "Expect:");   // 100-continue 대기 제거

  const std::string url = base_url_ + "/v1/images/edits";
  std::string response;

  curl_easy_setopt(curl.handle, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl.handle, CURLOPT_MIMEPOST, curl.mime);
  curl_easy_setopt(curl.handle, CURLOPT_HTTPHEADER, curl.headers);
  curl_easy_setopt(curl.handle, CURLOPT_WRITEFUNCTION, append_to_string);
  curl_easy_setopt(curl.handle, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl.handle, CURLOPT_TIMEOUT, timeout_seconds_);
  curl_easy_setopt(curl.handle, CURLOPT_CONNECTTIMEOUT, 30L);
  curl_easy_setopt(curl.handle, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl.handle, CURLOPT_USERAGENT, "vision_stylize/0.1");

  const CURLcode code = curl_easy_perform(curl.handle);
  if (code != CURLE_OK) {
    // 연결 자체가 안 된 것 — 다시 보내면 될 갈래다.
    throw ApiError(0, curl_easy_strerror(code), true);
  }

  long status = 0;
  curl_easy_getinfo(curl.handle, CURLINFO_RESPONSE_CODE, &status);
  if (status < 200 || status >= 300) {
    throw ApiError(status, extract_error_message(response), false);
  }

  // 응답은 base64 다 (b64_json). 기본 PNG.
  nlohmann::json json;
  try {
    json = nlohmann::json::parse(response);
  } catch (const nlohmann::json::exception & e) {
    throw ApiError(status, std::string("응답이 JSON 이 아니다: ") + e.what(), false);
  }

  if (!json.contains("data") || !json.at("data").is_array() || json.at("data").empty()) {
    throw ApiError(status, "응답에 data 배열이 없다: " + response.substr(0, 500), false);
  }
  const auto & first = json.at("data").at(0);
  if (!first.contains("b64_json") || !first.at("b64_json").is_string()) {
    throw ApiError(status, "응답에 b64_json 이 없다: " + response.substr(0, 500), false);
  }

  auto bytes = base64_decode(first.at("b64_json").get<std::string>());
  if (bytes.empty()) {
    throw ApiError(status, "base64 디코드 결과가 비어 있다", false);
  }
  return bytes;
}

std::vector<unsigned char> ApiClient::edit_image_with_retry(
  const StylizeRequest & request, int max_attempts) const
{
  if (max_attempts < 1) {
    max_attempts = 1;
  }

  for (int attempt = 1; ; ++attempt) {
    try {
      return edit_image(request);
    } catch (const ApiError & e) {
      // 판정은 ApiError 가 이미 했다. 여기서 상태 코드를 다시 보지 않는다.
      if (!e.retryable() || attempt >= max_attempts) {
        throw;
      }
      const auto wait = std::chrono::seconds(1 << (attempt - 1));   // 1s · 2s · 4s …
      std::this_thread::sleep_for(wait);
    }
  }
}

}  // namespace vision_stylize
