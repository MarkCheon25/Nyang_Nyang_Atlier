// cost_guard.hpp — 예상 비용과 중복 호출 방지
//
// 호출마다 돈이 나가므로 **보내기 전에** 얼마가 나갈지 계산해 보여주고 동의를 받는다.
// 그리고 사진·프롬프트·설정을 하나의 해시로 접어 결과 파일 이름에 넣는다. 같은
// 요청이면 이미 있는 파일을 쓰고 보내지 않는다. 덤으로 "이 PNG 가 어떤 프롬프트에서
// 나왔나" 를 파일 이름만 보고 알 수 있다.
//
// 출력 크기 파싱·검증도 여기 있다. 비용 계산이 출력 크기를 필요로 하므로 파서가
// 여기 있어야 하고, 파싱과 검증을 두 곳에 나누면 규칙이 흩어진다.
//
// ⚠️ **아래 토큰 계수는 전부 미확인이다** (llm_readme §1 미결). 계열 공통이라고
//    알려진 값을 적어 두었을 뿐, 첫 실호출로 확인해 고친다. 그때까지 이 값들이 내는
//    금액은 **자릿수 감각**이지 청구서가 아니다.

#ifndef VISION_STYLIZE__COST_GUARD_HPP_
#define VISION_STYLIZE__COST_GUARD_HPP_

#include <optional>
#include <string>

#include "vision_stylize/types.hpp"

namespace vision_stylize
{

/// 출력 크기 "<W>x<H>" 를 푼 값
struct OutputSize
{
  int width = 0;
  int height = 0;
};

/// "auto" 면 비어 있는 값을 돌려준다. 형식이 틀리면 PreflightError.
std::optional<OutputSize> parse_size(const std::string & size);

/// 출력 크기 규칙 — 양변이 16의 배수 · 최대변 ≤ 3840 · 종횡비 ≤ 3:1 ·
/// 총 픽셀 655,360 ~ 8,294,400. 어기면 PreflightError.
/// (이 규칙은 **출력에만** 걸린다. 우리가 보내는 사진은 얼마든지 작게 줄일 수 있다.)
void validate_output_size(const OutputSize & size);

/// 1M 토큰당 USD. llm_readme §1 확정사실 표.
struct Pricing
{
  double image_input_per_1m = 8.0;
  double image_output_per_1m = 30.0;
  double text_input_per_1m = 5.0;
};

struct CostEstimate
{
  long input_image_tokens = 0;
  long output_image_tokens = 0;
  long text_input_tokens = 0;
  double usd = 0.0;
  OutputSize assumed_output;   ///< size=auto 일 때 무엇으로 가정했는지
  bool output_size_assumed = false;

  std::string summary() const;
};

/// 요청 한 건의 예상 비용. 참조 그림도 입력 토큰에 넣는다 — 한 장이 늘면 토큰이
/// 늘기 때문이다. 이 값이 "참조를 함께 보내는 것이 값어치를 하는가" 를 숫자로 답한다.
CostEstimate estimate_cost(const StylizeRequest & request, const Pricing & pricing = Pricing{});

/// 사진 바이트 · 참조 바이트 · 프롬프트 원문 · 모델/크기/품질/배경을 하나로 접는다.
/// **전처리 후** 바이트를 넣는다 — 전처리 설정만 바꿔도 다른 요청이 된다.
///
/// 암호학적 해시가 아니다(FNV-1a 두 갈래). 여기서 필요한 것은 "같은 요청인가" 뿐이고,
/// 회차가 수백 번인 용도에서 충돌 확률은 무시할 수 있다. 의존을 늘리지 않는 쪽을 골랐다.
std::string request_hash(const StylizeRequest & request);

}  // namespace vision_stylize

#endif  // VISION_STYLIZE__COST_GUARD_HPP_
