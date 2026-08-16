// image_ops.hpp — 전처리와 후처리
//
// 픽셀을 만지는 일은 전부 이 파일에 모은다. 나머지 파일들은 바이트만 다룬다.
//
// 전처리(보내기 전) — 가장자리의 균일한 여백을 잘라내고, 색을 지우고, 긴 변을
//   정해진 크기로 줄인다.
//   · **크기 축소가 입력 토큰을 줄이는 유일한 손잡이다.** 흑백 변환은 토큰과 무관하다
//     (입력 토큰은 가로세로로만 정해진다 — llm_readme §1 확정사실)
//   · 흑백은 비용이 아니라 **컬러 출력 억제**용이다. 지시를 어길 재료를 미리 없앤다
//   · 여백 잘라내기가 은근히 크다 — 고양이가 차지하는 비율이 올라가 같은 토큰으로
//     더 많은 정보를 보낸다
//
// 후처리(받은 뒤) — 받은 PNG 를 Otsu 로 이진화해 2색으로 만든다.
//   ⚠️ **이진화본은 참고용이다. 다음 단계(SAM3) 입력은 원본 PNG 다.**
//   근거 — `cat_char.png` 실측(2026-08-16, 세션 260816-턴테이블): 그 그림은 2색이 아니라
//   3레벨이다(선 RGB20 18.0% / 몸 RGB152 33.7% / 배경 RGB244 41.5%). Otsu 임계값이 103 이라
//   몸 회색이 배경 쪽에 붙어 **채움이 통째로 사라진다.** 그런데 SAM3 가 낸 mask_00_cat 은
//   선이 아니라 그 회색 채움 실루엣이다. 이진화본을 SAM3 에 넣으면 실루엣을 잡을 근거를
//   우리 손으로 지우는 셈이 된다. 눈으로 볼 참고 산출물로만 남긴다.

#ifndef VISION_STYLIZE__IMAGE_OPS_HPP_
#define VISION_STYLIZE__IMAGE_OPS_HPP_

#include <optional>
#include <vector>

#include "vision_stylize/types.hpp"

namespace vision_stylize
{

/// 목표 종횡비에 입력을 어떻게 맞출 것인가.
/// `--size auto` 면 목표 종횡비가 없으므로 아무 것도 하지 않는다.
enum class FitMode
{
  kContain,   ///< 종횡비 유지, 남는 곳은 흰 여백 (고양이가 잘리지 않는다)
  kCover,     ///< 종횡비 유지, 넘치는 곳은 잘라냄
  kStretch    ///< 늘려서 맞춤
};

FitMode parse_fit_mode(const std::string & name);
const char * fit_mode_name(FitMode mode);

struct PreprocessOptions
{
  bool trim = true;            ///< 가장자리 균일 여백 잘라내기
  bool grayscale = true;       ///< 색 지우기 (컬러 출력 억제)
  int max_edge = 1024;         ///< 긴 변을 이 픽셀로 줄인다. 입력 토큰의 손잡이
  FitMode fit = FitMode::kContain;
  int trim_tolerance = 8;      ///< 여백 판정 허용 오차 (0~255)

  /// 목표 종횡비. `--size WxH` 가 주어졌을 때만 채워진다.
  std::optional<double> target_aspect;
};

/// 전처리. 결과는 항상 PNG 바이트다 (재인코딩 손실을 한 번으로 묶는다).
/// report 가 널이 아니면 무슨 일이 있었는지 채워 넣는다.
ImageBlob preprocess(
  const ImageBlob & src, const PreprocessOptions & options, PreprocessReport * report);

/// 후처리 — Otsu 이진화. 입력도 출력도 PNG 바이트.
/// ⚠️ 참고용 산출물이다. 헤더 앞머리의 주의를 읽을 것.
std::vector<unsigned char> binarize(const std::vector<unsigned char> & png_bytes);

}  // namespace vision_stylize

#endif  // VISION_STYLIZE__IMAGE_OPS_HPP_
