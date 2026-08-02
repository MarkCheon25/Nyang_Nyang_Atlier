// A(이미지 → 지도) 파이프라인의 조정값.
//
// 전부 기본값이 있어 Params{} 로 바로 쓸 수 있다. ROS 파라미터·YAML 은
// vision_node 가 이 구조체를 채우는 방식으로만 관여한다 — 코어는 ROS 를 모른다.
#ifndef ATLIER_VISION_PARAMS_HPP
#define ATLIER_VISION_PARAMS_HPP

namespace atlier::vision
{

struct Params
{
  // ── F2.4 종이 배치 ────────────────────────────────────────────────────
  double paper_w_mm{210.0};   ///< A4 세로 기준. 가로로 쓰려면 값을 맞바꾼다
  double paper_h_mm{297.0};
  double margin_mm{15.0};     ///< 사방 여백 → 작화영역 180 × 267

  // ── F2.3 스트로크 변환 ────────────────────────────────────────────────
  /// 재샘플링 점 간격. **이 값이 그대로 moveit2 데카르트 waypoint 밀도가 된다** —
  /// 작게 잡을수록 선은 매끄럽지만 점 수가 늘어 N2(15분) 예산을 압박한다.
  /// 기본 1.0mm 는 총 선길이 2~4m 기준 2000~4000점. 실측 후 조정할 것.
  double resample_step_mm{1.0};

  /// 이보다 짧은 스트로크는 노이즈로 보고 버린다.
  double min_stroke_len_mm{2.0};

  // ── F2.1 전처리 ───────────────────────────────────────────────────────
  int  blur_ksize{3};      ///< 홀수여야 한다. 0 이면 블러 생략
  bool invert{false};      ///< 흰 배경·검은 선이 기본. 반전된 입력이면 true

  // ── F2.2 윤곽 추출 ────────────────────────────────────────────────────
  /// ximgproc::thinning 으로 centerline 을 뽑는다. Canny 는 선의 **이중 윤곽**을
  /// 내므로 라인아트에는 부적합하다 (System Architecture.md §5.3).
  bool use_thinning{true};
};

/// 작화 가능 영역(여백을 뺀 안쪽)의 너비·높이(mm). 음수가 되면 여백이 과도한 것이다.
inline double DrawableWidthMm(const Params & p) { return p.paper_w_mm - 2.0 * p.margin_mm; }
inline double DrawableHeightMm(const Params & p) { return p.paper_h_mm - 2.0 * p.margin_mm; }

}  // namespace atlier::vision

#endif  // ATLIER_VISION_PARAMS_HPP
