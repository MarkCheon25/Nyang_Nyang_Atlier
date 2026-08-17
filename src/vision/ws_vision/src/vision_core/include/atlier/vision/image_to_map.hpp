// A. 이미지 → 지도 (BRD F2.1 ~ F2.4)
//
// 이 헤더는 OpenCV 에 의존한다. 지도를 **받기만** 하는 쪽(moveit2)은 이 헤더가
// 아니라 stroke_map.hpp 를 include 하면 된다.
#ifndef ATLIER_VISION_IMAGE_TO_MAP_HPP
#define ATLIER_VISION_IMAGE_TO_MAP_HPP

#include <opencv2/core.hpp>

#include <string>
#include <vector>

#include "atlier/vision/params.hpp"
#include "atlier/vision/stroke_map.hpp"

namespace atlier::vision
{

/// 픽셀 좌표 폴리라인 — F2.3 까지의 중간 표현.
///
/// 픽셀 단계에서 일부러 Stroke(x_mm) 를 쓰지 않는다. mm 로 바뀌는 곳은
/// F2.4(ScaleToPaper) 한 곳뿐이며, 단위 혼동을 타입으로 막기 위함이다.
using PolylinePx = std::vector<cv::Point2d>;

/// OpenCV findContours 의 출력 형태.
using ContourPx = std::vector<cv::Point>;

// ── 통합 진입점 ───────────────────────────────────────────────────────────

/// 이미지 한 장을 지도로 바꾼다. F2.1 → F2.2 → F2.3 → F2.4 를 순서대로 적용한다.
///
/// 입력은 **F1.2 적합성 검사를 통과한 이미지**라고 전제한다 — 흰 배경 고양이
/// 라인아트/실루엣. 이 함수는 적합성을 판정하지 않는다.
///
/// @throws std::invalid_argument 빈 이미지이거나 파라미터가 성립하지 않을 때
///         (여백이 용지보다 큰 경우 등)
StrokeMap BuildStrokeMap(const cv::Mat & image, const Params & params);

/// 파일에서 읽어 BuildStrokeMap 을 적용한다.
/// @throws std::runtime_error 파일을 읽거나 디코드하지 못했을 때
StrokeMap BuildStrokeMapFromFile(const std::string & path, const Params & params);

// ── 단계별 (BRD F2.1 ~ F2.4) ─────────────────────────────────────────────
// 통합 진입점이 순서대로 부르는 것과 같은 함수들이다. 단계마다 결과를 눈으로
// 확인할 수 있도록 공개해 둔다 (debug_dump.hpp 참조).

/// F2.1 전처리 — 그레이스케일 변환 · 이진화 · 노이즈 제거.
/// @return 0/255 이진 영상 (CV_8UC1), 선이 255(흰색)
cv::Mat Preprocess(const cv::Mat & src, const Params & params);

/// F2.2 도형·엣지·윤곽 추출. use_thinning 이면 centerline 을 먼저 뽑는다.
std::vector<ContourPx> ExtractContours(const cv::Mat & binary, const Params & params);

/// F2.3 윤곽 → 스트로크 변환. 곡선으로 피팅한 뒤 균일 간격으로 재추출하고,
/// 너무 짧은 것은 버린다. 좌표는 아직 **픽셀** 단위다.
std::vector<PolylinePx> ToPolylines(const std::vector<ContourPx> & contours, const Params & params);

/// F2.4 이미지 좌표 → A4 종이 좌표 스케일링 (여백 포함).
///
/// 종횡비를 유지한 채 작화영역에 맞춰 축소하고 중앙 정렬한다(letterbox).
/// @param src_size 폴리라인 좌표가 기준하는 원본 이미지 크기
/// @throws std::invalid_argument src_size 가 비었거나 작화영역이 0 이하일 때
StrokeMap ScaleToPaper(
  const std::vector<PolylinePx> & polylines, cv::Size src_size, const Params & params);

// ── 하네스 ────────────────────────────────────────────────────────────────

/// 파이프라인 검증용 더미 지도 — 이미지를 전혀 보지 않는다.
///
/// 작화영역 테두리 사각형 · 대각선 2개 · 중앙 원으로 이루어진다. 알고리즘이
/// 비어 있어도 좌표 규약 · 여백 · 중앙정렬 · mm 스케일 · SVG 렌더 · CSV 출력을
/// 관통 검증할 수 있다. **인쇄해서 자로 재면 여백이 실제로 맞는지 확인된다.**
StrokeMap MakeDummyMap(const Params & params);

}  // namespace atlier::vision

#endif  // ATLIER_VISION_IMAGE_TO_MAP_HPP
