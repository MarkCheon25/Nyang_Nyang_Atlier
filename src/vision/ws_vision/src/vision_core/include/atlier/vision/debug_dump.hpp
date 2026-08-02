// 단계별 결과를 눈으로 확인하기 위한 덤프.
//
// 알고리즘이 맞는지는 숫자가 아니라 그림으로 판단한다. 각 단계를 PNG 로 떨어뜨려
// 호스트에서 바로 열어보는 것이 이 파일의 목적이다. 컨테이너 GUI 없이 동작한다.
#ifndef ATLIER_VISION_DEBUG_DUMP_HPP
#define ATLIER_VISION_DEBUG_DUMP_HPP

#include <opencv2/core.hpp>

#include <string>
#include <vector>

#include "atlier/vision/image_to_map.hpp"
#include "atlier/vision/params.hpp"
#include "atlier/vision/stroke_map.hpp"

namespace atlier::vision
{

struct DumpOptions
{
  /// 출력 디렉터리. 없으면 만들어 쓴다.
  std::string out_dir{"out"};

  /// 종이 렌더(04)의 해상도. 300DPI 면 A4 가 2480 × 3508 픽셀이 된다.
  int render_dpi{300};
};

/// 출력 디렉터리를 만든다(이미 있으면 그대로 둔다).
/// @return 준비 성공 여부
bool EnsureOutDir(const DumpOptions & opts);

/// 01 — F2.1 전처리 결과 이진 영상.
bool DumpPreprocess(const cv::Mat & binary, const DumpOptions & opts);

/// 02 — F2.2 추출된 윤곽을 원본 위에 겹쳐 그린다.
bool DumpContours(
  const cv::Mat & src, const std::vector<ContourPx> & contours, const DumpOptions & opts);

/// 03 — F2.3 재샘플링된 폴리라인. **점을 하나씩 찍어 간격을 눈으로 확인**한다.
bool DumpPolylines(
  const cv::Mat & src, const std::vector<PolylinePx> & polylines, const DumpOptions & opts);

/// 04 — F2.4 종이 배치 결과를 실제 비율로 렌더링한다. 여백선·작화영역 테두리 포함.
bool DumpPaper(const StrokeMap & map, const Params & params, const DumpOptions & opts);

/// 좌표 수치를 그대로 본다. 형식: stroke_idx,point_idx,x_mm,y_mm
bool SaveCsv(const StrokeMap & map, const std::string & path);

}  // namespace atlier::vision

#endif  // ATLIER_VISION_DEBUG_DUMP_HPP
