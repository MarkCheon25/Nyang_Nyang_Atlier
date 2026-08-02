// 시뮬레이션이 답하는 것 — "펜이 종이에 **실제로** 어떻게 닿았는가".
//
// mock hardware(mock_components/GenericSystem)는 궤적을 그대로 따라간다고 가정하므로
// 계획과 실제의 차이가 언제나 0 이다. MuJoCo 를 쓰는 이유가 여기 있다 — 접촉·필압·
// 동역학 때문에 **계획대로 안 되는 것**을 보는 것. 그것이 BRD R2(필압 제어 실패,
// 가능성 "높음")의 사전 검증이다.
//
// 이 헤더는 ROS·MuJoCo 어디에도 의존하지 않는다. 궤적을 기록하는 쪽(시뮬 노드)과
// 읽는 쪽(확인 하네스)이 이 타입만 공유하면 되게 하기 위함이다.
#ifndef ATLIER_SIM_CONTACT_TRACE_HPP
#define ATLIER_SIM_CONTACT_TRACE_HPP

#include <cstddef>
#include <vector>

namespace atlier::sim
{

/// 종이 평면 위의 점 하나.
///
/// 좌표 규약은 **vision 의 `atlier::vision::Point2` 와 같다** — 원점은 종이 좌상단,
/// +x 오른쪽, +y 아래. 규약이 같아야 계획 지도와 실제 궤적을 그대로 겹쳐 그릴 수 있다.
struct Point2
{
  double x_mm{0.0};
  double y_mm{0.0};
};

/// 시뮬레이션이 한 스텝마다 남기는 기록 하나.
struct ContactSample
{
  double t_s{0.0};          ///< 시뮬 시작 기준 경과 시간
  double x_mm{0.0};         ///< 펜 끝의 종이 평면 좌표
  double y_mm{0.0};
  double force_n{0.0};      ///< 펜이 종이를 누르는 힘 — **R2 의 측정값**
  bool in_contact{false};   ///< 필압이 임계값을 넘어 실제로 선이 그어지는 중인가
};

/// 한 번의 작화에서 실제로 그려진 것 전부.
struct DrawnTrace
{
  std::vector<ContactSample> samples;
  double paper_w_mm{210.0};
  double paper_h_mm{297.0};
};

/// MuJoCo 월드 좌표를 종이 평면 좌표로 옮기는 데 필요한 배치 정보.
///
/// **MJCF 씬의 종이 배치와 반드시 일치해야 한다** — 여기 값과 씬이 어긋나면
/// 그려진 선이 엉뚱한 자리에 찍히고, 그 사실을 알아채기 어렵다.
struct PaperFrame
{
  double origin_x_m{0.0};   ///< 종이 좌상단 모서리의 월드 좌표(m)
  double origin_y_m{0.0};
  double surface_z_m{0.0};  ///< 종이 표면 높이(m) — 접촉 판정의 기준면
  double yaw_rad{0.0};      ///< 종이 +x_mm 방향이 월드에서 향하는 방위각
};

/// 월드 좌표(m) → 종이 평면 좌표(mm).
Point2 WorldToPaper(const PaperFrame & frame, double world_x_m, double world_y_m);

// ── 통계 ────────────────────────────────────────────────────────────────────
// 눈으로 보는 것(SVG)과 별개로, 숫자로 답해야 하는 질문들이 있다.

std::size_t SampleCount(const DrawnTrace & trace);

/// 연속 접촉 구간의 개수 = **실제로 그어진 선의 조각 수**.
///
/// 이 값이 계획 스트로크 수보다 많으면 **선이 끊겼다는 뜻**이다 — R2 가 말하는
/// "선 끊김"이 여기서 숫자로 잡힌다. 필압이 모자라 펜이 떴다가 다시 닿으면
/// 한 스트로크가 여러 조각으로 쪼개진다.
std::size_t SegmentCount(const DrawnTrace & trace);

/// 펜이 닿은 채 이동한 총 거리(mm). 접촉이 끊긴 구간은 세지 않는다.
/// vision 의 `TotalDrawLengthMm`(계획)과 짝지어 보면 얼마나 덜 그렸는지 나온다.
double DrawnLengthMm(const DrawnTrace & trace);

/// 접촉 중인 샘플들의 필압 통계. 접촉 샘플이 하나도 없으면 false.
///
/// 너무 약하면 선이 끊기고 너무 강하면 심이 부러지거나 종이가 찢어진다(R2).
/// 허용 범위는 실물 검증 대상이라 이 라이브러리는 판정하지 않고 값만 준다.
bool ForceStats(const DrawnTrace & trace, double & min_n, double & max_n, double & mean_n);

/// 접촉 시간 / 전체 시간. pen-down 비율 — N2 예산에서 공중 이동이 차지하는 몫을 본다.
double ContactDutyRatio(const DrawnTrace & trace);

/// 그려진 것의 바운딩 박스. 접촉 샘플이 없으면 false 를 반환하고 출력은 건드리지 않는다.
bool BoundingBox(const DrawnTrace & trace, Point2 & min_out, Point2 & max_out);

}  // namespace atlier::sim

#endif  // ATLIER_SIM_CONTACT_TRACE_HPP
