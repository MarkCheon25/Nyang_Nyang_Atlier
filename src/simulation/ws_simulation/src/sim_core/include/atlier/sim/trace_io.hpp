// 접촉 궤적의 파일 형식 — 시뮬 노드가 쓰고 확인 하네스가 읽는다.
//
// 두 프로세스를 파일로 갈라 둔 이유: 시뮬은 오래 돌고 확인은 여러 번 반복한다.
// 한 번 돌린 결과를 파일로 남겨 두면 렌더 설정을 바꿔가며 다시 볼 수 있다.
#ifndef ATLIER_SIM_TRACE_IO_HPP
#define ATLIER_SIM_TRACE_IO_HPP

#include <string>

#include "atlier/sim/contact_trace.hpp"
#include "atlier/sim/planned_map.hpp"

namespace atlier::sim
{

/// `trace.csv` 형식: `t_s,x_mm,y_mm,force_n,in_contact`
bool SaveTraceCsv(const DrawnTrace & trace, const std::string & path);

/// 위 형식을 읽는다. 파일이 없거나 형식이 어긋나면 std::runtime_error.
DrawnTrace LoadTraceCsv(const std::string & path);

/// 텍스트 파일 저장 (SVG·요약 등).
bool SaveText(const std::string & path, const std::string & content);

/// 사람이 읽는 요약 — 그려진 조각 수·길이·필압·접촉 비율, 계획이 있으면 대비까지.
/// `planned` 가 nullptr 이면 실제 값만 낸다.
std::string RenderSummary(const DrawnTrace & trace, const PlannedMap * planned);

}  // namespace atlier::sim

#endif  // ATLIER_SIM_TRACE_IO_HPP
