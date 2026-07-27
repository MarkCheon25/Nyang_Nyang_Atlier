// 작업이 어떤 경로로 흘러갔는지를 **눈으로 보는** 산출물.
//
// vision 의 SVG 가 "지도가 종이에 어떻게 앉는가"를 보여줬듯, 운영 계층의 결과는
// 그림이 아니라 **경로**다 — 어떤 상태를 거쳐 어디서 끝났는가. 상태 머신이
// 맞게 돌았는지는 숫자를 세는 것보다 이 경로를 보는 편이 빠르다.
#ifndef ATLIER_JOB_JOB_TIMELINE_HPP
#define ATLIER_JOB_JOB_TIMELINE_HPP

#include <string>
#include <vector>

#include "atlier/job/job.hpp"
#include "atlier/job/job_machine.hpp"

namespace atlier::job
{

/// stdout 용 — 시각·전이·부연을 한 줄씩. 거절된 시도는 `✗` 로 표시된다.
std::string RenderText(const Record & record, const std::vector<Transition> & history);

/// `timeline.csv` — `seq,at_s,accepted,from,to,label,note`
std::string RenderCsv(const std::vector<Transition> & history);

/// `timeline.md` — mermaid `stateDiagram-v2`. **실제로 밟은 전이만** 그린다.
/// GitHub·브라우저에서 그대로 렌더되므로 시나리오별 경로를 눈으로 비교할 수 있다.
std::string RenderMermaid(const Record & record, const std::vector<Transition> & history);

/// 작업 요약 — 스트로크 진행·실패·개입·수락기준 판정을 표 한 장으로.
std::string RenderSummary(const Record & record);

}  // namespace atlier::job

#endif  // ATLIER_JOB_JOB_TIMELINE_HPP
