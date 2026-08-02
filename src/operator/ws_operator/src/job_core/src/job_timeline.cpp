#include "atlier/job/job_timeline.hpp"

#include <algorithm>
#include <iomanip>
#include <set>
#include <sstream>
#include <utility>

namespace atlier::job
{
namespace
{

std::string Fixed(double value, int precision = 1)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(precision);
  out << value;
  return out.str();
}

/// CSV 필드 이스케이프 — 쉼표·따옴표가 든 note 가 열을 밀지 않게.
std::string CsvField(const std::string & text)
{
  if (text.find_first_of(",\"\n") == std::string::npos) {
    return text;
  }
  std::string out = "\"";
  for (const char character : text) {
    if (character == '"') {
      out += "\"\"";
    } else {
      out += character;
    }
  }
  return out + "\"";
}

}  // namespace

std::string RenderText(const Record & record, const std::vector<Transition> & history)
{
  std::ostringstream out;
  out << "seq    t(s)   전이\n";
  out << "───────────────────────────────────────────────────────────────────────────\n";
  for (std::size_t index = 0; index < history.size(); ++index) {
    const Transition & transition = history[index];
    out << std::setw(3) << index << "  "
        << std::setw(6) << Fixed(transition.at_s) << "  "
        << (transition.accepted ? "  " : "✗ ")
        << ToString(transition.from) << " → " << ToString(transition.to)
        << "  |  " << transition.label;
    if (!transition.note.empty()) {
      out << "\n                  ↳ " << transition.note;
    }
    out << "\n";
  }
  out << "───────────────────────────────────────────────────────────────────────────\n";
  out << "종료 상태: " << ToString(record.state) << "\n";
  return out.str();
}

std::string RenderCsv(const std::vector<Transition> & history)
{
  std::ostringstream out;
  out << "seq,at_s,accepted,from,to,label,note\n";
  for (std::size_t index = 0; index < history.size(); ++index) {
    const Transition & transition = history[index];
    out << index << ","
        << Fixed(transition.at_s) << ","
        << (transition.accepted ? "true" : "false") << ","
        << ToString(transition.from) << ","
        << ToString(transition.to) << ","
        << CsvField(transition.label) << ","
        << CsvField(transition.note) << "\n";
  }
  return out.str();
}

std::string RenderMermaid(const Record & record, const std::vector<Transition> & history)
{
  std::ostringstream out;
  out << "# 작업 " << record.id << " — 실제로 밟은 경로\n\n";
  out << "```mermaid\n";
  out << "stateDiagram-v2\n";
  out << "    classDef ok fill:#e6f4ea,stroke:#34a853,stroke-width:2px\n";
  out << "    classDef bad fill:#fce8e6,stroke:#ea4335,stroke-width:2px\n";
  out << "    classDef wait fill:#fef7e0,stroke:#fbbc04,stroke-width:2px\n\n";

  bool entered = false;
  std::set<std::string> seen_states;
  for (std::size_t index = 0; index < history.size(); ++index) {
    const Transition & transition = history[index];
    // 거절된 시도는 상태를 옮기지 않았으므로 경로에 그리지 않는다 (CSV·텍스트에는 남는다).
    if (!transition.accepted) {
      continue;
    }
    if (!entered) {
      out << "    [*] --> " << ToString(transition.from) << "\n";
      entered = true;
    }
    seen_states.insert(ToString(transition.from));
    seen_states.insert(ToString(transition.to));
    out << "    " << ToString(transition.from) << " --> " << ToString(transition.to)
        << ": " << (index + 1) << ". " << transition.label << "\n";
  }
  if (IsTerminal(record.state)) {
    out << "    " << ToString(record.state) << " --> [*]\n";
  }

  out << "\n";
  for (const std::string & state : seen_states) {
    if (state == "Completed") {
      out << "    class " << state << " ok\n";
    } else if (state == "Aborted" || state == "Failed" || state == "Rejected") {
      out << "    class " << state << " bad\n";
    } else if (state == "AwaitingReview" || state == "Paused") {
      out << "    class " << state << " wait\n";
    }
  }
  out << "```\n";
  return out.str();
}

std::string RenderSummary(const Record & record)
{
  std::ostringstream out;
  out << "작업 " << record.id << "\n";
  out << "  상태        : " << ToString(record.state)
      << (IsTerminal(record.state) ? " (종료)" : " (진행 중)") << "\n";
  if (!record.reject_reason.empty()) {
    out << "  거부 사유   : " << record.reject_reason << "\n";
  }
  out << "  계획        : 스트로크 " << record.plan.stroke_count
      << " 개 · 선길이 " << Fixed(record.plan.draw_len_mm) << " mm\n";
  out << "  실행        : " << record.strokes_done << "/" << record.plan.stroke_count
      << " 완료";
  if (!record.failed_strokes.empty()) {
    out << " · 실패 " << record.failed_strokes.size() << " 개 (";
    for (std::size_t index = 0; index < record.failed_strokes.size(); ++index) {
      out << (index ? "," : "") << record.failed_strokes[index];
    }
    out << ")";
  }
  out << "\n";
  if (record.aborted_at_stroke >= 0) {
    out << "  중단 지점   : 스트로크 " << record.aborted_at_stroke << " (F6.3 기록 대상)\n";
  }
  out << "  소요        : ";
  if (IsTerminal(record.state)) {
    out << Fixed(ElapsedS(record)) << " s\n";
  } else {
    out << "— (진행 중)\n";
  }
  out << "  사람 개입   : " << record.interventions.size() << " 회";
  for (const Intervention & intervention : record.interventions) {
    out << "  [" << Fixed(intervention.at_s) << "s @" << intervention.at_stroke << " "
        << ToString(intervention.command) << "]";
  }
  out << "\n";
  if (!record.result_image_path.empty()) {
    out << "  완성작      : " << record.result_image_path << "\n";
  }

  out << "  ── 수락 기준 ──\n";
  // AC1 은 "개입 없이 **완성**"이다 — 개입이 없어도 완주하지 않았으면 불합격이다.
  out << "  AC1 무개입 완주 : " << (MeetsAc1(record) ? "○" : "✗");
  if (record.state != State::Completed) {
    out << "  (완주하지 않음)";
  } else if (!WasUnattended(record)) {
    out << "  (완주했으나 개입 " << record.interventions.size() << " 회)";
  }
  out << "\n";
  out << "  AC2 20% 단축    : " << (MeetsAc2(record.plan) ? "○" : "✗")
      << "  (" << Fixed(OptimizationGainRatio(record.plan) * 100.0) << "%)\n";
  out << "  N2  15분 이내   : " << (MeetsN2(record) ? "○" : "✗")
      << "  (" << Fixed(ElapsedS(record)) << " s)\n";
  return out.str();
}

}  // namespace atlier::job
