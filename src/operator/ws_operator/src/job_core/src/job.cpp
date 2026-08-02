#include "atlier/job/job.hpp"

#include <stdexcept>

#include "atlier/job/policy.hpp"

namespace atlier::job
{

bool IsTerminal(State state)
{
  switch (state) {
    case State::Rejected:
    case State::Completed:
    case State::Aborted:
    case State::Failed:
      return true;
    default:
      return false;
  }
}

const char * ToString(State state)
{
  switch (state) {
    case State::Received: return "Received";
    case State::Ready: return "Ready";
    case State::Rejected: return "Rejected";
    case State::Planning: return "Planning";
    case State::Drawing: return "Drawing";
    case State::AwaitingReview: return "AwaitingReview";
    case State::Paused: return "Paused";
    case State::Capturing: return "Capturing";
    case State::Completed: return "Completed";
    case State::Aborted: return "Aborted";
    case State::Failed: return "Failed";
  }
  return "?";
}

const char * ToString(Command command)
{
  switch (command) {
    case Command::Start: return "Start";
    case Command::EmergencyStop: return "EmergencyStop";
    case Command::Pause: return "Pause";
    case Command::Resume: return "Resume";
    case Command::ReviewContinue: return "ReviewContinue";
    case Command::ReviewAbort: return "ReviewAbort";
  }
  return "?";
}

const char * ToString(Event event)
{
  switch (event) {
    case Event::InputAccepted: return "InputAccepted";
    case Event::InputRejected: return "InputRejected";
    case Event::PlanReady: return "PlanReady";
    case Event::PlanFailed: return "PlanFailed";
    case Event::StrokeDone: return "StrokeDone";
    case Event::StrokeFailed: return "StrokeFailed";
    case Event::ReviewTimeout: return "ReviewTimeout";
    case Event::DrawingDone: return "DrawingDone";
    case Event::CaptureDone: return "CaptureDone";
  }
  return "?";
}

double ElapsedS(const Record & record)
{
  // 음수가 "아직 없음"이다 — 시각 0 에 시작한 작업을 미시작으로 읽지 않기 위해서다.
  if (record.started_at_s < 0.0 || record.ended_at_s < record.started_at_s) {
    return 0.0;
  }
  return record.ended_at_s - record.started_at_s;
}

bool MeetsN2(const Record & record, double budget_s)
{
  const double elapsed = ElapsedS(record);
  // 아직 끝나지 않은 작업은 판정 대상이 아니다 — 0 을 "합격"으로 읽으면 안 된다.
  return elapsed > 0.0 && elapsed <= budget_s;
}

double OptimizationGainRatio(const PlanSummary & plan)
{
  if (plan.travel_before_s <= 0.0) {
    return 0.0;
  }
  return (plan.travel_before_s - plan.travel_after_s) / plan.travel_before_s;
}

bool MeetsAc2(const PlanSummary & plan, double threshold)
{
  return OptimizationGainRatio(plan) >= threshold;
}

bool WasUnattended(const Record & record)
{
  return record.interventions.empty();
}

bool MeetsAc1(const Record & record)
{
  return record.state == State::Completed && WasUnattended(record);
}

void Policy::Validate() const
{
  if (review_at_ratio <= 0.0 || review_at_ratio >= 1.0) {
    throw std::invalid_argument("review_at_ratio 는 0 과 1 사이여야 합니다 (F8.1 은 작화 도중 1회)");
  }
  if (review_timeout_s < 0.0) {
    throw std::invalid_argument("review_timeout_s 는 음수일 수 없습니다");
  }
  if (stroke_retry_count < 0) {
    throw std::invalid_argument("stroke_retry_count 는 음수일 수 없습니다");
  }
}

}  // namespace atlier::job
