#include "atlier/job/job_machine.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

namespace atlier::job
{

Machine::Machine(std::string job_id, Policy policy)
: policy_(policy)
{
  policy_.Validate();
  record_.id = std::move(job_id);
  record_.state = State::Received;
}

// ── 내부 헬퍼 ───────────────────────────────────────────────────────────────

Transition Machine::Reject(const std::string & note, double now_s, bool log)
{
  Transition transition;
  transition.accepted = false;
  transition.at_s = now_s;
  transition.from = record_.state;
  transition.to = record_.state;
  transition.label = "거절";
  transition.note = note;
  if (log) {
    history_.push_back(transition);
  }
  return transition;
}

Transition Machine::Commit(State to, std::string label, double now_s, std::string note)
{
  Transition transition;
  transition.accepted = true;
  transition.at_s = now_s;
  transition.from = record_.state;
  transition.to = to;
  transition.label = std::move(label);
  transition.note = std::move(note);

  record_.state = to;
  // 종료 상태에 처음 닿은 시각이 곧 작업의 끝이다 (N2 측정의 뒤쪽 경계).
  if (IsTerminal(to) && record_.ended_at_s < 0.0) {
    record_.ended_at_s = now_s;
  }

  history_.push_back(transition);
  return transition;
}

Transition Machine::Silent(double now_s) const
{
  Transition transition;
  transition.accepted = true;
  transition.at_s = now_s;
  transition.from = record_.state;
  transition.to = record_.state;
  return transition;
}

void Machine::RecordIntervention(Command command, double now_s)
{
  // Start 는 AC1 의 출발선 자체라 개입이 아니다 — 여기 넣으면 모든 작업이
  // "사람이 손댄 작업"이 되어 AC1 판정이 무의미해진다.
  if (command == Command::Start) {
    return;
  }
  Intervention intervention;
  intervention.at_s = record_.started_at_s >= 0.0 ? now_s - record_.started_at_s : 0.0;
  intervention.at_stroke = record_.state == State::Received || record_.state == State::Ready
    ? -1
    : record_.strokes_done;
  intervention.command = command;
  record_.interventions.push_back(intervention);
}

bool Machine::ReviewPending() const
{
  if (review_done_ || !policy_.review_enabled || record_.plan.stroke_count <= 0) {
    return false;
  }
  const int trigger =
    static_cast<int>(std::ceil(record_.plan.stroke_count * policy_.review_at_ratio));
  return record_.strokes_done + 1 >= trigger;
}

// ── 사람 명령 ───────────────────────────────────────────────────────────────

Transition Machine::Apply(Command command, double now_s)
{
  if (IsTerminal(record_.state)) {
    return Reject("이미 종료된 작업입니다", now_s);
  }

  switch (command) {
    case Command::Start: {
      if (record_.state != State::Ready) {
        return Reject("적합성 검사를 통과한 작업만 시작할 수 있습니다 (F1.2)", now_s);
      }
      record_.started_at_s = now_s;
      RecordIntervention(command, now_s);
      return Commit(State::Planning, "작화 시작 — AC1 출발선", now_s);
    }

    case Command::EmergencyStop: {
      // F6.4 — 어떤 상태에서든 받아들인다. 계획 수립 중(로봇 미동작)이어도
      // 결과는 "이 작업을 로봇에 내리지 않음"이다 (PF §3.3).
      record_.aborted_at_stroke = record_.strokes_done;
      RecordIntervention(command, now_s);
      const std::string note = policy_.pen_up_on_estop
        ? "pen-up 후 정지"
        : "pen-up 없이 즉시 정지 — 작업을 폐기하므로 blob 무관";
      return Commit(
        State::Aborted, "F6.4 긴급정지 — 상태 폐기", now_s,
        note + " · 중단 스트로크 " + std::to_string(record_.strokes_done));
    }

    case Command::Pause: {
      if (record_.state == State::Received || record_.state == State::Ready) {
        return Reject("아직 시작하지 않은 작업입니다", now_s);
      }
      if (record_.state == State::Paused) {
        return Reject("이미 일시정지 상태입니다", now_s);
      }
      resume_to_ = record_.state;
      RecordIntervention(command, now_s);
      return Commit(
        State::Paused, "F6.5 일시정지 — 상태 보존", now_s,
        std::string(policy_.pen_up_on_pause ? "pen-up 후 정지" : "pen-up 없이 정지") +
        " · 복귀 대상 " + ToString(resume_to_));
    }

    case Command::Resume: {
      if (record_.state != State::Paused) {
        return Reject("일시정지 상태가 아닙니다", now_s);
      }
      RecordIntervention(command, now_s);
      // 재개 단위는 스트로크다 — 스트로크 N 중간에 멈췄으면 N 을 처음부터 다시
      // 긋는다(중간 이음매 방지). 그래서 보존 상태가 인덱스 하나로 족하다 (PF §3.4).
      return Commit(
        resume_to_, "F6.5 재개 — 중단 스트로크부터", now_s,
        "스트로크 " + std::to_string(record_.strokes_done) + " 부터 이어서");
    }

    case Command::ReviewContinue: {
      if (record_.state != State::AwaitingReview) {
        return Reject("중간 확인 대기 상태가 아닙니다 (F8.1)", now_s);
      }
      RecordIntervention(command, now_s);
      return Commit(State::Drawing, "F8.1 사람 판단 — 계속", now_s);
    }

    case Command::ReviewAbort: {
      if (record_.state != State::AwaitingReview) {
        return Reject("중간 확인 대기 상태가 아닙니다 (F8.1)", now_s);
      }
      record_.aborted_at_stroke = record_.strokes_done;
      RecordIntervention(command, now_s);
      return Commit(State::Aborted, "F8.1 사람 판단 — 중단", now_s);
    }
  }
  return Reject("알 수 없는 명령", now_s);
}

// ── 파이프라인이 알리는 사실 ────────────────────────────────────────────────

Transition Machine::InputAccepted(double now_s)
{
  if (record_.state != State::Received) {
    return Reject("업로드 직후 상태가 아닙니다", now_s);
  }
  return Commit(State::Ready, "F1.2 적합성 통과", now_s);
}

Transition Machine::InputRejected(const std::string & reason, double now_s)
{
  if (record_.state != State::Received) {
    return Reject("업로드 직후 상태가 아닙니다", now_s);
  }
  record_.reject_reason = reason;
  // 로봇이 한 번도 움직이지 않은 종료 — 잘못된 입력을 가장 값싸게 막는 자리 (PF §3.1).
  return Commit(State::Rejected, "F1.2 거부", now_s, reason);
}

Transition Machine::PlanReady(const PlanSummary & plan, double now_s)
{
  if (record_.state != State::Planning) {
    return Reject("계획 수립 중이 아닙니다", now_s);
  }
  record_.plan = plan;
  const double gain = OptimizationGainRatio(plan) * 100.0;
  char note[160];
  std::snprintf(
    note, sizeof(note), "스트로크 %d 개 · 선길이 %.1f mm · F3.3 이동시간 %.1f→%.1f s (%.1f%% 단축)",
    plan.stroke_count, plan.draw_len_mm, plan.travel_before_s, plan.travel_after_s, gain);
  return Commit(State::Drawing, "F3 계획 완료", now_s, note);
}

Transition Machine::PlanFailed(const std::string & reason, double now_s)
{
  if (record_.state != State::Planning) {
    return Reject("계획 수립 중이 아닙니다", now_s);
  }
  return Commit(State::Failed, "계획 실패", now_s, reason);
}

Transition Machine::StrokeDone(double now_s)
{
  if (record_.state != State::Drawing) {
    return Reject("작화 중이 아닙니다", now_s);
  }
  ++record_.strokes_done;

  // F8.1 확인 지점 판정 — 진행률을 아는 것이 이 클래스뿐이라 여기서 한다.
  if (!review_done_ && policy_.review_enabled && record_.plan.stroke_count > 0) {
    const int trigger =
      static_cast<int>(std::ceil(record_.plan.stroke_count * policy_.review_at_ratio));
    if (record_.strokes_done >= trigger) {
      review_done_ = true;               // 작업당 1회 — 다시 걸리지 않는다
      review_entered_at_s_ = now_s;
      return Commit(
        State::AwaitingReview, "F8.1 확인 지점 도달", now_s,
        "스트로크 " + std::to_string(record_.strokes_done) + "/" +
        std::to_string(record_.plan.stroke_count) + " · pen-up → 촬영 자세 이동(F8.3)");
    }
  }
  return Silent(now_s);
}

Transition Machine::StrokeFailed(int stroke_index, double now_s)
{
  if (record_.state != State::Drawing) {
    return Reject("작화 중이 아닙니다", now_s);
  }
  record_.failed_strokes.push_back(stroke_index);
  ++record_.strokes_done;
  // 상태는 Drawing 그대로다. 실패로 사람을 부르면 AC1 위반이다 (PF §3.2).
  return Commit(
    State::Drawing, "F4.4 스트로크 실패 — 기록 후 계속", now_s,
    "스트로크 " + std::to_string(stroke_index) + " · 재시도 없음(Phase 1)");
}

Transition Machine::DrawingDone(double now_s)
{
  if (record_.state != State::Drawing) {
    return Reject("작화 중이 아닙니다", now_s);
  }
  std::string note;
  if (record_.plan.stroke_count > 0 && record_.strokes_done < record_.plan.stroke_count) {
    note = "⚠️ 계획 " + std::to_string(record_.plan.stroke_count) + " 개 중 " +
      std::to_string(record_.strokes_done) + " 개만 실행됨";
  }
  if (!policy_.capture_enabled) {
    // MuJoCo 에는 카메라가 없어 F8 을 끄고 완주한다 — AC1 검증에는 영향 없다 (PF §5.1).
    return Commit(
      State::Completed, "작화 완료 (F8.2 촬영 생략)", now_s,
      note.empty() ? "capture_enabled=false" : note);
  }
  return Commit(State::Capturing, "작화 완료 → F8.2 촬영", now_s, note);
}

Transition Machine::CaptureDone(const std::string & image_path, double now_s)
{
  if (record_.state != State::Capturing) {
    return Reject("촬영 단계가 아닙니다", now_s);
  }
  record_.result_image_path = image_path;
  return Commit(State::Completed, "F8.2 완성작 촬영 완료", now_s, image_path);
}

Transition Machine::Tick(double now_s)
{
  if (record_.state != State::AwaitingReview) {
    return Reject("대기 중인 판단이 없습니다", now_s, /*log=*/false);
  }
  if (now_s - review_entered_at_s_ < policy_.review_timeout_s) {
    return Reject("아직 대기 시간 내입니다", now_s, /*log=*/false);
  }

  // ⚠️ 여기서 RecordIntervention 을 부르지 않는다 — 무응답은 사람이 손댄 것이
  // 아니므로 AC1 판정에 걸리지 않는다. 그것이 해석 ①의 전부다.
  if (policy_.review_default_continue) {
    return Commit(
      State::Drawing, "F8.1 무응답 → 자동 계속", now_s,
      "대기 " + std::to_string(static_cast<int>(policy_.review_timeout_s)) + " s 초과 · 개입 아님");
  }
  record_.aborted_at_stroke = record_.strokes_done;
  return Commit(State::Aborted, "F8.1 무응답 → 자동 중단", now_s, "review_default_continue=false");
}

}  // namespace atlier::job
