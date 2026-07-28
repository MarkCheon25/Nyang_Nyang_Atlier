// 작업 상태 머신 — ⑥ 작업 관리의 알맹이.
//
// **이 클래스가 "어디까지 그렸는가"를 소유한다.** Process Flow §3.4 가 미결로
// 남겨둔 "상태 소유자(③ 로봇제어 / ⑥ 작업관리 경계)" 질문의 답이 여기다.
// F6.5 일시정지가 성립하려면 누군가는 이 상태를 들고 있어야 한다.
//
// ROS 에 의존하지 않는다 — rclcpp 를 여기 붙이지 말 것. ROS 없이 시나리오를
// 관통시켜 전이를 검증할 수 있는 것이 2층 구조의 이유다 (ROS 래핑은 job_manager).
#ifndef ATLIER_JOB_JOB_MACHINE_HPP
#define ATLIER_JOB_JOB_MACHINE_HPP

#include <string>
#include <vector>

#include "atlier/job/job.hpp"
#include "atlier/job/policy.hpp"

namespace atlier::job
{

/// 전이 한 번의 기록. 받아들여지지 않은 시도도 남긴다 — "그 상태에서 그 명령이
/// 왜 거절됐는지"가 운영 화면과 사후 분석 양쪽에서 필요하다.
struct Transition
{
  bool accepted{false};
  double at_s{0.0};
  State from{State::Received};
  State to{State::Received};
  std::string label;   ///< 사람이 읽는 이름 — "F6.4 긴급정지"
  std::string note;    ///< 거절 사유·부연
};

/// 작업 1건의 생애주기를 구동한다.
///
/// 시간은 밖에서 넣는다(`now_s`) — 시계를 들고 있으면 ROS 시간·시뮬 시간·테스트
/// 시간을 갈아끼울 수 없고, CLI 하네스가 15분짜리 작업을 순식간에 재생하지도
/// 못한다.
class Machine
{
public:
  explicit Machine(std::string job_id, Policy policy = {});

  // ── 사람 명령 (F6.4 · F6.5 · F8.1) ────────────────────────────────────────
  /// 받아들여지면 개입 로그에 남는다 (Start 제외 — AC1 의 출발선이므로).
  Transition Apply(Command command, double now_s);

  // ── 파이프라인이 알리는 사실 ──────────────────────────────────────────────
  Transition InputAccepted(double now_s);
  Transition InputRejected(const std::string & reason, double now_s);
  Transition PlanReady(const PlanSummary & plan, double now_s);
  Transition PlanFailed(const std::string & reason, double now_s);

  /// 스트로크 1개 완료. 여기서 F8.1 확인 지점 도달을 판정한다 —
  /// 진행률을 아는 것이 이 클래스뿐이라 그 판정도 여기 있어야 한다.
  Transition StrokeDone(double now_s);

  /// F4.4 — 실패를 기록하지만 상태는 Drawing 그대로다. 흐름을 죽이는 것은
  /// 긴급정지뿐이고, 스트로크 실패로 사람을 부르면 AC1 위반이다 (PF §3.2).
  Transition StrokeFailed(int stroke_index, double now_s);

  Transition DrawingDone(double now_s);
  Transition CaptureDone(const std::string & image_path, double now_s);

  /// 시간 경과만 알린다. AwaitingReview 에서 review_timeout_s 를 넘기면 자동
  /// 진행시킨다 — **이 전이는 개입으로 기록되지 않는다** (AC1 해석 ①).
  Transition Tick(double now_s);

  // ── 조회 ──────────────────────────────────────────────────────────────────
  const Record & record() const {return record_;}
  State state() const {return record_.state;}
  const Policy & policy() const {return policy_;}
  const std::vector<Transition> & history() const {return history_;}

  /// 다음 스트로크에서 F8.1 확인 지점에 닿는지. 화면 미리보기용.
  bool ReviewPending() const;

private:
  /// 받아들이지 않은 시도. `log=true` 면 이력에도 남는다 — 사후에 "왜 안 먹었나"를
  /// 묻게 되는 것은 사람이 누른 명령이지, 주기적으로 도는 Tick 이 아니다.
  Transition Reject(const std::string & note, double now_s, bool log = true);

  /// 상태 전이를 확정하고 이력에 남긴다. `to == from` 이어도 기록한다
  /// (F4.4 스트로크 실패처럼 상태는 그대로지만 남아야 하는 사건이 있다).
  Transition Commit(State to, std::string label, double now_s, std::string note = {});

  /// 상태도 안 바뀌고 이력에도 남길 것이 없는 성공 — 스트로크 1개 진행 같은 것.
  /// 300~500 개를 전부 남기면 이력이 진행률 로그로 파묻힌다.
  Transition Silent(double now_s) const;

  void RecordIntervention(Command command, double now_s);

  Record record_;
  Policy policy_;
  std::vector<Transition> history_;

  /// F6.5 가 "상태 보존"이라는 말의 실체 — 어디로 돌아갈지.
  State resume_to_{State::Drawing};

  /// 중간 확인은 **작업당 1회**다. 한 번 발동하면 다시 걸리지 않는다.
  bool review_done_{false};

  /// AwaitingReview 진입 시각 — 타임아웃 판정 기준.
  double review_entered_at_s_{0.0};
};

}  // namespace atlier::job

#endif  // ATLIER_JOB_JOB_MACHINE_HPP
