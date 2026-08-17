// job_manager — job_core 의 ROS 2 래퍼. **⑥ 작업 관리가 사는 자리다.**
//
// ⚠️ **경계 인터페이스(토픽/서비스/메시지 타입)는 아직 정하지 않았다.**
// vision↔moveit2 경계와 함께 정해야 해서다 — 진행률·계획 요약이 어떤 형식으로
// 올라오는지가 정해지지 않은 채로 이쪽 서비스 타입만 못 박으면 두 번 고치게 된다.
//
// 다만 **방향은 정해졌다** (SA §5.2 재확정):
//
//     React + roslibjs ──ws:9090── rosbridge ── job_manager ──(액션)──→ MoveIt2
//     Node.js ──HTTP── 업로드 파일 저장 · SQLite 이력 · 정적 서빙
//
//   · 웹이 부르는 것은 이 노드의 **서비스·토픽뿐**이다. rosbridge 는 액션을
//     지원하지 않으므로(SA §5.3) MoveIt2 액션 호출은 이 노드가 서버측에서 한다.
//   · 이력(F6.3)은 Node 가 rosbridge 로 구독해 SQLite 에 쓴다. 이 노드에 DB
//     드라이버를 붙이지 말 것 — "DB 에 직접 붙는 것은 하나"(SA §5.2).
//
// 열릴 경계의 얼개 (미확정 — 이름은 바뀔 수 있다):
//   서비스  ~/start · ~/emergency_stop · ~/pause · ~/resume · ~/review_decision
//   토픽    ~/state (상태·진행률 발행) · ~/record (종료 시 F6.3 레코드 발행)
//
// 지금 이 노드가 하는 일은 하나다 — 파라미터로 받은 정책을 Policy 로 옮겨
// 시나리오 하나를 관통시키고 결과를 로그로 남긴다. 파라미터·빌드·링크 경로가
// 살아 있는지 확인하는 용도다.
#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>
#include <string>

#include "atlier/job/job.hpp"
#include "atlier/job/job_machine.hpp"
#include "atlier/job/job_timeline.hpp"
#include "atlier/job/policy.hpp"

namespace atlier::job
{

class JobManagerNode : public rclcpp::Node
{
public:
  JobManagerNode()
  : Node("job_manager")
  {
    policy_.review_enabled = declare_parameter("review_enabled", policy_.review_enabled);
    policy_.review_at_ratio = declare_parameter("review_at_ratio", policy_.review_at_ratio);
    policy_.review_timeout_s = declare_parameter("review_timeout_s", policy_.review_timeout_s);
    policy_.review_default_continue =
      declare_parameter("review_default_continue", policy_.review_default_continue);
    policy_.capture_enabled = declare_parameter("capture_enabled", policy_.capture_enabled);
    policy_.stroke_retry_count =
      static_cast<int>(declare_parameter("stroke_retry_count", policy_.stroke_retry_count));
    policy_.pen_up_on_pause = declare_parameter("pen_up_on_pause", policy_.pen_up_on_pause);
    policy_.pen_up_on_estop = declare_parameter("pen_up_on_estop", policy_.pen_up_on_estop);

    self_check_ = declare_parameter("self_check", true);
  }

  /// 정책이 성립하는지, 코어 링크가 살아 있는지 한 번 확인한다.
  /// 생성자가 아니라 밖에서 부른다 — 실패가 노드 생성 자체를 무너뜨리지 않게.
  void SelfCheck()
  {
    try {
      policy_.Validate();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "정책이 성립하지 않습니다: %s", error.what());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "정책 적재 완료 — 중간확인 %s(%.0f%% 지점 · 무응답 %.0f s 후 %s) · 완성작 촬영 %s",
      policy_.review_enabled ? "켬" : "끔", policy_.review_at_ratio * 100.0,
      policy_.review_timeout_s, policy_.review_default_continue ? "자동 계속" : "자동 중단",
      policy_.capture_enabled ? "켬" : "끔");

    if (!self_check_) {
      RCLCPP_INFO(get_logger(), "self_check=false — 자기 점검을 건너뜁니다.");
    } else {
      RunSelfCheck();
    }

    RCLCPP_INFO(
      get_logger(),
      "경계 인터페이스(토픽/서비스)는 아직 열지 않았습니다 — vision↔moveit2 경계와 "
      "함께 정합니다. 상태 머신 전체를 눈으로 보려면: "
      "ros2 run job_core job_cli --scenario all --out-dir ~/data/out");
  }

private:
  /// 짧은 작업 하나를 관통시켜 코어가 실제로 도는지 본다 (가상 시계).
  void RunSelfCheck()
  {
    Machine machine("selfcheck", policy_);
    double now_s = 0.0;

    machine.InputAccepted(now_s);
    machine.Apply(Command::Start, now_s);

    PlanSummary plan;
    plan.stroke_count = 8;
    plan.draw_len_mm = 360.0;
    plan.travel_after_s = 16.0;
    plan.travel_before_s = 21.6;
    now_s += 3.0;
    machine.PlanReady(plan, now_s);

    while (machine.state() == State::Drawing &&
      machine.record().strokes_done < plan.stroke_count)
    {
      now_s += 2.0;
      machine.StrokeDone(now_s);
    }

    RCLCPP_INFO(
      get_logger(), "자기 점검: 스트로크 %d/%d 진행 후 상태 = %s (전이 %zu 건)",
      machine.record().strokes_done, plan.stroke_count, ToString(machine.state()),
      machine.history().size());

    if (machine.state() == State::AwaitingReview) {
      RCLCPP_INFO(
        get_logger(),
        "F8.1 확인 지점에서 대기 중입니다 — 실제 운영에서는 여기서 사람의 계속/중단을 "
        "기다리고, %.0f s 무응답이면 자동 %s 합니다.",
        policy_.review_timeout_s, policy_.review_default_continue ? "계속" : "중단");
    }
  }

  Policy policy_;
  bool self_check_{true};
};

}  // namespace atlier::job

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<atlier::job::JobManagerNode>();
  node->SelfCheck();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
