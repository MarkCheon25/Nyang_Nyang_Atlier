// ═══ 예제 2 · 자세 목표 (pose goal, IK) ══════════════════════════════════════
//
// 현재 **펜 끝(pen_tip)** 자세를 읽어 Z 로 -10cm 내린 곳을 목표로 준다.
// 절대 좌표를 박지 않고 현재 자세 기준 상대 이동이라 항상 도달 가능한 범위에 있다.
//
// 이 예제가 증명하는 것: **IK 가 플랜지가 아니라 펜 끝 기준으로 풀린다.**
// end effector 로그가 pen_tip 이고, 실제로 펜 끝이 10cm 내려가면 성공이다.
//
// 실행:  ros2 launch hcr5_examples example.launch.py example:=pose_goal

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "hcr5_pose_goal",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface mg(node, "hcr_arm");
  mg.setMaxVelocityScalingFactor(0.1);
  mg.setMaxAccelerationScalingFactor(0.1);

  RCLCPP_INFO(logger, "planning frame : %s", mg.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "end effector   : %s  ← pen_tip 이어야 한다",
              mg.getEndEffectorLink().c_str());

  // getCurrentPose() 는 end effector link 기준이다 = pen_tip
  auto current = mg.getCurrentPose();
  RCLCPP_INFO(logger, "현재 펜 끝 [%.3f %.3f %.3f]  (frame: %s)",
              current.pose.position.x, current.pose.position.y, current.pose.position.z,
              current.header.frame_id.c_str());

  auto target = current;
  target.pose.position.z -= 0.10;   // 10cm 아래
  RCLCPP_INFO(logger, "목표 펜 끝 [%.3f %.3f %.3f]",
              target.pose.position.x, target.pose.position.y, target.pose.position.z);

  mg.setPoseTarget(target);

  MoveGroupInterface::Plan plan;
  if (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(logger, "계획 성공 — 실행한다");
    auto result = mg.execute(plan);
    RCLCPP_INFO(logger, "실행 결과: %s",
                result == moveit::core::MoveItErrorCode::SUCCESS ? "성공" : "실패");

    auto after = mg.getCurrentPose();
    RCLCPP_INFO(logger, "도달 펜 끝 [%.3f %.3f %.3f]  (목표와 Z 오차 %.4f m)",
                after.pose.position.x, after.pose.position.y, after.pose.position.z,
                after.pose.position.z - target.pose.position.z);
  }
  else
  {
    RCLCPP_ERROR(logger, "계획 실패 — 목표가 작업영역 밖이거나 IK 해가 없다");
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
