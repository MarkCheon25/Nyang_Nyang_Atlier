// ═══ 예제 1 · 관절 목표 (joint-space goal) ═══════════════════════════════════
//
// 가장 단순한 형태. 현재 관절값을 읽어 joint_1 만 +0.3 rad(≈17°) 돌린다.
// IK 를 쓰지 않으므로 "계획 → 실행" 사슬만 순수하게 확인한다.
//
// 실행:  ros2 launch hcr5_examples example.launch.py example:=joint_goal
//        (demo.launch.py 가 떠 있어야 한다)

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "hcr5_joint_goal",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  // MoveGroupInterface 가 현재 상태(TF·joint_states)를 받으려면 노드가 돌아야 한다.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface mg(node, "hcr_arm");

  // 실기 연결 전이라도 항상 저속으로. joint_limits.yaml 의 한계에 곱해진다.
  mg.setMaxVelocityScalingFactor(0.1);
  mg.setMaxAccelerationScalingFactor(0.1);

  RCLCPP_INFO(logger, "planning frame : %s", mg.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "end effector   : %s  ← pen_tip 이어야 한다",
              mg.getEndEffectorLink().c_str());

  auto joints = mg.getCurrentJointValues();
  if (joints.size() != 6)
  {
    RCLCPP_ERROR(logger, "관절값을 %zu 개 받았다 (6개 기대). demo.launch.py 가 떠 있는가?",
                 joints.size());
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "현재 관절값 [%.3f %.3f %.3f %.3f %.3f %.3f]",
              joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

  joints[0] += 0.3;   // joint_1 만 +0.3 rad
  RCLCPP_INFO(logger, "목표 관절값 joint_1 = %.3f rad (%.1f°)", joints[0], joints[0] * 180.0 / M_PI);
  mg.setJointValueTarget(joints);

  MoveGroupInterface::Plan plan;
  if (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(logger, "계획 성공 — 실행한다");
    auto result = mg.execute(plan);
    RCLCPP_INFO(logger, "실행 결과: %s",
                result == moveit::core::MoveItErrorCode::SUCCESS ? "성공" : "실패");
  }
  else
  {
    RCLCPP_ERROR(logger, "계획 실패");
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
