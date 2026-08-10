// MoveIt2 first tutorial: plan and execute a single target pose on the Panda arm.
// Requires the demo (moveit_resources_panda_moveit_config demo.launch.py) running.

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("hello_moveit");

  // MoveGroup 스핀 스레드 (interactive marker / TF 갱신용)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  // auto move_group = MoveGroupInterface(node, "panda_arm");
  auto move_group = MoveGroupInterface(node, "hcr_arm");

  RCLCPP_INFO(logger, "Planning frame  : %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector    : %s", move_group.getEndEffectorLink().c_str());

  // 목표 pose: 로봇 앞쪽으로 살짝 이동
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1;
  target_pose.position.x = 0.28;
  target_pose.position.y = -0.2;
  target_pose.position.z = 0.5;
  move_group.setPoseTarget(target_pose);

  // 계획
  MoveGroupInterface::Plan plan;
  const bool success = static_cast<bool>(move_group.plan(plan));

  if (success) {
    RCLCPP_INFO(logger, "Plan succeeded. Executing...");
    move_group.execute(plan);
    RCLCPP_INFO(logger, "Execution finished.");
  } else {
    RCLCPP_ERROR(logger, "Plan failed.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
