#include <memory>
#include <vector>
#include <thread>
#include <cmath>  // M_PI, cos, sin 사용을 위해 추가

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("hello_moveit");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "hcr_arm");

  RCLCPP_INFO(logger, "Planning frame  : %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector    : %s", move_group.getEndEffectorLink().c_str());

  // VisualTools 초기화 (move_group 생성 이후)
  moveit_visual_tools::MoveItVisualTools visual_tools(
      node, move_group.getPlanningFrame(), rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group.getRobotModel());
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  auto const jmg = move_group.getRobotModel()->getJointModelGroup("hcr_arm");

  // 현재 자세를 원 위의 한 점(시작점)으로 사용
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

  // === 원 그리기 파라미터 ===
  const double radius = 0.10;   // 원 반지름 (10cm)
  const int num_points = 72;    // 원을 몇 각형으로 근사할지 (72 = 5도 간격, 매끈함)

  // 원의 중심: 현재 위치에서 x축으로 radius만큼 이동한 지점
  // (현재 위치 자체를 원 둘레 위의 한 점으로 쓰기 위함)
  const double center_x = start_pose.position.x + radius;
  const double center_z = start_pose.position.z;

  // === 원 웨이포인트 생성 ===
  // 각도를 0 ~ 2π(360도)까지 num_points개로 나눠서, 원 둘레 위의 점들을 순서대로 계산
  std::vector<geometry_msgs::msg::Pose> waypoints;

  for (int i = 0; i <= num_points; ++i) {
    double angle = 2.0 * M_PI * static_cast<double>(i) / num_points;

    geometry_msgs::msg::Pose p = start_pose;  // orientation은 시작 자세 그대로 유지
    p.position.x = center_x - radius * std::cos(angle);
    p.position.z = center_z + radius * std::sin(angle);

    waypoints.push_back(p);
  }

  // === 카터시안 경로 계산 ===
  // waypoints를 순서대로 직선으로 이어서, 촘촘한 점들이 결과적으로 원처럼 보이게 함
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;
  const double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

  RCLCPP_INFO(logger, "Cartesian path computed (%.2f%% achieved)", fraction * 100.0);

  // === 실행 전에 계산된 궤적을 RViz에 선으로 미리 시각화 ===
  auto const ee_link = move_group.getRobotModel()->getLinkModel("link6_1");
  visual_tools.publishTrajectoryLine(trajectory, ee_link, jmg);
  visual_tools.trigger();

  // === 경로가 충분히(95% 이상) 계산됐을 때만 실제로 로봇을 움직임 ===
  if (fraction > 0.95) {
    RCLCPP_INFO(logger, "Executing circle path...");
    move_group.execute(trajectory);
    RCLCPP_INFO(logger, "Execution finished.");
  } else {
    RCLCPP_ERROR(logger, "Cartesian path incomplete (%.2f%%). Not executing.", fraction * 100.0);
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}