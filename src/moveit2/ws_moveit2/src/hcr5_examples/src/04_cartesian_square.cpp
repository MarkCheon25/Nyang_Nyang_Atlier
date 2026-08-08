// ═══ 예제 3 · 데카르트 직선 경로 — 사각형 그리기 ═════════════════════════════
//
// **이 프로젝트의 핵심 동작이다.** 그리기 = 펜 끝이 정해진 선을 따라가는 것이고,
// 그 선을 만드는 도구가 computeCartesianPath 다.
//
// 현재 펜 끝 자세에서 XY 평면에 한 변 10cm 사각형을 그린다(자세는 유지).
//
// 반드시 볼 것: **달성률(fraction)**
//   1.00  전 구간 IK 가 풀렸다 — 정상
//   < 1.0 중간에 IK 가 끊겼거나 특이점/충돌을 만났다.
//         이 값이 실기에서 "선이 왜 일그러지는가"의 1차 지표가 된다.
//
// 실행:  ros2 launch hcr5_examples example.launch.py example:=cartesian_square

#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "hcr5_cartesian_square",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface mg(node, "hcr_arm");

  const double SIDE = 0.001;       // 한 변 10cm
  const double EEF_STEP = 0.0005;  // 5mm 간격으로 보간 — 촘촘할수록 선이 매끄럽다
  const double VEL_SCALE = 0.1;
  const double ACC_SCALE = 0.1;

  RCLCPP_INFO(logger, "end effector : %s", mg.getEndEffectorLink().c_str());

  auto start = mg.getCurrentPose().pose;
  RCLCPP_INFO(logger, "시작점 [%.3f %.3f %.3f]",
              start.position.x, start.position.y, start.position.z);

  // 사각형 꼭짓점 4개. 자세(orientation)는 시작값을 그대로 유지한다.
  std::vector<geometry_msgs::msg::Pose> waypoints;
  auto p = start;

  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE*2;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE*3;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE*4;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE*5;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE*6;  
  p.position.z += SIDE;  waypoints.push_back(p);  
  p.position.x += SIDE*7;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE*8;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE*9;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);  
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);  
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);  
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z -= SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);  
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);  
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);  
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  
  p.position.z += SIDE;  waypoints.push_back(p);  

  RCLCPP_INFO(logger, "바뀜");

  p.position.x += SIDE;  waypoints.push_back(p);
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  waypoints.push_back(p);
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  waypoints.push_back(p);
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  waypoints.push_back(p);
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  waypoints.push_back(p);
  p.position.z += SIDE;  waypoints.push_back(p);
  p.position.x += SIDE;  waypoints.push_back(p);
  p.position.z += SIDE;  waypoints.push_back(p); 
  
  moveit_msgs::msg::RobotTrajectory traj_msg;
  double fraction = mg.computeCartesianPath(waypoints, EEF_STEP, traj_msg);

  RCLCPP_INFO(logger, "데카르트 경로 달성률: %.1f%%  (구간 %zu 개)",
              fraction * 100.0, traj_msg.joint_trajectory.points.size());

  if (fraction < 0.99)
  {
    RCLCPP_ERROR(logger,
                 "경로를 %.1f%% 밖에 못 만들었다. 중간에 IK 가 끊겼거나 "
                 "작업영역/특이점 문제다. 시작 자세를 바꿔 다시 시도할 것.",
                 fraction * 100.0);
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  // computeCartesianPath 산출물은 시간 정보가 부실하다. TOTG 로 다시 시간을 입힌다.
  // (joint_limits.yaml 의 속도·가속도 한계가 여기서 실제로 쓰인다)
  robot_trajectory::RobotTrajectory rt(mg.getRobotModel(), "hcr_arm");
  rt.setRobotTrajectoryMsg(*mg.getCurrentState(), traj_msg);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(rt, VEL_SCALE, ACC_SCALE))
  {
    RCLCPP_ERROR(logger, "시간 매개변수화 실패 — joint_limits.yaml 의 가속도 한계를 확인할 것");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }
  rt.getRobotTrajectoryMsg(traj_msg);

  RCLCPP_INFO(logger, "총 소요시간 %.2f 초 — 실행한다",
              rclcpp::Duration(traj_msg.joint_trajectory.points.back().time_from_start).seconds());

  auto result = mg.execute(traj_msg);
  RCLCPP_INFO(logger, "실행 결과: %s",
              result == moveit::core::MoveItErrorCode::SUCCESS ? "성공" : "실패");

  auto end = mg.getCurrentPose().pose;
  const double dx = end.position.x - start.position.x;
  const double dy = end.position.y - start.position.y;
  const double dz = end.position.z - start.position.z;
  RCLCPP_INFO(logger, "시작점 복귀 오차: %.4f m (사각형이 닫혔는지의 지표)",
              std::sqrt(dx * dx + dy * dy + dz * dz));

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
