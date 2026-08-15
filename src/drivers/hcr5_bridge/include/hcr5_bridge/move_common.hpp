// movej · movel 이 공유하는 안전 게이트와 액션 전송.
//
// 왜 이 파일이 있나 — 운전절차.md §2 "속도" 표가 이 경로에 속도 안전망이
// **하나도 없다**고 적는다(MoveIt 스케일·joint_limits.yaml·JTC constraints·
// mqtt_cmd.py 3중가드 전부 액션 직송에는 안 걸린다). 유일하게 걸리는 것이
// 관절한계뿐이라 "계산이 곧 안전장치"였다 — 그 계산을 사람 손에서 여기로 옮긴다.
//
// 게이트 3개, 전부 **발행 전**에 건다:
//   ① 단위   — 넣은 숫자를 rad·실기 도 **양쪽으로 찍어 보여준다** (함정 ⑧)
//   ② 한계   — withinLimits() 실기 도 검사 + URDF 한계 검사
//   ③ 속도   — |Δ|/sec 가 상한을 넘으면 거부. --force 로만 통과
#ifndef HCR5_BRIDGE__MOVE_COMMON_HPP_
#define HCR5_BRIDGE__MOVE_COMMON_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "hcr5_bridge/joint_convention.hpp"

namespace hcr5_bridge
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using JointArray = std::array<double, kNumJoints>;

/// 기본 속도 상한(실기 도/s). 검C2 창 실측이 1.25°/s, 운전절차 ④~⑥ 이 2°/s 다.
/// 10°/s 는 실기 자율주행 실측(9.7°/s)과 같은 자리 — 그 위는 명령이 실기를 앞지른다.
inline constexpr double kDefaultMaxSpeedDegPerSec = 10.0;
/// --sec 를 안 주면 이 속도로 소요시간을 **자동 계산**한다. 운전절차 ④~⑥ 과 같은 값.
inline constexpr double kDefaultSpeedDegPerSec = 2.0;
/// 너무 짧은 궤적은 JTC 가 못 따라온다.
inline constexpr double kMinDurationSec = 0.5;

struct MoveOptions
{
  double sec = 0.0;                                  ///< 0 = 자동 계산
  double speed_deg_s = kDefaultSpeedDegPerSec;       ///< --sec 미지정 시 쓰는 속도
  double max_speed_deg_s = kDefaultMaxSpeedDegPerSec;
  bool deg_input = false;                            ///< --deg : 입력이 실기 도
  bool force = false;                                ///< --force : 속도 게이트 통과
  bool dry_run = false;                              ///< --dry-run : 계산만 하고 안 보낸다
  std::string host = "192.168.0.20";                 ///< movel 의 IK RPC 용
  int port = 1883;
};

/// 인자에서 옵션 플래그를 걷어내고, 남은 위치인자를 돌려준다.
/// 알 수 없는 `--` 플래그는 실패로 본다 — 오타를 조용히 무시하면 게이트가 헐거워진다.
inline bool parseOptions(
  const std::vector<std::string> & args, MoveOptions & opt, std::vector<std::string> & positional,
  std::string & err)
{
  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string & a = args[i];
    auto need_value = [&](double & dst) {
        if (i + 1 >= args.size()) { err = a + " 에 값이 없다"; return false; }
        try { dst = std::stod(args[++i]); } catch (...) { err = a + " 값이 숫자가 아니다"; return false; }
        return true;
      };
    if (a == "--deg") { opt.deg_input = true; }
    else if (a == "--force") { opt.force = true; }
    else if (a == "--dry-run") { opt.dry_run = true; }
    else if (a == "--sec") { if (!need_value(opt.sec)) { return false; } }
    else if (a == "--speed") { if (!need_value(opt.speed_deg_s)) { return false; } }
    else if (a == "--max-speed") { if (!need_value(opt.max_speed_deg_s)) { return false; } }
    else if (a == "--port") { double p; if (!need_value(p)) { return false; } opt.port = static_cast<int>(p); }
    else if (a == "--host") {
      if (i + 1 >= args.size()) { err = "--host 에 값이 없다"; return false; }
      opt.host = args[++i];
    } else if (a.rfind("--", 0) == 0) {
      err = "모르는 옵션: " + a; return false;
    } else {
      positional.push_back(a);
    }
  }
  return true;
}

/// URDF 한계. xacro 를 읽지 않고 상수로 든다 — 값의 원본은 hcr_robot.xacro 이고
/// 여기는 **더 안쪽**이어야 한다(운전절차 §5 함정 ⑩: URDF 한계는 실기보다 항상 안쪽).
/// 실기 ±360°(J3 ±165°) 검사가 withinLimits() 로 따로 걸리므로 이중 검사가 된다.

/// 목표를 사람이 읽을 수 있게 두 단위로 찍는다. 함정 ⑧ 은 이 표가 없어서 났다.
inline void printTarget(
  const char * label, const JointArray & urdf_rad, const JointArray & real_deg)
{
  std::printf("  %s\n", label);
  std::printf("    %-10s %10s %12s %12s\n", "관절", "URDF(rad)", "URDF(도)", "실기(도)");
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    std::printf(
      "    %-10s %10.6f %12.3f %12.3f\n",
      kUrdfJointNames[i].c_str(), urdf_rad[i], rad2deg(urdf_rad[i]), real_deg[i]);
  }
}

/// 게이트 ②③. 통과하면 소요시간(초)을 duration_sec 에 채운다.
inline bool checkGates(
  const JointArray & cur_real_deg, const JointArray & goal_real_deg, const MoveOptions & opt,
  double & duration_sec, std::string & err)
{
  // ② 한계 — 실기 공식 가동범위
  if (!withinLimits(goal_real_deg)) {
    err = "목표가 실기 공식 가동범위 밖이다 (±360°, J3 ±165°). withinLimits() 거부";
    return false;
  }

  // ③ 속도 — 가장 많이 도는 축이 기준이다
  double max_delta = 0.0;
  std::size_t max_axis = 0;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const double d = std::fabs(goal_real_deg[i] - cur_real_deg[i]);
    if (d > max_delta) { max_delta = d; max_axis = i; }
  }

  if (max_delta < 1e-9) {
    err = "목표가 현재 자세와 같다 — 보낼 것이 없다";
    return false;
  }

  duration_sec = (opt.sec > 0.0) ? opt.sec : (max_delta / opt.speed_deg_s);
  if (duration_sec < kMinDurationSec) { duration_sec = kMinDurationSec; }

  const double speed = max_delta / duration_sec;
  std::printf(
    "  속도  최대이동 %s %.3f° ÷ %.2fs = %.2f°/s  (상한 %.1f°/s)\n",
    kUrdfJointNames[max_axis].c_str(), max_delta, duration_sec, speed, opt.max_speed_deg_s);

  if (speed > opt.max_speed_deg_s) {
    if (!opt.force) {
      char buf[256];
      std::snprintf(
        buf, sizeof(buf),
        "속도 %.2f°/s 가 상한 %.1f°/s 를 넘는다 — 거부. "
        "--sec 를 늘리거나 --max-speed / --force 를 명시하라",
        speed, opt.max_speed_deg_s);
      err = buf;
      return false;
    }
    std::printf("  ⚠️ --force — 속도 상한을 넘겨 보낸다\n");
  }
  return true;
}

/// /joint_states 를 한 번 받아 현재 URDF 자세를 얻는다.
/// MQTT 가 아니라 ROS 로 읽는 이유 — movej 는 ros2_control 경로 안에서 완결돼야 한다.
inline bool readCurrentJointState(
  const rclcpp::Node::SharedPtr & node, JointArray & out_rad, std::chrono::seconds timeout,
  std::string & err)
{
  JointArray got{};
  bool have = false;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (have) { return; }
      JointArray tmp{};
      std::size_t found = 0;
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        for (std::size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k) {
          if (msg->name[k] == kUrdfJointNames[i]) { tmp[i] = msg->position[k]; ++found; break; }
        }
      }
      if (found == kNumJoints) { got = tmp; have = true; }
    });

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && !have && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!have) {
    err = "/joint_states 를 못 받았다 — 스택이 떠 있는지, 컨트롤러가 active 인지 확인하라";
    return false;
  }
  out_rad = got;
  return true;
}

/// FollowJointTrajectory 로 한 점을 보낸다. 도착 판정은 하지 않는다 —
/// error_code 는 판정력이 없다(운전절차 §5 함정 ④, JTC constraints 부재).
inline bool sendTrajectory(
  const rclcpp::Node::SharedPtr & node, const JointArray & goal_rad, double duration_sec,
  std::string & err)
{
  auto client = rclcpp_action::create_client<FollowJointTrajectory>(
    node, "/hcr_arm_controller/follow_joint_trajectory");

  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    err = "액션 서버가 없다 — /hcr_arm_controller/follow_joint_trajectory. 스택이 떠 있나?";
    return false;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names.assign(kUrdfJointNames.begin(), kUrdfJointNames.end());
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions.assign(goal_rad.begin(), goal_rad.end());
  pt.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  goal.trajectory.points.push_back(pt);

  auto goal_future = client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, goal_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    err = "goal 전송 실패";
    return false;
  }
  auto handle = goal_future.get();
  if (!handle) {
    err = "goal 이 거부됐다";
    return false;
  }
  std::printf("  → goal 접수. %.2fs 예정\n", duration_sec);

  auto result_future = client->async_get_result(handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    err = "결과 대기 실패";
    return false;
  }
  const auto result = result_future.get();
  std::printf(
    "  ← 종료 code=%d\n"
    "  ⚠️ error_code 는 도착 판정이 아니다 (JTC constraints 부재). "
    "도착은 `mqtt_cmd.py pos` 로 본다\n",
    result.result->error_code);
  return true;
}

}  // namespace hcr5_bridge

#endif  // HCR5_BRIDGE__MOVE_COMMON_HPP_
