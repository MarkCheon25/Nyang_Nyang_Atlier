// movel — TCP 목표 포즈를 관절값으로 풀어 ros2_control 경로로 보낸다.
//
//   ros2 run hcr5_bridge movel <x> <y> <z> <rx> <ry> <rz> [옵션]     (mm, 도)
//
// ⚠️ **경로는 직선이 아니다.** 시작·끝만 직교로 지정하고 그 사이는 JTC 가 관절 보간한다.
// 진짜 직선(컨트롤러 보간)은 `program/plan` 의 `move.selected:"linear"` 가 소유하고,
// 그건 ros2_control 을 안 타는 다른 층이다 (hcr5_mqtt/README.md §6, 실측 이탈 0.045mm/179.3mm).
// 이름이 movel 인 것은 **직교 좌표로 목표를 준다**는 뜻이지 직선 보간을 뜻하지 않는다.
//
// IK 는 로봇 컨트롤러가 푼다 — `robot/convertJointAngle` RPC. 우리 쪽 IK 를 새로 세우지
// 않는 이유는, 실기가 실제로 쓰는 해와 어긋나면 조용히 다른 자세로 가기 때문이다.

#include <cstdio>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "hcr5_bridge/move_common.hpp"
#include "hcr5_bridge/mqtt_client.hpp"

namespace
{

constexpr const char * kTopicGetPos = "get/command/pos";
constexpr const char * kTopicIk = "robot/convertJointAngle";

void usage()
{
  std::printf(
    "사용법:\n"
    "  ros2 run hcr5_bridge movel <x> <y> <z> <rx> <ry> <rz> [옵션]\n"
    "\n"
    "단위  위치 mm · 자세 도(度). 기준은 TCP (poseType:\"tcp\")\n"
    "\n"
    "⚠️ 경로는 직선이 아니다 — 시작·끝만 직교로 주고 사이는 관절 보간이다.\n"
    "   진짜 직선은 program/plan 의 linear 가 소유한다 (hcr5_mqtt/README.md §6).\n"
    "\n"
    "옵션\n"
    "  --sec N        소요시간. 안 주면 %.1f°/s 로 자동 계산\n"
    "  --speed D      자동 계산에 쓸 속도(실기 도/s). 기본 %.1f\n"
    "  --max-speed D  상한(실기 도/s). 기본 %.1f — 넘으면 거부\n"
    "  --force        속도 상한을 넘겨 보낸다\n"
    "  --dry-run      IK 해까지만 보고 발행하지 않는다\n"
    "  --host H       MQTT 브로커. 기본 192.168.0.20\n"
    "  --port P       기본 1883\n"
    "\n"
    "예\n"
    "  movel 490.0 -170.5 441.5 -180 0 0 --dry-run   홈 flange 자리 확인\n",
    hcr5_bridge::kDefaultSpeedDegPerSec, hcr5_bridge::kDefaultSpeedDegPerSec,
    hcr5_bridge::kDefaultMaxSpeedDegPerSec);
}

}  // namespace

int main(int argc, char ** argv)
{
  using namespace hcr5_bridge;

  rclcpp::init(argc, argv);
  const auto raw = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<std::string> args(raw.begin() + 1, raw.end());

  if (args.empty() || args[0] == "-h" || args[0] == "--help") {
    usage();
    rclcpp::shutdown();
    return args.empty() ? 1 : 0;
  }

  MoveOptions opt;
  std::vector<std::string> pos;
  std::string err;
  if (!parseOptions(args, opt, pos, err)) {
    std::fprintf(stderr, "✗ %s\n", err.c_str());
    rclcpp::shutdown();
    return 2;
  }
  if (opt.deg_input) {
    std::fprintf(stderr, "✗ movel 에 --deg 는 없다 — 위치는 늘 mm, 자세는 늘 도다\n");
    rclcpp::shutdown();
    return 2;
  }
  if (pos.size() != 6) {
    std::fprintf(stderr, "✗ x y z rx ry rz 6개가 필요하다 (받은 위치인자 %zu개)\n\n", pos.size());
    usage();
    rclcpp::shutdown();
    return 2;
  }

  std::array<double, 6> target_pose{};
  for (std::size_t i = 0; i < 6; ++i) {
    try {
      target_pose[i] = std::stod(pos[i]);
    } catch (...) {
      std::fprintf(stderr, "✗ %zu번째 값이 숫자가 아니다: %s\n", i + 1, pos[i].c_str());
      rclcpp::shutdown();
      return 2;
    }
  }

  std::printf(
    "movel — TCP 목표 pos(mm) [%.3f %.3f %.3f] rot(도) [%.3f %.3f %.3f]%s\n",
    target_pose[0], target_pose[1], target_pose[2],
    target_pose[3], target_pose[4], target_pose[5],
    opt.dry_run ? " · --dry-run" : "");

  // ── IK — 로봇 컨트롤러가 푼다 ────────────────────────────────────────────
  MqttClient mqtt(opt.host, opt.port, "hcr5-movel");
  if (!mqtt.start()) {
    std::fprintf(stderr, "✗ MQTT 접속 실패: %s:%d\n", opt.host.c_str(), opt.port);
    rclcpp::shutdown();
    return 3;
  }

  // 시드 — 현재 지령 자세. 시드가 나쁘면 IK 가 먼 해를 돌려준다.
  JointArray seed_real_deg{};
  {
    const auto r = mqtt.request(kTopicGetPos, Json::object());
    if (!r) {
      std::fprintf(stderr, "✗ %s 응답이 없다\n", kTopicGetPos);
      rclcpp::shutdown();
      return 3;
    }
    const auto it = r->find("joint");
    if (it == r->end() || !it->is_array() || it->size() != kNumJoints) {
      std::fprintf(stderr, "✗ %s 응답에 joint 6개가 없다\n", kTopicGetPos);
      rclcpp::shutdown();
      return 3;
    }
    for (std::size_t i = 0; i < kNumJoints; ++i) { seed_real_deg[i] = (*it)[i].get<double>(); }
  }

  JointArray goal_real_deg{};
  {
    Json payload = {
      {"info", {
         {"position", {{"x", target_pose[0]}, {"y", target_pose[1]}, {"z", target_pose[2]}}},
         {"orientation", {{"x", target_pose[3]}, {"y", target_pose[4]}, {"z", target_pose[5]}}},
         {"joint", seed_real_deg}}},
      {"poseType", "tcp"}};
    const auto r = mqtt.request(kTopicIk, payload);
    if (!r) {
      std::fprintf(stderr, "✗ IK 응답이 없다 (%s)\n", kTopicIk);
      rclcpp::shutdown();
      return 3;
    }
    const auto it = r->find("joint");
    if (it == r->end() || !it->is_array() || it->size() != kNumJoints) {
      std::fprintf(
        stderr, "✗ IK 가 해를 못 냈다 — 도달 불가능한 포즈이거나 특이점일 수 있다\n");
      rclcpp::shutdown();
      return 3;
    }
    for (std::size_t i = 0; i < kNumJoints; ++i) { goal_real_deg[i] = (*it)[i].get<double>(); }
  }

  std::printf("  IK 해 (시드 = 현재 지령 자세)\n");
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    std::printf(
      "    %-10s %10.4f°   (시드 대비 %+8.3f°)\n",
      kUrdfJointNames[i].c_str(), goal_real_deg[i], goal_real_deg[i] - seed_real_deg[i]);
  }

  const JointArray goal_rad = realDegToUrdfRad(goal_real_deg);

  auto node = rclcpp::Node::make_shared("hcr5_movel");

  // ── 게이트 ① 단위 ────────────────────────────────────────────────────────
  printTarget("목표", goal_rad, goal_real_deg);

  // ── 현재 자세 — 게이트의 기준은 ROS 가 보는 값이다 ────────────────────────
  JointArray cur_rad{};
  if (!readCurrentJointState(node, cur_rad, std::chrono::seconds(5), err)) {
    std::fprintf(stderr, "✗ %s\n", err.c_str());
    rclcpp::shutdown();
    return 3;
  }
  const JointArray cur_real_deg = urdfRadToRealDeg(cur_rad);
  printTarget("현재", cur_rad, cur_real_deg);

  // ── 게이트 ②③ 한계·속도 ──────────────────────────────────────────────────
  double duration = 0.0;
  if (!checkGates(cur_real_deg, goal_real_deg, opt, duration, err)) {
    std::fprintf(stderr, "✗ %s\n", err.c_str());
    rclcpp::shutdown();
    return 4;
  }

  if (opt.dry_run) {
    std::printf("  --dry-run — 발행하지 않는다\n");
    rclcpp::shutdown();
    return 0;
  }

  if (!sendTrajectory(node, goal_rad, duration, err)) {
    std::fprintf(stderr, "✗ %s\n", err.c_str());
    rclcpp::shutdown();
    return 5;
  }

  rclcpp::shutdown();
  return 0;
}
