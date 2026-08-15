// movej — 관절 목표 하나를 ros2_control 경로(FollowJointTrajectory)로 보낸다.
//
//   ros2 run hcr5_bridge movej <j1..j6> [옵션]
//   ros2 run hcr5_bridge movej home   [옵션]
//
// 입력 단위는 **기본 URDF 라디안**(ROS 표준 SI)이고, `--deg` 를 붙이면 실기 도(度)로 받는다.
// 어느 쪽이든 발행 전에 두 단위를 다 찍는다 — 운전절차 §5 함정 ⑧ 이 그 표가 없어서 났다.
//
// 이 실행기가 여는 것은 새 경로가 아니라 **게이트**다. 액션을 직접 쏘는 것과
// 도달하는 곳은 같고, 다른 것은 발행 전에 한계·속도를 검사한다는 점뿐이다.

#include <cstdio>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "hcr5_bridge/move_common.hpp"

namespace
{

void usage()
{
  std::printf(
    "사용법:\n"
    "  ros2 run hcr5_bridge movej <j1> <j2> <j3> <j4> <j5> <j6> [옵션]\n"
    "  ros2 run hcr5_bridge movej home [옵션]\n"
    "\n"
    "단위\n"
    "  기본       URDF 라디안 (ROS 표준 SI)\n"
    "  --deg      실기 도(度) — 펜던트·mqtt_cmd.py 와 같은 숫자\n"
    "\n"
    "속도 (액션 직송에는 다른 안전망이 없다 — 여기가 유일하다)\n"
    "  --sec N        소요시간. 안 주면 %.1f°/s 로 자동 계산\n"
    "  --speed D      자동 계산에 쓸 속도(실기 도/s). 기본 %.1f\n"
    "  --max-speed D  상한(실기 도/s). 기본 %.1f — 넘으면 거부\n"
    "  --force        속도 상한을 넘겨 보낸다\n"
    "  --dry-run      계산만 하고 발행하지 않는다\n"
    "\n"
    "예\n"
    "  movej home                       홈 복귀\n"
    "  movej --deg 10 -90 -90 -90 90 0  실기 도로 P1\n"
    "  movej 1.745329 0 1.570796 0 1.570796 0   같은 점을 rad 로\n",
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

  // ── 목표 해석 ────────────────────────────────────────────────────────────
  JointArray goal_real_deg{};
  bool named_home = false;

  if (pos.size() == 1 && pos[0] == "home") {
    goal_real_deg = kHomeRealDeg;
    named_home = true;
  } else if (pos.size() == kNumJoints) {
    JointArray in{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      try {
        in[i] = std::stod(pos[i]);
      } catch (...) {
        std::fprintf(stderr, "✗ %zu번째 값이 숫자가 아니다: %s\n", i + 1, pos[i].c_str());
        rclcpp::shutdown();
        return 2;
      }
    }
    if (opt.deg_input) {
      goal_real_deg = in;                        // 실기 도 그대로
    } else {
      goal_real_deg = urdfRadToRealDeg(in);      // URDF rad → 실기 도
    }
  } else {
    std::fprintf(stderr, "✗ 관절값 6개나 `home` 이 필요하다 (받은 위치인자 %zu개)\n\n", pos.size());
    usage();
    rclcpp::shutdown();
    return 2;
  }

  const JointArray goal_rad = realDegToUrdfRad(goal_real_deg);

  auto node = rclcpp::Node::make_shared("hcr5_movej");

  std::printf("movej — 입력 단위 %s%s\n",
    named_home ? "명명점 home" : (opt.deg_input ? "실기 도(--deg)" : "URDF 라디안(기본)"),
    opt.dry_run ? " · --dry-run" : "");

  // ── 게이트 ① 단위 ────────────────────────────────────────────────────────
  printTarget("목표", goal_rad, goal_real_deg);

  // ── 현재 자세 ────────────────────────────────────────────────────────────
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
