// movej — 관절 공간 이동. 관절값 6개를 주면 각 축이 그 각도로 간다.
//
//   ros2 run hcr5_bridge movej <j1> <j2> <j3> <j4> <j5> <j6> [옵션]
//   ros2 run hcr5_bridge movej home [옵션]
//
// 보간을 **컨트롤러가 소유한다** — `program/plan` 의 `move.selected:"joint"` 노드 하나를
// 적재하고 `program/play` 로 실행한다. 설계 근거 전체는 movej.md.
//
// ⚠️ **ros2_control 스택과 배타다.** `program/play` 는 컨트롤러가 자기 궤적을 실행하는 것이고
// `bringup.launch.py` 는 JTC 가 주기적으로 관절 지령을 쓴다 — 둘이 동시에 돌면 명령이 부딪힌다
// (README §1.2 · movel.md §1.3). 그래서 이 실행기는 `movel`·`servo` 와 같이 **rclcpp 의존이 없다.**
//
// 종전 구현(2026-08-16 오전까지)은 JTC 경로였다. 동작 자체는 옳았다 — 관절공간 보간이니
// 관절 궤적이 맞았고, `movel` 처럼 이름과 동작이 어긋난 적이 없다. 바뀐 것은 **자립성과 판정력**이다:
// 스택 없이 혼자 돌고, 도착을 사람이 아니라 프로그램이 판정한다 (movej.md §3.3).
//
// ✅ **`move.joint.velocity` 의 단위는 도/s 다** (2026-08-16 실측 **182점** · 6축 전부, movej.md §2.3).
// 지령 속도는 그대로 지켜진다 (K = 0.9987±0.0019). 다만 **가감속이 사다리꼴이 아니다** —
// 램프에 v/a 의 1.54배가 걸린다. 실측 모델:  t = d/v + 1.54·(v/a) + 0.20초  (RMS 잔차 0.037초).
// 오전 2점으로 냈던 k=1.017 은 사다리꼴을 강제한 탓에 부족분이 주행항으로 밀린 허상이었다.
// **6축이 같은 모형을 따른다** — 중력 방향(올림/내림)도 차이가 없다. 축별 보정은 필요 없다.
//
// 📎 `program/plan` 봉투의 **원본은 `hcr5_comm/README.md` §6** 이다. 이 파일과 `movel.cpp` 는
// 그 문서를 각자 구현한 두 벌이고, 다른 것은 `move.selected` 와 웨이포인트를 채우는 방법뿐이다.
// 한쪽을 고치면 다른 쪽도 봐야 한다 — 합치는 것은 movej.md §4 의 후속 항목.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include <uuid/uuid.h>

#include "hcr5_bridge/joint_convention.hpp"
#include "hcr5_bridge/mqtt_client.hpp"
#include "hcr5_bridge/pose_convention.hpp"
#include "hcr5_bridge/servo_gate.hpp"

namespace
{

using hcr5_bridge::Json;
using hcr5_bridge::MqttClient;
using hcr5_bridge::Vec3;
using hcr5_bridge::kNumJoints;

using JointArray = std::array<double, kNumJoints>;

constexpr const char * kTopicGetPos = "get/command/pos";
constexpr const char * kTopicFk = "robot/convertPose";
constexpr const char * kTopicClear = "program/clear";
constexpr const char * kTopicPlan = "program/plan";
constexpr const char * kTopicPlay = "program/play";
constexpr const char * kTopicStop = "program/stop";
constexpr const char * kTopicMoveStop = "move/stop";
constexpr const char * kTopicProgramEnd = "program/end";
constexpr const char * kTopicEventMotion = "event/motion";

/// 각속도 기본값(도/s). 단위가 미확정이던 동안은 종전 JTC 구현의 숫자(2.0)를 그대로 뒀는데,
/// 2026-08-16 실측으로 도/s 가 확정되면서 그 보수값을 유지할 이유가 없어졌다.
/// 펜던트 캡처의 joint 기본값 50 보다는 아래에 둔다 (hcr5_comm/README.md §6.1.1).
constexpr double kDefaultSpeedDegPerSec = 30.0;
/// 각속도 상한. 펜던트 기본값과 같은 자리 — 이 위는 사람이 명시적으로 올려야 한다.
constexpr double kDefaultMaxSpeedDegPerSec = 50.0;
/// 각가속도 기본값. `hcr5_comm/README.md` §6.1.1 의 joint 노드 기본값과 같다.
/// ⚠️ 기본 속도가 30°/s 로 올라가면서 **게이트 ③ 이 9° 를 요구한다**(v²/a — 실측 램프각은
/// 0.77·v²/a = 6.9° 라 게이트가 보수적인 것이다, kRampFactor 참조). 그보다 짧은 이동은
/// 게이트 ③ 이 거부한다 — 거부 메시지가 그 구간에서 낼 수 있는 최대 속도를 알려주므로
/// `--speed` 를 낮추면 된다. 짧은 이동을 자주 한다면 `--accel` 을 올리는 쪽이 맞다.
constexpr double kDefaultAccelDegPerSec2 = 100.0;
/// 최대 이동 축의 각변위 상한. movel 의 직교 거리 상한(movel.md §2.4)에 대응하는 칸이다 —
/// `--speed` 는 각속도만 보므로 **총 이동량은 아무도 안 본다**. 그 구멍을 메운다.
constexpr double kDefaultMaxDeltaDeg = 90.0;
/// 이보다 작으면 이동으로 치지 않는다. 목표가 사실상 현재 자세면 보낼 것이 없고,
/// 부동소수 찌꺼기가 남으면 **속도 게이트에 먼저 걸려** 엉뚱한 이유로 거부된다 — 그 오독을 막는다.
constexpr double kMinDeltaDeg = 0.01;
/// tcp 와 flange 를 같은 점으로 봐도 되는 한계(mm). 로봇 반복정밀도(±0.1mm)보다 안쪽이다.
constexpr double kPoseSameTolMm = 0.01;

/// 가감속 시간 계수 — 램프에 걸리는 시간은 `v/a` 가 아니라 **1.54·(v/a)** 다.
/// 사다리꼴이 아니라 저크가 다듬어진 프로파일이라는 뜻이고, 지령 가속도의 실효값은 0.65·a 다.
/// 2026-08-16 실측 **182점** (6축 전부 · d 30~360° · v 10~90°/s · a 25~400°/s²) 회귀:
///   B = 1.5392 ± 0.0053. movej.md §2.3.
/// **이 값은 단단하다** — `a` 만 25→400 으로 흔들어 v/a 를 16배 벌린 독립 지렛대(30점)에서도
/// 1.5409 ± 0.0085 로 같은 값이 나왔다. 축도, 중력 방향도, 세션 시각도 안 가린다.
/// 🔴 **π/2 = 1.5708 은 아니다** — 6.0σ 로 배제된다. 한때 코사인형 램프를 시사한다고 적어 두었으나
///    v/a 지렛대를 벌리자 갈렸다. (통계오차 기준이고 축별·설계별 계통은 안 갈랐다.)
constexpr double kRampFactor = 1.54;
/// 속도·거리·가속도 무엇과도 상관없이 붙는 고정 오버헤드(초). 발행~플레이 왕복과 정지 판정 몫이다.
/// 182점 회귀로 c = 0.2034 ± 0.0055. **소수 셋째 자리는 의미가 없다** — 회차 산포가 0.035초다.
/// 🔴 한때 "세션 중 0.189 → 0.226 으로 드리프트한다" 고 적어 두었으나 **기각됐다**(업무목록 L26).
///    같은 지령 80회를 29분(중간 12.7분 정지)에 걸쳐 돌려 시간축만 흔들었더니 기울기가
///    −0.010±0.005 / +0.003±0.005 초/10분 이었다 — 크기도 방향도 계통적이지 않다.
///    앞서 본 단조 증가는 **묶음당 n=14~24 짜리 평균 네 개(각 표준오차 0.007~0.011초)에서
///    추세를 읽은 것**이었다. 오후 두 묶음(n=80, 30)은 다시 0.203 / 0.201 로 돌아왔다.
/// 아래 회귀 감시 밴드(±0.20초)는 그래도 좁히지 마라 — 회차 산포 0.035초에 축별·설계별
/// 미세 편차가 얹혀 묶음평균이 0.19~0.22 사이에서 흔들린다.
constexpr double kFixedOverheadSec = 0.20;

struct Options
{
  double speed = kDefaultSpeedDegPerSec;         ///< move.joint.velocity 로 실린다
  double accel = kDefaultAccelDegPerSec2;        ///< move.joint.acceleration 으로 실린다
  double max_speed = kDefaultMaxSpeedDegPerSec;
  double max_delta = kDefaultMaxDeltaDeg;
  double timeout_sec = 60.0;
  bool deg_input = false;                        ///< --deg : 입력이 실기 도(度)
  bool force = false;
  bool dry_run = false;
  bool plan_only = false;                        ///< 적재까지만 — program/play 를 안 쏜다
  std::string host = "192.168.0.20";
  int port = 1883;
};

void usage()
{
  std::printf(
    "사용법:\n"
    "  ros2 run hcr5_bridge movej <j1> <j2> <j3> <j4> <j5> <j6> [옵션]\n"
    "  ros2 run hcr5_bridge movej home [옵션]\n"
    "\n"
    "관절값 6개를 주면 각 축이 그 각도로 간다. **기준 좌표계 인자가 없다** —\n"
    "관절공간에는 원점이 없다 (두산 movej 도 movel 과 달리 ref 가 없다. movej.md §3.1).\n"
    "플랜지 경로는 구속하지 않는다 — 직선이 필요하면 movel 을 쓴다.\n"
    "\n"
    "단위\n"
    "  기본       URDF 라디안 (ROS 표준 SI)\n"
    "  --deg      실기 도(度) — 펜던트·mqtt_cmd.py 와 같은 숫자\n"
    "  발행 전에 rad · URDF도 · 실기도 **세 칸을 다 찍는다** (README §3.8 단위 폭주 사고)\n"
    "\n"
    "⚠️ ros2_control 스택(bringup.launch.py)과 **동시에 쓰지 않는다** — 명령이 부딪힌다.\n"
    "⚠️ 서보가 꺼져 있으면 발행 전에 죽는다 — `mqtt_cmd.py servo on` 으로 켠다.\n"
    "\n"
    "옵션\n"
    "  --deg          입력을 실기 도로 받는다\n"
    "  --speed D      각속도. 기본 %.1f — 컨트롤러 노드에 그대로 실린다\n"
    "  --accel A      각가속도. 기본 %.1f (컨트롤러 joint 기본값)\n"
    "  --max-speed D  각속도 상한. 기본 %.1f — 넘으면 거부\n"
    "  --max-delta D  최대 이동 축의 각변위 상한. 기본 %.1f — 넘으면 거부\n"
    "  --timeout S    도착 대기 상한 초. 기본 60 — 넘으면 move/stop 을 쏜다\n"
    "  --force        속도·각변위·프로파일 게이트를 넘겨 보낸다\n"
    "  --dry-run      계산과 FK 까지만 보고 아무것도 발행하지 않는다\n"
    "  --plan-only    프로그램 적재까지만 하고 program/play 를 안 쏜다 — 로봇이 안 움직인다\n"
    "  --host H       MQTT 브로커. 기본 192.168.0.20\n"
    "  --port P       기본 1883\n"
    "\n"
    "예\n"
    "  movej home                                홈 복귀\n"
    "  movej --deg 10 -90 -90 -90 90 0           실기 도로 P1\n"
    "  movej 1.745329 0 1.570796 0 1.570796 0    같은 점을 rad 로\n",
    kDefaultSpeedDegPerSec, kDefaultAccelDegPerSec2,
    kDefaultMaxSpeedDegPerSec, kDefaultMaxDeltaDeg);
}

/// 인자 파싱. 모르는 `--` 플래그는 실패로 본다 — 오타를 조용히 무시하면 게이트가 헐거워진다.
bool parse(
  const std::vector<std::string> & args, Options & opt, std::vector<std::string> & pos,
  std::string & err)
{
  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string & a = args[i];
    auto value = [&](double & dst) {
        if (i + 1 >= args.size()) { err = a + " 에 값이 없다"; return false; }
        try { dst = std::stod(args[++i]); } catch (...) { err = a + " 값이 숫자가 아니다"; return false; }
        return true;
      };
    if (a == "--deg") { opt.deg_input = true; }
    else if (a == "--force") { opt.force = true; }
    else if (a == "--dry-run") { opt.dry_run = true; }
    else if (a == "--plan-only") { opt.plan_only = true; }
    else if (a == "--speed") { if (!value(opt.speed)) { return false; } }
    else if (a == "--accel") { if (!value(opt.accel)) { return false; } }
    else if (a == "--max-speed") { if (!value(opt.max_speed)) { return false; } }
    else if (a == "--max-delta") { if (!value(opt.max_delta)) { return false; } }
    else if (a == "--timeout") { if (!value(opt.timeout_sec)) { return false; } }
    else if (a == "--port") { double p; if (!value(p)) { return false; } opt.port = static_cast<int>(p); }
    else if (a == "--host") {
      if (i + 1 >= args.size()) { err = "--host 에 값이 없다"; return false; }
      opt.host = args[++i];
    } else if (a.rfind("--", 0) == 0) {
      err = "모르는 옵션: " + a;
      return false;
    } else {
      pos.push_back(a);
    }
  }
  if (opt.speed <= 0.0 || opt.accel <= 0.0) { err = "--speed · --accel 은 양수여야 한다"; return false; }
  if (opt.max_speed <= 0.0 || opt.max_delta <= 0.0) {
    err = "--max-speed · --max-delta 는 양수여야 한다";
    return false;
  }
  return true;
}

Vec3 readVec(const Json & j)
{
  return Vec3{j.value("x", 0.0), j.value("y", 0.0), j.value("z", 0.0)};
}

Json writeVec(const Vec3 & v)
{
  return Json{{"x", v[0]}, {"y", v[1]}, {"z", v[2]}};
}

std::string makeUuid()
{
  uuid_t u;
  uuid_generate_time(u);
  char buf[37];
  uuid_unparse_lower(u, buf);
  return std::string(buf);
}

/// 한 점의 직교 표현 — 위치와 자세.
struct Pose
{
  Vec3 pos{};
  Vec3 ori{};
};

/// 목표를 사람이 읽을 수 있게 **세 단위로** 찍는다 (게이트 ⓪).
/// README §3.8 의 단위 폭주 사고가 이 표가 없어서 났다. rclcpp 를 떼면서
/// `move_common.hpp::printTarget()` 을 여기로 옮겨 왔다 — 이 표만은 버릴 수 없다.
void printTarget(const char * label, const JointArray & urdf_rad, const JointArray & real_deg)
{
  using hcr5_bridge::kUrdfJointNames;
  using hcr5_bridge::rad2deg;
  std::printf("  %s\n", label);
  std::printf("    %-10s %10s %12s %12s\n", "관절", "URDF(rad)", "URDF(도)", "실기(도)");
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    std::printf(
      "    %-10s %10.6f %12.3f %12.3f\n",
      kUrdfJointNames[i].c_str(), urdf_rad[i], rad2deg(urdf_rad[i]), real_deg[i]);
  }
}

/// 한 점의 세 표현 묶음 — tcp·flange·joint.
/// 웨이포인트는 이 셋을 **동시에** 담고 셋은 서로 정합해야 한다 (hcr5_comm/README.md §6.1.2).
/// movel 은 한쪽 pose 를 양쪽에 복사하지만, 여기는 FK 로 tcp·flange 를 **각각** 얻어 넣는다.
Json poseTriple(const Pose & tcp, const Pose & flange, const JointArray & joint)
{
  auto one = [](const Pose & p) {
      return Json{{"position", writeVec(p.pos)}, {"orientation", writeVec(p.ori)}};
    };
  return Json{{"tcp", one(tcp)}, {"flange", one(flange)}, {"joint", joint}};
}

/// 웨이포인트 한 칸. `relative` 블록은 `selected:"fixed"` 일 때도 **늘 실린다**
/// (2026-08-16 펜던트 캡처 확인) — 빼면 컨트롤러 파서가 어떻게 반응하는지 모르므로 그대로 채운다.
Json waypointSlot(
  const Pose & tcp, const Pose & flange, const JointArray & joint,
  const Json & from, bool result_zero)
{
  const Json here = poseTriple(tcp, flange, joint);
  const Json nul{{"x", nullptr}, {"y", nullptr}, {"z", nullptr}};
  const Json zero{{"x", 0}, {"y", 0}, {"z", 0}};
  return Json{
    {"selected", "fixed"},
    {"fixed", here},
    {"relative", {
       {"from", from},
       {"to", from},
       {"result", {
          {"position", result_zero ? zero : nul},
          {"orientation", result_zero ? zero : nul}}}}},
    {"variable", nullptr}};
}

/// program/plan 페이로드 — MOVE(joint) 노드 하나짜리 최소 프로그램.
/// 형식의 원본은 hcr5_comm/README.md §6 이고, movel 의 linear 판과 다른 것은
/// `move.selected` 와 노드 이름뿐이다. ⚠️ `joint` 노드에는 `radius` 가 없다 (§6.1.1).
Json buildPlan(
  const Pose & cur_tcp, const Pose & cur_flange, const JointArray & cur_joint,
  const Pose & goal_tcp, const Pose & goal_flange, const JointArray & goal_joint,
  const Options & opt)
{
  const Json from = poseTriple(cur_tcp, cur_flange, cur_joint);

  // 네 종류가 모두 실리고 `selected` 가 그중 하나를 고른다 (§6.1.1).
  // 안 고른 셋은 펜던트 기본값 그대로 둔다 — 우리가 건드릴 이유가 없다.
  const Json move{
    {"selected", "joint"},
    {"joint", {
       // 🔴 velocity 의 단위가 미확정이다. 도/s 가설로 싣고 도착 시간으로 대조한다 (movej.md §2.3).
       {"velocity", opt.speed},
       {"acceleration", opt.accel},
       // 구간이 하나뿐이라 시작·끝 모두 정지다. 스트로크 체인(§6.3)은 이 계층 밖이다.
       {"continues", false},
       {"startVelocity", 0},
       {"endVelocity", 0},
       {"repeat", 1}}},
    {"linear", {
       {"radius", 0}, {"velocity", 500}, {"acceleration", 1000}, {"continues", false},
       {"startVelocity", 0}, {"endVelocity", 0}, {"repeat", 1}}},
    {"arc", {
       {"velocity", 250}, {"acceleration", 500}, {"continues", false},
       {"startVelocity", 0}, {"endVelocity", 0}, {"repeat", 1}}},
    {"circle", {
       {"velocity", 500}, {"acceleration", 1000}, {"continues", false},
       {"startVelocity", 0}, {"endVelocity", 0}, {"repeat", 1}}}};

  const Json initialize{
    {"uuid", makeUuid()},
    {"name", "Initialize"},
    {"type", "INITIALIZE"},
    {"child", Json::array()},
    {"time", 0},
    {"attr", {{"skip", false}, {"always", false}}}};

  const Json move_node{
    {"uuid", makeUuid()},
    {"name", "MOVEJ"},
    {"type", "MOVE"},
    {"child", Json::array()},
    {"time", 0},
    {"attr", {
       {"skip", false},
       {"repeat", 1},
       {"frame", "flange"},
       {"coordinate", "base"},
       {"options", {{"vision", {{"position", "continuously"}, {"type", "general"}}}}},
       {"move", move},
       {"waypoint", {
          // middlePoint 는 arc·circle 용이고 joint 에서 쓰이지 않는다. 캡처와 같이
          // 현재 자세로 채워 둔다 — 비워서 파서가 어떻게 되는지는 모른다.
          {"middlePoint", waypointSlot(cur_tcp, cur_flange, cur_joint, from, false)},
          {"endPoint", waypointSlot(goal_tcp, goal_flange, goal_joint, from, true)}}}}}};

  auto frame_def = [](const char * name) {
      Json f;
      f["name"] = name;
      f["position"] = Json{{"x", 0}, {"y", 0}, {"z", 0}};
      f["orientation"] = Json{{"x", 0}, {"y", 0}, {"z", 0}};
      return f;
    };
  auto root = [](const Json & uuid, const Json & child) {
      Json r;
      r["uuid"] = uuid;
      r["type"] = "ROOT";
      r["name"] = nullptr;
      r["time"] = 0;
      r["attr"] = Json::object();
      r["child"] = child;
      return r;
    };

  // 조립식으로 쓴다 — 8칸짜리 중첩 브레이스는 nlohmann 의 initializer_list 해석이
  // 어긋나고(컴파일 에러), 무엇보다 사람이 칸을 세기 어렵다.
  Json plan;
  plan["variables"] = Json::object();
  plan["coordinates"]["base"] = frame_def("Base");
  plan["coordinates"]["tcp"] = frame_def("TCP");
  // 전역 속도 배율(%). 캡처값 그대로 100 — 노드 velocity 를 그대로 쓰겠다는 뜻이다.
  plan["velocity"] = 100;
  plan["repeat"] = false;
  plan["program"] = root(makeUuid(), Json::array({initialize, move_node}));
  plan["thread"] = root(0, Json::array());
  plan["subprogram"] = Json::array();
  plan["name"] = "hcr5_bridge_movej";
  return plan;
}

}  // namespace

int main(int argc, char ** argv)
{
  using namespace hcr5_bridge;

  const std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty() || args[0] == "-h" || args[0] == "--help") {
    usage();
    return args.empty() ? 1 : 0;
  }

  Options opt;
  std::vector<std::string> pos;
  std::string err;
  if (!parse(args, opt, pos, err)) {
    std::fprintf(stderr, "✗ %s\n\n", err.c_str());
    usage();
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
        return 2;
      }
    }
    // ⚠️ 이 변환을 빼먹으면 로봇이 **조용히 전혀 다른 자세로 간다** (joint_convention.hpp).
    goal_real_deg = opt.deg_input ? in : urdfRadToRealDeg(in);
  } else {
    std::fprintf(stderr, "✗ 관절값 6개나 `home` 이 필요하다 (받은 위치인자 %zu개)\n\n", pos.size());
    usage();
    return 2;
  }
  const JointArray goal_rad = realDegToUrdfRad(goal_real_deg);

  std::printf(
    "movej — 입력 단위 %s%s\n",
    named_home ? "명명점 home" : (opt.deg_input ? "실기 도(--deg)" : "URDF 라디안(기본)"),
    opt.dry_run ? " · --dry-run" : "");

  MqttClient mqtt(opt.host, opt.port, "hcr5-movej");
  if (!mqtt.start()) {
    std::fprintf(stderr, "✗ MQTT 접속 실패: %s:%d\n", opt.host.c_str(), opt.port);
    return 3;
  }

  // ── 현재 자세 — 기준은 **MQTT** 다 ────────────────────────────────────────
  // ROS `/joint_states` 가 아니다. 이 경로는 스택을 안 타므로 그 토픽이 아예 없다 (movej.md §1.3).
  // 한 번의 응답이 tcp·flange·joint 를 다 준다 — 현재점 웨이포인트는 이것으로 완성된다.
  JointArray cur_real_deg{};
  Pose cur_tcp, cur_flange;
  {
    const auto r = mqtt.request(kTopicGetPos, Json::object());
    if (!r) {
      std::fprintf(stderr, "✗ %s 응답이 없다 — 실기가 켜져 있고 랜이 붙었는지 확인하라\n", kTopicGetPos);
      return 3;
    }
    const auto jt = r->find("joint");
    if (jt == r->end() || !jt->is_array() || jt->size() != kNumJoints ||
      r->find("tcp") == r->end() || r->find("flange") == r->end())
    {
      std::fprintf(stderr, "✗ %s 응답에 joint 6개 · tcp · flange 가 다 있지 않다\n", kTopicGetPos);
      return 3;
    }
    for (std::size_t i = 0; i < kNumJoints; ++i) { cur_real_deg[i] = (*jt)[i].get<double>(); }
    cur_tcp.pos = readVec(r->at("tcp").at("position"));
    cur_tcp.ori = readVec(r->at("tcp").at("orientation"));
    cur_flange.pos = readVec(r->at("flange").at("position"));
    cur_flange.ori = readVec(r->at("flange").at("orientation"));
  }
  const JointArray cur_rad = realDegToUrdfRad(cur_real_deg);

  // ── 게이트 ⓪ 단위 ────────────────────────────────────────────────────────
  printTarget("현재", cur_rad, cur_real_deg);
  printTarget("목표", goal_rad, goal_real_deg);

  // 업무목록 L14 — 활성 툴이 세션 사이에 바뀌어 tcp↔flange 오프셋이 128.05 / 10 / 0 으로 표류했다.
  // 지금 값을 찍어 둔다. 아래 FK 대체 판단도 이 값에 걸린다.
  const double tcp_offset = norm(sub(cur_tcp.pos, cur_flange.pos));
  std::printf("  현재 tcp↔flange 오프셋 %.3f mm (L14 — 세션 사이에 바뀐다)\n", tcp_offset);

  // ── 게이트 ① 각변위 ──────────────────────────────────────────────────────
  // 가장 많이 도는 축이 기준이다. `--speed` 는 각속도만 보므로 총 이동량은 여기서만 걸린다.
  double max_delta = 0.0;
  std::size_t max_axis = 0;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const double d = std::fabs(goal_real_deg[i] - cur_real_deg[i]);
    if (d > max_delta) { max_delta = d; max_axis = i; }
  }
  std::printf(
    "  이동  최대 축 %s  %.3f°  (상한 %.1f°)\n",
    kUrdfJointNames[max_axis].c_str(), max_delta, opt.max_delta);

  if (max_delta < kMinDeltaDeg) {
    std::fprintf(
      stderr,
      "✗ 최대 각변위 %.4f° 가 %.2f° 미만이다 — 목표가 사실상 현재 자세다. 보낼 것이 없다\n",
      max_delta, kMinDeltaDeg);
    return 4;
  }
  if (max_delta > opt.max_delta) {
    if (!opt.force) {
      std::fprintf(
        stderr,
        "✗ 각변위 %.3f° 가 상한 %.1f° 를 넘는다 — 거부.\n"
        "   자릿수를 잘못 치면 그대로 그만큼 돈다. "
        "의도한 것이면 --max-delta 를 올리거나 --force 를 명시하라\n",
        max_delta, opt.max_delta);
      return 4;
    }
    std::printf("  ⚠️ --force — 각변위 상한 %.1f° 를 넘겨 보낸다\n", opt.max_delta);
  }

  // ── 게이트 ② 속도 ────────────────────────────────────────────────────────
  if (opt.speed > opt.max_speed) {
    if (!opt.force) {
      std::fprintf(
        stderr,
        "✗ 속도 %.2f°/s 가 상한 %.1f°/s 를 넘는다 — 거부. "
        "--max-speed 를 올리거나 --force 를 명시하라\n",
        opt.speed, opt.max_speed);
      return 4;
    }
    std::printf("  ⚠️ --force — 속도 상한 %.1f°/s 를 넘겨 보낸다\n", opt.max_speed);
  }

  // ── 게이트 ③ 속도 프로파일이 물리적으로 가능한가 ──────────────────────────
  // 2026-08-05 의 150033 "특이점" 은 **오진**이었다. 실제 원인은 가감속 거리 부족이다
  // (hcr5_comm/README.md §8 함정 1). 관절공간에서도 같은 조건인지는 미실측이지만 계산은 공짜다.
  // 이 관문의 판정은 **사다리꼴 기준**으로 남겨 둔다. 실측 램프각은 0.77·v²/a 로 이보다 좁아
  // (아래 kRampFactor 참조) 이 식은 실제로 가능한 이동도 일부 거부한다 — 30% 보수적이다.
  // 안전한 쪽으로 틀린 것이라 그대로 두었다. 통과시키려면 --force.
  const double ramp = opt.speed * opt.speed / opt.accel;      // 가속각 + 감속각 (보수적)
  const double expect_sec = max_delta / opt.speed + kRampFactor * opt.speed / opt.accel + kFixedOverheadSec;
  std::printf(
    "  속도  %.2f°/s · 가속 %.1f°/s² → 가감속에 %.3f° 필요 (구간 %.3f°, 실측 기준으로는 %.3f°)\n"
    "  예상 소요시간 %.2f초   (실측 모델 d/v + %.2f·v/a + %.2f초, 2026-08-16 182점·6축)\n",
    opt.speed, opt.accel, ramp, max_delta, 0.5 * kRampFactor * ramp, expect_sec, kRampFactor,
    kFixedOverheadSec);
  if (ramp > max_delta) {
    const double reachable = std::sqrt(max_delta * opt.accel);
    if (!opt.force) {
      std::fprintf(
        stderr,
        "✗ 구간이 짧아 %.2f°/s 에 도달하지 못한다 — 거부.\n"
        "   이 구간에서 낼 수 있는 최대는 약 %.2f°/s 다. "
        "--speed 를 낮추거나 --force 를 명시하라.\n"
        "   (이 조건이 150033 을 '특이점' 으로 오진하게 만든 그 조건이다)\n",
        opt.speed, reachable);
      return 4;
    }
    std::printf("  ⚠️ --force — 도달 불가능한 속도 프로파일을 그대로 보낸다\n");
  }

  // ── 게이트 ④ 관절 한계 ───────────────────────────────────────────────────
  if (!withinLimits(goal_real_deg)) {
    std::fprintf(
      stderr, "✗ 목표가 실기 공식 가동범위 밖이다 (±360°, J3 ±165°). withinLimits() 거부\n");
    return 4;
  }

  // ── FK — 목표 관절값의 직교 표현을 얻는다 ────────────────────────────────
  // 경로 결정에는 안 쓰인다. 웨이포인트가 tcp·flange·joint 세 표현을 **동시에** 담고
  // 셋이 정합해야 하기 때문에 채우는 것뿐이다 (movej.md §1.2).
  Pose goal_tcp, goal_flange;
  {
    auto fk = [&](const char * pose_type, Pose & out) {
        const Json payload{{"info", {{"joint", goal_real_deg}}}, {"poseType", pose_type}};
        const auto r = mqtt.request(kTopicFk, payload);
        if (!r || r->find("position") == r->end() || r->find("orientation") == r->end()) {
          return false;
        }
        out.pos = readVec(r->at("position"));
        out.ori = readVec(r->at("orientation"));
        return true;
      };

    if (!fk("tcp", goal_tcp)) {
      std::fprintf(stderr, "✗ FK(tcp) 가 해를 못 냈다 (%s) — 도달 불가능한 자세일 수 있다\n", kTopicFk);
      return 3;
    }
    if (!fk("flange", goal_flange)) {
      // ✅ poseType:"flange" 는 2026-08-16 에 **받는 것을 확인했다** (movej.md §1.4) — 이 분기는
      // 평소 안 탄다. 그래도 남겨 둔다: 활성 툴이 세션 사이에 바뀌는 실기(L14)에서 컨트롤러
      // 동작이 바뀔 여지가 있고, 대체 가능 여부를 **지금 측정한 오프셋**으로 판정할 수 있어서다.
      if (tcp_offset <= kPoseSameTolMm) {
        goal_flange = goal_tcp;
        std::printf(
          "  ⚠️ FK(flange) 가 거부됐다. 다만 현재 tcp↔flange 오프셋이 %.4f mm (≤%.2f) 라\n"
          "     지금 활성 툴에서는 두 점이 같다 — tcp 해를 flange 칸에도 넣는다\n",
          tcp_offset, kPoseSameTolMm);
      } else {
        std::fprintf(
          stderr,
          "✗ FK(flange) 가 거부됐고 tcp↔flange 오프셋이 %.3f mm 라 대체할 수 없다.\n"
          "   %s 가 poseType:\"flange\" 를 안 받는 것으로 보인다 — 확인 후 코드를 고쳐야 한다.\n"
          "   (틀린 flange 좌표를 채워 보내면 컨트롤러가 어느 표현을 쓰는지에 따라 "
          "조용히 다른 자세로 갈 수 있다)\n",
          tcp_offset, kTopicFk);
        return 3;
      }
    }
  }
  std::printf(
    "  FK   목표 tcp    [%9.3f %9.3f %9.3f]  rot [%8.3f %8.3f %8.3f]\n"
    "       목표 flange [%9.3f %9.3f %9.3f]  rot [%8.3f %8.3f %8.3f]\n",
    goal_tcp.pos[0], goal_tcp.pos[1], goal_tcp.pos[2],
    goal_tcp.ori[0], goal_tcp.ori[1], goal_tcp.ori[2],
    goal_flange.pos[0], goal_flange.pos[1], goal_flange.pos[2],
    goal_flange.ori[0], goal_flange.ori[1], goal_flange.ori[2]);
  std::printf(
    "  참고 플랜지 직선거리 %.3f mm — **이 경로가 그 직선을 그린다는 뜻이 아니다.**\n"
    "       관절공간 보간이라 플랜지는 일반적으로 호를 그린다. 직선이 필요하면 movel 을 쓴다\n",
    norm(sub(goal_flange.pos, cur_flange.pos)));

  if (opt.dry_run) {
    std::printf("  --dry-run — 발행하지 않는다\n");
    return 0;
  }

  // ── 게이트 ⑤ 서보 상태 ───────────────────────────────────────────────────
  // 여기까지의 게이트는 전부 **내가 보낼 값**만 봤다. 이 칸이 **로봇이 받을 준비**를 본다.
  // `--plan-only` 는 적재만 하고 program/play 를 안 쏘므로 서보가 필요 없다 (근거는 servo_gate.hpp).
  if (!opt.plan_only) {
    std::string servo;
    if (!waitServoOn(mqtt, servo)) {
      std::fprintf(
        stderr,
        "✗ 서보가 켜져 있지 않다 (status/operation = %s) — 발행하지 않는다.\n"
        "   이 게이트가 없으면 clear·plan·play 세 ack 가 전부 code:0 으로 정상인데 로봇은\n"
        "   1mm 도 안 움직이고, 타임아웃 %.0f초를 다 기다린 뒤에야 죽는다 (2026-08-16 실측).\n"
        "   켜려면: python3 tools/mqtt_cmd.py servo on\n",
        servo.empty() ? "응답 없음" : servo.c_str(), opt.timeout_sec);
      return 4;
    }
    std::printf("  서보  %s\n", servo.c_str());
  }

  // ── 도착 감시를 **발행 전에** 건다 ───────────────────────────────────────
  // ack 는 접수일 뿐 도착이 아니다. 도착은 program/end · event/motion 이 알린다.
  // 구독을 발행 뒤에 걸면 짧은 이동에서 신호를 놓친다.
  std::mutex m;
  std::condition_variable cv;
  bool done = false;
  std::string done_by;
  auto mark = [&](const char * who) {
      { std::lock_guard<std::mutex> lk(m); if (done) { return; } done = true; done_by = who; }
      cv.notify_all();
    };
  mqtt.subscribe(kTopicProgramEnd, [&](const std::string &, const Json &) { mark("program/end"); });
  mqtt.subscribe(kTopicEventMotion, [&](const std::string &, const Json &) { mark("event/motion"); });

  bool fault = false;
  std::string fault_msg;
  auto on_error = [&](const std::string & topic, const Json & j) {
      std::lock_guard<std::mutex> lk(m);
      if (!fault) { fault = true; fault_msg = topic + " " + j.dump(); }
      cv.notify_all();
    };
  mqtt.subscribe("error/command", on_error);
  mqtt.subscribe("error/event", on_error);
  mqtt.subscribe("event/collision", on_error);

  // ── 적재 · 실행 ──────────────────────────────────────────────────────────
  if (!mqtt.request(kTopicClear, Json::object())) {
    std::fprintf(stderr, "✗ %s ack 가 없다\n", kTopicClear);
    return 5;
  }
  std::printf("  → %s ack\n", kTopicClear);

  const Json plan = buildPlan(
    cur_tcp, cur_flange, cur_real_deg, goal_tcp, goal_flange, goal_real_deg, opt);
  if (!mqtt.request(kTopicPlan, plan, std::chrono::milliseconds(5000))) {
    std::fprintf(
      stderr,
      "✗ %s 가 거부됐다 (ack 없음 또는 code≠0) — 프로그램 트리 형식을 확인하라\n", kTopicPlan);
    return 5;
  }
  std::printf("  → %s ack — MOVE(joint) 노드 1개 적재 (%zu bytes)\n",
    kTopicPlan, plan.dump().size());

  if (opt.plan_only) {
    // ⚠️ "펜던트에서 확인하라" 고 안내하면 안 된다 — 2026-08-16 실측으로 **안 보이는 것**이
    // 확인됐다. 실행 버퍼와 펜던트 파일 저장소(HTW Storage)는 다른 층이다.
    std::printf(
      "  --plan-only — %s 를 쏘지 않는다. 로봇은 안 움직인다.\n"
      "  ⚠️ 이 프로그램은 펜던트 파일 목록에 안 뜬다 — 실행 버퍼와 파일 저장소는 다른 층이다.\n"
      "     적재 확인은 위 ack 와 status/program(PROGRAM_STATE_INIT)으로 한다.\n"
      "     지우려면 %s 를 쏜다.\n", kTopicPlay, kTopicClear);
    return 0;
  }

  const auto t_play = std::chrono::steady_clock::now();
  if (!mqtt.request(kTopicPlay, Json::object())) {
    std::fprintf(stderr, "✗ %s ack 가 없다 — 서보가 켜져 있는지 확인하라\n", kTopicPlay);
    return 5;
  }
  std::printf("  → %s ack — 실행 시작. ⚠️ 로봇이 움직인다\n", kTopicPlay);

  // ── 도착 대기 ────────────────────────────────────────────────────────────
  {
    std::unique_lock<std::mutex> lk(m);
    const bool ok = cv.wait_for(
      lk, std::chrono::duration<double>(opt.timeout_sec), [&] { return done || fault; });

    if (fault) {
      lk.unlock();
      std::fprintf(stderr, "✗ 실행 중 이상 신호: %s\n", fault_msg.c_str());
      mqtt.publish(kTopicStop, Json::object());
      mqtt.publish(kTopicMoveStop, Json::object());
      std::fprintf(stderr, "   %s · %s 발행\n", kTopicStop, kTopicMoveStop);
      return 6;
    }
    if (!ok) {
      lk.unlock();
      std::fprintf(stderr, "✗ %.1f초 안에 도착 신호가 없다 — 정지시킨다\n", opt.timeout_sec);
      mqtt.publish(kTopicStop, Json::object());
      mqtt.publish(kTopicMoveStop, Json::object());
      std::fprintf(stderr, "   %s · %s 발행. 자세는 `mqtt_cmd.py pos` 로 확인하라\n",
        kTopicStop, kTopicMoveStop);
      return 6;
    }
    std::printf("  ← 도착 (%s)\n", done_by.c_str());
  }
  const double actual_sec =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t_play).count();

  // ── 시간 회귀 감시 — 단위·프로파일 모두 확정됐고, 이제 이 표시는 감시용이다 ─
  // 확정 근거(2026-08-16 실측 182점·6축 전부, movej.md §2.3): 거리·속도·가속도·시각을 따로 흔들어
  //   · 초과분이 **거리와 무관**  → 주행항은 d/v 그대로, 지령 속도는 지켜진다 (K = 0.9987±0.0019)
  //   · 초과분이 **속도에 비례**  → 램프 시간이 v/a 의 1.54배. 사다리꼴이 아니다
  //   · a 를 25~400 으로 흔들어 v/a 를 16배 벌려도 같은 계수 → 지령 가속도는 실제로 먹는다
  //   · 여섯 축이 같은 식을 따르고 중력 방향(올림/내림)도 차이가 없다 → 축별 보정 불필요
  //   · 같은 지령 80회를 29분에 걸쳐 돌려도 시간에 끌리지 않는다 → c 는 진짜 상수다 (L26)
  // 위 expect_sec 이 그 모델이므로 **비율은 1.00 근처**여야 정상이다 (실측 최대 잔차 0.14초).
  // 벗어나면 컨트롤러 설정이 바뀐 것이다. 전역 배율을 먼저 의심하라 — `get/velocity` 가 1 이
  // 아니면 노드 velocity 에 그것이 곱해진다(hcr5_comm/README.md §6.2). 배율 0.5 면 주행항이 두 배다.
  {
    const double ratio = actual_sec / expect_sec;
    const double resid = actual_sec - expect_sec;
    std::printf(
      "  시간  실측 %.2f초 / 예상 %.2f초 = %.3f  (잔차 %+.2f초)\n", actual_sec, expect_sec, ratio,
      resid);
    if (std::fabs(resid) < 0.20) {
      std::printf("        → 정상. 실측 모델(도/s · 램프 1.54·v/a)이 그대로 유지되고 있다\n");
    } else if (resid > 0.0) {
      std::printf(
        "     ⚠️ 모델보다 **%.2f초 느렸다**. 전역 배율(`get/velocity`)이 1 인지 먼저 확인하라 —\n"
        "        1 이 아니면 노드 velocity 에 그 값이 곱해져 주행항이 %.2f배로 늘어난다.\n"
        "        배율이 1 인데도 이 값이면 컨트롤러 가감속 프로파일이 바뀐 것이다\n",
        resid, ratio);
    } else {
      std::printf(
        "     ⚠️ 모델보다 **%.2f초 빨랐다** — 확정된 도/s 해석과 어긋나는 방향이다.\n"
        "        추가 이동 전에 --speed 를 낮추고 컨트롤러 설정을 확인하라\n",
        -resid);
    }
    if (actual_sec < 1.0) {
      std::printf(
        "     ⚠️ 실측이 1초 미만이라 고정 오버헤드(%.2f초)가 비율을 지배한다 — 감시로 쓰지 마라\n",
        kFixedOverheadSec);
    }
  }

  // ── 도착 검증 — 신호를 믿지 않고 실제 자세를 되읽는다 ─────────────────────
  {
    const auto r = mqtt.request(kTopicGetPos, Json::object());
    if (!r) {
      std::printf("  ⚠️ 도착 자세를 못 읽었다 — `mqtt_cmd.py pos` 로 확인하라\n");
      return 0;
    }
    const auto jt = r->find("joint");
    if (jt == r->end() || !jt->is_array() || jt->size() != kNumJoints) {
      std::printf("  ⚠️ 도착 응답에 joint 6개가 없다 — `mqtt_cmd.py pos` 로 확인하라\n");
      return 0;
    }
    double worst = 0.0;
    std::size_t worst_axis = 0;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double e = std::fabs((*jt)[i].get<double>() - goal_real_deg[i]);
      if (e > worst) { worst = e; worst_axis = i; }
    }
    std::printf(
      "  실제  최대 오차 %s %.4f° (목표 대비)\n", kUrdfJointNames[worst_axis].c_str(), worst);
  }
  return 0;
}
