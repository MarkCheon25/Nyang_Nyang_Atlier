// movel — 직교 좌표 **직선** 이동. 점 A 에서 점 B 로 플랜지가 직선을 그리며 한 번에 간다.
//
//   ros2 run hcr5_bridge movel <좌표계> <x> <y> <z> [옵션]     (mm)
//
// 보간을 **컨트롤러가 소유한다** — `program/plan` 의 `move.selected:"linear"` 노드 하나를
// 적재하고 `program/play` 로 실행한다. PC 는 목표만 주고 궤적을 만들지 않는다.
// 실측 직선 이탈 0.045mm / 179.3mm 구간으로 로봇 반복정밀도(±0.1mm)보다 좋다
// (hcr5_comm/README.md §6.2). 설계 근거 전체는 movel.md.
//
// ⚠️ **ros2_control 스택과 배타다.** `program/play` 는 컨트롤러가 자기 궤적을 실행하는 것이고
// `bringup.launch.py` 는 JTC 가 주기적으로 관절 지령을 쓴다 — 둘이 동시에 돌면 명령이 부딪힌다
// (README §1.2). 그래서 이 실행기는 `servo` 와 같이 **rclcpp 의존이 없다.** 스택이 죽어 있어도
// 돌고, 스택이 살아 있으면 **띄우면 안 된다.**
//
// 종전 구현(2026-08-15까지)은 JTC 경로였고 플랜지 경로가 직선이 아니었다 — 시작·끝만 직교로
// 주고 사이는 관절 보간이라 일반적으로 호(곡선)가 됐다. 이름과 동작이 어긋나 있었고
// 두산 `movel`(컨트롤러가 직선 보간을 소유) 대조가 그것을 드러냈다 (movel.md §3).
//
// 📎 `program/plan` 봉투의 **원본은 `hcr5_comm/README.md` §6** 이다. 이 파일과 `movej.cpp`
// (2026-08-16 부터 같은 경로)는 그 문서를 각자 구현한 두 벌이고, 다른 것은 `move.selected` 와
// 웨이포인트를 채우는 방법뿐이다. 한쪽을 고치면 다른 쪽도 봐야 한다 — 합치는 것은 movej.md §4.
// ⚠️ 특히 `poseTriple()` 이 여기서는 tcp·flange 에 **같은 pose 를 복사**한다. movej 는 FK 를 두 번
// 불러 양쪽을 각각 채운다 — 그쪽이 옳다. 이 파일의 이관은 movej.md §4 의 후속 항목이다.

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
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
using hcr5_bridge::Mat3;
using hcr5_bridge::MqttClient;
using hcr5_bridge::Vec3;
using hcr5_bridge::kNumJoints;

constexpr const char * kTopicGetPos = "get/command/pos";
constexpr const char * kTopicIk = "robot/convertJointAngle";
constexpr const char * kTopicClear = "program/clear";
constexpr const char * kTopicPlan = "program/plan";
constexpr const char * kTopicPlay = "program/play";
constexpr const char * kTopicStop = "program/stop";
constexpr const char * kTopicMoveStop = "move/stop";
constexpr const char * kTopicProgramEnd = "program/end";
constexpr const char * kTopicEventMotion = "event/motion";

/// 선속도 기본값(mm/s). 컨트롤러 기본은 500 이지만 그 값이 2026-08-05 사고의 한 축이었다
/// (hcr5_comm/README.md §8 함정 1 — 50 으로 낮추자 무에러 완주). 실기 앞에서는 느린 쪽이 기본이다.
constexpr double kDefaultSpeedMmPerSec = 50.0;
/// 가속도 기본값(mm/s²). §6.1 의 linear 기본값과 같다.
constexpr double kDefaultAccelMmPerSec2 = 1000.0;
/// 직교 이동거리 상한(mm). movel.md §2.4 — 상대이동에서 `10` 을 `100` 으로 오타 내면
/// 그대로 10배를 가는데 관절한계·각속도 게이트는 **직교 거리를 안 본다**. 이 칸이 그 구멍이다.
constexpr double kDefaultMaxDistMm = 100.0;
/// 이보다 짧으면 이동으로 치지 않는다(mm). 로봇 반복정밀도가 ±0.1mm 라 그 아래는 의미가 없고,
/// `base` 로 현재 좌표를 그대로 주면 부동소수 찌꺼기(1e-4mm)가 남아 **속도 게이트에 먼저 걸려**
/// "속도가 안 나온다" 는 엉뚱한 이유로 거부된다 — 그 오독을 막는 자리다.
constexpr double kMinDistMm = 0.01;

struct Options
{
  std::string frame;                                ///< base | world | tool
  Vec3 xyz{};
  double speed = kDefaultSpeedMmPerSec;
  double accel = kDefaultAccelMmPerSec2;
  double max_dist = kDefaultMaxDistMm;
  double timeout_sec = 60.0;
  bool force = false;
  bool dry_run = false;
  bool plan_only = false;                           ///< 적재까지만 — program/play 를 안 쏜다
  std::string pose_type = "flange";                 ///< 제어점 — flange | tcp
  std::string host = "192.168.0.20";
  int port = 1883;
};

void usage()
{
  std::printf(
    "사용법:\n"
    "  ros2 run hcr5_bridge movel <좌표계> <x> <y> <z> [옵션]        단위 mm\n"
    "\n"
    "좌표계 — **기준 원점**을 고른다 (제어점을 고르는 것이 아니다)\n"
    "  base    실기 base 절대        목표 = (x, y, z)\n"
    "  world   base 축 방향 상대 Δ   목표 = 현재 + (x, y, z)\n"
    "  tool    현재 툴 축 기준 상대 Δ 목표 = 현재 + R·(x, y, z)\n"
    "\n"
    "자세(rx·ry·rz)는 인자에 없다 — **현재 자세를 그대로 유지한다**.\n"
    "경로는 플랜지 기준 **직선**이다 (컨트롤러가 보간을 소유한다).\n"
    "\n"
    "⚠️ ros2_control 스택(bringup.launch.py)과 **동시에 쓰지 않는다** — 명령이 부딪힌다.\n"
    "⚠️ 서보가 꺼져 있으면 발행 전에 죽는다 — `mqtt_cmd.py servo on` 으로 켠다.\n"
    "\n"
    "옵션\n"
    "  --speed V      선속도 mm/s. 기본 %.0f (컨트롤러 기본 500 은 사고 이력이 있다)\n"
    "  --accel A      가속도 mm/s². 기본 %.0f\n"
    "  --max-dist D   직교 이동거리 상한 mm. 기본 %.0f — 넘으면 거부\n"
    "  --tcp          제어점을 TCP 로 (기본은 flange)\n"
    "  --timeout S    도착 대기 상한 초. 기본 60 — 넘으면 move/stop 을 쏜다\n"
    "  --force        거리·속도프로파일 게이트를 넘겨 보낸다\n"
    "  --dry-run      목표와 IK 해까지만 보고 아무것도 발행하지 않는다\n"
    "  --plan-only    프로그램 적재까지만 하고 program/play 를 안 쏜다 — 로봇이 안 움직인다\n"
    "  --host H       MQTT 브로커. 기본 192.168.0.20\n"
    "  --port P       기본 1883\n"
    "\n"
    "예\n"
    "  movel world 0 0 20 --dry-run        수직 20mm ↑ (계산만)\n"
    "  movel tool 0 0 -10                  펜 방향으로 10mm\n"
    "  movel base 490 -170.5 441.5         홈 flange 자리로 절대이동\n",
    kDefaultSpeedMmPerSec, kDefaultAccelMmPerSec2, kDefaultMaxDistMm);
}

/// 인자 파싱. 모르는 `--` 플래그는 실패로 본다 — 오타를 조용히 무시하면 게이트가 헐거워진다.
bool parse(const std::vector<std::string> & args, Options & opt, std::string & err)
{
  std::vector<std::string> pos;
  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string & a = args[i];
    auto value = [&](double & dst) {
        if (i + 1 >= args.size()) { err = a + " 에 값이 없다"; return false; }
        try { dst = std::stod(args[++i]); } catch (...) { err = a + " 값이 숫자가 아니다"; return false; }
        return true;
      };
    if (a == "--force") { opt.force = true; }
    else if (a == "--dry-run") { opt.dry_run = true; }
    else if (a == "--plan-only") { opt.plan_only = true; }
    else if (a == "--tcp") { opt.pose_type = "tcp"; }
    else if (a == "--flange") { opt.pose_type = "flange"; }
    else if (a == "--speed") { if (!value(opt.speed)) { return false; } }
    else if (a == "--accel") { if (!value(opt.accel)) { return false; } }
    else if (a == "--max-dist") { if (!value(opt.max_dist)) { return false; } }
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

  if (pos.size() != 4) {
    err = "<좌표계> <x> <y> <z> 4개가 필요하다 (받은 위치인자 " + std::to_string(pos.size()) + "개)";
    return false;
  }
  opt.frame = pos[0];
  if (opt.frame != "base" && opt.frame != "world" && opt.frame != "tool") {
    err = "좌표계가 '" + opt.frame + "' 다. base | world | tool 중 하나여야 한다";
    return false;
  }
  for (std::size_t i = 0; i < 3; ++i) {
    try {
      opt.xyz[i] = std::stod(pos[i + 1]);
    } catch (...) {
      err = "좌표 " + std::to_string(i + 1) + "번째 값이 숫자가 아니다: " + pos[i + 1];
      return false;
    }
  }
  if (opt.speed <= 0.0 || opt.accel <= 0.0) { err = "--speed · --accel 은 양수여야 한다"; return false; }
  if (opt.max_dist <= 0.0) { err = "--max-dist 는 양수여야 한다"; return false; }
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

/// 한 점의 세 표현 묶음 — tcp·flange·joint.
/// 웨이포인트는 이 셋을 **동시에** 담는다 (§6.1.2). 셋이 서로 어긋나면 컨트롤러가 어느 것을
/// 쓰는지 알 수 없으므로 전부 **같은 점**으로 채운다 — joint 는 IK RPC 가 준 해이고
/// 왕복 FK 오차 0.000mm 로 확인했다.
///
/// 🔴 **이 전제는 2026-08-16 에 깨졌다 — 실기 사용 정지 중.**
/// 위 논리는 컨트롤러 툴 오프셋이 **0** 이라 tcp 와 flange 가 실제로 같은 점일 때만 성립했다.
/// 펜 TCP `(−3.675, −1.725, 120.560)` 가 등록되면서 둘은 **120.628mm** 떨어졌고,
/// 이 함수는 이제 물리적으로 불가능한 조합을 싣는다. 컨트롤러가 어느 슬롯을 채택하는지 미확인.
/// 고치려면 슬롯별로 제 좌표계 값을 넣어야 한다 — 판정·해제 조건은 `movel.md` 머리말.
Json poseTriple(const Vec3 & pos, const Vec3 & ori, const std::array<double, kNumJoints> & joint)
{
  const Json pose{{"position", writeVec(pos)}, {"orientation", writeVec(ori)}};
  return Json{{"tcp", pose}, {"flange", pose}, {"joint", joint}};
}

/// 웨이포인트 한 칸. `relative` 블록은 `selected:"fixed"` 일 때도 **늘 실린다**
/// (2026-08-16 펜던트 캡처 확인) — 빼면 컨트롤러 파서가 어떻게 반응하는지 모르므로 그대로 채운다.
Json waypointSlot(
  const Vec3 & pos, const Vec3 & ori, const std::array<double, kNumJoints> & joint,
  const Json & from, bool result_zero)
{
  const Json here = poseTriple(pos, ori, joint);
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

/// program/plan 페이로드 — MOVE 노드 하나짜리 최소 프로그램.
/// 노드 하나 = 구간 하나이므로 점 A→B 를 분할 없이 쭉 한 번에 간다 (movel.md §1.2).
/// 형식의 원본은 2026-08-16 펜던트 캡처다 (세션 260816-컨스터블).
Json buildPlan(
  const Vec3 & cur_pos, const Vec3 & cur_ori, const std::array<double, kNumJoints> & cur_joint,
  const Vec3 & goal_pos, const Vec3 & goal_ori,
  const std::array<double, kNumJoints> & goal_joint, const Options & opt)
{
  const Json from = poseTriple(cur_pos, cur_ori, cur_joint);

  // 네 종류가 모두 실리고 `selected` 가 그중 하나를 고른다 (§6.1.1).
  // 안 고른 셋은 펜던트 기본값 그대로 둔다 — 우리가 건드릴 이유가 없다.
  const Json move{
    {"selected", "linear"},
    {"linear", {
       // ⚠️ radius 는 0 이어야 한다. radius=50 이면 궤적이 그 웨이포인트를 22.3mm
       // 떨어져 지나간다 (§6.2 실측). 선 모양을 지키는 유일한 조합이다.
       {"radius", 0},
       {"velocity", opt.speed},
       {"acceleration", opt.accel},
       // 구간이 하나뿐이라 시작·끝 모두 정지다. 스트로크 체인(§6.3)은 이 계층 밖이다.
       {"continues", false},
       {"startVelocity", 0},
       {"endVelocity", 0},
       {"repeat", 1}}},
    {"joint", {
       {"velocity", 50}, {"acceleration", 100}, {"continues", false},
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
    {"name", "MOVEL"},
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
          // middlePoint 는 linear 에서 쓰이지 않는다(arc·circle 용). 캡처와 같이
          // 현재 자세로 채워 둔다 — 비워서 파서가 어떻게 되는지는 모른다.
          {"middlePoint", waypointSlot(cur_pos, cur_ori, cur_joint, from, false)},
          {"endPoint", waypointSlot(goal_pos, goal_ori, goal_joint, from, true)}}}}}};

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
  plan["name"] = "hcr5_bridge_movel";
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
  std::string err;
  if (!parse(args, opt, err)) {
    std::fprintf(stderr, "✗ %s\n\n", err.c_str());
    usage();
    return 2;
  }

  MqttClient mqtt(opt.host, opt.port, "hcr5-movel");
  if (!mqtt.start()) {
    std::fprintf(stderr, "✗ MQTT 접속 실패: %s:%d\n", opt.host.c_str(), opt.port);
    return 3;
  }

  // ── 현재 자세 — 기준은 **MQTT** 다 ────────────────────────────────────────
  // ROS `/joint_states` 가 아니다. 이 경로는 스택을 안 타므로 그 토픽이 아예 없다 (movel.md §1.3).
  Vec3 cur_pos{}, cur_ori{};
  std::array<double, kNumJoints> seed_real_deg{};
  {
    const auto r = mqtt.request(kTopicGetPos, Json::object());
    if (!r) {
      std::fprintf(stderr, "✗ %s 응답이 없다 — 실기가 켜져 있고 랜이 붙었는지 확인하라\n", kTopicGetPos);
      return 3;
    }
    const auto jt = r->find("joint");
    const auto pt = r->find(opt.pose_type);
    if (jt == r->end() || !jt->is_array() || jt->size() != kNumJoints || pt == r->end()) {
      std::fprintf(stderr, "✗ %s 응답에 joint 6개 또는 %s 가 없다\n", kTopicGetPos, opt.pose_type.c_str());
      return 3;
    }
    for (std::size_t i = 0; i < kNumJoints; ++i) { seed_real_deg[i] = (*jt)[i].get<double>(); }
    cur_pos = readVec(pt->at("position"));
    cur_ori = readVec(pt->at("orientation"));
  }

  // ── 목표 계산 — 좌표계 토큰은 **기준 원점**을 고른다 ──────────────────────
  // 자세는 건드리지 않는다. 받은 orientation 을 그대로 되돌려 준다 (movel.md §2.3).
  Vec3 goal_pos{};
  if (opt.frame == "base") {
    goal_pos = opt.xyz;
  } else if (opt.frame == "world") {
    goal_pos = add(cur_pos, opt.xyz);
  } else {  // tool — 툴 축 방향으로 Δ 를 민다. R 은 2026-08-16 실측 확정 규약(pose_convention.hpp).
    const Mat3 R = rotationFromRealDeg(cur_ori);
    goal_pos = add(cur_pos, applyRotation(R, opt.xyz));
  }
  const Vec3 goal_ori = cur_ori;
  const Vec3 delta = sub(goal_pos, cur_pos);
  const double dist = norm(delta);

  std::printf(
    "movel — 좌표계 %s · 제어점 %s%s\n"
    "  현재  pos(mm) [%9.3f %9.3f %9.3f]  rot(도) [%9.3f %9.3f %9.3f]\n"
    "  목표  pos(mm) [%9.3f %9.3f %9.3f]  rot(도) [%9.3f %9.3f %9.3f]  ← 자세 유지\n"
    "  이동  Δ(mm)   [%9.3f %9.3f %9.3f]  거리 %.3f mm\n",
    opt.frame.c_str(), opt.pose_type.c_str(), opt.dry_run ? " · --dry-run" : "",
    cur_pos[0], cur_pos[1], cur_pos[2], cur_ori[0], cur_ori[1], cur_ori[2],
    goal_pos[0], goal_pos[1], goal_pos[2], goal_ori[0], goal_ori[1], goal_ori[2],
    delta[0], delta[1], delta[2], dist);

  // ── 게이트 ① 직교 이동거리 ───────────────────────────────────────────────
  // 기존 게이트(관절한계·각속도)가 **직교 거리를 안 보는** 구멍을 메운다 (movel.md §2.4).
  if (dist < kMinDistMm) {
    std::fprintf(
      stderr,
      "✗ 이동거리 %.4f mm 는 %.2f mm 미만이다 — 목표가 사실상 현재 위치다. 보낼 것이 없다\n",
      dist, kMinDistMm);
    return 4;
  }
  if (dist > opt.max_dist) {
    if (!opt.force) {
      std::fprintf(
        stderr,
        "✗ 이동거리 %.3f mm 가 상한 %.1f mm 를 넘는다 — 거부.\n"
        "   상대이동에서 자릿수를 잘못 치면 그대로 그만큼 간다. "
        "의도한 것이면 --max-dist 를 올리거나 --force 를 명시하라\n",
        dist, opt.max_dist);
      return 4;
    }
    std::printf("  ⚠️ --force — 이동거리 상한 %.1f mm 를 넘겨 보낸다\n", opt.max_dist);
  }

  // ── 게이트 ② 속도 프로파일이 물리적으로 가능한가 ──────────────────────────
  // 2026-08-05 의 150033 "특이점" 은 **오진**이었다. 실제 원인은 가감속 거리 부족이다
  // (hcr5_comm/README.md §8 함정 1). 자세를 의심하기 전에 여기를 먼저 계산한다.
  const double ramp = opt.speed * opt.speed / opt.accel;   // 가속거리 + 감속거리
  std::printf(
    "  속도  %.1f mm/s · 가속 %.1f mm/s² → 가감속에 %.2f mm 필요 (구간 %.3f mm)\n",
    opt.speed, opt.accel, ramp, dist);
  if (ramp > dist) {
    const double reachable = std::sqrt(dist * opt.accel);
    if (!opt.force) {
      std::fprintf(
        stderr,
        "✗ 구간이 짧아 %.1f mm/s 에 도달하지 못한다 — 거부.\n"
        "   이 구간에서 낼 수 있는 최대는 약 %.1f mm/s 다. "
        "--speed 를 낮추거나 --force 를 명시하라.\n"
        "   (이 조건이 150033 을 '특이점' 으로 오진하게 만든 그 조건이다)\n",
        opt.speed, reachable);
      return 4;
    }
    std::printf("  ⚠️ --force — 도달 불가능한 속도 프로파일을 그대로 보낸다\n");
  }

  // ── IK — 로봇 컨트롤러가 푼다 ────────────────────────────────────────────
  // 우리 쪽 IK 를 새로 세우지 않는 이유는, 실기가 실제로 쓰는 해와 어긋나면
  // 조용히 다른 자세로 가기 때문이다. 시드는 현재 자세 — 시드가 나쁘면 먼 해를 준다.
  std::array<double, kNumJoints> goal_real_deg{};
  {
    const Json payload{
      {"info", {
         {"position", writeVec(goal_pos)},
         {"orientation", writeVec(goal_ori)},
         {"joint", seed_real_deg}}},
      {"poseType", opt.pose_type}};
    const auto r = mqtt.request(kTopicIk, payload);
    if (!r) {
      std::fprintf(
        stderr, "✗ IK 가 해를 못 냈다 — 도달 불가능한 점이거나 특이점일 수 있다 (%s)\n", kTopicIk);
      return 3;
    }
    const auto it = r->find("joint");
    if (it == r->end() || !it->is_array() || it->size() != kNumJoints) {
      std::fprintf(stderr, "✗ IK 응답에 joint 6개가 없다\n");
      return 3;
    }
    for (std::size_t i = 0; i < kNumJoints; ++i) { goal_real_deg[i] = (*it)[i].get<double>(); }
  }

  std::printf("  IK 해 (시드 = 현재 자세)\n");
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    std::printf(
      "    %-10s %10.4f°   (현재 대비 %+8.3f°)\n",
      kUrdfJointNames[i].c_str(), goal_real_deg[i], goal_real_deg[i] - seed_real_deg[i]);
  }

  // ── 게이트 ③ 관절 한계 ───────────────────────────────────────────────────
  if (!withinLimits(goal_real_deg)) {
    std::fprintf(
      stderr, "✗ IK 해가 실기 공식 가동범위 밖이다 (±360°, J3 ±165°). withinLimits() 거부\n");
    return 4;
  }

  if (opt.dry_run) {
    std::printf("  --dry-run — 발행하지 않는다\n");
    return 0;
  }

  // ── 게이트 ⑤ 서보 상태 ───────────────────────────────────────────────────
  // 앞의 게이트는 전부 **내가 보낼 값**만 봤다. 이 칸이 **로봇이 받을 준비**를 본다.
  // movel 은 서보가 켜진 세션에서만 돌려봐서 이 구멍이 안 드러났었다 — 근거는 servo_gate.hpp.
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

  const Json plan =
    buildPlan(cur_pos, cur_ori, seed_real_deg, goal_pos, goal_ori, goal_real_deg, opt);
  if (!mqtt.request(kTopicPlan, plan, std::chrono::milliseconds(5000))) {
    std::fprintf(
      stderr,
      "✗ %s 가 거부됐다 (ack 없음 또는 code≠0) — 프로그램 트리 형식을 확인하라\n", kTopicPlan);
    return 5;
  }
  std::printf("  → %s ack — MOVE(linear) 노드 1개 적재 (%zu bytes)\n",
    kTopicPlan, plan.dump().size());

  if (opt.plan_only) {
    // ⚠️ "펜던트에서 확인하라" 고 안내하면 안 된다 — 2026-08-16 실측으로 **안 보이는 것**이
    // 확인됐다. program/plan 은 컨트롤러 실행 버퍼에 올라가고, 펜던트 파일 목록(HTW Storage)의
    // `.file` 은 펜던트가 별도로 "적용"할 때 서버에 올리는 다른 층이다. 적재 확인은
    // ack(위에서 이미 받았다) 와 status/program → PROGRAM_STATE_INIT 으로 한다.
    std::printf(
      "  --plan-only — %s 를 쏘지 않는다. 로봇은 안 움직인다.\n"
      "  ⚠️ 이 프로그램은 펜던트 파일 목록에 안 뜬다 — 실행 버퍼와 파일 저장소는 다른 층이다.\n"
      "     적재 확인은 위 ack 와 status/program(PROGRAM_STATE_INIT)으로 한다.\n"
      "     지우려면 %s 를 쏜다.\n", kTopicPlay, kTopicClear);
    return 0;
  }

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
      std::fprintf(
        stderr, "✗ %.1f초 안에 도착 신호가 없다 — 정지시킨다\n", opt.timeout_sec);
      mqtt.publish(kTopicStop, Json::object());
      mqtt.publish(kTopicMoveStop, Json::object());
      std::fprintf(stderr, "   %s · %s 발행. 자세는 `mqtt_cmd.py pos` 로 확인하라\n",
        kTopicStop, kTopicMoveStop);
      return 6;
    }
    std::printf("  ← 도착 (%s)\n", done_by.c_str());
  }

  // ── 도착 검증 — 신호를 믿지 않고 실제 자세를 되읽는다 ─────────────────────
  {
    const auto r = mqtt.request(kTopicGetPos, Json::object());
    if (!r || r->find(opt.pose_type) == r->end()) {
      std::printf("  ⚠️ 도착 자세를 못 읽었다 — `mqtt_cmd.py pos` 로 확인하라\n");
      return 0;
    }
    const Vec3 got = readVec(r->at(opt.pose_type).at("position"));
    const double e = norm(sub(got, goal_pos));
    std::printf(
      "  실제  pos(mm) [%9.3f %9.3f %9.3f]  목표와의 오차 %.3f mm\n",
      got[0], got[1], got[2], e);
  }
  return 0;
}
