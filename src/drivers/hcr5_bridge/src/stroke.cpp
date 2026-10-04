// stroke — 점 열(列)을 **한 프로그램**으로 이어 그린다. 소묘 스트로크의 본 경로.
//
//   ros2 run hcr5_bridge stroke <경로.json> [옵션]
//
// `movel` 과 같은 문(`program/plan`)이지만 **노드 수가 다르다.** movel 은 MOVE 노드 하나 =
// 구간 하나라 점 71개를 그리려면 clear→plan→play 를 70번 돌아야 하고, 매 구간 끝에서
// 속도가 0 을 찍어 선이 점열이 된다. 이 실행기는 노드 N 개를 **한 프로그램**에 실어
// `continues:true` 로 잇는다 — 웨이포인트 전환에서 속도가 19~20mm/s 를 유지한다
// (hcr5_comm/README.md §6.2 실측). 체인 구성은 같은 문서 §6.3 을 그대로 따른다.
//
// ⚠️ **ros2_control 스택과 배타다.** movel·movej 와 같은 이유다 (README §1.2).
// 그래서 이 실행기도 rclcpp 의존이 없다.
//
// 📌 **L28(웨이포인트 tcp·flange 슬롯 모순)을 이 파일은 안 밟는다.** movel 은 한쪽 pose 를
// 양쪽에 복사해서 펜 TCP 오프셋 120.628mm 만큼 모순이 실렸지만, 여기는 `movej` 와 같이
// **IK 를 poseType:"tcp" 로 풀고 FK 를 tcp·flange 각각 불러** 세 슬롯을 정합하게 채운다.
// 컨트롤러가 어느 슬롯을 채택하든 같은 점이므로 "무엇을 쓰는지" 판정이 불필요하다.
//
// 🔴 **z 를 상수로 두는 것은 임시다.** 펜홀더 스프링 행정은 **1.069mm** 뿐이고(260816_펜TCP
// 실측), 작업면 오차가 그보다 크면 어딘가는 박히고 어딘가는 뜬다. 그래서 z 를 xy 의 함수로
// 받는 자리(`--plane`)를 처음부터 열어 뒀다 — 상수 `--z` 는 티치점이 1개인 특수 케이스다.
//
// 📎 `program/plan` 봉투의 원본은 `hcr5_comm/README.md` §6 이다. 이 파일은 `movel.cpp` ·
// `movej.cpp` 에 이은 세 번째 구현이고, 다른 것은 **노드가 여러 개**라는 점뿐이다.
// 한쪽을 고치면 나머지도 봐야 한다.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <fstream>
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
constexpr const char * kTopicIk = "robot/convertJointAngle";
constexpr const char * kTopicFk = "robot/convertPose";
constexpr const char * kTopicClear = "program/clear";
constexpr const char * kTopicPlan = "program/plan";
constexpr const char * kTopicPlay = "program/play";
constexpr const char * kTopicStop = "program/stop";
constexpr const char * kTopicMoveStop = "move/stop";
constexpr const char * kTopicProgramEnd = "program/end";
constexpr const char * kTopicEventMotion = "event/motion";

/// 스트로크 선속도(mm/s). movel 기본 50 보다 낮춘다 — 짧은 구간이 섞인 경로에서
/// 50 은 가감속 거리(v²/a)를 못 채우는 세그먼트를 만든다. 20 이면 세그 0.4mm 면 충분하다.
constexpr double kDefaultSpeedMmPerSec = 20.0;
constexpr double kDefaultAccelMmPerSec2 = 1000.0;

/// 접근·이탈(펜을 들고 오가는 구간)은 그림 품질과 무관하므로 조금 빠르게.
constexpr double kDefaultApproachSpeedMmPerSec = 50.0;

/// 펜을 든 높이(mm). 첫 점 위로 이만큼 올라가서 내려온다.
constexpr double kDefaultHoverMm = 30.0;

/// 세그먼트 길이 상한(mm). movel 의 `--max-dist` 와 같은 자리 — 좌표를 잘못 읽으면
/// 그 길이만큼 그대로 간다. 경로 파일이 통째로 어긋난 것을 여기서 잡는다.
constexpr double kDefaultMaxSegMm = 100.0;

/// 인접 점 사이 관절 변화 상한(도). **이 게이트가 스트로크에서 가장 중요하다** —
/// IK 가 점마다 다른 해 브랜치를 고르면 직교공간으로는 1mm 옆인데 관절은 180° 뒤집힌다.
/// 시드를 이전 점의 해로 주어 대부분 막지만, 막혔는지는 검사해야 안다.
constexpr double kDefaultMaxJointJumpDeg = 20.0;

/// 이보다 짧은 세그먼트는 점을 버려 없앤다(mm).
///
/// 🔴 **이 값이 0.05 였을 때 실기가 `150033` 으로 거부했다** (2026-08-17, 260817-쿠르베).
/// 세 실험이 세그 길이 하나를 가리켰다 — 최단 5.590mm 경로는 20mm/s 로 완주했고,
/// 1.598mm 가 섞인 경로는 같은 속도에서 거부, 5mm/s 로 낮추면 완주는 하되 에러가 떴다.
/// 컨트롤러는 웨이포인트 간격이 너무 좁으면 그 구간의 궤적을 못 만들고, 그것을
/// "there might be singular points" 라고 말한다 — **특이점이 아니다.** 실측 민감도는
/// 최대 0.23 도/mm 로 상한의 1/20 이었다(게이트 ④).
///
/// 원본 경로를 고치지 않고 여기서 흡수한다. 2mm 는 연필 선폭 수준이라 그림이 안 상한다.
constexpr double kDefaultMinSegMm = 2.0;

/// 관절 민감도 상한(도/mm) — **특이점 게이트**.
///
/// 🔴 2026-08-17 (260817-쿠르베) 에 이 게이트가 없어서 `150033
/// CTRLPORT_ERROR_ROBOT_MOTION_IS_NOT_RUNNABLE` 을 맞았다. 관절한계·관절점프 게이트를
/// 다 통과했는데도 컨트롤러가 거부했다 — 두 게이트가 보는 것이 다르기 때문이다:
///
///   - 관절 점프 게이트: 이웃 점의 IK 해가 **얼마나 다른가** (해 브랜치가 바뀌었나)
///   - 이 게이트:        그 차이를 **주어진 직교 속도로 낼 수 있는가**
///
/// 특이점 **근처**에서는 IK 해가 멀쩡히 존재하고 이웃 해와도 가깝다. 그런데 직교공간
/// 1mm 를 가는 데 관절이 10° 돌아야 하면, 20mm/s 로 그으려는 순간 200°/s 가 필요해진다.
/// 즉 **해의 존재가 아니라 야코비안의 조건수**가 문제다 — 점만 봐서는 안 보이고 속도를
/// 곱해야 보인다. 컨트롤러 메시지가 "there might be singular points" 라고 말하는 그것이다.
///
/// 눈금의 근거: HCR-5 공식 최대 관절속도는 180°/s 다. 스트로크 속도 20mm/s 에서
/// 민감도 9 도/mm 면 정확히 180°/s 가 된다. 그 절반을 상한으로 둔다.
constexpr double kDefaultMaxSensitivityDegPerMm = 4.5;

/// 연속 블렌딩(`continues:true`) 체인의 최대 길이. 0 = 무제한.
///
/// 🔴 **컨트롤러 룩어헤드 한계다** (2026-08-17, 260817-쿠르베 실측).
/// 체인이 길면 `program/play` 가 `150033` 으로 거부하는데, 에러의 `params` 가 노드 수·세그
/// 길이를 바꿔도 **늘 `{"0":0,"1":8}`** 이었다 — 8 은 인덱스가 아니라 한계값이다:
///
///   체인 3 → ✅ · 체인 8 → ❌ · 체인 9 → ❌   (전부 같은 params 8)
///
/// 그래서 체인을 이 길이마다 끊는다. 끊는 곳에서 속도가 0을 찍지만 `radius:0` 이라
/// 궤적은 그 웨이포인트를 정확히 통과하므로(§6.2, 통과오차 0.34mm) **선은 안 끊긴다.**
/// 느려질 뿐이다. 룩어헤드 한계를 실측으로 더 좁히면 이 값을 올려도 된다.
constexpr std::size_t kDefaultChain = 6;

/// 실기 최대 관절속도(도/s). 민감도 게이트가 "이 속도면 몇 도/s 가 필요한가" 를
/// 사람에게 보여줄 때 쓰는 기준선이다.
constexpr double kMaxJointSpeedDegPerSec = 180.0;

struct Options
{
  std::string path;
  double z = -19.0;
  double hover = kDefaultHoverMm;
  double speed = kDefaultSpeedMmPerSec;
  double accel = kDefaultAccelMmPerSec2;
  double approach_speed = kDefaultApproachSpeedMmPerSec;
  double max_seg = kDefaultMaxSegMm;
  double max_jump = kDefaultMaxJointJumpDeg;
  double max_sens = kDefaultMaxSensitivityDegPerMm;
  double min_seg = kDefaultMinSegMm;
  std::size_t chain = kDefaultChain;
  std::size_t limit = 0;          ///< 0 = 전부. 노드 수 한계를 탐색할 때 앞 N 점만 쓴다
  bool dry_run = false;
  bool plan_only = false;
  bool force = false;
  std::string host = "192.168.0.20";
  int port = 1883;
  double timeout_sec = 180.0;     ///< 경로 전체를 도는 시간이라 movel 보다 길다
};

void usage()
{
  std::printf(
    "stroke — 점 열을 한 프로그램으로 이어 그린다 (program/plan, MOVE 노드 N개)\n"
    "\n"
    "  ros2 run hcr5_bridge stroke <경로.json> [옵션]\n"
    "\n"
    "경로 파일 — 최상위 \"points\": [[x,y,(z)], ...] 배열. base 좌표계.\n"
    "  단위는 \"units\" 필드(\"m\"|\"mm\")를 보고, 없으면 값 크기로 추정한다.\n"
    "  ⚠️ 점의 z 는 **안 쓴다** — z 는 --z 가 정한다 (아래).\n"
    "\n"
    "옵션\n"
    "  --z Z              펜이 종이에 닿는 z (mm, base, **TCP 기준**). 기본 %.1f\n"
    "  --hover H          펜을 든 높이 mm. 기본 %.0f\n"
    "  --speed V          스트로크 선속도 mm/s. 기본 %.0f\n"
    "  --accel A          가속도 mm/s². 기본 %.0f\n"
    "  --approach-speed V 접근·이탈 속도 mm/s. 기본 %.0f\n"
    "  --max-seg D        세그먼트 길이 상한 mm. 기본 %.0f — 넘으면 거부\n"
    "  --min-seg D        이보다 짧은 세그는 점을 버려 없앤다 mm. 기본 %.1f\n"
    "                     ⚠️ 낮추면 150033 이 난다 — 컨트롤러가 좁은 간격의 궤적을 못 만든다\n"
    "  --max-jump D       인접 점 관절 변화 상한 도. 기본 %.0f — 넘으면 거부\n"
    "  --max-sens D       관절 민감도 상한 도/mm. 기본 %.1f — **특이점 게이트**.\n"
    "                     넘으면 그 점에서 관절속도가 폭발한다 (150033 의 진짜 원인)\n"
    "  --chain N          연속 블렌딩 체인의 최대 길이. 기본 %zu, 0=무제한.\n"
    "                     ⚠️ 컨트롤러 룩어헤드 한계 — 넘기면 150033 이 난다 (실측 8에서 실패)\n"
    "  --limit N          앞 N 점만 쓴다 (0=전부). 한계 탐색·부분 시험용\n"
    "  --dry-run          IK·FK·게이트까지만. 아무것도 발행하지 않는다\n"
    "  --plan-only        program/plan 까지만. play 를 안 쏘므로 **로봇이 안 움직인다**\n"
    "  --force            거리·관절점프 게이트를 넘겨 보낸다\n"
    "  --host H --port P  MQTT. 기본 %s:%d\n"
    "  --timeout S        도착 대기 초. 기본 %.0f\n"
    "\n"
    "⚠️ --dry-run 도 --plan-only 도 안 주면 **로봇이 그림을 그린다.**\n"
    "⚠️ ros2_control 스택(bringup.launch.py)과 배타다 — 띄운 채로 쓰지 마라.\n",
    -19.0, kDefaultHoverMm, kDefaultSpeedMmPerSec, kDefaultAccelMmPerSec2,
    kDefaultApproachSpeedMmPerSec, kDefaultMaxSegMm, kDefaultMinSegMm,
    kDefaultMaxJointJumpDeg, kDefaultMaxSensitivityDegPerMm, kDefaultChain,
    "192.168.0.20", 1883, 180.0);
}

bool parse(const std::vector<std::string> & args, Options & opt, std::string & err)
{
  std::size_t i = 0;
  auto value = [&](auto & dst) {
      if (i + 1 >= args.size()) { err = args[i] + " 에 값이 없다"; return false; }
      try {
        if constexpr (std::is_same_v<std::decay_t<decltype(dst)>, int>) {
          dst = std::stoi(args[i + 1]);
        } else if constexpr (std::is_same_v<std::decay_t<decltype(dst)>, std::size_t>) {
          dst = static_cast<std::size_t>(std::stoul(args[i + 1]));
        } else if constexpr (std::is_same_v<std::decay_t<decltype(dst)>, std::string>) {
          dst = args[i + 1];
        } else {
          dst = std::stod(args[i + 1]);
        }
      } catch (...) { err = args[i] + " 값을 못 읽었다: " + args[i + 1]; return false; }
      ++i;
      return true;
    };

  for (; i < args.size(); ++i) {
    const std::string & a = args[i];
    if (a == "--z") { if (!value(opt.z)) { return false; } }
    else if (a == "--hover") { if (!value(opt.hover)) { return false; } }
    else if (a == "--speed") { if (!value(opt.speed)) { return false; } }
    else if (a == "--accel") { if (!value(opt.accel)) { return false; } }
    else if (a == "--approach-speed") { if (!value(opt.approach_speed)) { return false; } }
    else if (a == "--max-seg") { if (!value(opt.max_seg)) { return false; } }
    else if (a == "--max-jump") { if (!value(opt.max_jump)) { return false; } }
    else if (a == "--max-sens") { if (!value(opt.max_sens)) { return false; } }
    else if (a == "--min-seg") { if (!value(opt.min_seg)) { return false; } }
    else if (a == "--chain") { if (!value(opt.chain)) { return false; } }
    else if (a == "--limit") { if (!value(opt.limit)) { return false; } }
    else if (a == "--dry-run") { opt.dry_run = true; }
    else if (a == "--plan-only") { opt.plan_only = true; }
    else if (a == "--force") { opt.force = true; }
    else if (a == "--host") { if (!value(opt.host)) { return false; } }
    else if (a == "--port") { if (!value(opt.port)) { return false; } }
    else if (a == "--timeout") { if (!value(opt.timeout_sec)) { return false; } }
    else if (!a.empty() && a[0] == '-') { err = "모르는 옵션: " + a; return false; }
    else if (opt.path.empty()) { opt.path = a; }
    else { err = "경로 파일이 둘 이상이다: " + a; return false; }
  }

  if (opt.path.empty()) { err = "경로 파일이 없다"; return false; }
  if (opt.speed <= 0.0 || opt.accel <= 0.0) { err = "--speed · --accel 은 양수여야 한다"; return false; }
  if (opt.approach_speed <= 0.0) { err = "--approach-speed 는 양수여야 한다"; return false; }
  if (opt.hover <= 0.0) { err = "--hover 는 양수여야 한다"; return false; }
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

// `sub`·`norm` 은 pose_convention.hpp 가 이미 갖고 있다 — 여기서 다시 정의하면 모호해진다.
using hcr5_bridge::norm;
using hcr5_bridge::sub;

/// 한 점의 직교 표현 — 위치와 자세.
struct Pose
{
  Vec3 pos{};
  Vec3 ori{};
};

/// 경로 위의 한 점 — 세 표현이 다 채워진 상태.
/// 웨이포인트는 tcp·flange·joint 를 **동시에** 담고 셋이 정합해야 한다 (§6.1.2).
struct Node
{
  Pose tcp;
  Pose flange;
  JointArray joint{};
};

/// xy 만 읽는다 — z 는 옵션이 정한다.
struct Pt
{
  double x = 0.0;
  double y = 0.0;
};

/// 경로 파일을 읽어 xy 열을 낸다. 단위 판정이 이 함수의 요점이다 —
/// m 로 준 파일을 mm 로 읽으면 그림이 1000배가 되고 게이트가 잡기 전에 IK 가 먼저 죽는다.
bool loadPath(const std::string & path, std::vector<Pt> & out, std::string & err)
{
  std::ifstream f(path);
  if (!f) { err = "파일을 못 연다: " + path; return false; }

  Json j;
  try { f >> j; } catch (const std::exception & e) {
    err = std::string("JSON 파싱 실패: ") + e.what();
    return false;
  }

  const auto it = j.find("points");
  if (it == j.end() || !it->is_array() || it->empty()) {
    err = "최상위 \"points\" 배열이 없거나 비었다";
    return false;
  }

  // 단위 — 선언이 있으면 그것을 믿고, 없으면 크기로 추정한다.
  // 이 로봇의 작업 반경은 수백 mm 라, 좌표 절댓값이 10 을 안 넘으면 m 로 본다.
  double scale = 1.0;
  std::string unit = j.value("units", std::string());
  if (unit == "m") {
    scale = 1000.0;
  } else if (unit == "mm") {
    scale = 1.0;
  } else {
    double maxabs = 0.0;
    for (const auto & p : *it) {
      if (!p.is_array() || p.size() < 2) { continue; }
      maxabs = std::max({maxabs, std::fabs(p[0].get<double>()), std::fabs(p[1].get<double>())});
    }
    scale = (maxabs < 10.0) ? 1000.0 : 1.0;
    unit = (scale == 1000.0) ? "m (추정)" : "mm (추정)";
  }

  out.clear();
  for (const auto & p : *it) {
    if (!p.is_array() || p.size() < 2) { err = "points 원소가 [x,y,...] 배열이 아니다"; return false; }
    out.push_back(Pt{p[0].get<double>() * scale, p[1].get<double>() * scale});
  }

  std::printf("  경로  %s — %zu 점, 단위 %s\n", path.c_str(), out.size(), unit.c_str());
  return true;
}

/// 한 점의 세 표현 묶음 — tcp·flange·joint.
/// ⚠️ **movel 의 poseTriple() 과 다르다.** 저쪽은 같은 pose 를 양쪽에 복사해 L28 을 만들었다.
/// 여기는 FK 로 각각 얻은 값을 제 슬롯에 넣는다 (movej 와 같은 방식).
Json poseTriple(const Node & n)
{
  auto one = [](const Pose & p) {
      return Json{{"position", writeVec(p.pos)}, {"orientation", writeVec(p.ori)}};
    };
  return Json{{"tcp", one(n.tcp)}, {"flange", one(n.flange)}, {"joint", n.joint}};
}

/// 웨이포인트 한 칸. `relative` 블록은 `selected:"fixed"` 일 때도 **늘 실린다**
/// (2026-08-16 펜던트 캡처 확인) — 빼면 컨트롤러 파서가 어떻게 반응하는지 모르므로 그대로 채운다.
Json waypointSlot(const Node & n, const Json & from, bool result_zero)
{
  const Json here = poseTriple(n);
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

/// MOVE(linear) 노드 하나. 체인의 어느 자리인지에 따라 속도 이음매가 달라진다 (§6.3):
///
///     시작 노드:  {startVelocity:0, endVelocity:V, continues:true,  radius:0}
///     중간 노드:  {startVelocity:V, endVelocity:V, continues:true,  radius:0}
///     종료 노드:  {startVelocity:V, endVelocity:0, continues:false, radius:0}
///
/// ⚠️ `radius` 는 **늘 0** 이다. radius=50 이면 궤적이 웨이포인트를 22.3mm 떨어져 지나간다
/// (§6.2 실측) — 선 모양을 지키는 조합은 `radius:0` + `continues:true` 뿐이다.
Json moveNode(
  const Node & prev, const Node & cur, const Json & from,
  double vel, double accel, double start_vel, double end_vel, bool continues,
  const char * name)
{
  const Json move{
    {"selected", "linear"},
    {"linear", {
       {"radius", 0},
       {"velocity", vel},
       {"acceleration", accel},
       {"continues", continues},
       {"startVelocity", start_vel},
       {"endVelocity", end_vel},
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

  return Json{
    {"uuid", makeUuid()},
    {"name", name},
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
          // middlePoint 는 linear 에서 안 쓰인다(arc·circle 용). 캡처 형식을 지키되
          // 이 노드의 **시작점**으로 채운다 — 비워서 파서가 어떻게 되는지는 모른다.
          {"middlePoint", waypointSlot(prev, from, false)},
          {"endPoint", waypointSlot(cur, from, true)}}}}}};
}

/// program/plan 페이로드 — INITIALIZE + MOVE 노드 N 개.
/// **이것이 movel 과의 유일한 구조 차이다.** 저쪽은 노드가 늘 1개다.
Json buildPlan(const std::vector<Json> & move_nodes, const Node & cur)
{
  const Json initialize{
    {"uuid", makeUuid()},
    {"name", "Initialize"},
    {"type", "INITIALIZE"},
    {"child", Json::array()},
    {"time", 0},
    {"attr", {{"skip", false}, {"always", false}}}};

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

  Json child = Json::array({initialize});
  for (const auto & n : move_nodes) { child.push_back(n); }

  (void)cur;
  Json plan;
  plan["variables"] = Json::object();
  plan["coordinates"]["base"] = frame_def("Base");
  plan["coordinates"]["tcp"] = frame_def("TCP");
  // 전역 속도 배율(%). 100 = 노드 velocity 를 그대로 쓰겠다는 뜻.
  // ⚠️ 이것과 별개로 `set/velocity`(0~1) 전역 배율이 곱해진다 (§6.2) — 실측 속도가
  //    지령의 절반이면 그쪽을 먼저 본다.
  plan["velocity"] = 100;
  plan["repeat"] = false;
  plan["program"] = root(makeUuid(), child);
  plan["thread"] = root(0, Json::array());
  plan["subprogram"] = Json::array();
  plan["name"] = "hcr5_bridge_stroke";
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

  std::printf(
    "stroke — z %.2f mm (TCP 기준) · hover %.1f mm · 속도 %.1f mm/s%s%s\n",
    opt.z, opt.hover, opt.speed,
    opt.dry_run ? " · --dry-run" : "", opt.plan_only ? " · --plan-only" : "");

  std::vector<Pt> pts;
  if (!loadPath(opt.path, pts, err)) {
    std::fprintf(stderr, "✗ %s\n", err.c_str());
    return 2;
  }

  if (opt.limit > 0 && opt.limit < pts.size()) {
    std::printf("  --limit %zu — 앞 %zu 점만 쓴다 (원본 %zu)\n", opt.limit, opt.limit, pts.size());
    pts.resize(opt.limit);
  }
  if (pts.size() < 2) {
    std::fprintf(stderr, "✗ 점이 2개 미만이다 — 그릴 것이 없다\n");
    return 2;
  }

  // ── 짧은 세그 제거 — **150033 을 막는 자리다** ───────────────────────────
  // 웨이포인트 간격이 좁으면 컨트롤러가 그 구간의 궤적을 못 만들고 거부한다
  // (2026-08-17 실측, kDefaultMinSegMm 주석 참조). 원본을 고치지 않고 여기서 흡수한다.
  //
  // ⚠️ 폐곡선의 **마지막 점은 지킨다.** 그리기가 시작점으로 정확히 돌아와야 선이 닫히는데,
  //    끝점을 버리면 직전 점에서 끊긴다. 끝점이 너무 가까우면 그 **앞의 점**을 버린다.
  {
    const bool closed =
      pts.size() > 2 &&
      std::hypot(pts.back().x - pts.front().x, pts.back().y - pts.front().y) < opt.min_seg;

    std::vector<Pt> keep{pts.front()};
    std::size_t dropped = 0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
      const double d = std::hypot(pts[i].x - keep.back().x, pts[i].y - keep.back().y);
      if (d < opt.min_seg) { ++dropped; continue; }
      keep.push_back(pts[i]);
    }
    // 끝점이 버려졌으면 되살리고, 대신 그 앞을 하나 뺀다.
    const double tail =
      std::hypot(pts.back().x - keep.back().x, pts.back().y - keep.back().y);
    if (tail > 1e-9 && !closed) {
      if (keep.size() > 1 &&
        std::hypot(pts.back().x - keep.back().x, pts.back().y - keep.back().y) < opt.min_seg)
      {
        keep.pop_back();
      }
      keep.push_back(pts.back());
      --dropped;
    }
    if (dropped > 0) {
      std::printf(
        "  짧은 세그 %zu 개를 버렸다 (%.2f mm 미만 — 150033 방지) → %zu 점\n",
        dropped, opt.min_seg, keep.size());
    }
    pts.swap(keep);
  }

  // ── 경로 통계 · 게이트 ① 세그먼트 길이 ───────────────────────────────────
  {
    double total = 0.0, longest = 0.0, shortest = 1e9;
    std::size_t worst = 0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
      const double d = std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
      total += d;
      shortest = std::min(shortest, d);
      if (d > longest) { longest = d; worst = i; }
    }
    double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
    for (const auto & p : pts) {
      xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
      ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
    }
    std::printf(
      "  경로장 %.1f mm · 세그 %zu 개 (최단 %.3f · 최장 %.3f) · "
      "bbox x[%.1f %.1f] y[%.1f %.1f] = %.1f×%.1f mm\n",
      total, pts.size() - 1, shortest, longest, xmin, xmax, ymin, ymax,
      xmax - xmin, ymax - ymin);
    std::printf("  예상 스트로크 시간 %.0f 초 (경로장 ÷ 속도, 가감속 제외)\n", total / opt.speed);

    if (longest > opt.max_seg) {
      if (!opt.force) {
        std::fprintf(
          stderr,
          "✗ 세그먼트 %zu 가 %.3f mm 로 상한 %.1f mm 를 넘는다 — 거부.\n"
          "   경로 파일의 단위나 좌표계가 어긋났을 수 있다. "
          "의도한 것이면 --max-seg 를 올리거나 --force 를 명시하라\n",
          worst, longest, opt.max_seg);
        return 4;
      }
      std::printf("  ⚠️ --force — 세그먼트 상한 %.1f mm 를 넘겨 보낸다\n", opt.max_seg);
    }

    // 게이트 ② 속도 프로파일 — 가감속 거리가 세그먼트에 들어가는가.
    // 2026-08-05 의 150033 "특이점" 은 오진이었고 실제 원인이 이것이다 (§8 함정 ①).
    // ⚠️ 다만 continues:true 로 이은 **중간** 세그먼트는 감속을 안 하므로 이 조건이
    //    구간마다 걸리지는 않는다. 그래도 최단 세그를 기준으로 알려는 준다.
    const double ramp = opt.speed * opt.speed / opt.accel;
    std::printf("  가감속에 %.3f mm 필요 (최단 세그 %.3f mm)", ramp, shortest);
    if (ramp > shortest) {
      std::printf(" ← 최단 세그가 더 짧다. 체인 양 끝에서만 문제가 된다");
    }
    std::printf("\n");
  }

  MqttClient mqtt(opt.host, opt.port, "hcr5-stroke");
  if (!mqtt.start()) {
    std::fprintf(stderr, "✗ MQTT 접속 실패: %s:%d\n", opt.host.c_str(), opt.port);
    return 3;
  }

  // ── 현재 자세 — 기준은 **MQTT** 다 ────────────────────────────────────────
  // 이 경로는 스택을 안 타므로 `/joint_states` 가 아예 없다 (movel.md §1.3).
  // 자세(orientation)는 여기서 읽은 것을 **전 구간 그대로 쓴다** — 펜을 수직으로
  // 세운 채 xy 만 움직인다는 뜻이고, 회전 규약을 몰라도 성립한다 (movel.md §2.3).
  Node cur;
  {
    const auto r = mqtt.request(kTopicGetPos, Json::object());
    if (!r) {
      std::fprintf(stderr, "✗ %s 응답이 없다 — 실기가 켜져 있고 랜이 붙었는지 확인하라\n", kTopicGetPos);
      return 3;
    }
    if (r->find("joint") == r->end() || r->find("tcp") == r->end() || r->find("flange") == r->end()) {
      std::fprintf(stderr, "✗ %s 응답에 joint·tcp·flange 가 다 있지 않다\n", kTopicGetPos);
      return 3;
    }
    const auto & jt = r->at("joint");
    if (!jt.is_array() || jt.size() != kNumJoints) {
      std::fprintf(stderr, "✗ %s 응답의 joint 가 6개가 아니다\n", kTopicGetPos);
      return 3;
    }
    for (std::size_t i = 0; i < kNumJoints; ++i) { cur.joint[i] = jt[i].get<double>(); }
    cur.tcp.pos = readVec(r->at("tcp").at("position"));
    cur.tcp.ori = readVec(r->at("tcp").at("orientation"));
    cur.flange.pos = readVec(r->at("flange").at("position"));
    cur.flange.ori = readVec(r->at("flange").at("orientation"));
  }

  const double tcp_offset = norm(sub(cur.tcp.pos, cur.flange.pos));
  std::printf(
    "  현재  tcp    [%9.3f %9.3f %9.3f]  rot [%8.3f %8.3f %8.3f]\n"
    "        flange [%9.3f %9.3f %9.3f]  ← tcp 와 %.3f mm 떨어져 있다\n",
    cur.tcp.pos[0], cur.tcp.pos[1], cur.tcp.pos[2],
    cur.tcp.ori[0], cur.tcp.ori[1], cur.tcp.ori[2],
    cur.flange.pos[0], cur.flange.pos[1], cur.flange.pos[2], tcp_offset);

  // 펜이 등록돼 있지 않으면 z 의 의미가 통째로 달라진다 — 조용히 틀리는 것을 막는다.
  if (tcp_offset < 1.0) {
    std::printf(
      "  ⚠️ tcp 와 flange 가 사실상 같은 점이다 (%.3f mm). 펜 TCP 가 등록돼 있지 않으면\n"
      "     --z 는 **플랜지 높이**로 해석된다 — 펜 길이만큼 파고든다. 툴 설정을 확인하라\n",
      tcp_offset);
  }

  // ── 노드 열 만들기 — IK 로 관절, FK 로 tcp·flange ─────────────────────────
  // IK 는 **poseType:"tcp"** 로 푼다. 그래야 --z 가 펜 끝의 높이가 된다.
  // 시드는 **직전 점의 해**다 — 시드가 나쁘면 IK 가 먼 해(다른 브랜치)를 준다.
  const Vec3 ori = cur.tcp.ori;

  auto solve = [&](const Vec3 & tcp_pos, const JointArray & seed, Node & out, std::string & e) {
      const Json ik_payload{
        {"info", {
           {"position", writeVec(tcp_pos)},
           {"orientation", writeVec(ori)},
           {"joint", seed}}},
        {"poseType", "tcp"}};
      const auto r = mqtt.request(kTopicIk, ik_payload);
      if (!r || r->find("joint") == r->end() || !r->at("joint").is_array() ||
        r->at("joint").size() != kNumJoints)
      {
        e = "IK 가 해를 못 냈다 — 도달 불가능한 점이거나 특이점일 수 있다";
        return false;
      }
      for (std::size_t i = 0; i < kNumJoints; ++i) { out.joint[i] = r->at("joint")[i].get<double>(); }

      if (!withinLimits(out.joint)) {
        e = "IK 해가 실기 공식 가동범위 밖이다 (±360°, J3 ±165°)";
        return false;
      }

      // FK 두 번 — 웨이포인트의 tcp·flange 슬롯을 **각각** 채운다.
      // 이것이 L28(같은 pose 를 양쪽에 복사)을 안 밟는 이유다.
      auto fk = [&](const char * pose_type, Pose & p) {
          const Json payload{{"info", {{"joint", out.joint}}}, {"poseType", pose_type}};
          const auto fr = mqtt.request(kTopicFk, payload);
          if (!fr || fr->find("position") == fr->end() || fr->find("orientation") == fr->end()) {
            return false;
          }
          p.pos = readVec(fr->at("position"));
          p.ori = readVec(fr->at("orientation"));
          return true;
        };
      if (!fk("tcp", out.tcp)) { e = "FK(tcp) 가 해를 못 냈다"; return false; }
      if (!fk("flange", out.flange)) { e = "FK(flange) 가 해를 못 냈다"; return false; }
      return true;
    };

  // 경로: [현재] → hover(첫 점 위) → 첫 점 → … → 끝 점 → hover(끝 점 위)
  // hover 두 칸은 펜을 들고 오가는 구간이라 그림에 안 그려진다.
  struct Step
  {
    Vec3 pos;
    const char * name;
  };
  std::vector<Step> steps;
  steps.push_back(Step{Vec3{pts.front().x, pts.front().y, opt.z + opt.hover}, "APPROACH"});
  steps.push_back(Step{Vec3{pts.front().x, pts.front().y, opt.z}, "PEN_DOWN"});
  for (std::size_t i = 1; i < pts.size(); ++i) {
    steps.push_back(Step{Vec3{pts[i].x, pts[i].y, opt.z}, "STROKE"});
  }
  steps.push_back(Step{Vec3{pts.back().x, pts.back().y, opt.z + opt.hover}, "PEN_UP"});

  std::printf("  IK·FK 를 %zu 점에 건다 (RPC %zu 회) …\n", steps.size(), steps.size() * 3);

  std::vector<Node> nodes;
  nodes.reserve(steps.size());

  /// 노드별 진단 — 민감도 게이트가 걸렸을 때 **어디가 나쁜지** 사람에게 보여주기 위한 것.
  struct Diag
  {
    double seg = 0.0;      ///< 이 노드의 직교 이동거리 (mm)
    double jump = 0.0;     ///< 최대 관절 변화 (도)
    double sens = 0.0;     ///< jump / seg (도/mm)
    std::size_t axis = 0;  ///< 가장 많이 도는 축
  };
  std::vector<Diag> diag;
  diag.reserve(steps.size());

  {
    JointArray seed = cur.joint;
    for (std::size_t i = 0; i < steps.size(); ++i) {
      Node n;
      std::string e;
      if (!solve(steps[i].pos, seed, n, e)) {
        std::fprintf(
          stderr, "✗ 점 %zu/%zu (%s, [%.3f %.3f %.3f]) — %s\n",
          i + 1, steps.size(), steps[i].name,
          steps[i].pos[0], steps[i].pos[1], steps[i].pos[2], e.c_str());
        return 3;
      }

      // ── 게이트 ③ 관절 점프 ────────────────────────────────────────────────
      // **스트로크에서 가장 위험한 실패 방식이 여기다.** 직교공간으로는 1mm 옆인데
      // IK 가 다른 해 브랜치를 고르면 관절이 크게 튄다 — 에러 없이 팔이 휘둘린다.
      const JointArray & prev = nodes.empty() ? cur.joint : nodes.back().joint;
      double jump = 0.0;
      std::size_t jump_axis = 0;
      for (std::size_t k = 0; k < kNumJoints; ++k) {
        const double d = std::fabs(n.joint[k] - prev[k]);
        if (d > jump) { jump = d; jump_axis = k; }
      }
      // 첫 점(현재 자세 → 접근점)은 실제로 크게 움직이는 것이 정상이라 검사에서 뺀다.
      if (i > 0 && jump > opt.max_jump) {
        if (!opt.force) {
          std::fprintf(
            stderr,
            "✗ 점 %zu/%zu (%s) 에서 %s 가 %.3f° 튄다 — 상한 %.1f°. 거부.\n"
            "   직교공간으로는 이웃한 점인데 IK 가 다른 해 브랜치를 골랐다는 뜻이다.\n"
            "   그대로 보내면 에러 없이 팔이 크게 휘둘린다. --force 는 이 경우 위험하다\n",
            i + 1, steps.size(), steps[i].name,
            kUrdfJointNames[jump_axis].c_str(), jump, opt.max_jump);
          return 4;
        }
        std::printf("  ⚠️ --force — 점 %zu 의 관절 점프 %.3f° 를 넘긴다\n", i + 1, jump);
      }

      // ── 게이트 ④ 관절 민감도 (특이점) ─────────────────────────────────────
      // 게이트 ③ 은 "해가 얼마나 다른가" 를 봤다. 여기는 **그 차이를 이 속도로 낼 수
      // 있는가** 를 본다. 특이점 근처에서는 해가 멀쩡히 있고 이웃 해와도 가까운데,
      // 직교 1mm 에 관절이 크게 돌아 관절속도가 폭발한다. 컨트롤러가 이것을
      // "there might be singular points"(150033) 로 거부한다.
      const Vec3 prev_pos = (i == 0) ? cur.tcp.pos : steps[i - 1].pos;
      const double seg = norm(sub(steps[i].pos, prev_pos));
      const double sens = (seg > 1e-9) ? (jump / seg) : 0.0;
      diag.push_back(Diag{seg, jump, sens, jump_axis});

      // 첫 점(현재 자세 → 접근점)은 자세를 크게 바꾸는 것이 정상이라 뺀다.
      if (i > 0 && sens > opt.max_sens) {
        const double vel = (steps[i].name == std::string("STROKE")) ? opt.speed : opt.approach_speed;
        if (!opt.force) {
          std::fprintf(
            stderr,
            "✗ 점 %zu/%zu (%s) 의 관절 민감도가 %.2f 도/mm 로 상한 %.2f 를 넘는다 — 거부.\n"
            "   %s 가 %.3f mm 구간에서 %.3f° 돈다. %.1f mm/s 로 그으면 **%.0f °/s** 가 필요한데\n"
            "   실기 최대는 %.0f °/s 다. 특이점 근처라는 뜻이고, 그대로 보내면 컨트롤러가\n"
            "   150033 CTRLPORT_ERROR_ROBOT_MOTION_IS_NOT_RUNNABLE 로 거부한다.\n"
            "   → --speed 를 낮추거나, 이 점을 피하도록 경로/자세(rz)를 바꿔야 한다\n",
            i + 1, steps.size(), steps[i].name, sens, opt.max_sens,
            kUrdfJointNames[jump_axis].c_str(), seg, jump, vel, sens * vel,
            kMaxJointSpeedDegPerSec);
          return 4;
        }
        std::printf("  ⚠️ --force — 점 %zu 의 민감도 %.2f 도/mm 를 넘긴다\n", i + 1, sens);
      }

      seed = n.joint;
      nodes.push_back(n);
    }
  }

  // ── 민감도 상위 — 게이트를 통과해도 어디가 아슬아슬한지는 봐야 한다 ───────
  {
    std::vector<std::size_t> idx(diag.size());
    for (std::size_t i = 0; i < idx.size(); ++i) { idx[i] = i; }
    // 첫 점은 자세 전환이라 늘 크다 — 순위에서 뺀다.
    idx.erase(idx.begin());
    std::sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) {
        return diag[a].sens > diag[b].sens;
      });
    const std::size_t show = std::min<std::size_t>(5, idx.size());
    std::printf("  관절 민감도 상위 %zu (도/mm — 클수록 특이점에 가깝다)\n", show);
    for (std::size_t k = 0; k < show; ++k) {
      const std::size_t i = idx[k];
      const double vel = (steps[i].name == std::string("STROKE")) ? opt.speed : opt.approach_speed;
      std::printf(
        "    점 %2zu %-9s 세그 %7.3f mm · %-10s %7.3f° → %5.2f 도/mm "
        "(%.0f mm/s 에서 %5.0f °/s)\n",
        i + 1, steps[i].name, diag[i].seg, kUrdfJointNames[diag[i].axis].c_str(),
        diag[i].jump, diag[i].sens, vel, diag[i].sens * vel);
    }
  }

  // ── 지령 z 와 FK 가 되돌려 준 z 의 대조 ──────────────────────────────────
  // IK→FK 왕복 오차는 0.000mm 로 확인돼 있지만(movel.md §1.4), 그것은 **같은 poseType**
  // 안에서의 이야기다. 여기서 보는 것은 "내가 --z 로 준 값이 정말 펜 끝의 높이인가" 다.
  {
    double zmax = -1e9, zmin = 1e9;
    for (std::size_t i = 1; i + 1 < nodes.size(); ++i) {   // hover 두 칸 제외
      zmax = std::max(zmax, nodes[i].tcp.pos[2]);
      zmin = std::min(zmin, nodes[i].tcp.pos[2]);
    }
    std::printf(
      "  펜 구간 tcp.z 범위 [%.4f %.4f] mm — 지령 %.4f 와의 최대 차 %.4f mm\n",
      zmin, zmax, opt.z, std::max(std::fabs(zmax - opt.z), std::fabs(zmin - opt.z)));
  }

  // ── MOVE 노드 열 — 체인 구성은 §6.3 을 그대로 따른다 ─────────────────────
  const Json from = poseTriple(cur);
  std::vector<Json> move_nodes;
  move_nodes.reserve(nodes.size());

  const std::size_t stroke_begin = 1;                  // PEN_DOWN 직후부터가 그려지는 구간
  const std::size_t stroke_end = nodes.size() - 2;     // PEN_UP 직전까지
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const Node & prev = (i == 0) ? cur : nodes[i - 1];
    const bool in_chain = (i > stroke_begin && i <= stroke_end);

    double vel = opt.approach_speed;
    double sv = 0.0, ev = 0.0;
    bool cont = false;
    const char * name = steps[i].name;

    if (i == stroke_begin) {
      // PEN_DOWN — 내려오는 구간. 정지에서 시작해 정지로 끝난다.
      vel = opt.approach_speed;
      sv = 0.0;
      ev = 0.0;
      cont = false;
    } else if (in_chain) {
      // 체인 안에서의 0-based 순번. `--chain N` 마다 끊어 룩어헤드 한계를 피한다.
      const std::size_t k = i - (stroke_begin + 1);
      const bool cut_here =
        (i == stroke_end) || (opt.chain > 0 && (k + 1) % opt.chain == 0);
      const bool cut_before =
        (k == 0) || (opt.chain > 0 && k % opt.chain == 0);

      vel = opt.speed;
      sv = cut_before ? 0.0 : opt.speed;   // 앞이 끊겼으면 정지에서 출발
      ev = cut_here ? 0.0 : opt.speed;     // 여기서 끊으면 정지로 닫는다
      cont = !cut_here;
    }

    move_nodes.push_back(
      moveNode(prev, nodes[i], from, vel, opt.accel, sv, ev, cont, name));
  }

  const Json plan = buildPlan(move_nodes, cur);
  const std::size_t plan_bytes = plan.dump().size();
  {
    const std::size_t n_stroke = stroke_end - stroke_begin;
    const std::size_t n_cut =
      (opt.chain > 0) ? ((n_stroke + opt.chain - 1) / opt.chain) : 1;
    std::printf(
      "  프로그램 — MOVE 노드 %zu 개 (스트로크 %zu · 접근이탈 3) · %zu bytes (%.1f KB)\n"
      "  체인 — 최대 %zu 마다 끊는다 → 이음매 %zu 곳에서 속도가 0을 찍는다 "
      "(선은 안 끊긴다 · radius:0)\n",
      move_nodes.size(), n_stroke, plan_bytes, plan_bytes / 1024.0,
      opt.chain, (n_cut > 0) ? n_cut - 1 : 0);
  }

  if (opt.dry_run) {
    std::printf("  --dry-run — 발행하지 않는다\n");
    return 0;
  }

  // ── 게이트 ⑤ 서보 상태 ───────────────────────────────────────────────────
  // 이 게이트가 없으면 clear·plan·play 세 ack 가 전부 code:0 인데 로봇은 1mm 도
  // 안 움직이고, 타임아웃을 다 기다린 뒤에야 죽는다 (servo_gate.hpp).
  if (!opt.plan_only) {
    std::string servo;
    if (!waitServoOn(mqtt, servo)) {
      std::fprintf(
        stderr,
        "✗ 서보가 켜져 있지 않다 (status/operation = %s) — 발행하지 않는다.\n"
        "   켜려면: ros2 run hcr5_bridge servo on\n",
        servo.empty() ? "응답 없음" : servo.c_str());
      return 4;
    }
    std::printf("  서보  %s\n", servo.c_str());
  }

  // ── 도착 감시를 **발행 전에** 건다 ───────────────────────────────────────
  // ack 는 접수일 뿐 도착이 아니다. 구독을 발행 뒤에 걸면 신호를 놓친다.
  std::mutex m;
  std::condition_variable cv;
  bool done = false;
  std::string done_by;
  auto mark = [&](const char * who) {
      { std::lock_guard<std::mutex> lk(m); if (done) { return; } done = true; done_by = who; }
      cv.notify_all();
    };
  // ⚠️ `event/motion` 은 **노드마다** 날 수 있다. 여러 구간을 한 프로그램으로 도는
  //    이 경로에서 전체 완료를 뜻하는 것은 `program/end` 다 — 그쪽만 완료로 본다.
  mqtt.subscribe(kTopicProgramEnd, [&](const std::string &, const Json &) { mark("program/end"); });

  // ⚠️ **에러는 래치하고 세어 둔다.** 2026-08-17 에 `program/end` 와 `error/command` 가
  // 경합해서, 로봇이 150033 을 냈는데도 "완료" 로 보고된 회차가 있었다 — 먼저 온 쪽이
  // 대기를 깨우기 때문이다. 완료로 빠져나가더라도 아래에서 반드시 다시 본다.
  bool fault = false;
  std::size_t fault_count = 0;
  std::string fault_msg;
  auto on_error = [&](const std::string & topic, const Json & j) {
      std::lock_guard<std::mutex> lk(m);
      ++fault_count;
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

  // 노드가 많아 페이로드가 크다 — movel 의 5초보다 넉넉히 준다.
  if (!mqtt.request(kTopicPlan, plan, std::chrono::milliseconds(15000))) {
    std::fprintf(
      stderr,
      "✗ %s 가 거부됐다 (ack 없음 또는 code≠0) — %zu bytes / MOVE 노드 %zu 개.\n"
      "   **노드 수나 페이로드 크기 한계일 수 있다** (미실측 항목). --limit 로 점을 줄여\n"
      "   어디서 갈리는지 좁혀 보라 — 예: --limit 10 → 20 → 40\n",
      kTopicPlan, plan_bytes, move_nodes.size());
    return 5;
  }
  std::printf("  → %s ack — MOVE 노드 %zu 개 적재 (%zu bytes)\n",
    kTopicPlan, move_nodes.size(), plan_bytes);

  if (opt.plan_only) {
    std::printf(
      "  --plan-only — %s 를 쏘지 않는다. 로봇은 안 움직인다.\n"
      "  ⚠️ 이 프로그램은 펜던트 파일 목록에 안 뜬다 — 실행 버퍼와 파일 저장소는 다른 층이다.\n"
      "     지우려면 %s 를 쏜다.\n", kTopicPlay, kTopicClear);
    return 0;
  }

  if (!mqtt.request(kTopicPlay, Json::object())) {
    std::fprintf(stderr, "✗ %s ack 가 없다\n", kTopicPlay);
    return 5;
  }
  std::printf("  → %s ack — 실행 시작. ⚠️ 로봇이 그린다\n", kTopicPlay);

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
      std::fprintf(stderr, "✗ %.1f초 안에 %s 가 없다 — 정지시킨다\n", opt.timeout_sec, kTopicProgramEnd);
      mqtt.publish(kTopicStop, Json::object());
      mqtt.publish(kTopicMoveStop, Json::object());
      std::fprintf(stderr, "   %s · %s 발행. 자세는 `mqtt_cmd.py pos` 로 확인하라\n",
        kTopicStop, kTopicMoveStop);
      return 6;
    }
    std::printf("  ← 완료 (%s)\n", done_by.c_str());
  }

  // ── 완료 뒤에도 에러를 다시 본다 ─────────────────────────────────────────
  // `program/end` 가 먼저 도착해 대기를 깨웠어도, 같은 실행에서 난 에러는 유효하다.
  // 늦게 오는 것까지 잡으려고 짧게 더 기다린다 — 이 유예가 없으면 "완료" 가 거짓말을 한다.
  {
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, std::chrono::milliseconds(500), [&] { return fault; });
    if (fault) {
      const std::size_t n = fault_count;
      const std::string msg = fault_msg;
      lk.unlock();
      std::fprintf(
        stderr,
        "✗ **완료 신호가 왔지만 실행 중 에러가 %zu 건 났다** — 이 회차를 성공으로 보면 안 된다.\n"
        "   %s\n"
        "   150033 이면 웨이포인트 간격이 좁아 궤적을 못 만든 것이다 — --min-seg 를 올려라\n",
        n, msg.c_str());
      return 6;
    }
  }

  // ── 도착 검증 — 신호를 믿지 않고 실제 자세를 되읽는다 ─────────────────────
  {
    const auto r = mqtt.request(kTopicGetPos, Json::object());
    if (!r || r->find("tcp") == r->end()) {
      std::printf("  ⚠️ 도착 자세를 못 읽었다 — `mqtt_cmd.py pos` 로 확인하라\n");
      return 0;
    }
    const Vec3 got = readVec(r->at("tcp").at("position"));
    const Vec3 want = nodes.back().tcp.pos;
    std::printf(
      "  실제  tcp [%9.3f %9.3f %9.3f]  마지막 점(PEN_UP)과의 오차 %.3f mm\n",
      got[0], got[1], got[2], norm(sub(got, want)));
  }
  return 0;
}
