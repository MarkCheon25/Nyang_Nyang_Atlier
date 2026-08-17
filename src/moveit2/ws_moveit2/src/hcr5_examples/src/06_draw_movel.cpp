// ═══ 예제 6 · 같은 그림을 movel(점대점 직선) 로 그린다 ═══════════════════════
//
// 05 와 **똑같은 종이·똑같은 좌표**를 쓴다 (`include/hcr5_examples/a4_paper.hpp`).
// 다른 것은 **실행 전략 하나**다.
//
//   05  윤곽선 전체를 computeCartesianPath 한 번에 넘긴다 → 궤적 1개, 멈추지 않고 완주
//   06  윤곽선의 변 하나마다 따로 계획·실행한다          → 궤적 70개, 꼭짓점마다 정지
//
// ⚠️ 먼저 알아둘 것 — **선 모양은 05 와 같다.**
// ────────────────────────────────────────
// 05 의 computeCartesianPath 도 웨이포인트 사이를 데카르트 직선으로 잇는다. 그래서
// 지나가는 자취(기하)는 두 예제가 **완전히 동일**하다. "여러 직선으로 표현된다"는
// 것은 06 이 새로 만드는 성질이 아니라 05 도 이미 그랬다.
//
// 06 이 실제로 바꾸는 것은 **동역학과 명령 단위**다.
//   · 꼭짓점마다 속도가 0 으로 떨어졌다 다시 붙는다 (05 는 TOTG 가 통과시킨다)
//   · 로봇에 내리는 명령이 "궤적 1개" 가 아니라 "목표점 70개" 가 된다
//
// 왜 이게 필요한가 — 실기 명령 채널이 점대점뿐이기 때문이다
// ────────────────────────────────────────────────────────
// HCR-5 컨트롤러에서 우리가 확보한 이동 명령은 `move/joint/here` 하나이고, 이것은
// **한 번 발행하면 목표까지 로봇이 알아서 가는** 점대점 명령이다 (연속 궤적을 매
// 주기 밀어넣는 서보 인터페이스가 아니다). 05 의 250구간 연속 궤적을 그대로
// 태우면 로봇이 자기 이동을 계속 선점당하며 움직이는 목표를 쫓게 된다
// (실측 추종오차 0.8846° ≈ 반경 538mm 에서 8mm — 선 품질에 치명적).
//
// 06 은 그 점대점 모델을 **RViz 에서 미리 재현**한다. 여기서 확인할 것:
//   ① 변마다 계획이 풀리는가 (달성률 100%)
//   ② 정지·재가속을 70번 하면 시간이 얼마나 드는가
//   ③ 어느 변이 너무 짧은가 — 실기 데드밴드(0.01°)에 먹히거나 명령 왕복이
//      이동시간보다 길어지는 구간을 미리 골라낸다
//
// 지금은 아직 **MoveIt 이 궤적을 만들고 mock 이 실행**한다. 실기 연동은
// 이 구조를 그대로 두고 실행 계층만 `hcr_bridge` 로 바꾸는 순서다.
//
// 실행:
//   터미널1  ros2 launch hcr5_moveit_config demo.launch.py
//   터미널2  ros2 run hcr5_viz pen_trail --ros-args -p min_point_distance:=0.0002
//   터미널3  ros2 launch hcr5_examples example.launch.py example:=draw_movel
//
//   RViz 는 손댈 것이 없다 — moveit.rviz 에 디스플레이가 이미 등록돼 있다.
//     /pen_trail/trail            빨강 = 실제 자취
//     /hcr5_examples/target_shape 주황=경로 · 하늘색 점=정지하는 꼭짓점 · 흰색=종이
//
//   실기에 태우기 전 검토는 execute:=false 로 한다. 06 은 구간마다 시작 상태를
//   앞 구간의 끝으로 이어 붙이므로, 로봇을 안 움직이고도 **전 구간이 제대로
//   계획된다** (05 의 execute:=false 는 그렇지 않다 — 05 헤더 §8 참조).
//
//   ⚠️ execute:=false 는 로봇을 안 움직이므로 **0.1초 만에 끝난다.** 그냥 두면
//      볼 것이 없다 — move_group 이 구간을 계산할 때마다 /display_planned_path 에
//      한 구간짜리 미리보기를 내보내는데, 72개가 0.1초 안에 서로를 덮어써서
//      화면에는 깜빡임만 남는다. 그래서 §7-b 가 **72구간을 한 메시지로 묶어 다시
//      발행**한다. 그때 RViz 가 처음부터 끝까지 한 번에 재생한다.
//      재생을 보는 동안 노드는 preview_hold 초(기본 30) 동안 살아 있는다.
//
// 파라미터
//   z_left  −0.012 · z_right −0.014   종이 좌·우변의 Z [m]
//   use_measured_z false               true 면 teach 한 Z 를 그대로 쓴다
//   margin  0.015 · draw_size 0.0 · eef_step 0.002 · hover 0.03
//   vel_scale 0.1 · acc_scale 0.1 · execute true · go_home true · pen_yaw_deg 0.0
//   settle    0.0                      꼭짓점마다 추가로 멈추는 시간 [s] (실기 왕복 흉내)
//   short_seg 0.002                    이보다 짧은 변을 "짧은 구간"으로 집계 [m]
//   preview_hold 30.0                  execute:=false 재생을 보는 동안 살아 있는 시간 [s]

#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

#include <hcr5_examples/a4_paper.hpp>

using hcr5_examples::A4Paper;
using hcr5_examples::ContourFit;
using hcr5_examples::kContour;

// ── 파라미터 헬퍼 ────────────────────────────────────────────────────────────
// NodeOptions 에 automatically_declare_parameters_from_overrides(true) 를 줬으므로
// 런치/CLI 로 넘어온 값은 **이미 선언돼 있다.** 그 상태에서 declare_parameter 를
// 부르면 예외가 난다. 그래서 "없을 때만 선언" 한다.
template <typename T>
static T param(const rclcpp::Node::SharedPtr & n, const std::string & name, const T & def)
{
  if (!n->has_parameter(name)) { n->declare_parameter<T>(name, def); }
  return n->get_parameter(name).get_value<T>();
}

static double dist(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return std::sqrt(std::pow(b.position.x - a.position.x, 2) +
                   std::pow(b.position.y - a.position.y, 2) +
                   std::pow(b.position.z - a.position.z, 2));
}

// ── movel 한 구간 ────────────────────────────────────────────────────────────
// 현재 상태(cursor)에서 target 까지 **데카르트 직선 하나**를 계획·실행한다.
// 이것이 실기의 이동 명령 1건에 대응한다.
struct Seg {
  bool   ok{false};
  double fraction{0.0};
  double length{0.0};
  double duration{0.0};
  size_t points{0};
  moveit_msgs::msg::RobotTrajectory traj;   // RViz 미리보기용 (execute:=false)
};

static Seg movel(
  moveit::planning_interface::MoveGroupInterface & mg,
  moveit::core::RobotState & cursor,               // in/out — 구간 끝 상태로 갱신된다
  const geometry_msgs::msg::Pose & from,
  const geometry_msgs::msg::Pose & target,
  double eef_step, double vel, double acc, bool execute,
  const rclcpp::Logger & log, size_t idx)
{
  Seg r;
  r.length = dist(from, target);

  // 시작 상태를 명시한다. execute 면 실기(mock)의 현재 상태, 아니면 앞 구간의 끝.
  // 이 한 줄이 execute:=false 검토를 성립시킨다 — 로봇이 안 움직여도 다음 구간이
  // 앞 구간 끝에서 이어서 계획된다.
  if (execute) { mg.setStartStateToCurrentState(); } else { mg.setStartState(cursor); }

  moveit_msgs::msg::RobotTrajectory traj;
  r.fraction = mg.computeCartesianPath({target}, eef_step, traj);
  if (r.fraction < 0.999) {
    RCLCPP_ERROR(log, "[구간 %zu] 달성률 %.1f%% — (%.1f, %.1f, %.1f) → (%.1f, %.1f, %.1f) mm",
                 idx, r.fraction * 100.0,
                 from.position.x * 1000.0, from.position.y * 1000.0, from.position.z * 1000.0,
                 target.position.x * 1000.0, target.position.y * 1000.0, target.position.z * 1000.0);
    return r;
  }

  // computeCartesianPath 산출물은 시간 정보가 부실하다. TOTG 로 다시 시간을 입힌다.
  // 구간마다 따로 걸므로 **양 끝 속도가 0** 이다 — 이것이 점대점의 정체다.
  robot_trajectory::RobotTrajectory rt(mg.getRobotModel(), mg.getName());
  rt.setRobotTrajectoryMsg(cursor, traj);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(rt, vel, acc)) {
    RCLCPP_ERROR(log, "[구간 %zu] 시간 매개변수화 실패", idx);
    return r;
  }
  rt.getRobotTrajectoryMsg(traj);

  r.points   = traj.joint_trajectory.points.size();
  r.duration = rclcpp::Duration(traj.joint_trajectory.points.back().time_from_start).seconds();
  r.traj     = traj;

  if (execute) {
    if (mg.execute(traj) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(log, "[구간 %zu] 실행 실패", idx);
      return r;
    }
  }

  cursor = rt.getLastWayPoint();   // 다음 구간의 시작점
  r.ok = true;
  return r;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "hcr5_draw_movel",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto log = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  auto spinner = std::thread([&exec]() { exec.spin(); });

  const double draw_size = param<double>(node, "draw_size", 0.0);
  const double eef_step  = param<double>(node, "eef_step", 0.002);
  const double hover     = param<double>(node, "hover", 0.03);
  const double margin    = param<double>(node, "margin", 0.015);
  const double z_left    = param<double>(node, "z_left", -0.012);
  const double z_right   = param<double>(node, "z_right", -0.014);
  const bool   meas_z    = param<bool>(node, "use_measured_z", false);
  const double pen_yaw   = param<double>(node, "pen_yaw_deg", 0.0);
  const double vel       = param<double>(node, "vel_scale", 0.1);
  const double acc       = param<double>(node, "acc_scale", 0.1);
  const bool   execute   = param<bool>(node, "execute", true);
  const bool   go_home   = param<bool>(node, "go_home", true);
  const double settle    = param<double>(node, "settle", 0.0);
  const double short_seg = param<double>(node, "short_seg", 0.002);
  const double preview_hold = param<double>(node, "preview_hold", 30.0);

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface mg(node, "hcr_arm");
  mg.setMaxVelocityScalingFactor(vel);
  mg.setMaxAccelerationScalingFactor(acc);

  RCLCPP_INFO(log, "엔드이펙터: %s · 기준 프레임: %s",
              mg.getEndEffectorLink().c_str(), mg.getPlanningFrame().c_str());

  // ── 1. 홈 자세로 ───────────────────────────────────────────────────────────
  if (go_home) {
    mg.setNamedTarget("home");
    if (mg.move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(log, "홈 자세 이동 실패 — SRDF 의 group_state 'home' 확인");
      exec.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }
    RCLCPP_INFO(log, "홈 자세 도달");
  }

  // ── 2. 종이 · 도형 ─────────────────────────────────────────────────────────
  const A4Paper paper(z_left, z_right, meas_z, pen_yaw);
  const ContourFit fit(kContour, paper, margin, draw_size);

  if (!meas_z) {
    RCLCPP_INFO(log, "종이 Z 지정: 좌 %.1f mm · 우 %.1f mm", z_left * 1000.0, z_right * 1000.0);
    RCLCPP_WARN(log,
      "teach 평면 대비 좌변 %+.2f mm · 우변 %+.2f mm (+ 는 펜이 종이 위로 뜬 것). "
      "그대로 두면 뜬 쪽은 선이 안 나온다 — use_measured_z:=true 로 확인할 것.",
      A4Paper::gapLeft(z_left) * 1000.0, A4Paper::gapRight(z_right) * 1000.0);
  } else {
    RCLCPP_INFO(log, "종이 Z: teach 값 사용 (좌 %.2f · 우 %.2f mm)",
                hcr5_examples::kLL.z * 1000.0, hcr5_examples::kRL.z * 1000.0);
  }
  if (fit.clamped()) {
    RCLCPP_WARN(log, "draw_size %.0f mm 는 종이(여백 %.0f mm 제외)를 넘는다 — %.0f mm 로 줄인다",
                draw_size * 1000.0, margin * 1000.0,
                fit.fitScale() * std::max(fit.widthPx(), fit.heightPx()) * 1000.0);
  }
  RCLCPP_INFO(log, "종이: %.1f×%.1f mm · 여백 %.0f mm",
              paper.width() * 1000.0, paper.height() * 1000.0, margin * 1000.0);
  RCLCPP_INFO(log, "입력 도형: %zu 점 · bbox %.0f×%.0f px → %.1f×%.1f mm (배율 %.5f m/px)",
              kContour.size(), fit.widthPx(), fit.heightPx(),
              fit.drawnWidth() * 1000.0, fit.drawnHeight() * 1000.0, fit.scale());

  // 그릴 경로: 윤곽선 전체 + 첫 점으로 복귀(닫기)
  std::vector<geometry_msgs::msg::Pose> shape;
  shape.reserve(kContour.size() + 1);
  for (const auto & p : kContour) { shape.push_back(fit.pose(p, 0.0)); }
  shape.push_back(shape.front());

  const auto start_above = fit.pose(kContour.front(), hover);

  // 변 길이 분포 — 실기 점대점에서 어디가 비효율인지 미리 본다.
  {
    double lo = 1e9, hi = 0.0, sum = 0.0;
    size_t n_short = 0;
    for (size_t i = 1; i < shape.size(); ++i) {
      const double d = dist(shape[i - 1], shape[i]);
      lo = std::min(lo, d); hi = std::max(hi, d); sum += d;
      if (d < short_seg) { ++n_short; }
    }
    RCLCPP_INFO(log,
      "변 %zu 개 · 총 길이 %.1f mm · 최단 %.2f · 최장 %.2f · 평균 %.2f mm · "
      "%.0fmm 미만 %zu 개",
      shape.size() - 1, sum * 1000.0, lo * 1000.0, hi * 1000.0,
      sum * 1000.0 / (shape.size() - 1), short_seg * 1000.0, n_short);
  }

  // ── 3. 마커 ────────────────────────────────────────────────────────────────
  // transient_local: RViz 를 나중에 켜거나 토픽을 나중에 추가해도 보이게 한다.
  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/hcr5_examples/target_shape", rclcpp::QoS(1).transient_local().reliable());
  {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker m;
    m.header.frame_id = mg.getPlanningFrame();
    m.header.stamp = node->now();
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;      // points 가 절대좌표라 pose 는 항등

    m.ns = "target_shape";  m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.scale.x = 0.001;               // LINE_STRIP 은 scale.x 만 쓴다 (선 굵기)
    m.color.r = 1.0f; m.color.g = 0.55f; m.color.b = 0.1f; m.color.a = 1.0f;  // 주황
    for (const auto & q : shape) { m.points.push_back(q.position); }
    arr.markers.push_back(m);

    // 종이 테두리 — 도형이 종이 안에 들어갔는지 눈으로 확인된다.
    visualization_msgs::msg::Marker pm = m;
    pm.ns = "paper"; pm.id = 1;
    pm.scale.x = 0.002;
    pm.color.r = 1.0f; pm.color.g = 1.0f; pm.color.b = 1.0f; pm.color.a = 0.8f;
    pm.points.clear();
    for (const auto & ab : {std::pair<double, double>{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}}) {
      pm.points.push_back(paper.at(ab.first, ab.second, 0.0).position);
    }
    arr.markers.push_back(pm);

    // **로봇이 실제로 정지하는 점.** 05 에는 없던 표시다 — movel 의 단위가 눈에 보인다.
    visualization_msgs::msg::Marker vm = m;
    vm.ns = "via"; vm.id = 2;
    vm.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    vm.scale.x = vm.scale.y = vm.scale.z = 0.003;
    vm.color.r = 0.2f; vm.color.g = 0.8f; vm.color.b = 1.0f; vm.color.a = 1.0f;  // 하늘색
    vm.points.clear();
    for (const auto & q : shape) { vm.points.push_back(q.position); }
    arr.markers.push_back(vm);

    marker_pub->publish(arr);
    RCLCPP_INFO(log, "마커 발행: 주황=경로 · 하늘색=정지 꼭짓점 %zu 개 · 흰색=종이",
                shape.size());
  }

  // ── 4. 펜을 든 채로 시작점 위까지 ──────────────────────────────────────────
  // 여기만 데카르트가 아니라 일반 계획(move)이다. 홈에서 종이까지는 300mm 가까이
  // 내려가는 큰 이동이라 직선을 강제하면 IK 가 끊기거나 특이점을 스치기 쉽다.
  {
    mg.setPoseTarget(start_above);
    MoveGroupInterface::Plan plan;
    const bool planned = (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!planned) {
      RCLCPP_ERROR(log, "시작점 위(%.1f, %.1f, %.1f mm) 계획 실패 — 종이가 작업영역 밖인가",
                   start_above.position.x * 1000.0, start_above.position.y * 1000.0,
                   start_above.position.z * 1000.0);
      exec.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }
    if (execute && mg.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(log, "시작점 위로 이동 실패");
      exec.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }
    mg.clearPoseTargets();
    RCLCPP_INFO(log, "시작점 위 %s (펜 든 높이 %.0f mm)",
                execute ? "도달" : "계획 완료 (execute:=false — 안 움직인다)", hover * 1000.0);
  }

  // ── 5. 자취 초기화 ─────────────────────────────────────────────────────────
  {
    auto cli = node->create_client<std_srvs::srv::Empty>("/pen_trail/clear");
    if (cli->wait_for_service(std::chrono::milliseconds(500))) {
      cli->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
      RCLCPP_INFO(log, "이전 자취 삭제 (/pen_trail/clear)");
    } else {
      RCLCPP_INFO(log, "pen_trail 이 없다 — 자취 초기화 생략");
    }
  }

  // ── 6. movel 연쇄 ──────────────────────────────────────────────────────────
  // 펜 내림 → 변 하나씩 → 펜 듦. 전부 같은 단위(직선 1개 = 명령 1건)다.
  moveit::core::RobotState cursor(*mg.getCurrentState());
  if (!execute) {
    // 안 움직였으므로 커서를 §4 에서 계획한 접근 자세로 옮겨 놓는다.
    // 그래야 첫 하강 구간이 실제와 같은 자리에서 시작한다.
    mg.setStartStateToCurrentState();
    MoveGroupInterface::Plan plan;
    mg.setPoseTarget(start_above);
    if (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      robot_trajectory::RobotTrajectory rt(mg.getRobotModel(), mg.getName());
      rt.setRobotTrajectoryMsg(cursor, plan.trajectory);
      cursor = rt.getLastWayPoint();
    }
    mg.clearPoseTargets();
  }

  std::vector<geometry_msgs::msg::Pose> targets;
  targets.reserve(shape.size() + 2);
  targets.push_back(shape.front());                            // 하강 (펜 내림)
  for (size_t i = 1; i < shape.size(); ++i) { targets.push_back(shape[i]); }
  targets.push_back(start_above);                              // 상승 (펜 듦)

  const auto t0 = std::chrono::steady_clock::now();
  auto prev = start_above;

  // execute:=false 는 로봇을 안 움직이므로 순식간에 끝난다 — 볼 것이 없다.
  // 그래서 계획한 궤적을 모아 두었다가 RViz 에 재생시킨다 (§7-b).
  const moveit::core::RobotState preview_start(cursor);
  std::vector<moveit_msgs::msg::RobotTrajectory> preview;

  size_t done = 0, failed_at = 0;
  double total_len = 0.0, total_dur = 0.0;
  double worst_frac = 1.0; size_t worst_idx = 0;
  double slowest = 0.0;    size_t slowest_idx = 0;

  for (size_t i = 0; i < targets.size(); ++i) {
    const auto seg = movel(mg, cursor, prev, targets[i], eef_step, vel, acc, execute, log, i);
    if (!seg.ok) { failed_at = i + 1; break; }

    total_len += seg.length;
    total_dur += seg.duration;
    if (seg.fraction < worst_frac) { worst_frac = seg.fraction; worst_idx = i; }
    if (seg.duration > slowest)    { slowest = seg.duration;    slowest_idx = i; }
    if (!execute) { preview.push_back(seg.traj); }
    prev = targets[i];
    ++done;

    if (execute && settle > 0.0) {
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(settle * 1000)));
    }
    if (i % 10 == 0 || i + 1 == targets.size()) {
      RCLCPP_INFO(log, "구간 %zu/%zu · 누적 %.1f mm · %.1f 초",
                  i + 1, targets.size(), total_len * 1000.0, total_dur);
    }
  }

  const double wall =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  // ── 7. 결과 ────────────────────────────────────────────────────────────────
  RCLCPP_INFO(log, "──────── 결과 (movel) ────────");
  RCLCPP_INFO(log, "구간 %zu/%zu 성공 · 총 이동 %.1f mm",
              done, targets.size(), total_len * 1000.0);
  if (execute) {
    // 벽시계 − 궤적시간 = 구간마다 무는 계획·왕복 비용. 실기에서도 명령 1건마다
    // 같은 성격의 비용을 문다 (MQTT 왕복 + 컨트롤러 가감속). 점대점 전략의 대가다.
    RCLCPP_INFO(log, "궤적 시간 합 %.1f 초 · 벽시계 %.1f 초 (차이 %.1f 초 = 계획·왕복 비용, "
                     "구간당 %.0f ms)",
                total_dur, wall, wall - total_dur,
                done ? (wall - total_dur) * 1000.0 / done : 0.0);
  } else {
    RCLCPP_INFO(log, "궤적 시간 합 %.1f 초 (실행하면 여기에 구간당 계획·왕복 비용이 더 붙는다)",
                total_dur);
  }
  RCLCPP_INFO(log, "최저 달성률 %.1f%% (구간 %zu) · 최장 구간 %.2f 초 (구간 %zu)",
              worst_frac * 100.0, worst_idx, slowest, slowest_idx);

  if (failed_at) {
    RCLCPP_ERROR(log, "구간 %zu 에서 멈췄다 — 위 [구간 %zu] 로그에 좌표가 있다",
                 failed_at, failed_at - 1);
  } else if (execute) {
    const auto end = mg.getCurrentPose().pose;
    RCLCPP_INFO(log, "종료점 오차 %.2f mm", dist(end, start_above) * 1000.0);
    RCLCPP_INFO(log, "RViz 에서 주황(계획)과 빨강(자취)을 겹쳐 보면 편차가 보인다.");
  } else {
    RCLCPP_INFO(log, "execute:=false — 전 구간 계획만 확인했다. 로봇은 움직이지 않았다.");
  }

  // ── 7-b. RViz 재생 (execute:=false 일 때만) ────────────────────────────────
  // execute:=false 는 로봇을 안 움직이므로 0.1초 만에 끝나 눈으로 볼 것이 없다.
  // 계획해 둔 구간 궤적 전부를 DisplayTrajectory 로 한 번에 발행하면, RViz 의
  // MotionPlanning → Planned Path 가 **유령 로봇으로 처음부터 끝까지 재생**한다.
  // 실기 없이 movel 연쇄를 통째로 검토하는 자리가 이것이다.
  //
  // 재생 속도는 moveit.rviz 의 `State Display Time` 이 정한다 (현재 0.05 s/웨이포인트).
  // 더 느리게 보려면 RViz 에서 그 값을 올리거나 REALTIME 으로 바꾼다.
  //
  // execute:=true 에서는 **일부러 발행하지 않는다.** 실제 로봇이 이미 움직였는데
  // 유령까지 재생하면 둘이 헷갈린다 (moveit.rviz 의 Loop Animation 주석과 같은 이유).
  if (!execute && !preview.empty()) {
    auto disp_pub = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", rclcpp::QoS(1).transient_local().reliable());

    moveit_msgs::msg::DisplayTrajectory disp;
    disp.model_id = mg.getRobotModel()->getName();
    moveit::core::robotStateToRobotStateMsg(preview_start, disp.trajectory_start);
    disp.trajectory = preview;
    disp_pub->publish(disp);

    RCLCPP_INFO(log, "RViz 재생 발행: %zu 구간 · 궤적 %.1f 초 — MotionPlanning 의 "
                     "Planned Path 가 유령 로봇으로 재생한다", preview.size(), total_dur);
    RCLCPP_INFO(log, "재생을 보는 동안 %.0f 초 살아 있는다 (preview_hold 로 조절). "
                     "Ctrl-C 로 언제든 끝낼 수 있다", preview_hold);
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(preview_hold * 1000)));
  } else {
    // 마커를 남겨 두기 위해 잠시 더 살아 있는다 (transient_local 이라 죽어도 남지만,
    // 늦게 붙은 구독자가 확실히 받도록 여유를 둔다).
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return failed_at ? 1 : 0;
}
