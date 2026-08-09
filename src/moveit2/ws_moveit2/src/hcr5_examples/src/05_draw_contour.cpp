// ═══ 예제 5 · 윤곽선 그리기 — 비전이 뽑은 좌표를 그대로 그린다 ═══════════════
//
// **이것이 프로젝트의 실제 작업이다.** 지금까지의 예제(사각형)는 우리가 좌표를
// 만들어 넣었지만, 여기서는 **비전이 이미지에서 뽑아온 윤곽선 좌표**를 받는다.
//
// 지금은 좌표가 이 파일 안에 박혀 있다(고양이 이미지, 70점). 나중에 vision 이
// 토픽/서비스로 넘겨주면 kContour 를 그것으로 갈아끼우면 된다 — **나머지는 그대로다.**
//
// 무엇을 확인하려는 것인가
// ────────────────────────
//   ① 픽셀 좌표가 로봇 좌표로 제대로 옮겨지는가 (뒤집히거나 거울상이 되지 않는가)
//   ② 그 경로 전체에서 IK 가 풀리는가            ← 달성률(fraction)
//   ③ 실제로 그려진 자취가 원본 도형과 같은가    ← 초록(계획) vs 빨강(자취) 비교
//
// ③ 을 위해 **입력 도형을 초록 선으로 발행**한다. `hcr5_viz/pen_trail` 의 빨간
// 자취와 겹쳐 보면 편차가 눈에 보인다. 이것이 선 품질의 1차 계측 수단이다.
//
// 실행:
//   터미널1  ros2 launch hcr5_moveit_config demo.launch.py
//   터미널2  ros2 run hcr5_viz pen_trail --ros-args -p min_point_distance:=0.0002
//   터미널3  ros2 launch hcr5_examples example.launch.py example:=draw_contour
//
//   RViz 는 손댈 것이 없다 — moveit.rviz 에 두 디스플레이가 이미 등록돼 있다.
//     /pen_trail/trail            빨강 = 실제 자취
//     /hcr5_examples/target_shape 초록 = 입력 도형  (Transient Local 로 구독해야 한다)
//
// 크기를 바꿔가며 실험 (재빌드 불필요):
//   ros2 launch hcr5_examples example.launch.py example:=draw_contour draw_size:=0.25
//   ros2 launch hcr5_examples example.launch.py example:=draw_contour execute:=false
//
// ⚠️ `ros2 launch` 에 `--ros-args -p x:=y` 를 붙여도 **노드로 전달되지 않는다.**
//    위처럼 `key:=value` 런치 인자로 줄 것 (example.launch.py 가 타입을 붙여 실어 준다).
//
// 파라미터:  draw_size 긴 변 [m] 0.15 · eef_step 0.002 · hover 0.03
//            vel_scale 0.1 · acc_scale 0.1 · execute true · go_home true
//
// ⚠️ `pen_trail` 의 기본 `min_point_distance` 는 2mm 다. 이 도형의 최단 변이
//    draw_size=0.15 에서 약 1.2mm 라 기본값으로는 세부가 뭉개진다. 위처럼 낮출 것.

#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

// ── 입력 도형 ────────────────────────────────────────────────────────────────
// 고양이 이미지에서 뽑은 윤곽선. OpenCV `findContours` 산출물 형태 그대로 —
// **픽셀 좌표**이고, 원점은 이미지 **좌상단**, v(세로)는 **아래로** 증가한다.
// 마지막 점에서 첫 점으로 돌아가는 닫힌 경로다(코드가 자동으로 이어 붙인다).
struct Px { int u, v; };

static const std::vector<Px> kContour = {
  { 56,   0}, { 47,  12}, { 46,  84}, { 38, 103}, { 36, 127}, {  5, 131}, {  4, 139},
  {  7, 142}, { 37, 142}, { 42, 153}, { 15, 167}, { 17, 175}, { 50, 167}, { 61, 179},
  { 51, 234}, { 50, 272}, { 69, 337}, { 85, 480}, { 70, 489}, { 62, 500}, { 60, 512},
  { 64, 523}, { 83, 535}, {322, 535}, {340, 529}, {386, 525}, {417, 511}, {447, 478},
  {459, 435}, {455, 399}, {437, 344}, {435, 317}, {448, 294}, {479, 281}, {479, 246},
  {462, 237}, {432, 241}, {405, 260}, {389, 285}, {382, 314}, {383, 339}, {403, 405},
  {407, 435}, {395, 463}, {371, 475}, {374, 445}, {371, 388}, {358, 338}, {343, 305},
  {304, 257}, {234, 208}, {211, 178}, {217, 167}, {250, 175}, {253, 165}, {227, 156},
  {231, 142}, {260, 142}, {264, 138}, {264, 133}, {258, 129}, {233, 128}, {232, 106},
  {224,  83}, {223,   8}, {215,   0}, {206,   0}, {160,  41}, {111,  42}, { 66,   0},
};

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

// ── 데카르트 구간 하나를 계획·시간매개변수화·실행 ────────────────────────────
struct SegResult { double fraction; size_t points; double duration; bool executed; };

static SegResult runCartesian(
  moveit::planning_interface::MoveGroupInterface & mg,
  const std::vector<geometry_msgs::msg::Pose> & wps,
  double eef_step, double vel, double acc, bool execute,
  const rclcpp::Logger & log, const char * label)
{
  moveit_msgs::msg::RobotTrajectory traj;
  const double fraction = mg.computeCartesianPath(wps, eef_step, traj);

  SegResult r{fraction, traj.joint_trajectory.points.size(), 0.0, false};
  if (fraction < 0.999) {
    RCLCPP_ERROR(log, "[%s] 달성률 %.1f%% — 경로가 끊겼다", label, fraction * 100.0);
    return r;
  }

  // computeCartesianPath 산출물은 시간 정보가 부실하다. TOTG 로 다시 시간을 입힌다.
  robot_trajectory::RobotTrajectory rt(mg.getRobotModel(), mg.getName());
  rt.setRobotTrajectoryMsg(*mg.getCurrentState(), traj);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(rt, vel, acc)) {
    RCLCPP_ERROR(log, "[%s] 시간 매개변수화 실패 — joint_limits.yaml 의 가속도 한계 확인", label);
    return r;
  }
  rt.getRobotTrajectoryMsg(traj);

  r.points = traj.joint_trajectory.points.size();
  r.duration =
    rclcpp::Duration(traj.joint_trajectory.points.back().time_from_start).seconds();

  if (!execute) {
    RCLCPP_INFO(log, "[%s] 계획만 (execute:=false) — 구간 %zu 개 · %.1f 초",
                label, r.points, r.duration);
    return r;
  }

  RCLCPP_INFO(log, "[%s] 실행 — 구간 %zu 개 · %.1f 초", label, r.points, r.duration);
  r.executed = (mg.execute(traj) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!r.executed) { RCLCPP_ERROR(log, "[%s] 실행 실패", label); }
  return r;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "hcr5_draw_contour",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto log = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  auto spinner = std::thread([&exec]() { exec.spin(); });

  // ── 파라미터 ───────────────────────────────────────────────────────────────
  const double draw_size = param<double>(node, "draw_size", 0.15);   // 긴 변 [m]
  const double eef_step  = param<double>(node, "eef_step", 0.002);   // 보간 간격 [m]
  const double hover     = param<double>(node, "hover", 0.03);       // 펜 든 높이 [m]
  const double vel       = param<double>(node, "vel_scale", 0.1);
  const double acc       = param<double>(node, "acc_scale", 0.1);
  const bool   execute   = param<bool>(node, "execute", true);
  const bool   go_home   = param<bool>(node, "go_home", true);

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface mg(node, "hcr_arm");

  RCLCPP_INFO(log, "엔드이펙터: %s · 기준 프레임: %s",
              mg.getEndEffectorLink().c_str(), mg.getPlanningFrame().c_str());

  // ── 1. 홈 자세로 ───────────────────────────────────────────────────────────
  // 왜 필요한가: **펜이 수직으로 아래를 향하는 자세**에서 시작해야 종이 평면
  // (수평 XY 평면)에 그릴 수 있다. SRDF 의 `home` 은 실기가 `move/joint/home` 으로
  // 실제 가는 자세이고, 그 자세에서 pen_tip 의 자세는 RPY (180°, 0, 0) — 즉 펜이
  // 바닥을 향한다. 여기서 얻은 orientation 을 전 구간 유지하면 펜이 기울지 않는다.
  if (go_home) {
    mg.setNamedTarget("home");
    if (mg.move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(log, "홈 자세 이동 실패 — SRDF 의 group_state 'home' 확인");
      exec.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }
    RCLCPP_INFO(log, "홈 자세 도달");
  }

  const auto origin = mg.getCurrentPose().pose;
  RCLCPP_INFO(log, "그리기 평면: z = %.4f m · 중심 (%.4f, %.4f)",
              origin.position.z, origin.position.x, origin.position.y);

  // ── 2. 픽셀 → 로봇 좌표 ────────────────────────────────────────────────────
  // 이미지는 좌상단 원점 · v 아래로 증가. 로봇은 pen 이 −Z 를 보는 수평 XY 평면.
  //
  // 로봇 베이스에 서서 +X 방향(바깥)을 보는 관찰자를 기준으로 잡는다.
  //     관찰자의 "위쪽"(멀어지는 쪽) = +X      ← 이미지의 v 가 작은 쪽(위)
  //     관찰자의 "왼쪽"              = +Y      ← 이미지의 u 가 작은 쪽(왼쪽)
  //
  //     x = cx − (v − v_c) · s
  //     y = cy − (u − u_c) · s
  //
  // v 와 u 에 **둘 다 음부호**가 붙는 것이 핵심이다. 한쪽만 뒤집으면 거울상이 된다
  // (이미지 v 가 아래로 증가하는 것을 보정하는 부호와, 관찰자 오른쪽이 −Y 인 것을
  //  보정하는 부호가 각각 하나씩이라 결과적으로 방향이 보존된다).
  int u_lo = kContour[0].u, u_hi = kContour[0].u;
  int v_lo = kContour[0].v, v_hi = kContour[0].v;
  for (const auto & p : kContour) {
    u_lo = std::min(u_lo, p.u); u_hi = std::max(u_hi, p.u);
    v_lo = std::min(v_lo, p.v); v_hi = std::max(v_hi, p.v);
  }
  const double w_px = u_hi - u_lo, h_px = v_hi - v_lo;
  const double s = draw_size / std::max(w_px, h_px);          // [m/px]
  const double u_c = 0.5 * (u_lo + u_hi), v_c = 0.5 * (v_lo + v_hi);

  RCLCPP_INFO(log, "입력 도형: %zu 점 · bbox %.0f×%.0f px → %.1f×%.1f mm (배율 %.5f m/px)",
              kContour.size(), w_px, h_px, w_px * s * 1000.0, h_px * s * 1000.0, s);

  auto toPose = [&](const Px & p, double dz) {
    geometry_msgs::msg::Pose q = origin;      // orientation 유지 = 펜이 계속 수직
    q.position.x = origin.position.x - (p.v - v_c) * s;
    q.position.y = origin.position.y - (p.u - u_c) * s;
    q.position.z = origin.position.z + dz;
    return q;
  };

  // 그릴 경로: 윤곽선 전체 + 첫 점으로 복귀(닫기)
  std::vector<geometry_msgs::msg::Pose> shape;
  shape.reserve(kContour.size() + 1);
  for (const auto & p : kContour) { shape.push_back(toPose(p, 0.0)); }
  shape.push_back(shape.front());

  // ── 3. 입력 도형을 초록 선으로 발행 ────────────────────────────────────────
  // transient_local: RViz 를 나중에 켜거나 토픽을 나중에 추가해도 보이게 한다.
  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/hcr5_examples/target_shape", rclcpp::QoS(1).transient_local().reliable());
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = mg.getPlanningFrame();
    m.header.stamp = node->now();
    m.ns = "target_shape";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;      // points 가 절대좌표라 pose 는 항등
    m.scale.x = 0.001;               // LINE_STRIP 은 scale.x 만 쓴다 (선 굵기)
    m.color.r = 0.1f; m.color.g = 1.0f; m.color.b = 0.1f; m.color.a = 1.0f;
    for (const auto & q : shape) { m.points.push_back(q.position); }
    arr.markers.push_back(m);
    marker_pub->publish(arr);
    RCLCPP_INFO(log, "입력 도형 발행: /hcr5_examples/target_shape (초록)");
  }

  // ── 4. 펜을 든 채로 시작점 위까지 ──────────────────────────────────────────
  // 이 이동을 그리기와 분리하지 않으면, 중심에서 시작점까지 가는 직선이
  // 자취에 그대로 남아 도형에 없는 선이 하나 더 그려진다.
  {
    std::vector<geometry_msgs::msg::Pose> approach;
    auto up = origin;      up.position.z += hover;
    approach.push_back(up);
    approach.push_back(toPose(kContour.front(), hover));
    auto r = runCartesian(mg, approach, eef_step, vel, acc, execute, log, "이동");
    if (!r.executed && execute) {
      exec.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }
  }

  // ── 5. 자취 초기화 ─────────────────────────────────────────────────────────
  // pen_trail 이 떠 있으면 여기까지의 이동 자취를 지운다. 그래야 빨간 선에
  // **그린 것만** 남는다. 노드가 없으면 그냥 넘어간다(이 예제는 뷰어에 의존하지 않는다).
  {
    auto cli = node->create_client<std_srvs::srv::Empty>("/pen_trail/clear");
    if (cli->wait_for_service(std::chrono::milliseconds(500))) {
      cli->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
      RCLCPP_INFO(log, "이전 자취 삭제 (/pen_trail/clear)");
    } else {
      RCLCPP_INFO(log, "pen_trail 이 없다 — 자취 초기화 생략 (그리기는 그대로 진행)");
    }
  }

  // ── 6. 펜 내리고 → 그리고 → 펜 들기 ───────────────────────────────────────
  // 한 번의 데카르트 경로로 묶는다. 수직 하강·상승은 위에서 내려다보면 점이라
  // 자취에 군더더기를 남기지 않는다.
  std::vector<geometry_msgs::msg::Pose> path;
  path.reserve(shape.size() + 2);
  path.push_back(shape.front());                              // 하강 (펜 내림)
  path.insert(path.end(), shape.begin(), shape.end());        // 그리기
  path.push_back(toPose(kContour.front(), hover));            // 상승 (펜 듦)

  auto draw = runCartesian(mg, path, eef_step, vel, acc, execute, log, "그리기");

  // ── 7. 결과 ────────────────────────────────────────────────────────────────
  RCLCPP_INFO(log, "──────── 결과 ────────");
  RCLCPP_INFO(log, "요청 웨이포인트 %zu · 달성률 %.1f%% · 궤적 구간 %zu · 소요 %.1f 초",
              path.size(), draw.fraction * 100.0, draw.points, draw.duration);

  if (draw.fraction < 0.999) {
    // 어디서 끊겼는지 알려준다 — 달성률 숫자만으로는 원인 추적이 안 된다.
    double total = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
      const auto & a = path[i - 1].position;
      const auto & b = path[i].position;
      total += std::hypot(std::hypot(b.x - a.x, b.y - a.y), b.z - a.z);
    }
    double target = total * draw.fraction, acc_len = 0.0;
    size_t idx = 0;
    for (size_t i = 1; i < path.size(); ++i) {
      const auto & a = path[i - 1].position;
      const auto & b = path[i].position;
      acc_len += std::hypot(std::hypot(b.x - a.x, b.y - a.y), b.z - a.z);
      if (acc_len >= target) { idx = i; break; }
    }
    RCLCPP_ERROR(log,
      "경로가 %zu 번째 웨이포인트 부근에서 끊겼다 (전체 %zu). "
      "draw_size 를 줄이거나(현재 %.0f mm) 도형 중심을 옮겨 볼 것.",
      idx, path.size(), draw_size * 1000.0);
  } else if (draw.executed) {
    const auto end = mg.getCurrentPose().pose;
    const auto & want = path.back().position;
    RCLCPP_INFO(log, "종료점 오차 %.2f mm",
                std::hypot(std::hypot(end.position.x - want.x, end.position.y - want.y),
                           end.position.z - want.z) * 1000.0);
    RCLCPP_INFO(log, "RViz 에서 초록(입력)과 빨강(자취)을 겹쳐 보면 편차가 보인다.");
  }

  // 마커를 남겨 두기 위해 잠시 더 살아 있는다 (transient_local 이라 죽어도 남지만,
  // 늦게 붙은 구독자가 확실히 받도록 여유를 둔다).
  rclcpp::sleep_for(std::chrono::seconds(1));

  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return draw.fraction >= 0.999 ? 0 : 1;
}
