// ═══ 예제 5 · 윤곽선 그리기 — 실제 A4 용지 위에 그린다 ═══════════════════════
//
// **이것이 프로젝트의 실제 작업이다.** 지금까지의 예제(사각형)는 우리가 좌표를
// 만들어 넣었지만, 여기서는 **비전이 이미지에서 뽑아온 윤곽선 좌표**를 받는다.
//
// 지금은 좌표가 이 파일 안에 박혀 있다(고양이 이미지, 70점). 나중에 vision 이
// 토픽/서비스로 넘겨주면 kContour 를 그것으로 갈아끼우면 된다 — **나머지는 그대로다.**
//
// 종전에는 홈 자세의 pen_tip 위치(공중 z=291.5mm)를 그리기 평면으로 삼았다.
// 실기에 종이를 놓고 네 모서리를 teach 했으므로, 이제 **그 종이 위에** 그린다.
// 좌표는 아래 kLL/kLU/kRU/kRL 에 기록돼 있다 (teach 기록이므로 이 파일이 정본).
//
// 무엇을 확인하려는 것인가
// ────────────────────────
//   ① 픽셀 좌표가 종이 위로 제대로 옮겨지는가 (뒤집히거나 거울상이 되지 않는가)
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
// 값을 바꿔가며 실험 (재빌드 불필요):
//   ros2 launch hcr5_examples example.launch.py example:=draw_contour execute:=false
//   ros2 launch hcr5_examples example.launch.py example:=draw_contour z_right:=-0.018
//   ros2 launch hcr5_examples example.launch.py example:=draw_contour use_measured_z:=true
//
// ⚠️ `ros2 launch` 에 `--ros-args -p x:=y` 를 붙여도 **노드로 전달되지 않는다.**
//    위처럼 `key:=value` 런치 인자로 줄 것 (example.launch.py 가 타입을 붙여 실어 준다).
//
// 파라미터
//   z_left  −0.012 · z_right −0.014   종이 좌·우변의 Z [m] — 펜 높이를 여기서 잡는다
//   use_measured_z false               true 면 teach 한 Z 를 그대로 쓴다
//   margin  0.015                      종이 가장자리 여백 [m]
//   draw_size 0.0                      0 이면 종이에 맞춰 자동 · >0 이면 긴 변 [m] 강제
//   eef_step 0.002 · hover 0.03 · vel_scale 0.1 · acc_scale 0.1
//   execute true · go_home true · pen_yaw_deg 0.0
//
// ⚠️ `pen_trail` 의 기본 `min_point_distance` 는 2mm 다. 이 도형의 최단 변이
//    약 1.2mm 라 기본값으로는 세부가 뭉개진다. 위처럼 낮출 것.

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

// ── 종이 (A4) ────────────────────────────────────────────────────────────────
// 실기에서 teach 한 네 모서리 [m, 로봇 베이스 프레임]. **이 파일이 정본이다** —
// 종이를 옮기거나 다시 재면 여기를 고치고 커밋한다 (측정 기록이므로 런치 인자로
// 빼지 않았다). 이름은 위에서 내려다본 기준 — +X 가 오른쪽, +Y 가 위쪽이다.
//
//        LU (432.09, 107.84)      RU (639.24, 107.82)     ← +Y (위)
//              ┌──────────────────────┐
//              │                      │
//              │        A4            │  긴 변 297 → +Y
//              │                      │
//              └──────────────────────┘
//        LL (435.07,−186.48)      RL (644.00,−186.14)
//                     짧은 변 210 → +X
//
// 실측 검토 (tools 없이 이 자리에서 확인한 값)
//   변 길이   LL→RL 209.01 · LU→RU 207.23 (공칭 210)
//             LL→LU 294.34 · RL→RU 294.00 (공칭 297)
//     → XY 는 1~3mm 짧게 찍혔다. 모서리 직각도도 ±0.9° 흔들린다.
//   평면성   네 점의 최소제곱 평면 이탈이 ±0.008mm — **Z 는 서로 완벽히 일관된다.**
//   기울기   좌변 −12.36/−12.39, 우변 −18.28/−18.29.
//            X 로 209mm 가는 동안 Z 가 5.92mm 내려간다 = 1.63° 기울어진 책상이다.
//
//   즉 "Z 오차"로 보였던 5.9mm 는 측정 잡음이 아니라 **실제 기울기**다.
//   ±0.008mm 로 평면에 붙어 있는 값이 잡음일 수는 없다. z_right 를 −14 로 두면
//   우변에서 펜이 종이보다 4.3mm 떠서 그 영역은 아예 안 그려진다.
//   use_measured_z:=true 로 teach 값을 그대로 쓰는 경로를 열어 두었다.
struct Corner { double x, y, z; };
static constexpr Corner kLL{0.43507, -0.18648, -0.01236};   // 왼쪽 아래
static constexpr Corner kLU{0.43209,  0.10784, -0.01239};   // 왼쪽 위
static constexpr Corner kRU{0.63924,  0.10782, -0.01829};   // 우측 위
static constexpr Corner kRL{0.64400, -0.18614, -0.01828};   // 우측 아래

// teach 할 때 티치펜던트가 보여준 자세는 네 점 모두 Rx=179.9° Ry=0 Rz=−90° 였다.
// 우리 URDF 의 홈에서 pen_tip 자세는 RPY(180°, 0, 0) 이다. 둘의 차이는 **펜 축을
// 중심으로 한 90° 회전뿐**이고 (툴 Z 축은 양쪽 다 정확히 (0,0,−1) = 수직 하향),
// 펜은 축 대칭이라 그려지는 선에는 영향이 없다. 실기 TCP 프레임의 X 축 정의가
// 우리 tool0 과 90° 다른 것으로 보이며, 네 점의 Rz 가 모두 같으므로 어느 쪽이든
// 그리기 결과는 동일하다. 손목 자세만 달라진다 — 필요하면 pen_yaw_deg 로 준다.

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
  const double draw_size = param<double>(node, "draw_size", 0.0);    // 0 = 종이에 맞춤
  const double eef_step  = param<double>(node, "eef_step", 0.002);   // 보간 간격 [m]
  const double hover     = param<double>(node, "hover", 0.03);       // 펜 든 높이 [m]
  const double margin    = param<double>(node, "margin", 0.015);     // 가장자리 여백 [m]
  const double z_left    = param<double>(node, "z_left", -0.012);    // 좌변 Z [m]
  const double z_right   = param<double>(node, "z_right", -0.014);   // 우변 Z [m]
  const bool   meas_z    = param<bool>(node, "use_measured_z", false);
  const double pen_yaw   = param<double>(node, "pen_yaw_deg", 0.0);  // 펜 축 회전 [deg]
  const double vel       = param<double>(node, "vel_scale", 0.1);
  const double acc       = param<double>(node, "acc_scale", 0.1);
  const bool   execute   = param<bool>(node, "execute", true);
  const bool   go_home   = param<bool>(node, "go_home", true);

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface mg(node, "hcr_arm");

  RCLCPP_INFO(log, "엔드이펙터: %s · 기준 프레임: %s",
              mg.getEndEffectorLink().c_str(), mg.getPlanningFrame().c_str());

  // ── 1. 홈 자세로 ───────────────────────────────────────────────────────────
  // 왜 필요한가: 종이는 로봇 베이스보다 **아래**(z ≈ −12mm)에 있고 홈의 pen_tip 은
  // z = 291.5mm 다. 홈을 거쳐 가면 매번 같은 팔 형상에서 출발하므로, 종이로 내려갈
  // 때 어깨/팔꿈치가 뒤집히는(configuration flip) 일이 없다.
  if (go_home) {
    mg.setNamedTarget("home");
    if (mg.move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(log, "홈 자세 이동 실패 — SRDF 의 group_state 'home' 확인");
      exec.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }
    RCLCPP_INFO(log, "홈 자세 도달");
  }

  // ── 2. 펜 자세 ─────────────────────────────────────────────────────────────
  // 전 구간 고정한다. 펜이 기울면 선 굵기가 변하고 접촉점이 명령 위치에서 밀린다.
  //   R = Rz(yaw) · Rx(180°)  →  쿼터니언 (x,y,z,w) = (cos(yaw/2), sin(yaw/2), 0, 0)
  // yaw=0 이면 (1,0,0,0) 으로 홈의 pen_tip 자세와 정확히 같다 (툴 Z = (0,0,−1)).
  // 종이가 1.63° 기울어 있지만 펜은 수직으로 둔다 — 기울기를 따라가면 IK 해가
  // 바뀌는 위험 대비 이득이 없다 (1.63° 에서 접촉점 이동은 펜 반지름 × 0.028).
  geometry_msgs::msg::Quaternion pen_q;
  {
    const double h = 0.5 * pen_yaw * M_PI / 180.0;
    pen_q.x = std::cos(h); pen_q.y = std::sin(h); pen_q.z = 0.0; pen_q.w = 0.0;
  }

  // ── 3. 종이 평면 ───────────────────────────────────────────────────────────
  // teach 한 XY 는 그대로 쓰고, Z 만 z_left / z_right 로 갈아끼운다.
  // use_measured_z:=true 면 teach 값을 그대로 쓴다.
  Corner LL = kLL, LU = kLU, RU = kRU, RL = kRL;
  if (!meas_z) {
    LL.z = LU.z = z_left;
    RL.z = RU.z = z_right;
    RCLCPP_INFO(log, "종이 Z 지정: 좌 %.1f mm · 우 %.1f mm", z_left * 1000.0, z_right * 1000.0);
    // teach 평면 대비 얼마나 뜨는지/눌리는지 — 안 그려지는 원인의 1순위다.
    const double d_l = 1000.0 * (z_left  - 0.5 * (kLL.z + kLU.z));
    const double d_r = 1000.0 * (z_right - 0.5 * (kRL.z + kRU.z));
    RCLCPP_WARN(log,
      "teach 평면 대비 좌변 %+.2f mm · 우변 %+.2f mm (+ 는 펜이 종이 위로 뜬 것). "
      "우변이 %.1f mm 이상 뜨면 그쪽 선은 안 나온다 — use_measured_z:=true 로 확인할 것.",
      d_l, d_r, 1.0);
  } else {
    RCLCPP_INFO(log, "종이 Z: teach 값 사용 (좌 %.2f · 우 %.2f mm)",
                kLL.z * 1000.0, kRL.z * 1000.0);
  }

  // 종이 정규좌표 (a,b) ∈ [0,1]² → 로봇 좌표. a 는 좌→우(+X), b 는 아래→위(+Y).
  // **겹선형 보간**이라 네 모서리를 정확히 지난다. 종이가 완전한 직사각형이
  // 아니어도(실측 직각도 ±0.9°) 네 변을 그대로 따라가며, Z 기울기도 자동으로 실린다.
  auto onPaper = [&](double a, double b, double dz) {
    const double w00 = (1 - a) * (1 - b), w10 = a * (1 - b);
    const double w11 = a * b,             w01 = (1 - a) * b;
    geometry_msgs::msg::Pose q;
    q.orientation = pen_q;
    q.position.x = w00 * LL.x + w10 * RL.x + w11 * RU.x + w01 * LU.x;
    q.position.y = w00 * LL.y + w10 * RL.y + w11 * RU.y + w01 * LU.y;
    q.position.z = w00 * LL.z + w10 * RL.z + w11 * RU.z + w01 * LU.z + dz;
    return q;
  };

  // 종이의 실제 변 길이 (마주보는 두 변의 평균). 배율 계산의 기준이다.
  auto dist = [](const Corner & p, const Corner & q) {
    return std::sqrt((p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y) + (p.z-q.z)*(p.z-q.z));
  };
  const double W = 0.5 * (dist(kLL, kRL) + dist(kLU, kRU));   // 짧은 변 ≈ 0.208
  const double H = 0.5 * (dist(kLL, kLU) + dist(kRL, kRU));   // 긴 변   ≈ 0.294

  // ── 4. 픽셀 → 종이 좌표 ────────────────────────────────────────────────────
  // 이미지는 좌상단 원점 · u 오른쪽 · v 아래로 증가.
  // 종이는 (위에서 내려다볼 때) a 오른쪽 · b 위쪽.
  //
  //     a = 0.5 + (u − u_c)·s / W       u 증가 → 오른쪽
  //     b = 0.5 − (v − v_c)·s / H       v 증가 → 아래쪽   ← 부호 하나만 뒤집는다
  //
  // 종전 코드는 v→+X, u→+Y 로 보내 이미지를 90° 눕혔다. 그때는 평면이 공중의
  // 무한 평면이라 상관없었지만, 종이는 세로가 긴 A4 다. 세로로 긴 이 도형
  // (475×535 px)을 눕히면 210mm 폭에 535px 을 밀어넣게 돼 크게 못 그린다.
  // 지금처럼 세우면 종이와 도형의 세로/가로 방향이 맞는다.
  //
  // 거울상이 아닌 이유: +Z 에서 내려다보는 시점에서 a 는 화면 오른쪽, b 는 화면
  // 위쪽이다. 여기에 u→오른쪽, v→아래쪽을 그대로 대응시켰으므로 이미지가 보이는
  // 그대로 놓인다. 펜은 종이 **윗면**에 그리고 사람도 위에서 보므로 뒤집히지 않는다.
  int u_lo = kContour[0].u, u_hi = kContour[0].u;
  int v_lo = kContour[0].v, v_hi = kContour[0].v;
  for (const auto & p : kContour) {
    u_lo = std::min(u_lo, p.u); u_hi = std::max(u_hi, p.u);
    v_lo = std::min(v_lo, p.v); v_hi = std::max(v_hi, p.v);
  }
  const double w_px = u_hi - u_lo, h_px = v_hi - v_lo;
  const double u_c = 0.5 * (u_lo + u_hi), v_c = 0.5 * (v_lo + v_hi);

  // 배율: 여백을 뺀 종이 안에 꽉 채운다. draw_size > 0 이면 그 값을 쓰되,
  // 종이를 넘으면 잘려 나가는 대신 줄여서 맞춘다 (경고와 함께).
  const double s_fit = std::min((W - 2 * margin) / w_px, (H - 2 * margin) / h_px);
  double s = s_fit;
  if (draw_size > 0.0) {
    s = draw_size / std::max(w_px, h_px);
    if (s > s_fit) {
      RCLCPP_WARN(log, "draw_size %.0f mm 는 종이(여백 %.0f mm 제외)를 넘는다 — %.0f mm 로 줄인다",
                  draw_size * 1000.0, margin * 1000.0, s_fit * std::max(w_px, h_px) * 1000.0);
      s = s_fit;
    }
  }

  RCLCPP_INFO(log, "종이: %.1f×%.1f mm · 여백 %.0f mm", W * 1000.0, H * 1000.0, margin * 1000.0);
  RCLCPP_INFO(log, "입력 도형: %zu 점 · bbox %.0f×%.0f px → %.1f×%.1f mm (배율 %.5f m/px)",
              kContour.size(), w_px, h_px, w_px * s * 1000.0, h_px * s * 1000.0, s);

  auto toPose = [&](const Px & p, double dz) {
    return onPaper(0.5 + (p.u - u_c) * s / W,
                   0.5 - (p.v - v_c) * s / H, dz);
  };

  // 그릴 경로: 윤곽선 전체 + 첫 점으로 복귀(닫기)
  std::vector<geometry_msgs::msg::Pose> shape;
  shape.reserve(kContour.size() + 1);
  for (const auto & p : kContour) { shape.push_back(toPose(p, 0.0)); }
  shape.push_back(shape.front());

  // 그려질 영역을 로봇 좌표로 찍어 둔다 — 실기에서 종이 위인지 눈으로 대조할 값.
  {
    auto c0 = toPose({u_lo, v_hi}, 0.0).position;   // 도형 좌하단
    auto c1 = toPose({u_hi, v_lo}, 0.0).position;   // 도형 우상단
    RCLCPP_INFO(log, "그릴 영역: X %.1f~%.1f · Y %.1f~%.1f · Z %.2f~%.2f mm",
                c0.x * 1000.0, c1.x * 1000.0, c0.y * 1000.0, c1.y * 1000.0,
                std::min(c0.z, c1.z) * 1000.0, std::max(c0.z, c1.z) * 1000.0);
  }

  // ── 5. 입력 도형 + 종이 테두리를 마커로 발행 ───────────────────────────────
  // transient_local: RViz 를 나중에 켜거나 토픽을 나중에 추가해도 보이게 한다.
  // 종이 테두리(흰 선)를 같이 그려야 도형이 종이 안에 들어갔는지 눈으로 확인된다.
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

    visualization_msgs::msg::Marker pm = m;
    pm.ns = "paper";
    pm.id = 1;
    pm.scale.x = 0.002;
    pm.color.r = 1.0f; pm.color.g = 1.0f; pm.color.b = 1.0f; pm.color.a = 0.8f;
    pm.points.clear();
    for (const auto & ab : {std::pair<double,double>{0,0}, {1,0}, {1,1}, {0,1}, {0,0}}) {
      pm.points.push_back(onPaper(ab.first, ab.second, 0.0).position);
    }
    arr.markers.push_back(pm);

    marker_pub->publish(arr);
    RCLCPP_INFO(log, "마커 발행: /hcr5_examples/target_shape — 초록=도형 · 흰색=종이 테두리");
  }

  // ── 6. 펜을 든 채로 시작점 위까지 ──────────────────────────────────────────
  // 이 이동을 그리기와 분리하지 않으면, 접근 직선이 자취에 그대로 남아 도형에
  // 없는 선이 하나 더 그려진다.
  //
  // 여기만 데카르트가 아니라 **일반 계획(move)** 을 쓴다. 홈에서 종이 위까지는
  // 300mm 가까이 내려가는 큰 이동이라, 직선을 강제하면 중간에서 IK 가 끊기거나
  // 특이점을 스치기 쉽다. 자취는 어차피 바로 다음에 지우므로 경로 모양은 상관없다.
  const auto start_above = toPose(kContour.front(), hover);
  {
    mg.setMaxVelocityScalingFactor(vel);
    mg.setMaxAccelerationScalingFactor(acc);
    mg.setPoseTarget(start_above);
    if (execute) {
      if (mg.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(log,
          "시작점 위(%.1f, %.1f, %.1f mm)로 이동 실패 — 종이가 작업영역 밖이거나 "
          "hover 가 너무 낮다", start_above.position.x * 1000.0,
          start_above.position.y * 1000.0, start_above.position.z * 1000.0);
        exec.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
      }
      RCLCPP_INFO(log, "시작점 위 도달 (펜 든 높이 %.0f mm)", hover * 1000.0);
    } else {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool ok = (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(log, "[이동] 계획만 (execute:=false) — %s", ok ? "성공" : "실패");
      if (!ok) { exec.cancel(); spinner.join(); rclcpp::shutdown(); return 1; }
    }
    mg.clearPoseTargets();
  }

  // ── 7. 자취 초기화 ─────────────────────────────────────────────────────────
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

  // ── 8. 펜 내리고 → 그리고 → 펜 들기 ───────────────────────────────────────
  // 한 번의 데카르트 경로로 묶는다. 수직 하강·상승은 위에서 내려다보면 점이라
  // 자취에 군더더기를 남기지 않는다.
  //
  // ⚠️ execute:=false 면 §6 에서 로봇이 실제로 안 움직였으므로, 아래 경로는 홈에서
  //    종이까지 내려가는 구간이 통째로 앞에 붙는다. 달성률이 낮게 나오는 것이
  //    정상이다 — 도형 자체의 IK 를 보려면 execute:=true 로 볼 것.
  std::vector<geometry_msgs::msg::Pose> path;
  path.reserve(shape.size() + 2);
  path.push_back(shape.front());                              // 하강 (펜 내림)
  path.insert(path.end(), shape.begin(), shape.end());        // 그리기
  path.push_back(start_above);                                // 상승 (펜 듦)

  auto draw = runCartesian(mg, path, eef_step, vel, acc, execute, log, "그리기");

  // ── 9. 결과 ────────────────────────────────────────────────────────────────
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
    const auto & at = path[std::min(idx, path.size() - 1)].position;
    RCLCPP_ERROR(log,
      "경로가 %zu 번째 웨이포인트 부근에서 끊겼다 (전체 %zu) — (%.1f, %.1f, %.1f mm). "
      "draw_size 로 줄여 보거나(현재 긴 변 %.0f mm) 종이를 로봇 쪽으로 당길 것.",
      idx, path.size(), at.x * 1000.0, at.y * 1000.0, at.z * 1000.0,
      std::max(w_px, h_px) * s * 1000.0);
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
