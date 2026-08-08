// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  예제 3 · 데카르트 직선 경로 (사각형 그리기)  — 한 줄씩 해설본           ║
// ║                                                                          ║
// ║  ⚠️ 읽기용 사본. 빌드되지 않는다.                                         ║
// ║     원본: ws_moveit2/src/hcr5_examples/src/03_cartesian_square.cpp       ║
// ╚══════════════════════════════════════════════════════════════════════════╝
//
// ═══ 이 예제가 왜 가장 중요한가 ═══════════════════════════════════════════════
//
//   그리기 = 펜 끝이 **정해진 선을 따라가는 것**이다.
//   예제 2(setPoseTarget)로는 이게 안 된다 — 끝점만 맞고 가는 길은 제멋대로다.
//
//     setPoseTarget      A ~~~~(관절공간 보간, 실제 경로는 곡선)~~~~> B
//     computeCartesianPath  A ────(데카르트 직선)────> B
//
//   종이 위에 직선을 그으려면 후자여야 한다. 그래서 이 함수가
//   **그리기 파이프라인의 핵심 도구**다.
//
// ═══ 이 예제가 답하는 질문 ═══════════════════════════════════════════════════
//
//   "우리 로봇이 평면에 선을 그릴 수 있는가?"
//
//   답은 **달성률(fraction)** 하나로 나온다. 1.00 이면 전 구간 IK 가 풀렸다는 뜻.
//   이 값이 낮으면 실기에 가기 전에 설계를 바꿔야 한다.
//
// ═══ 전체 흐름 ═══════════════════════════════════════════════════════════════
//
//   현재 자세 읽기
//     → 경유점(waypoint) 4개로 사각형 정의
//       → computeCartesianPath 로 직선 보간 궤적 생성   ← 달성률 확인
//         → TOTG 로 시간 다시 입히기                     ← joint_limits 사용
//           → 실행
//             → 시작점 복귀 오차로 폐합 확인


// ─────────────────────────────────────────────────────────────────────────────
// 【1】 헤더 — 예제 1·2보다 많다
// ─────────────────────────────────────────────────────────────────────────────

#include <thread>
#include <vector>
// waypoints 를 std::vector 로 담는다.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
// 경유점 타입. 예제 2는 PoseStamped 를 썼지만 여기서는 순수 Pose 배열이 필요하다
// (computeCartesianPath 의 인자 타입이 std::vector<geometry_msgs::msg::Pose>).

#include <moveit_msgs/msg/robot_trajectory.hpp>
// computeCartesianPath 의 **출력** 타입.
// ★ MoveGroupInterface::Plan 이 아니라 RobotTrajectory 메시지가 나온다는 점에 주의.
//   그래서 plan() 을 거치지 않고 바로 execute(traj_msg) 로 넘긴다.

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <moveit/robot_trajectory/robot_trajectory.hpp>
// robot_trajectory::RobotTrajectory — 궤적의 **C++ 객체** 표현.
// 메시지(moveit_msgs::msg::RobotTrajectory)와 다르다:
//   메시지  = 통신용 평평한 데이터
//   객체    = 로봇 모델과 연결된, 처리 가능한 형태
// 시간 매개변수화 같은 후처리는 객체 쪽에서만 된다.

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
// TOTG (Time-Optimal Trajectory Generation).
// 궤적의 각 점에 "언제 지날지"를 계산해 넣는 알고리즘.
// 속도·가속도 한계를 지키면서 최단 시간이 되도록 배분한다.


int main(int argc, char** argv)
{
  // ───────────────────────────────────────────────────────────────────────────
  // 【2】 초기화 — 예제 1·2와 동일한 정형
  // ───────────────────────────────────────────────────────────────────────────

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


  // ───────────────────────────────────────────────────────────────────────────
  // 【3】 조정 가능한 상수 — 실험할 때 여기만 바꾸면 된다
  // ───────────────────────────────────────────────────────────────────────────

  const double SIDE = 0.10;
  // 사각형 한 변 [m]. 10cm.
  // A4 는 210×297mm 이므로, 실제 그리기를 흉내내려면 0.21 / 0.297 로 키워본다.
  // ★ 키울수록 달성률이 떨어질 가능성이 높다 — 그게 "작업영역 찾기" 실험이다.

  const double EEF_STEP = 0.005;
  // 보간 간격 [m]. 5mm 마다 IK 를 한 번 푼다는 뜻.
  //
  // ★ 이 값의 트레이드오프
  //   작게 (1mm)  선이 매끄럽다 · IK 호출이 많아 느리다 · 궤적 점이 많아진다
  //   크게 (2cm)  빠르다 · 선이 각지고, 중간에 장애물/특이점을 놓칠 수 있다
  //
  //   한 변 10cm 를 5mm 로 나누면 20점, 네 변이니 약 80점.
  //   실제 결과가 72점이었다 (시작/끝 중복 제거 등으로 조금 줄어든다).
  //
  //   ※ 실기에서는 이 값이 **명령 전송 주기**와 맞물린다. 점이 너무 많으면
  //     MQTT 로 다 못 보낸다. 3단계에서 다시 조정 대상이 된다.

  const double VEL_SCALE = 0.1;
  const double ACC_SCALE = 0.1;
  // TOTG 에 넘길 스케일. joint_limits.yaml 한계에 곱해진다.
  // ★ 예제 1·2와 달리 setMaxVelocityScalingFactor() 를 쓰지 않는다.
  //   computeCartesianPath 는 MoveGroupInterface 의 스케일 설정을 **쓰지 않기 때문**이다.
  //   시간은 아래 TOTG 단계에서 따로 입히므로, 스케일도 거기에 넘겨야 한다.


  RCLCPP_INFO(logger, "end effector : %s", mg.getEndEffectorLink().c_str());
  // ★ 아래 waypoints 는 **이 링크**가 지나갈 경로다. pen_tip 이어야 한다.


  // ───────────────────────────────────────────────────────────────────────────
  // 【4】 시작점 확보
  // ───────────────────────────────────────────────────────────────────────────

  auto start = mg.getCurrentPose().pose;
  // PoseStamped 에서 .pose 만 꺼낸다 (헤더는 어차피 planning frame 이라 자명).

  RCLCPP_INFO(logger, "시작점 [%.3f %.3f %.3f]",
              start.position.x, start.position.y, start.position.z);


  // ───────────────────────────────────────────────────────────────────────────
  // 【5】 경유점(waypoint) 정의 — 사각형 만들기
  // ───────────────────────────────────────────────────────────────────────────

  std::vector<geometry_msgs::msg::Pose> waypoints;
  // computeCartesianPath 는 이 **점들을 순서대로 직선으로 이어** 경로를 만든다.
  //
  // ★ 시작점은 넣지 않는다. 함수가 "현재 위치에서 첫 waypoint 까지"를
  //   자동으로 이어주기 때문이다. 넣으면 길이 0짜리 구간이 생겨 경고가 날 수 있다.

  auto p = start;
  // p 를 "펜을 옮기는 커서"처럼 쓴다. start 복사본이므로 orientation 이 유지된다.
  //
  // ★ orientation 을 그대로 두는 것이 중요하다.
  //   사각형을 그리는 동안 펜 기울기가 변하면 안 되기 때문이다.
  //   (실제 그리기에서는 펜이 종이에 수직이어야 하므로 여기서 자세를 정한다)

  p.position.x += SIDE;  waypoints.push_back(p);   // ① 오른쪽으로 10cm
  p.position.y += SIDE;  waypoints.push_back(p);   // ② 위로 10cm
  p.position.x -= SIDE;  waypoints.push_back(p);   // ③ 왼쪽으로 10cm
  p.position.y -= SIDE;  waypoints.push_back(p);   // ④ 아래로 10cm → 시작점 복귀
  //
  // p 를 누적 수정하므로 **절대 좌표가 아니라 이어지는 경로**가 된다.
  //   ①  (x+S, y  )
  //   ②  (x+S, y+S)
  //   ③  (x  , y+S)
  //   ④  (x  , y  )  = 시작점
  //
  // ★ base_link 의 XY 평면에 그린다. 실제 종이는 책상 위(대략 XY 평면)이므로
  //   개념적으로 맞다. 다만 **종이의 실제 평면과 정렬돼 있지는 않다** —
  //   그건 5단계 캘리브레이션에서 맞춘다.


  // ───────────────────────────────────────────────────────────────────────────
  // 【6】 데카르트 경로 계산  ★ 이 예제의 핵심
  // ───────────────────────────────────────────────────────────────────────────

  moveit_msgs::msg::RobotTrajectory traj_msg;
  // 출력이 담길 곳. 함수가 채워준다.

  double fraction = mg.computeCartesianPath(waypoints, EEF_STEP, traj_msg);
  //
  // ═══ 이 함수가 내부에서 하는 일 ═══
  //
  //   1. 현재 자세 → waypoint① → ② → ③ → ④ 를 잇는 직선을 그린다
  //   2. 그 직선을 EEF_STEP(5mm) 간격으로 잘게 나눈다
  //   3. **각 점마다 IK 를 푼다** (KDL 솔버)
  //   4. 앞 점의 해를 다음 IK 의 초기값으로 써서 관절값이 연속되게 한다
  //   5. 충돌 검사 (avoid_collisions 기본값 true)
  //   6. IK 가 실패하거나 충돌하면 **거기서 멈추고** 그때까지의 궤적만 반환
  //
  // ═══ 반환값 fraction ═══
  //
  //   요청한 경로 중 **몇 %를 실제로 만들었는가**.
  //
  //     1.00  전 구간 성공 — 정상
  //     0.65  65% 지점에서 막혔다 (IK 실패 / 특이점 / 충돌)
  //     0.00  첫 점부터 실패
  //
  //   ★ 이 값이 실기에서 "선이 왜 일그러지는가"의 1차 지표가 된다.
  //     낮게 나올 때의 대응:
  //       · 시작 자세를 바꾼다 (팔이 뻗은 상태보다 굽힌 상태가 여유 있다)
  //       · 사각형 위치·크기를 바꾼다 → **A4 를 어디 놓을지가 여기서 정해진다**
  //       · IK 솔버 교체 (KDL → TRAC-IK)
  //       · 자세 제약(orientation constraint) 추가
  //
  // ═══ Jazzy 의 시그니처 변화 ═══
  //
  //   구버전: computeCartesianPath(waypoints, eef_step, jump_threshold, traj, ...)
  //   Jazzy : computeCartesianPath(waypoints, eef_step, traj, ...)
  //
  //   jump_threshold 는 "관절값이 갑자기 튀면 중단"하는 안전장치였는데,
  //   오작동이 잦아 **deprecated** 되었다 (인자는 남아 있지만 무시된다).
  //   옛 예제를 그대로 복사하면 컴파일은 되지만 그 인자가 의미가 없다.

  RCLCPP_INFO(logger, "데카르트 경로 달성률: %.1f%%  (구간 %zu 개)",
              fraction * 100.0, traj_msg.joint_trajectory.points.size());
  // points.size() = 보간된 궤적점 개수. 실측 72개.
  // 이 점 하나하나가 관절값 6개 + 시각을 갖는다.
  // ★ 3단계에서 MQTT 로 실기에 보낼 것이 바로 이 점들이다.

  if (fraction < 0.99)
  {
    // 0.99 를 기준으로 삼는 이유: 부동소수 오차로 1.00 이 0.9999… 로 나올 수 있다.
    RCLCPP_ERROR(logger,
                 "경로를 %.1f%% 밖에 못 만들었다. 중간에 IK 가 끊겼거나 "
                 "작업영역/특이점 문제다. 시작 자세를 바꿔 다시 시도할 것.",
                 fraction * 100.0);
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
    // ★ 부분 궤적을 실행하지 않는다. 반쪽짜리 사각형을 그리는 것보다
    //   실패를 명확히 알리는 게 낫다.
  }


  // ───────────────────────────────────────────────────────────────────────────
  // 【7】 시간 매개변수화 (TOTG)  ← 빼먹으면 실행이 이상해진다
  // ───────────────────────────────────────────────────────────────────────────
  //
  // ★ 왜 필요한가
  //
  //   computeCartesianPath 가 만든 궤적은 **기하학적 경로**다.
  //   "어디를 지날지"는 있는데 "언제 지날지"가 부실하다 —
  //   각 점의 time_from_start 가 0이거나 비현실적인 값이다.
  //
  //   이 상태로 실행하면 컨트롤러가
  //     · 궤적을 거부하거나
  //     · 순간이동하듯 움직이려다 한계를 넘거나
  //     · 뚝뚝 끊기게 움직인다
  //
  //   그래서 속도·가속도 한계를 지키는 시간표를 다시 계산해 넣는다.
  //
  // ★ plan() 경로에서는 이 단계가 자동이다
  //   예제 1·2는 move_group 내부의 AddTimeOptimalParameterization 어댑터가
  //   알아서 해줬다. computeCartesianPath 는 move_group 을 거치지 않고
  //   **클라이언트 쪽에서 직접 계산**하므로 우리가 해야 한다.

  robot_trajectory::RobotTrajectory rt(mg.getRobotModel(), "hcr_arm");
  // 메시지를 처리 가능한 객체로 만들 그릇.
  //   mg.getRobotModel()  URDF+SRDF 로부터 만들어진 로봇 모델
  //   "hcr_arm"           어느 그룹의 궤적인지

  rt.setRobotTrajectoryMsg(*mg.getCurrentState(), traj_msg);
  // 메시지 → 객체 변환.
  // 첫 인자가 **기준 상태(reference state)** 다. 궤적 메시지는 관절값만 담고
  // 나머지 상태(그룹에 없는 관절, 부착물 등)는 안 담기 때문에,
  // 현재 상태를 바탕으로 채워 넣는다.
  //
  // getCurrentState() 는 shared_ptr 을 주므로 * 로 역참조한다.

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  // 시간 매개변수화 알고리즘 객체.
  // 대안: IterativeParabolicTimeParameterization (구식, 덜 정확)
  //       RuckigSmoothing (저크 제한까지, 더 부드러움)
  // ★ 그리기처럼 매끄러움이 중요한 작업은 나중에 Ruckig 검토 대상이다.

  if (!totg.computeTimeStamps(rt, VEL_SCALE, ACC_SCALE))
  {
    // ★ 여기가 joint_limits.yaml 의 **가속도 한계**를 실제로 쓰는 지점이다.
    //   has_acceleration_limits: false 이면 여기서 실패한다.
    //   (같은 문제가 예제 1·2에서는 move_group 안에서 터졌다 — 상세본 §7.1)
    RCLCPP_ERROR(logger, "시간 매개변수화 실패 — joint_limits.yaml 의 가속도 한계를 확인할 것");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  rt.getRobotTrajectoryMsg(traj_msg);
  // 객체 → 메시지 역변환. traj_msg 가 이제 시간 정보를 갖게 된다.
  // (같은 변수에 덮어쓴다)

  RCLCPP_INFO(logger, "총 소요시간 %.2f 초 — 실행한다",
              rclcpp::Duration(traj_msg.joint_trajectory.points.back().time_from_start).seconds());
  // 마지막 점의 time_from_start = 전체 소요시간. 실측 7.00초.
  // ★ 이 값이 스케일(0.1)에 반비례한다. 0.2 로 올리면 대략 절반이 된다.


  // ───────────────────────────────────────────────────────────────────────────
  // 【8】 실행
  // ───────────────────────────────────────────────────────────────────────────

  auto result = mg.execute(traj_msg);
  // ★ 예제 1·2와 다르다: Plan 이 아니라 **RobotTrajectory 메시지**를 직접 넘긴다.
  //   computeCartesianPath 가 Plan 을 만들지 않기 때문이다.
  //   execute() 에는 두 오버로드가 있어서 둘 다 받는다.
  //
  // 이후 경로는 예제 1과 동일:
  //   /execute_trajectory → TrajectoryExecutionManager
  //   → FollowJointTrajectory → JTC → ros2_control → 하드웨어

  RCLCPP_INFO(logger, "실행 결과: %s",
              result == moveit::core::MoveItErrorCode::SUCCESS ? "성공" : "실패");


  // ───────────────────────────────────────────────────────────────────────────
  // 【9】 폐합 검증 — 사각형이 제대로 닫혔는가
  // ───────────────────────────────────────────────────────────────────────────

  auto end = mg.getCurrentPose().pose;
  const double dx = end.position.x - start.position.x;
  const double dy = end.position.y - start.position.y;
  const double dz = end.position.z - start.position.z;

  RCLCPP_INFO(logger, "시작점 복귀 오차: %.4f m (사각형이 닫혔는지의 지표)",
              std::sqrt(dx * dx + dy * dy + dz * dz));
  //
  // ★ 왜 이걸 재는가
  //
  //   네 변을 돌면 원리적으로 시작점에 정확히 돌아와야 한다.
  //   오차가 크면 어딘가에서 **누적 오차**가 생긴 것이다:
  //     · IK 근사해가 조금씩 어긋남
  //     · 컨트롤러 추종 오차
  //     · 궤적이 중간에 잘림
  //
  //   mock 에서는 0.0000 m 가 나왔다 (기계 오차가 없으니 당연).
  //   ★ 실기에서는 이 값이 **0이 아니게 되고, 그게 곧 선 품질 지표**가 된다.
  //     HCR-5 반복정밀도가 ±0.1mm 이므로 그 언저리면 정상,
  //     mm 단위로 벌어지면 추종 문제를 의심한다.


  // ───────────────────────────────────────────────────────────────────────────
  // 【10】 정리 종료
  // ───────────────────────────────────────────────────────────────────────────

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}


// ═══════════════════════════════════════════════════════════════════════════════
// 【부록 A】 세 예제의 관계
// ═══════════════════════════════════════════════════════════════════════════════
//
//              | 입력         | IK  | 중간 경로      | 그리기에 쓸 수 있나
//   -----------|--------------|-----|----------------|--------------------
//   예제 1     | 관절각 6개   | ✗   | 관절공간 보간  | ✗ (홈 자세 이동용)
//   예제 2     | 목표 자세    | ○   | 관절공간 보간  | ✗ (끝점만 맞음)
//   예제 3     | 경유점 배열  | ○○  | **데카르트 직선** | ★ 이것
//
//   ※ 예제 3의 IK 는 점 하나가 아니라 **수십~수백 점**에 대해 연속으로 풀린다.
//     그래서 IK 솔버의 안정성이 여기서 처음으로 문제가 된다.
//
// ═══════════════════════════════════════════════════════════════════════════════
// 【부록 B】 2단계에서 확장할 방향
// ═══════════════════════════════════════════════════════════════════════════════
//
//   1. 도형을 늘린다 — 원, 지그재그, A4 크기 사각형
//      → **달성률 100% 가 유지되는 영역**을 찾는다
//      → 그 결과가 "A4 를 로봇 앞 어디에 놓아야 하는가"의 답이다
//
//   2. 자취를 선으로 시각화 (hcr_viz)
//      → /joint_states 구독 → TF 에서 pen_tip → Marker(LINE_STRIP)
//      → 입력한 도형과 실제 지나간 선을 **겹쳐 보고 편차를 수치화**
//
//   3. 궤적을 파일로 저장
//      → 3단계에서 MoveIt 없이 브릿지 단독 테스트에 쓴다
//      → 가짜 HCR-5 스텁의 입력으로도 그대로 쓴다
//
// ═══════════════════════════════════════════════════════════════════════════════
// 【부록 C】 진단표
// ═══════════════════════════════════════════════════════════════════════════════
//
//  증상                                  | 의심 지점
//  -------------------------------------|--------------------------------------
//  달성률이 낮다 (< 100%)                | 시작 자세 / 도형 크기 / 특이점 / IK 솔버
//  달성률 0%                             | 시작 자세가 이미 한계. EEF_STEP 확인
//  "시간 매개변수화 실패"                 | joint_limits.yaml 가속도 (§7.1)
//  실행은 성공인데 움직임이 뚝뚝 끊김      | EEF_STEP 이 너무 큼 / 스케일 문제
//  복귀 오차가 크다                       | IK 누적 오차 or 궤적 잘림
//
// 관련 문서: src/moveit2/docs/ysh_HCR5_ros2_control_구축_상세.md
//            src/moveit2/ws_moveit2/src/hcr5_examples/README.md
