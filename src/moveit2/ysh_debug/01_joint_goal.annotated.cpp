// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  예제 1 · 관절 목표 (joint-space goal)  — 한 줄씩 해설본                 ║
// ║                                                                          ║
// ║  ⚠️ 이것은 **읽기용 사본**이다. 빌드되지 않는다.                          ║
// ║     실제 빌드되는 원본:                                                  ║
// ║     ws_moveit2/src/hcr5_examples/src/01_joint_goal.cpp                   ║
// ║     원본을 고치면 이 파일도 같이 고쳐야 한다 (자동 동기화 없음).          ║
// ╚══════════════════════════════════════════════════════════════════════════╝
//
// 【이 예제가 증명하는 것】
//   IK(역기구학)를 전혀 쓰지 않고, "계획 → 실행" 사슬만 순수하게 확인한다.
//   관절 각도를 직접 지정하므로 "그 자세가 가능한가"를 따질 필요가 없다.
//   → 여기서 실패하면 IK 문제가 아니라 **컨트롤러/통신 문제**다.
//
// 【전체 흐름】
//   rclcpp 초기화
//     → 노드 생성 (파라미터 자동 선언 옵션 필수)
//       → 스핀 스레드 시작 (이게 없으면 현재 상태를 못 받는다)
//         → MoveGroupInterface 생성 (로봇 모델 로드 + move_group 에 연결)
//           → 현재 관절값 읽기
//             → 목표 설정 → plan() → execute()
//               → 정리 종료


// ─────────────────────────────────────────────────────────────────────────────
// 【1】 헤더
// ─────────────────────────────────────────────────────────────────────────────

#include <thread>
// std::thread 를 쓰기 위해. 아래 【4】에서 executor 를 별도 스레드로 돌린다.
// MoveGroupInterface 는 "현재 로봇 상태"를 토픽(/joint_states)과 TF 로 받는데,
// 노드가 spin 하지 않으면 그 메시지가 **영원히 도착하지 않는다.**

#include <rclcpp/rclcpp.hpp>
// ROS 2 C++ 클라이언트 라이브러리. Node, 로거, executor, init/shutdown 전부 여기.

#include <moveit/move_group_interface/move_group_interface.hpp>
// MoveGroupInterface — move_group 노드에 대한 **클라이언트 래퍼**다.
// 이 클래스가 내부적으로 하는 일:
//   · robot_description 파라미터로 로봇 모델(URDF+SRDF) 로드
//   · /move_action 액션 클라이언트 생성 (계획 요청용)
//   · CurrentStateMonitor 시작 (/joint_states 구독 + TF 리스너)
//   · /execute_trajectory 액션 클라이언트 생성 (실행 요청용)
// 즉 "MoveIt 을 쓴다"는 건 대부분 이 클래스를 쓴다는 뜻이다.
//
// ※ Jazzy 부터 확장자가 .h → .hpp 로 바뀌었다. 옛 예제는 .h 로 되어 있을 수 있다.


int main(int argc, char** argv)
{
  // ───────────────────────────────────────────────────────────────────────────
  // 【2】 ROS 2 초기화
  // ───────────────────────────────────────────────────────────────────────────

  rclcpp::init(argc, argv);
  // DDS 미들웨어를 띄우고, 커맨드라인의 --ros-args 를 파싱한다.
  // launch 파일이 넘긴 파라미터(robot_description 등)도 이 시점에 들어온다.
  // 반드시 노드 생성 **전에** 호출해야 한다.


  // ───────────────────────────────────────────────────────────────────────────
  // 【3】 노드 생성  ← 여기가 가장 자주 틀리는 곳
  // ───────────────────────────────────────────────────────────────────────────
  
  auto node = std::make_shared<rclcpp::Node>(
      "hcr5_joint_goal",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  //
  // ★ automatically_declare_parameters_from_overrides(true) 가 **필수**다.
  //
  //   ROS 2 는 원래 파라미터를 코드에서 declare_parameter() 로 **미리 선언**해야
  //   쓸 수 있다. 그런데 MoveGroupInterface 가 요구하는 파라미터는
  //     robot_description                (URDF 문자열)
  //     robot_description_semantic       (SRDF 문자열)
  //     robot_description_kinematics.*   (IK 솔버 설정)
  //     robot_description_planning.*     (joint_limits)
  //   처럼 개수도 많고 이름도 동적이다. 전부 손으로 선언할 수 없다.
  //
  //   이 옵션을 켜면 **외부에서 넘어온 파라미터를 자동으로 선언**해 준다.
  //   끄면(기본값) MoveGroupInterface 가 로봇 모델을 못 찾아 죽는다:
  //     "Failed to fetch parameter 'robot_description'"
  //
  //   → 그 파라미터를 실제로 넣어주는 것이 launch/example.launch.py 다.
  //     그래서 `ros2 run` 으로 직접 띄우면 안 된다.

  auto logger = node->get_logger();
  // RCLCPP_INFO 등에 넘길 로거. 노드 이름이 로그 앞에 붙는다.


  // ───────────────────────────────────────────────────────────────────────────
  // 【4】 스핀 스레드  ← 두 번째로 자주 틀리는 곳
  // ───────────────────────────────────────────────────────────────────────────

  rclcpp::executors::SingleThreadedExecutor executor;
  // executor = 콜백을 실제로 실행하는 엔진.
  // 구독 콜백, 타이머, 액션 응답이 전부 executor 를 통해 처리된다.

  executor.add_node(node);
  // 이 노드의 콜백들을 executor 가 관리하도록 등록.

  auto spinner = std::thread([&executor]() { executor.spin(); });
  //
  // ★ 왜 **별도 스레드**인가?
  //
  //   executor.spin() 은 블로킹이다 — 호출하면 안 돌아온다.
  //   그런데 우리는 main 흐름에서 plan()/execute() 를 순차적으로 호출해야 한다.
  //   그래서 spin 은 뒷 스레드에 맡기고, main 은 계속 진행한다.
  //
  //   이걸 빼면 어떻게 되나:
  //     getCurrentJointValues() 가 **영원히 블로킹**되거나 빈 값을 반환한다.
  //     /joint_states 메시지가 도착해도 처리할 executor 가 없기 때문이다.
  //     증상: "Failed to fetch current robot state" 또는 무한 대기.
  //
  //   대안: rclcpp::executors::MultiThreadedExecutor 를 쓰거나
  //         rclcpp::spin_some() 을 루프에서 호출하는 방법도 있다.


  // ───────────────────────────────────────────────────────────────────────────
  // 【5】 MoveGroupInterface 생성
  // ───────────────────────────────────────────────────────────────────────────

  using moveit::planning_interface::MoveGroupInterface;
  // 이름이 길어서 별칭. (using 문 자체는 선택)

  MoveGroupInterface mg(node, "hcr_arm");
  //
  // "hcr_arm" = **SRDF 에 정의된 플래닝 그룹 이름**이다.
  //   hcr5_moveit_config/config/hcr5.srdf 에 이렇게 들어 있다:
  //     <group name="hcr_arm">
  //       <chain base_link="base_link" tip_link="pen_tip"/>
  //     </group>
  //
  // ★ tip_link 가 pen_tip 이라는 것이 이 프로젝트의 핵심이다.
  //   그래서 이 그룹으로 계획하면 **펜 끝이 기준점**이 된다.
  //   (기본값이었다면 link6_1 = 플랜지가 기준이 됐을 것이다)
  //
  // 생성자가 블로킹으로 하는 일:
  //   1. robot_description 파라미터 읽어 로봇 모델 생성
  //   2. move_group 노드의 액션 서버가 뜰 때까지 대기
  //   3. CurrentStateMonitor 시작
  // → move_group 이 안 떠 있으면 여기서 오래 멈추거나 예외가 난다.

  mg.setMaxVelocityScalingFactor(0.1);
  mg.setMaxAccelerationScalingFactor(0.1);
  //
  // joint_limits.yaml 의 한계값에 **곱해지는** 비율이다 (0.0 ~ 1.0).
  //   실효 속도 = max_velocity(3.1416 rad/s) × 0.1 = 0.314 rad/s ≈ 18°/s
  //   실효 가속 = max_acceleration(3.5 rad/s²) × 0.1 = 0.35 rad/s² ≈ 20°/s²
  //
  // 실기 연결 전까지는 낮게 유지한다. mock 이라도 습관을 들여야
  // 실기 붙일 때 실수하지 않는다.


  // ───────────────────────────────────────────────────────────────────────────
  // 【6】 진단 로그 — 설정이 의도대로인지 확인
  // ───────────────────────────────────────────────────────────────────────────

  RCLCPP_INFO(logger, "planning frame : %s", mg.getPlanningFrame().c_str());
  // 계획의 기준 좌표계. 보통 SRDF virtual_joint 의 parent_frame 또는 URDF 루트.
  // 우리는 base_link 가 나온다. 모든 pose 목표가 이 프레임 기준으로 해석된다.

  RCLCPP_INFO(logger, "end effector   : %s  ← pen_tip 이어야 한다",
              mg.getEndEffectorLink().c_str());
  //
  // ★ 이 한 줄이 이 프로젝트에서 가장 중요한 검증이다.
  //
  //   SRDF 에 <end_effector> 를 정의하지 않았는데도 pen_tip 이 나오는 이유:
  //   MoveGroupInterface 는 EE 가 명시되지 않으면 **그룹 체인의 마지막 링크**를
  //   EE 로 삼는다. 우리 체인이 base_link → … → pen_tip 이므로 pen_tip 이다.
  //
  //   여기서 link6_1 이 나오면 → Planning Group 을 조인트 나열로 잡은 것.
  //   Setup Assistant 에서 'Add Kin. Chain' 으로 다시 설정해야 한다.


  // ───────────────────────────────────────────────────────────────────────────
  // 【7】 현재 관절값 읽기
  // ───────────────────────────────────────────────────────────────────────────

  auto joints = mg.getCurrentJointValues();
  //
  // 반환형: std::vector<double>, 그룹에 속한 관절 순서대로.
  // 값의 출처: CurrentStateMonitor → /joint_states 토픽
  //            → joint_state_broadcaster → ros2_control 하드웨어
  //
  // 즉 이 값이 정상이면 **하드웨어까지의 읽기 경로가 살아 있다**는 뜻이다.
  // (mock 이든 실기든 동일한 경로다 — 그래서 mock 검증이 의미가 있다)
  //
  // ⚠️ 【4】의 스핀 스레드가 없으면 여기서 멈추거나 빈 벡터가 온다.

  if (joints.size() != 6)
  {
    // 방어 코드. 6개가 아니면 대개 두 경우다:
    //   · demo.launch.py 가 안 떠 있다 (상태를 못 받음 → 빈 벡터)
    //   · 그룹 정의가 잘못됐다
    RCLCPP_ERROR(logger, "관절값을 %zu 개 받았다 (6개 기대). demo.launch.py 가 떠 있는가?",
                 joints.size());
    executor.cancel();   // spin 루프 종료 신호
    spinner.join();      // 스레드가 실제로 끝날 때까지 대기
    rclcpp::shutdown();  // ROS 정리
    return 1;            // 비정상 종료 코드
  }

  RCLCPP_INFO(logger, "현재 관절값 [%.3f %.3f %.3f %.3f %.3f %.3f]",
              joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
  // 예: [0.000 0.000 1.567 -1.590 -1.470 0.000]
  // 이 값은 hcr5.ros2_control.xacro 의 <param name="initial_value"> 에서 온다
  // (mock 하드웨어는 시작 시 그 값으로 초기화된다).


  // ───────────────────────────────────────────────────────────────────────────
  // 【8】 목표 설정
  // ───────────────────────────────────────────────────────────────────────────

  joints[0] += 0.3;
  // joint_1(베이스 회전)만 +0.3 rad ≈ +17.2°.
  //
  // ★ 절대값을 박지 않고 **현재값 기준 상대 이동**인 이유:
  //   어느 자세에서 실행하든 대체로 도달 가능하기 때문이다.
  //   절대값을 박으면 로봇이 이미 그 근처에 있을 때 "안 움직인 것처럼" 보이거나,
  //   관절 한계 밖이면 계획이 실패한다.
  //
  // ★ 여기서 joint_1 의 **음수 방향이 열려 있는 것**이 중요하다.
  //   URDF 를 실기값으로 고치기 전에는 lower="0.0" 이라, 0 근처에서 음수로
  //   가려 하면 348° 대회전으로 계획됐다. (상세본 §5.1)

  RCLCPP_INFO(logger, "목표 관절값 joint_1 = %.3f rad (%.1f°)",
              joints[0], joints[0] * 180.0 / M_PI);

  mg.setJointValueTarget(joints);
  //
  // 관절 공간 목표를 설정한다. **IK 를 호출하지 않는다** — 이미 관절값이니까.
  //
  // 비교:
  //   setJointValueTarget(vector<double>)  관절 공간. IK 불필요. 항상 해가 있음
  //   setPoseTarget(Pose)                  데카르트 공간. **IK 필요**. 해가 없을 수 있음
  //   setNamedTarget("home")               SRDF 의 group_state 이름으로
  //
  // 이 예제가 IK 를 안 쓰는 이유가 이것이다. 실패하면 원인이 IK 가 아님이 확실해진다.


  // ───────────────────────────────────────────────────────────────────────────
  // 【9】 계획 (plan)
  // ───────────────────────────────────────────────────────────────────────────

  MoveGroupInterface::Plan plan;
  // 계획 결과를 담을 구조체. 안에 trajectory_(RobotTrajectory), planning_time_ 등.

  if (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    // plan() 이 하는 일:
    //   1. /move_action 액션으로 move_group 에 MotionPlanRequest 전송
    //   2. move_group 이 파이프라인 실행:
    //        어댑터(ValidateWorkspaceBounds, CheckStartStateBounds,
    //               CheckStartStateCollision)
    //        → 플래너(OMPL / RRTConnect)
    //        → 응답 어댑터(AddTimeOptimalParameterization) ★
    //   3. 결과 궤적을 plan 에 채워 반환
    //
    // ★ AddTimeOptimalParameterization 단계에서 joint_limits.yaml 의
    //   **가속도 한계**가 필요하다. 없으면 여기서 실패한다:
    //     "No acceleration limit was defined for joint joint_1!"
    //   URDF <limit> 태그에는 가속도 항목이 아예 없어서 반드시 yaml 로 준다.
    //   (상세본 §7.1)
    //
    // 반환형 MoveItErrorCode 는 int 로 암묵 변환되지만, 명시적으로
    // SUCCESS 와 비교하는 편이 읽기 좋다.

    RCLCPP_INFO(logger, "계획 성공 — 실행한다");

    // ─────────────────────────────────────────────────────────────────────────
    // 【10】 실행 (execute)
    // ─────────────────────────────────────────────────────────────────────────

    auto result = mg.execute(plan);
    //
    // execute() 가 하는 일:
    //   1. /execute_trajectory 액션으로 move_group 에 궤적 전송
    //   2. move_group 의 TrajectoryExecutionManager 가
    //      moveit_controllers.yaml 을 보고 담당 컨트롤러를 찾음
    //      → hcr_arm_controller, action_ns: follow_joint_trajectory
    //   3. FollowJointTrajectory 액션으로 JointTrajectoryController 에 전달
    //   4. JTC 가 ros2_control 하드웨어의 position 인터페이스에 씀
    //   5. 완료까지 **블로킹**
    //
    // ★ 여기가 계획과 실행의 경계다.
    //   3번에서 실패하면: "Action client not connected to action server:
    //                      hcr_arm_controller/follow_joint_trajectory"
    //   → 컨트롤러가 안 떠 있다는 뜻. ros2_controllers.yaml 의
    //     command_interfaces 가 비어 있으면 이렇게 된다. (상세본 §7.2)
    //
    // ※ plan()+execute() 대신 mg.move() 를 쓰면 둘을 한 번에 한다.
    //   나눠 쓰는 이유: 계획만 하고 사람이 검토한 뒤 실행하는 흐름
    //   (실기에서 필요)과 궤적 데이터를 따로 저장하는 흐름을 위해서다.

    RCLCPP_INFO(logger, "실행 결과: %s",
                result == moveit::core::MoveItErrorCode::SUCCESS ? "성공" : "실패");
  }
  else
  {
    RCLCPP_ERROR(logger, "계획 실패");
    // 관절 공간 목표에서 계획이 실패하는 경우:
    //   · 목표가 관절 한계 밖
    //   · 시작 상태나 목표 상태가 자기충돌
    //   · 시작 상태를 못 읽음
  }


  // ───────────────────────────────────────────────────────────────────────────
  // 【11】 정리 종료  ← 순서가 중요하다
  // ───────────────────────────────────────────────────────────────────────────

  executor.cancel();
  // spin() 루프에 종료 신호. 이걸 안 하면 spinner.join() 에서 영원히 멈춘다.

  spinner.join();
  // 스레드가 실제로 끝날 때까지 대기.
  // join() 없이 std::thread 소멸자가 호출되면 std::terminate 로 프로그램이 죽는다.

  rclcpp::shutdown();
  // ROS 컨텍스트 정리. 발행 중인 메시지 flush, DDS 자원 해제.

  return 0;
}


// ═══════════════════════════════════════════════════════════════════════════════
// 【부록】 이 예제로 진단할 수 있는 것
// ═══════════════════════════════════════════════════════════════════════════════
//
//  증상                                    | 의심 지점
//  ---------------------------------------|--------------------------------------
//  로봇 모델 로드 실패                      | 【3】 파라미터. ros2 run 으로 띄웠나?
//  관절값을 못 받음 / 무한 대기             | 【4】 스핀 스레드. demo 가 떠 있나?
//  end effector 가 link6_1                 | SRDF 그룹이 chain 이 아님
//  "No acceleration limit"                 | joint_limits.yaml 가속도 (§7.1)
//  "Action client not connected"           | ros2_controllers.yaml 인터페이스 (§7.2)
//  실행 성공인데 RViz 가 안 움직임           | Scene Robot alpha. 주황은 Goal State다
//
// 관련 문서: src/moveit2/docs/ysh_HCR5_ros2_control_구축_상세.md
