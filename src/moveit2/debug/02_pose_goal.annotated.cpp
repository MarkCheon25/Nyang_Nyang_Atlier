// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  예제 2 · 자세 목표 (pose goal, IK)  — 한 줄씩 해설본                    ║
// ║                                                                          ║
// ║  ⚠️ 읽기용 사본. 빌드되지 않는다.                                         ║
// ║     원본: ws_moveit2/src/hcr5_examples/src/02_pose_goal.cpp              ║
// ╚══════════════════════════════════════════════════════════════════════════╝
//
// 【예제 1과 무엇이 다른가】
//
//   예제 1: 관절 각도 6개를 직접 지정        → IK 불필요, 항상 해가 있다
//   예제 2: "펜 끝이 이 위치·자세에 있어라"  → **IK 필요**, 해가 없을 수 있다
//
//   즉 이 예제부터 "그 자세가 물리적으로 가능한가"라는 질문이 생긴다.
//
// 【이 예제가 증명하는 것】
//   ★ IK 가 **플랜지(link6_1)가 아니라 펜 끝(pen_tip) 기준으로** 풀린다.
//
//   그리기 작업에서 우리가 제어하고 싶은 것은 펜 끝이다. 만약 IK 기준점이
//   플랜지라면, "펜 끝을 종이 위 (x,y)에 놓아라"를 매번 손으로 변환해야 한다.
//   SRDF 체인의 tip_link 를 pen_tip 으로 잡았기 때문에 그럴 필요가 없다.
//
// 【검증 방법】
//   현재 펜 끝 자세를 읽어 Z 로 -10cm 내린 곳을 목표로 준다.
//   실행 후 도달 위치를 다시 읽어 목표와의 오차를 출력한다.
//   오차가 ~0 이면 IK · 계획 · 실행이 전부 펜 끝 기준으로 일관되게 돌았다는 뜻.


#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
// 헤더 구성은 예제 1과 동일하다. 각 헤더의 역할은 01_joint_goal.annotated.cpp 참조.
// (pose 를 다루지만 geometry_msgs 를 따로 include 하지 않아도 되는 이유:
//  move_group_interface.hpp 가 이미 geometry_msgs/msg/pose_stamped.hpp 를 끌어온다)


int main(int argc, char** argv)
{
  // ───────────────────────────────────────────────────────────────────────────
  // 【1】 초기화 · 노드 · 스핀 — 예제 1과 동일한 정형(boilerplate)
  // ───────────────────────────────────────────────────────────────────────────

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "hcr5_pose_goal",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  // automatically_declare_parameters_from_overrides(true) 가 필수인 이유는
  // 예제 1 【3】참조. 이 예제는 추가로 **kinematics 파라미터**가 꼭 필요하다 —
  // IK 를 쓰기 때문이다. launch 가 넘기는
  //   moveit_config.robot_description_kinematics
  // 가 없으면 setPoseTarget 이후 계획이 항상 실패한다.

  auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  // getCurrentPose() 도 CurrentStateMonitor + TF 를 쓰므로 스핀이 반드시 필요하다.
  // 오히려 예제 1보다 더 중요하다 — TF 조회까지 하기 때문이다.

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface mg(node, "hcr_arm");
  mg.setMaxVelocityScalingFactor(0.1);
  mg.setMaxAccelerationScalingFactor(0.1);


  // ───────────────────────────────────────────────────────────────────────────
  // 【2】 진단 로그
  // ───────────────────────────────────────────────────────────────────────────

  RCLCPP_INFO(logger, "planning frame : %s", mg.getPlanningFrame().c_str());
  // ★ 이 프레임이 아래 목표 pose 의 **기준 좌표계**가 된다.
  //   우리는 base_link. 즉 "position.z -= 0.1" 은 base_link 기준으로 아래로 10cm.
  //   (setPoseReferenceFrame() 으로 바꿀 수 있다)

  RCLCPP_INFO(logger, "end effector   : %s  ← pen_tip 이어야 한다",
              mg.getEndEffectorLink().c_str());
  // ★ 이 링크가 아래 getCurrentPose()/setPoseTarget() 의 **대상**이다.
  //   pen_tip 이면 "펜 끝"을 옮기는 것이고, link6_1 이면 "플랜지"를 옮기는 것이다.
  //   150mm 차이가 나므로 그리기에서는 결정적이다.


  // ───────────────────────────────────────────────────────────────────────────
  // 【3】 현재 자세 읽기
  // ───────────────────────────────────────────────────────────────────────────

  auto current = mg.getCurrentPose();
  //
  // 반환형: geometry_msgs::msg::PoseStamped
  //   .header.frame_id  기준 프레임 (= planning frame = base_link)
  //   .pose.position    x, y, z          [m]
  //   .pose.orientation x, y, z, w       쿼터니언
  //
  // 내부 동작: TF 에서 planning_frame → end_effector_link 변환을 조회한다.
  //            즉 tf2_echo base_link pen_tip 과 같은 값이다.
  //
  // ★ 인자를 주지 않으면 **end effector link 기준**이다.
  //   getCurrentPose("link6_1") 처럼 다른 링크를 지정할 수도 있다.
  //   두 값을 비교하면 펜 오프셋 150mm 가 그대로 나온다.

  RCLCPP_INFO(logger, "현재 펜 끝 [%.3f %.3f %.3f]  (frame: %s)",
              current.pose.position.x, current.pose.position.y, current.pose.position.z,
              current.header.frame_id.c_str());
  // 실측 예: [-0.016 -0.605 0.419]  (frame: base_link)


  // ───────────────────────────────────────────────────────────────────────────
  // 【4】 목표 자세 만들기
  // ───────────────────────────────────────────────────────────────────────────

  auto target = current;
  // 현재 자세를 통째로 복사한다.
  // ★ 이렇게 하면 **orientation(자세)이 그대로 유지**된다.
  //   펜의 기울기를 바꾸지 않고 위치만 옮기겠다는 뜻이다.
  //   그리기에서는 펜 각도가 유지되어야 하므로 이 방식이 맞다.
  //
  //   orientation 을 직접 만들려면 쿼터니언을 계산해야 하는데,
  //   w=1, x=y=z=0 같은 임의값을 넣으면 대개 IK 해가 없어 계획이 실패한다.
  //   (튜토리얼 예제가 자주 실패하는 이유가 이것이다)

  target.pose.position.z -= 0.10;
  // base_link 기준으로 아래로 10cm.
  //
  // ★ 상대 이동인 이유: 절대 좌표를 박으면 로봇 자세나 설치 위치에 따라
  //   작업영역 밖이 되어 계획이 실패한다. 상대 이동은 어디서 실행하든
  //   대체로 안전하다.

  RCLCPP_INFO(logger, "목표 펜 끝 [%.3f %.3f %.3f]",
              target.pose.position.x, target.pose.position.y, target.pose.position.z);

  mg.setPoseTarget(target);
  //
  // ★ 여기서 IK 가 걸린다. 정확히는:
  //   setPoseTarget 자체는 목표를 저장만 하고, 실제 IK 는 plan() 안에서 돈다.
  //   move_group 이 pose 목표를 받으면
  //     kinematics.yaml 의 kdl_kinematics_plugin/KDLKinematicsPlugin 으로
  //     "pen_tip 이 이 자세가 되는 관절값 6개"를 찾는다.
  //
  //   KDL 은 수치해석(반복법) 솔버라 다음 특성이 있다:
  //     · 초기값에 따라 해를 못 찾을 수 있다 (timeout 0.005s)
  //     · 여러 해 중 하나만 준다 (어느 것인지 보장 없음)
  //     · 특이점 근처에서 불안정하다
  //   → 데카르트 경로(예제 3)에서 이 특성이 문제로 드러날 수 있다.
  //     그때 TRAC-IK 등으로 교체를 검토한다.
  //
  // ※ PoseStamped 대신 Pose 를 넘겨도 된다. 그때는 planning frame 기준으로 해석.


  // ───────────────────────────────────────────────────────────────────────────
  // 【5】 계획 · 실행
  // ───────────────────────────────────────────────────────────────────────────

  MoveGroupInterface::Plan plan;
  if (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(logger, "계획 성공 — 실행한다");

    auto result = mg.execute(plan);
    RCLCPP_INFO(logger, "실행 결과: %s",
                result == moveit::core::MoveItErrorCode::SUCCESS ? "성공" : "실패");

    // ─────────────────────────────────────────────────────────────────────────
    // 【6】 도달 검증 — 이 예제의 결론부
    // ─────────────────────────────────────────────────────────────────────────

    auto after = mg.getCurrentPose();
    // 실행이 끝난 뒤 다시 읽는다. 같은 경로(TF)로 읽으므로 비교가 유효하다.

    RCLCPP_INFO(logger, "도달 펜 끝 [%.3f %.3f %.3f]  (목표와 Z 오차 %.4f m)",
                after.pose.position.x, after.pose.position.y, after.pose.position.z,
                after.pose.position.z - target.pose.position.z);
    //
    // ★ 이 오차가 무엇을 말해주나
    //
    //   mock 하드웨어에서는 명령한 관절값이 그대로 상태가 되므로,
    //   오차가 0에 가까워야 정상이다 (실측 -0.0001 m = 0.1mm).
    //   0.1mm 는 IK 수치해석의 수렴 오차이지 기계 오차가 아니다.
    //
    //   이 값이 크게 나온다면:
    //     · IK 가 근사해로 수렴 (solver timeout 이 너무 짧음)
    //     · 관절 한계에 걸려 목표에 못 감
    //     · EE 링크가 예상과 다름
    //
    //   ※ 실기에 붙이면 이 오차에 **기계 오차·추종 지연**이 더해진다.
    //     그때 이 숫자가 곧 선 품질 지표가 된다. HCR-5 반복정밀도는 ±0.1mm.
  }
  else
  {
    RCLCPP_ERROR(logger, "계획 실패 — 목표가 작업영역 밖이거나 IK 해가 없다");
    //
    // 예제 1과 달리 이 예제는 **정상적으로도 실패할 수 있다.**
    // pose 목표는 IK 해가 없으면 원리적으로 도달 불가이기 때문이다.
    //
    // 실패 원인 후보:
    //   · 목표가 작업반경(915mm) 밖
    //   · 관절 한계 때문에 그 자세를 못 만듦
    //   · 그 자세가 자기충돌
    //   · KDL 이 timeout 안에 수렴 실패 (해는 있는데 못 찾음)  ← 재실행하면 될 수도
  }


  // ───────────────────────────────────────────────────────────────────────────
  // 【7】 정리 종료 (예제 1과 동일. 순서 중요)
  // ───────────────────────────────────────────────────────────────────────────

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}


// ═══════════════════════════════════════════════════════════════════════════════
// 【부록 A】 관절 목표 vs 자세 목표 — 언제 무엇을 쓰나
// ═══════════════════════════════════════════════════════════════════════════════
//
//                  | setJointValueTarget      | setPoseTarget
//  ----------------|--------------------------|---------------------------------
//  입력            | 관절 각도 6개            | 위치(x,y,z) + 자세(쿼터니언)
//  IK              | 불필요                   | 필요 (kinematics.yaml)
//  해의 존재       | 한계 안이면 항상 있음    | 없을 수 있음
//  경로            | 관절 공간 보간           | 관절 공간 보간 (끝점만 맞춤)
//  용도            | 홈 자세, 대기 자세       | "여기를 짚어라"
//
//  ★ 주의: setPoseTarget 도 **중간 경로는 데카르트 직선이 아니다.**
//    끝점만 그 자세일 뿐, 가는 길은 관절 공간에서 보간된다.
//    직선으로 가야 한다면 예제 3의 computeCartesianPath 를 써야 한다.
//    → 그리기가 예제 3을 쓰는 이유가 이것이다.
//
// ═══════════════════════════════════════════════════════════════════════════════
// 【부록 B】 이 예제로 진단할 수 있는 것
// ═══════════════════════════════════════════════════════════════════════════════
//
//  증상                              | 의심 지점
//  ---------------------------------|-----------------------------------------
//  end effector 가 link6_1          | SRDF 체인 tip_link. 펜 끝이 아님
//  계획이 항상 실패                  | kinematics 파라미터 누락 (launch 확인)
//  가끔 실패, 재실행하면 성공        | KDL timeout(0.005s). 값을 늘리거나 TRAC-IK
//  도달 오차가 mm 단위로 큼          | IK 근사 수렴 또는 관절 한계 접촉
//
// 관련 문서: src/moveit2/docs/HCR5_ros2_control_구축_상세.md §5.2 (펜 TCP)
