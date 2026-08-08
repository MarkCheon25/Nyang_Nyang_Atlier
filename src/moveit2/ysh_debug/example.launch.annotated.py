# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  example.launch.py  — 한 줄씩 해설본                                      ║
# ║                                                                          ║
# ║  ⚠️ 읽기용 사본. 실행되지 않는다.                                          ║
# ║     원본: ws_moveit2/src/hcr5_examples/launch/example.launch.py          ║
# ╚══════════════════════════════════════════════════════════════════════════╝
#
# ═══ 이 파일이 없으면 무슨 일이 일어나나 ══════════════════════════════════════
#
#   $ ros2 run hcr5_examples joint_goal
#   [ERROR] Failed to fetch parameter 'robot_description'
#   → MoveGroupInterface 가 로봇 모델을 만들지 못하고 죽는다.
#
#   왜? MoveGroupInterface 는 로봇을 **자기 노드의 파라미터**에서 읽는다.
#   move_group 노드가 갖고 있는 걸 빌려 쓰는 게 아니다.
#   그래서 예제 노드에게도 똑같은 파라미터 뭉치를 넣어줘야 한다.
#
#   그 뭉치를 만들어 넣어주는 것이 이 launch 파일의 유일한 역할이다.
#
# ═══ 넣어야 하는 파라미터 4종 ═════════════════════════════════════════════════
#
#   robot_description             URDF 전문(xacro 를 펼친 XML 문자열)
#   robot_description_semantic    SRDF (플래닝 그룹 · 자기충돌 매트릭스)
#   robot_description_kinematics  IK 솔버 설정 (kinematics.yaml)
#   robot_description_planning    관절 한계 (joint_limits.yaml)
#
#   앞의 둘이 없으면 로봇 모델 자체가 안 만들어지고,
#   셋째가 없으면 pose 목표(예제 2·3)가 항상 실패하고,
#   넷째가 없으면 시간 매개변수화가 실패한다.


from launch import LaunchDescription
# launch 파일은 반드시 generate_launch_description() 에서
# LaunchDescription 객체를 돌려줘야 한다. ros2 launch 가 그걸 찾아 실행한다.

from launch.actions import DeclareLaunchArgument
# 커맨드라인 인자를 선언한다. `example:=pose_goal` 처럼 넘길 수 있게 해준다.

from launch.substitutions import LaunchConfiguration
# 선언된 인자의 **값을 나중에 꺼내 쓰는** 객체.
#
# ★ 왜 그냥 문자열이 아닌가 (초보자가 가장 헷갈리는 부분)
#   launch 파일은 두 단계로 동작한다:
#     1) 파이썬 코드가 실행되어 "실행 계획"을 만든다   ← 지금 이 코드
#     2) launch 시스템이 그 계획을 해석해 실제로 띄운다 ← 인자 값이 정해지는 시점
#   즉 이 파이썬 코드가 돌 때는 example 의 값이 아직 **없다.**
#   그래서 "나중에 이 값으로 치환하라"는 자리표시자(substitution)를 쓴다.

from launch_ros.actions import Node
# ROS 노드를 띄우는 액션.

from moveit_configs_utils import MoveItConfigsBuilder
# ★ MoveIt 이 제공하는 헬퍼. moveit_config 패키지에서 설정 파일들을 긁어모아
#   Node 에 바로 넘길 수 있는 dict 형태로 만들어 준다.
#   이게 없으면 URDF 를 xacro 로 직접 펼치고, yaml 을 손으로 읽어
#   dict 를 조립해야 한다 (수십 줄).


def generate_launch_description():
    # ros2 launch 가 이 이름의 함수를 찾아 호출한다. 이름을 바꾸면 안 된다.

    # ─────────────────────────────────────────────────────────────────────────
    # 【1】 MoveIt 설정 뭉치 만들기
    # ─────────────────────────────────────────────────────────────────────────

    moveit_config = (
        MoveItConfigsBuilder("hcr5", package_name="hcr5_moveit_config")
        .to_moveit_configs()
    )
    #
    # 첫 인자 "hcr5" = **로봇 이름**. 파일 이름 규칙에 쓰인다:
    #     config/hcr5.srdf,  config/hcr5.urdf.xacro
    #   Setup Assistant 가 이 이름으로 파일을 만들었기 때문에 맞춰야 한다.
    #
    # package_name = 설정이 들어 있는 패키지.
    #
    # ★ 생성자가 하는 일 (내부 동작)
    #   1. hcr5_moveit_config/.setup_assistant 파일을 읽는다
    #      → 거기에 "URDF 는 hcr5_description 패키지의 이 경로" 라고 적혀 있다
    #   2. 그 xacro 를 찾아둔다
    #
    # to_moveit_configs() 가 실제로 읽어들이는 것들:
    #   · URDF xacro 를 펼쳐 문자열로            → robot_description
    #   · config/hcr5.srdf                        → robot_description_semantic
    #   · config/kinematics.yaml                  → robot_description_kinematics
    #   · config/joint_limits.yaml                → robot_description_planning
    #   · config/moveit_controllers.yaml          → trajectory_execution
    #   · config/*_planning.yaml                  → planning_pipelines
    #
    # ★ 나중에 여기를 손볼 수 있다
    #   .joint_limits(file_path="...") 로 **다른 경로의 joint_limits 를** 쓸 수 있다.
    #   docstring 이 "Absolute or relative path" 라고 명시하고, 내부가
    #   `self._package_path / file_path` 이므로 **절대경로를 주면 그게 이긴다**
    #   (pathlib 의 성질). 즉 hcr5_description 쪽 정본을 가리킬 수 있다.
    #
    #   이 파일은 Setup Assistant 가 생성하는 이름이 아니라 **재생성에도 살아남으므로**,
    #   그 일원화를 여기서 하면 손보정 값이 되돌아가는 문제가 사라진다.
    #   (상세본 §7.3.1)

    # ─────────────────────────────────────────────────────────────────────────
    # 【2】 인자 값 참조 준비
    # ─────────────────────────────────────────────────────────────────────────

    example = LaunchConfiguration("example")
    # "example 이라는 인자의 값"을 가리키는 자리표시자.
    # 지금은 값이 없고, 실행 시점에 채워진다.

    # ─────────────────────────────────────────────────────────────────────────
    # 【3】 실행 계획 반환
    # ─────────────────────────────────────────────────────────────────────────

    return LaunchDescription([

        DeclareLaunchArgument(
            "example",
            default_value="joint_goal",
            description="joint_goal | pose_goal | cartesian_square",
        ),
        # 인자 선언. 이게 있어야 `example:=pose_goal` 이 먹는다.
        # 선언하지 않고 LaunchConfiguration 만 쓰면 실행 시 에러가 난다.
        #
        # default_value 덕분에 인자 없이 실행해도 joint_goal 이 돈다.
        # description 은 `ros2 launch ... --show-args` 에 표시된다.

        Node(
            package="hcr5_examples",
            executable=example,
            # ★ 실행파일 이름에 자리표시자를 그대로 쓴다.
            #   실행 시점에 "joint_goal" / "pose_goal" / "cartesian_square" 로 치환된다.
            #   이 세 이름은 CMakeLists.txt 의 add_executable() 타겟명과 같아야 한다.
            #
            #   ※ 원본 소스 파일명은 01_joint_goal.cpp 처럼 번호가 붙어 있지만,
            #     CMake 에서 타겟명은 번호를 뗐다. 숫자로 시작하는 실행파일명은
            #     다루기 번거롭기 때문이다.

            output="screen",
            # 노드의 stdout/stderr 를 터미널에 그대로 뿌린다.
            # 이게 없으면 RCLCPP_INFO 로그가 안 보여서 결과를 알 수 없다.

            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            # ★ 이 네 줄이 이 파일의 존재 이유다.
            #
            #   각 항목은 {"파라미터이름": 값} 형태의 dict 이고,
            #   리스트로 주면 launch 가 전부 합쳐 노드에 넘긴다.
            #
            #   노드 쪽에서는 NodeOptions().automatically_declare_parameters_from_overrides(true)
            #   덕분에 자동으로 선언되어 MoveGroupInterface 가 읽을 수 있게 된다.
            #   (한쪽만 있으면 안 된다 — launch 가 넣어주고, 노드가 받아들여야 한다)
            #
            #   ※ trajectory_execution / planning_pipelines 는 넣지 않았다.
            #     그건 move_group **서버**가 쓰는 것이고, 우리 클라이언트에는
            #     불필요하다. 실행 요청은 액션으로 move_group 에 넘기므로
            #     컨트롤러 설정을 클라이언트가 알 필요가 없다.
        ),
    ])


# ═══════════════════════════════════════════════════════════════════════════════
# 【부록】 사용법과 전제
# ═══════════════════════════════════════════════════════════════════════════════
#
#   전제 — 다른 터미널에서 MoveIt 이 떠 있어야 한다:
#       ros2 launch hcr5_moveit_config demo.launch.py
#       ros2 launch hcr5_moveit_config demo.launch.py use_rviz:=false   # 헤드리스
#
#   실행:
#       ros2 launch hcr5_examples example.launch.py example:=joint_goal
#       ros2 launch hcr5_examples example.launch.py example:=pose_goal
#       ros2 launch hcr5_examples example.launch.py example:=cartesian_square
#
#   인자 확인:
#       ros2 launch hcr5_examples example.launch.py --show-args
#
# ─── 왜 demo.launch.py 가 따로 떠 있어야 하나 ─────────────────────────────────
#
#   demo.launch.py 가 띄우는 것:
#       move_group             계획 서버 (우리 예제가 액션으로 요청)
#       ros2_control_node      컨트롤러 매니저 + 하드웨어
#       robot_state_publisher  URDF → TF 발행
#       rviz2                  (use_rviz:=true 일 때)
#
#   우리 예제는 **클라이언트**라 이들이 없으면 아무것도 못 한다.
#   MoveGroupInterface 생성자가 move_group 을 기다리다 멈춘다.
#
#   ⚠️ demo.launch.py 를 두 번 띄우지 말 것. controller_manager 가 둘이 되어
#      /joint_states 가 충돌한다.
