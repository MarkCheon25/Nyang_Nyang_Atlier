"""HCR-5 MoveIt2 예제 실행기.

MoveGroupInterface 는 robot_description · SRDF · kinematics · joint_limits 를
**자기 노드의 파라미터로** 요구한다. `ros2 run` 으로 그냥 띄우면 이것들이 없어
로봇 모델을 못 만들고 죽는다. 그래서 이 launch 가 대신 넣어준다.

사용:
    ros2 launch hcr5_examples example.launch.py example:=joint_goal
    ros2 launch hcr5_examples example.launch.py example:=pose_goal
    ros2 launch hcr5_examples example.launch.py example:=cartesian_square
    ros2 launch hcr5_examples example.launch.py example:=draw_contour

draw_contour 는 인자를 받는다 (재빌드 없이 실험 가능):
    ros2 launch hcr5_examples example.launch.py example:=draw_contour draw_size:=0.25
    ros2 launch hcr5_examples example.launch.py example:=draw_contour execute:=false

⚠️ `ros2 launch` 에 `--ros-args -p x:=y` 를 붙여도 **노드로 전달되지 않는다.**
   노드 파라미터는 반드시 아래처럼 launch 인자로 선언해 parameters 에 실어야 한다.
   (다른 예제들은 이 인자를 무시한다 — automatically_declare 라 선언만 되고 안 쓰인다)

전제: 다른 터미널에서 demo.launch.py 가 떠 있어야 한다.
    ros2 launch hcr5_moveit_config demo.launch.py

참고 — 이 파일은 Setup Assistant 가 건드리지 않는다(생성 대상 이름이 아니다).
그래서 joint_limits 를 여기서 명시적으로 넘긴다. 나중에 이 경로를
hcr5_description 쪽 정본으로 바꾸면, Setup Assistant 재생성에도 값이 살아남는다.
(상세: src/moveit2/docs/ysh_HCR5_ros2_control_구축_상세.md §7.3.1)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

# draw_contour 가 읽는 노드 파라미터. (이름, 기본값, 타입, 설명)
# 다른 예제는 이 값들을 읽지 않으므로 실려도 무해하다.
#
# ⚠️ 타입을 반드시 지정해야 한다. LaunchConfiguration 은 **문자열**이라
#    그냥 넘기면 노드에 string 으로 선언되고, get_value<double>() 이 예외를 던진다.
TUNABLES = [
    ("draw_size", "0.15",  float, "도형의 긴 변 길이 [m]"),
    ("eef_step",  "0.002", float, "데카르트 경로 보간 간격 [m] — 작을수록 선이 매끄럽다"),
    ("hover",     "0.03",  float, "펜을 들고 이동할 높이 [m]"),
    ("vel_scale", "0.1",   float, "속도 스케일 (0~1)"),
    ("acc_scale", "0.1",   float, "가속도 스케일 (0~1)"),
    ("execute",   "true",  bool,  "false 면 계획만 하고 실행하지 않는다"),
    ("go_home",   "true",  bool,  "false 면 현재 자세에서 바로 그린다"),
]


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("hcr5", package_name="hcr5_moveit_config")
        .to_moveit_configs()
    )

    args = [
        DeclareLaunchArgument(
            "example",
            default_value="joint_goal",
            description="joint_goal | pose_goal | cartesian_square | "
                        "4_cartesian_square | draw_contour",
        ),
    ] + [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, _, desc in TUNABLES
    ]

    return LaunchDescription(args + [
        Node(
            package="hcr5_examples",
            executable=LaunchConfiguration("example"),
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                # 런치 인자 → 노드 파라미터 (타입 명시 필수, 위 주석 참조)
                {
                    name: ParameterValue(LaunchConfiguration(name), value_type=typ)
                    for name, _, typ, _ in TUNABLES
                },
            ],
        ),
    ])
