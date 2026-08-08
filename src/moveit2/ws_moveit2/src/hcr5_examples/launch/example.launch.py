"""HCR-5 MoveIt2 예제 실행기.

MoveGroupInterface 는 robot_description · SRDF · kinematics · joint_limits 를
**자기 노드의 파라미터로** 요구한다. `ros2 run` 으로 그냥 띄우면 이것들이 없어
로봇 모델을 못 만들고 죽는다. 그래서 이 launch 가 대신 넣어준다.

사용:
    ros2 launch hcr5_examples example.launch.py example:=joint_goal
    ros2 launch hcr5_examples example.launch.py example:=pose_goal
    ros2 launch hcr5_examples example.launch.py example:=cartesian_square

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
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("hcr5", package_name="hcr5_moveit_config")
        .to_moveit_configs()
    )

    example = LaunchConfiguration("example")

    return LaunchDescription([
        DeclareLaunchArgument(
            "example",
            default_value="joint_goal",
            description="joint_goal | pose_goal | cartesian_square",
        ),
        Node(
            package="hcr5_examples",
            executable=example,
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        ),
    ])
