#  MoveIt2 데모 기동 — sim(mock) / 실기가 갈리는 진입점
#
#    ros2 launch hcr_moveit_config demo.launch.py                    # mock (기본)
#    ros2 launch hcr_moveit_config demo.launch.py \
#        hardware_plugin:=hcr_bridge/HcrSystemInterface rw_rate:=30 is_async:=true
#                                                                    # 실기 읽기 (안 움직인다)
#    ros2 launch hcr_moveit_config demo.launch.py \
#        hardware_plugin:=hcr_bridge/HcrSystemInterface rw_rate:=30 is_async:=true \
#        allow_motion:=true                            # 실기 쓰기 ⚠️ 로봇이 움직인다
#
#  인자는 xacro mappings 로 흘러 hcr_robot.urdf.xacro → hcr_robot.xacro → ros2_control
#  매크로에 닿는다. 기본값의 원본은 hcr_robot_description/urdf/hcr_robot.xacro 한 곳이다.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # ⚠️ MoveItConfigsBuilder 는 호출되는 그 자리에서 xacro 를 전개한다.
    #    LaunchConfiguration 객체를 mappings 에 그냥 넣으면 문자열로 안 풀려
    #    조용히 깨진다 — OpaqueFunction 안에서 perform(context) 로 실제 값을 뽑아 넘긴다.
    #    빈 값은 빼고 넘겨서 xacro 쪽 기본값이 그대로 살게 한다 (기본값 원본은 한 곳이다).
    mappings = {
        name: LaunchConfiguration(name).perform(context)
        for name in ("hardware_plugin", "rw_rate", "is_async", "allow_motion")
    }
    mappings = {k: v for k, v in mappings.items() if v}

    moveit_config = (
        MoveItConfigsBuilder("hcr_robot", package_name="hcr_moveit_config")
        .robot_description(
            file_path="config/hcr_robot.urdf.xacro", mappings=mappings
        )
        .robot_description_semantic(file_path="config/hcr_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("hcr_moveit_config"), "launch", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("hcr_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    hcr_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hcr_arm_controller", "-c", "/controller_manager"],
    )

    return [
        rviz_node,
        static_tf_node,
        robot_state_publisher,
        move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        hcr_arm_controller_spawner,
    ]


def generate_launch_description():
    # 기본값을 비워 두는 것은 의도다 — 비면 mappings 에서 빠지고 xacro 기본값이 쓰인다.
    # 여기에 값을 복사해 두면 xacro 기본값이 바뀌어도 launch 가 옛 값을 조용히 덮는다.
    declared_args = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        ),
        DeclareLaunchArgument(
            "hardware_plugin",
            default_value="",
            description="ros2_control 하드웨어 플러그인. 비우면 xacro 기본값"
            " mock_components/GenericSystem. 실기는 hcr_bridge/HcrSystemInterface",
        ),
        DeclareLaunchArgument(
            "rw_rate",
            default_value="",
            description="하드웨어 컴포넌트의 read/write 주기(Hz, 양의 정수)."
            " 비우면 xacro 기본값 100. 실기는 30 (상태 버스 실측 29.1Hz)",
        ),
        DeclareLaunchArgument(
            "is_async",
            default_value="",
            description="하드웨어 read/write 를 워커 스레드로 분리할지."
            " 비우면 xacro 기본값 false. 실기는 true (명령 RPC 왕복 115~137ms)",
        ),
        DeclareLaunchArgument(
            "allow_motion",
            default_value="",
            description="⚠️ 실기를 실제로 움직일지의 안전 잠금."
            " 비우면 xacro 기본값 false = write() 무동작(실기 읽기 전용)."
            " true 로 켤 때만 로봇이 움직인다 — 서보 ON·e-stop 대기·입회 전제",
        ),
    ]

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])
