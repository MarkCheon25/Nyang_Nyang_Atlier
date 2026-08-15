# bringup.launch.py — HCR-5 를 ros2_control 로 세운다. **MoveIt 없이.**
#
#   ros2 launch hcr5_bridge bringup.launch.py                              mock — 실기 불필요
#   ros2 launch hcr5_bridge bringup.launch.py real:=true                   실기 읽기 — 안 움직인다
#   ros2 launch hcr5_bridge bringup.launch.py real:=true allow_motion:=true  실기 쓰기 ⚠️ 움직인다
#
# 이 파일이 하는 일은 hcr_moveit_config/demo.launch.py 에서 **move_group 을 뺀 것**이다.
# 남는 것은 controller_manager · robot_state_publisher · 컨트롤러 2개 · RViz 뿐이고,
# 그래서 MoveIt 패키지를 하나도 참조하지 않는다 (URDF 원본인 hcr_robot_description 만 쓴다).
#
# 서보 사이클
#   기동: 컨트롤러가 active 가 된 **뒤** 서보 ON  (순서가 뒤집히면 함정 ⑤ 가 산다)
#         — allow_motion:=true 이고 auto_servo 가 켜져 있을 때만. launch 훅이 한다
#   종료: move/stop → 서보 OFF. **둘 다 플러그인 on_deactivate 안에서** 일어난다
#
# ⚠️ 종료 쪽을 launch 훅에서 플러그인으로 옮겼다 (2026-08-16).
#    종전에는 OnShutdown 훅이 servo off 를 실행했는데 **SIGINT 에서 안 도는 것이 실측됐다** —
#    pkill -INT 로 스택을 내린 뒤 status/operation 이 여전히 SERVO_ON 이었다(2026-08-15).
#    즉 Ctrl+C 경로 자체가 서보를 못 껐다. controller_manager 는 종료 시 on_deactivate 를
#    보장하므로 그쪽이 유일하게 믿을 수 있는 자리다.
#    아래 OnShutdown 훅은 **지웠다** — 안 도는 훅을 남겨 두면 도는 것처럼 읽힌다.
#
# 서보를 자동으로 끄는 것은 원래 설계(브레이크 판단은 사람 몫)를 바꾼 것이다 —
# 서보를 켠 채 방치하면 축온이 오르기 때문이다(운전절차 §5 함정 ⑦, 58~61°C 6축 트립).
# 자세가 위험해 브레이크를 걸고 싶지 않으면 auto_servo:=false 로 끈다 —
# 그러면 기동 시 서보 ON 도, 종료 시 서보 OFF 도 둘 다 안 한다.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _if_real(real, when_true, when_false):
    """real:=true 냐에 따라 문자열 하나를 고른다. xacro 인자는 문자열이어야 한다."""
    return PythonExpression(
        ["'", when_true, "' if '", real, "' == 'true' else '", when_false, "'"]
    )


def generate_launch_description():
    real = LaunchConfiguration("real")
    allow_motion = LaunchConfiguration("allow_motion")
    auto_servo = LaunchConfiguration("auto_servo")
    use_rviz = LaunchConfiguration("use_rviz")
    host = LaunchConfiguration("host")

    # ── URDF ────────────────────────────────────────────────────────────────
    # 실기/mock 두 조합만 낸다. 값의 근거는 hcr_robot.ros2_control.xacro 머리말이고,
    # 여기서 고르는 것은 그중 **검증된 조합**뿐이다 (검증.md V-2·V-10).
    #   실기  hcr5_bridge/HcrSystemInterface · rw_rate 30 · is_async true
    #   mock  mock_components/GenericSystem  · rw_rate 100 · is_async false
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("hcr_robot_description"), "urdf", "hcr_robot.xacro"]
    )
    # ⚠️ ParameterValue(value_type=str) 로 감싸야 한다 — 안 감싸면 launch 가 전개된 URDF 를
    #    YAML 로 파싱하려다 죽는다 ("Unable to parse the value of parameter robot_description").
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ", xacro_file,
                    " hardware_plugin:=",
                    _if_real(
                        real, "hcr5_bridge/HcrSystemInterface", "mock_components/GenericSystem"),
                    " rw_rate:=", _if_real(real, "30", "100"),
                    " is_async:=", _if_real(real, "true", "false"),
                    " allow_motion:=", allow_motion,
                    # 종료 시 서보 OFF 는 플러그인이 한다 (on_deactivate).
                    # auto_servo 하나로 기동 ON 과 종료 OFF 를 같이 여닫는다.
                    " servo_off_on_deactivate:=", auto_servo,
                ]
            ),
            value_type=str,
        )
    }

    controllers_yaml = os.path.join(
        get_package_share_directory("hcr5_bridge"), "config", "ros2_controllers.yaml"
    )

    # ── 노드 ────────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hcr_arm_controller", "-c", "/controller_manager"],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(use_rviz),
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("hcr5_bridge"), "rviz", "hcr5.rviz"]),
        ],
        parameters=[robot_description],
    )

    # ── 서보 사이클 ──────────────────────────────────────────────────────────
    # 조건: 실기 쓰기 모드이면서 auto_servo 가 켜져 있을 때만.
    # mock 이나 읽기 전용에서 서보를 켜면 발열만 생기고 얻는 것이 없다.
    servo_condition = IfCondition(
        PythonExpression(
            ["'", allow_motion, "' == 'true' and '", auto_servo, "' == 'true'"]
        )
    )

    servo_on = ExecuteProcess(
        cmd=["ros2", "run", "hcr5_bridge", "servo", "on", "--host", host],
        output="screen",
        condition=servo_condition,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "real", default_value="false",
                description="true 면 실기 플러그인(hcr5_bridge/HcrSystemInterface·30Hz·async). "
                            "false 면 mock — 실기가 없어도 뜬다",
            ),
            DeclareLaunchArgument(
                "allow_motion", default_value="false",
                description="⚠️ true 면 write() 가 실기를 움직인다. e-stop 앞에 있을 것",
            ),
            DeclareLaunchArgument(
                "auto_servo", default_value="true",
                description="기동 시 서보 ON · 종료 시 서보 OFF. allow_motion:=true 일 때만 작동. "
                            "브레이크를 걸고 싶지 않은 자세면 false",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("host", default_value="192.168.0.20"),

            robot_state_publisher,
            static_tf,
            ros2_control_node,
            jsb_spawner,
            rviz,

            # jsb 가 뜬 뒤에 arm 컨트롤러를 올린다 — 동시에 올리면 CM 이 붐빈다.
            RegisterEventHandler(
                OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
            ),
            # 컨트롤러가 active 가 된 **뒤** 서보 ON.
            # 이 순서라야 함정 ⑤(기동 시 자동 발행 2건)가 무해하게 남는다 —
            # on_activate 가 명령 버퍼를 현재 자세로 채워 두기 때문이다.
            RegisterEventHandler(
                OnProcessExit(target_action=arm_spawner, on_exit=[servo_on])
            ),
            # 종료 훅은 없다 — 머리말 참조. 서보 OFF 는 플러그인 on_deactivate 가 한다.
        ]
    )
