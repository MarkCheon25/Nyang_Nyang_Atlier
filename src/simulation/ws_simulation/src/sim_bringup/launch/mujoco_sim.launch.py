"""MuJoCo 시뮬레이션 기동 — HCR-5 + 펜 + 종이.

⚠️ **스켈레톤이다.** robot_description 생성과 컨트롤러 스폰까지는 서 있고,
MJCF 변환·씬 로드 경로는 아직 실동작을 확인하지 않았다 (README "구현 상태" 참조).

기동 순서:
  1. xacro → robot_description  (hcr_robot_pen.xacro — 펜 + MuJoCo 하드웨어 플러그인)
  2. robot_description → MJCF   (mujoco_ros2_control 의 변환 스크립트)
  3. MuJoCo + controller_manager 기동
  4. joint_state_broadcaster · joint_trajectory_controller 스폰
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory('sim_bringup')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='MuJoCo 뷰어를 띄운다. headless 로 돌리려면 false '
                    '(확인 경로가 SVG 파일이라 GUI 없이도 결과는 나온다)')

    # 1. xacro → robot_description
    #    원본 hcr_robot.xacro 를 감싸 펜을 붙이고 하드웨어 플러그인만 갈아끼운다.
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare('sim_bringup'), 'urdf', 'hcr_robot_pen.xacro']),
    ])
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # 2~3. MuJoCo + controller_manager
    #
    # ⚠️ TODO — mujoco_ros2_control 이 씬을 받는 파라미터 이름과 MJCF 변환 시점을
    # 아직 확정하지 않았다. 변환 스크립트(robot_description_to_mjcf.sh)는
    # robot_description 을 읽어 MJCF 를 만들고, 기본 scene.xml 은 그 결과를
    # include 하는 규약이다. 여기 mjcf/scene.xml 이 같은 규약을 따르고 있으므로
    # 남은 것은 "언제 변환을 돌리고 어느 경로를 넘기는가" 하나다.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description,
            os.path.join(bringup_share, 'config', 'controllers.yaml'),
            {
                'mujoco_model_path': os.path.join(bringup_share, 'mjcf', 'scene.xml'),
                'mujoco_viewer': LaunchConfiguration('gui'),
            },
        ],
    )

    # 4. 컨트롤러 스폰 — mock 때와 같은 것을 쓴다 (N6)
    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    trajectory_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher,
        controller_manager,
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster,
                          on_exit=[trajectory_controller])),
        joint_state_broadcaster,
    ])
