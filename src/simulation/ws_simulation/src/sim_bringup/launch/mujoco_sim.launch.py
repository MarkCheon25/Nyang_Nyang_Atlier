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

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true 면 MuJoCo Simulate 창을 띄우지 않는다 '
                    '(확인 경로가 SVG 파일이라 GUI 없이도 결과는 나온다)')
    scene_arg = DeclareLaunchArgument(
        'mujoco_scene', default_value='/home/rosuser/data/mjcf/scene.xml',
        description='MuJoCo 씬 경로. **변환 산출물이라 git 제외 영역에 있다** — '
                    'robot_description_to_mjcf.sh 의 -o 와 맞춰야 한다')

    # 1. xacro → robot_description
    #    원본 hcr_robot.xacro 를 감싸 펜을 붙이고 하드웨어 플러그인만 갈아끼운다.
    #    ⚠️ 씬 경로와 headless 는 **hardware 의 <param> 으로** 들어간다 —
    #    controller_manager 노드 파라미터가 아니다 (mujoco_ros2_control 규약).
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare('sim_bringup'), 'urdf', 'hcr_robot_pen.xacro']),
        ' mujoco_scene:=', LaunchConfiguration('mujoco_scene'),
        ' headless:=', LaunchConfiguration('headless'),
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
    # ⚠️ **MJCF 변환은 미리 해 두어야 한다** (이 launch 는 변환하지 않는다):
    #
    #   xacro $(ros2 pkg prefix sim_bringup --share)/urdf/hcr_robot_pen.xacro > /tmp/hcr_pen.urdf
    #   /opt/ros/jazzy/share/mujoco_ros2_control/scripts/robot_description_to_mjcf.sh \
    #     -u /tmp/hcr_pen.urdf \
    #     -m $(ros2 pkg prefix sim_bringup --share)/mjcf/mujoco_inputs.xml \
    #     --scene $(ros2 pkg prefix sim_bringup --share)/mjcf/scene.xml \
    #     -o ~/data/mjcf -s -c --no-fuse
    #
    # `--no-fuse` 가 없으면 펜이 link6_1 에 병합돼 사라지고, `-m` 이 없으면
    # actuator 가 0 개로 나와 아무것도 제어되지 않는다 (둘 다 실측). README 참조.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description,
            os.path.join(bringup_share, 'config', 'controllers.yaml'),
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
        headless_arg,
        scene_arg,
        robot_state_publisher,
        controller_manager,
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster,
                          on_exit=[trajectory_controller])),
        joint_state_broadcaster,
    ])
