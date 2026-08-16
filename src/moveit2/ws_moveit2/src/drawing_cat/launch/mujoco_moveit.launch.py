"""MoveIt2(머리) ↔ MuJoCo(몸통) 연동 — moveit2 컨테이너 쪽 기동.

두 컨테이너가 `network_mode: host` 로 같은 ROS 그래프에 있고, DDS 로 붙는다.
역할 분담이 명확해야 한다:

    simulation 컨테이너 (mujoco_sim.launch.py) — 몸통
        robot_state_publisher · ros2_control_node(MuJoCo 하드웨어)
        joint_state_broadcaster · joint_trajectory_controller

    moveit2 컨테이너 (이 파일) — 머리
        move_group · rviz2   ← **이 둘만 띄운다**

⚠️ **`demo.launch.py` 를 대신 쓰면 안 된다.** 저건 mock_hardware 단독 데모용이라
몸통까지 자기가 띄운다. 아래 5 개가 시뮬레이션 쪽과 정면으로 겹친다:

    ros2_control_node              → /controller_manager 가 도메인에 둘
    joint_state_broadcaster        → JSB 가 둘 → /joint_states 오염
                                     (mujoco_sim.launch.py 독스트링이 경고한 그 증상)
    hcr_arm_controller spawner     → MuJoCo 쪽엔 그 이름의 컨트롤러가 없다
    robot_state_publisher          → /tf 이중 발행
    static_transform_publisher     → hcr_robot_pen.xacro 가 이미 world → base_link
                                     fixed 조인트를 갖고 있다 → TF 부모가 둘

⚠️ **펜은 아직 MoveIt 의 TCP 가 아니다.** move_group 은 펜이 없는
`hcr_robot.urdf.xacro` 를 쓰고, MuJoCo 는 펜이 붙은 `hcr_robot_pen.xacro` 를 쓴다.
따라서 (a) 펜은 MoveIt 충돌 검사에 들어가지 않고, (b) `pen_tip` 프레임이 MoveIt
쪽에 존재하지 않는다. 펜 길이(150 mm) 보정은 지금은 응용 코드가 직접 한다.
F4.1(종이→로봇 좌표 변환)을 제대로 풀려면 이 결정을 다시 봐야 한다.

기동 순서:
  1. 양쪽 컨테이너의 ROS_DOMAIN_ID 를 같은 값(0 아님)으로 맞춘다
  2. simulation:  ros2 launch sim_bringup mujoco_sim.launch.py
     → `ros2 control list_controllers` 로 컨트롤러 2 개 active 확인
  3. moveit2:     ros2 launch drawing_cat mujoco_moveit.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 원본 hcr_moveit_config/launch/moveit.rviz 에 pen_trace Marker 디스플레이를
    # 추가한 것. 원본에는 Marker 가 없어서 매번 손으로 Add → Marker 를 해야 한다.
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('drawing_cat'), 'rviz', 'draw_cat.rviz']),
        description='RViz 설정 파일 경로 (기본값: pen_trace 마커가 포함된 draw_cat.rviz)')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='false 면 move_group 만 띄운다 (헤드리스 확인용)')

    # MuJoCo 전용 컨트롤러 설정. **원본 hcr_moveit_config 는 건드리지 않는다** —
    # hanwha_robot_arm/ 은 외부에서 가져와 포팅한 공용 자산이고 mock_hardware
    # 데모가 계속 동작해야 한다. 그래서 복사본을 moveit2 쪽 로컬 패키지에 둔다.
    #
    # trajectory_execution() 의 file_path 는 절대경로를 받는다 (내부가
    # `self._package_path / file_path` 라 pathlib 규칙상 절대경로가 우선한다).
    drawing_cat_config = os.path.join(
        get_package_share_directory('drawing_cat'), 'config')
    moveit_controllers_mujoco = os.path.join(
        drawing_cat_config, 'moveit_controllers_mujoco.yaml')
    # 2026-08-16: joint_limits_mujoco.yaml 우회를 제거했다.
    # 실측 URDF(markch/hcr5_ros2)가 joint_1 을 lower=-4.712388 로 고쳤으므로
    # 하한 0 문제가 사라졌다. 오히려 우회값(±6.283185)이 실측 상한(7.853981)보다
    # 좁아 성능을 깎아먹던 상태였다. hcr_moveit_config 의 joint_limits.yaml 을 쓴다.

    moveit_config = (
        MoveItConfigsBuilder('hcr_robot', package_name='hcr_moveit_config')
        .robot_description(file_path='config/hcr_robot.urdf.xacro')
        .robot_description_semantic(file_path='config/hcr_robot.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .trajectory_execution(file_path=moveit_controllers_mujoco)
        .planning_pipelines(pipelines=['ompl'])
        # ⚠️ publish_robot_description 은 **false** 여야 한다. 시뮬레이션 쪽
        # robot_state_publisher 가 이미 /robot_description 을 (펜 포함 버전으로)
        # 발행하고 있어서, 여기서도 켜면 서로 다른 내용이 같은 토픽에 실린다.
        # semantic(SRDF)은 발행자가 여기뿐이라 켜둔다.
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # Fixed Frame 은 base_link 그대로 두면 된다 — 시뮬 쪽 URDF 가 world 를 루트로
    # 갖지만 world → base_link 가 identity fixed 라 base_link 도 TF 에 항상 있다.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        rviz_config_arg,
        use_rviz_arg,
        move_group_node,
        rviz_node,
    ])
