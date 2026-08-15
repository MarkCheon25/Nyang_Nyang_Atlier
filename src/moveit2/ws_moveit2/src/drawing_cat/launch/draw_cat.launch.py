"""고양이 스트로크를 그린다 — 응용 노드만 띄운다.

⚠️ **move_group 이 이미 떠 있어야 한다.** 이 노드는 MoveGroupInterface 클라이언트라
계획·실행을 전부 move_group 에 맡긴다. 먼저 백엔드를 올릴 것:

    simulation 컨테이너:  ros2 launch sim_bringup mujoco_sim.launch.py
    moveit2 컨테이너:     ros2 launch drawing_cat mujoco_moveit.launch.py
    moveit2 컨테이너:     ros2 launch drawing_cat draw_cat.launch.py   ← 이 파일

시뮬레이터가 없으면 `/joint_states` 발행자가 없어서 getCurrentPose() 가 (0,0,0) 을
돌려주고 fraction 이 전부 0.00 이 된다 (실측). RViz 로 자취를 보려면 마커가
volatile QoS 라 RViz 가 이 노드보다 **먼저** 떠 있어야 한다.

동작을 바꾸는 값은 전부 config/draw_cat_params.yaml 에 있다 — 스트로크 좌표,
축척, 펜업 높이, 마커 색까지. `params_file:=<경로>` 로 다른 파일을 쓸 수 있다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('drawing_cat'), 'config', 'draw_cat_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='draw_cat 노드 파라미터 파일 경로')

    # robot_description · SRDF · kinematics 는 MoveGroupInterface 가 자기 쪽에서도
    # 로봇 모델을 만들기 때문에 필요하다. `ros2 run` 으로 그냥 띄우면 이것들이
    # 없어서 "No kinematics plugins defined" 가 뜨고 fraction 이 0 이 된다.
    moveit_config = (
        MoveItConfigsBuilder('hcr_robot', package_name='hcr_moveit_config')
        .robot_description(file_path='config/hcr_robot.urdf.xacro')
        .robot_description_semantic(file_path='config/hcr_robot.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    draw_cat_node = Node(
        package='drawing_cat',
        executable='draw_cat',
        output='screen',
        parameters=[moveit_config.to_dict(), LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_file_arg, draw_cat_node])
