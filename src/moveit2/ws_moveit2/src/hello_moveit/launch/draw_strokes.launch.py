import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    MOVEIT_CONFIG_PKG = 'hcr_moveit_config'

    # 1. 노드 자체 파라미터 정의
    node_parameters = [
        {
            'automatically_declare_parameters_from_overrides': True,
            'draw_size': 0.15,
            'eef_step': 0.002,
            'hover': 0.03,
            'vel_scale': 0.1,
            'acc_scale': 0.1,
            'execute': True,
            'go_home': True,
            'expected_parts': 4,
        }
    ]

    # 2. joint_limits.yaml 안전하게 읽어서 파라미터 딕셔너리로 변환해 주입
    try:
        moveit_config_share = get_package_share_directory(MOVEIT_CONFIG_PKG)
        joint_limits_path = os.path.join(moveit_config_share, 'config', 'joint_limits.yaml')
        
        if os.path.isfile(joint_limits_path):
            with open(joint_limits_path, 'r') as f:
                joint_limits_dict = yaml.safe_load(f)
                
            print(f"\n[SUCCESS] joint_limits.yaml 정상 파싱 완료: {joint_limits_path}\n")
            # 딕셔너리 형태로 파라미터 리스트에 추가 (경로 대신 딕셔너리 전달)
            node_parameters.append(joint_limits_dict)
        else:
            print(f"\n[WARN] 파일을 찾을 수 없습니다: {joint_limits_path}\n")
    except Exception as e:
        print(f"\n[WARN] joint_limits.yaml 로드 중 예외 발생: {e}\n")

    draw_strokes_node = Node(
        package='hello_moveit',
        executable='draw_strokes',
        name='draw_strokes_node',
        output='screen',
        parameters=node_parameters
    )

    return LaunchDescription([
        draw_strokes_node
    ])