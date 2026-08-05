"""실기 상태 브릿지 — MQTT motion/joint/position → /joint_states.

기본은 읽기 전용이다 (allow_motion:=false). 동작 지령 서비스를 열려면 명시해야 한다:
    ros2 launch hcr_bridge state_bridge.launch.py allow_motion:=true
⚠️ PC MQTT 명령은 펜던트 데드맨을 거치지 않는다 — e-stop 대기 상태에서만 켤 것.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    allow_motion = LaunchConfiguration("allow_motion")

    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="192.168.0.20",
                              description="HCR-5 컨트롤러 IP (랜선 직결)"),
        DeclareLaunchArgument("port", default_value="1883",
                              description="MQTT 브로커 포트"),
        DeclareLaunchArgument("allow_motion", default_value="false",
                              description="동작 지령 서비스 활성화 — 로봇이 움직인다"),
        Node(
            package="hcr_bridge",
            executable="state_bridge_node",
            name="hcr_state_bridge",
            output="screen",
            parameters=[{
                "host": host,
                "port": port,
                "allow_motion": allow_motion,
                "publish_rate": 50.0,
            }],
        ),
    ])
