"""펜 끝 자취 시각화 노드 실행.

MoveIt 파라미터가 전혀 필요 없다 — TF 만 읽기 때문이다.
그래서 hcr5_examples 의 example.launch.py 와 달리 MoveItConfigsBuilder 가 없다.

사용:
    ros2 launch hcr5_viz pen_trail.launch.py
    ros2 launch hcr5_viz pen_trail.launch.py min_point_distance:=0.0005   # 더 촘촘하게
    ros2 launch hcr5_viz pen_trail.launch.py tip_frame:=link6_1           # 플랜지 자취와 비교

전제: demo.launch.py 가 떠 있어야 TF 가 나온다.

※ ParameterValue(..., value_type=...) 가 필요한 이유
  LaunchConfiguration 은 **문자열 치환**이다. 그대로 넘기면 노드가
  declare_parameter("min_point_distance", 0.002) 로 double 로 선언해 둔 것과
  타입이 어긋나 다음 에러가 난다:
      Wrong parameter type, parameter {min_point_distance} is of type {double},
      setting it to {string} is not allowed.
  value_type 을 주면 launch 가 문자열을 그 타입으로 변환해 넘긴다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (인자이름, 기본값, 타입, 설명)
ARGS = [
    ("base_frame", "base_link", str, "자취를 그릴 기준 좌표계"),
    ("tip_frame", "pen_tip", str, "따라갈 프레임. link6_1 로 바꾸면 플랜지 자취를 본다"),
    ("min_point_distance", "0.002", float, "이만큼 움직여야 점을 찍는다 [m]"),
    ("max_points", "20000", int, "자취 최대 점 개수 (넘으면 오래된 것부터 버림)"),
    ("publish_rate", "30.0", float, "발행 주기 [Hz]"),
    ("line_width", "0.002", float, "선 굵기 [m]"),
    ("tip_size", "0.008", float, "현재 위치 구슬 지름 [m]"),
]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc)
         for n, d, _, desc in ARGS]
        + [
            Node(
                package="hcr5_viz",
                executable="pen_trail",
                name="pen_trail",
                output="screen",
                parameters=[{
                    n: ParameterValue(LaunchConfiguration(n), value_type=t)
                    for n, _, t, _ in ARGS
                }],
            ),
        ]
    )
