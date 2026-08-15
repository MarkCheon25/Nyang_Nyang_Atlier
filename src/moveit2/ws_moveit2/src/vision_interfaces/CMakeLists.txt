cmake_minimum_required(VERSION 3.8)
project(vision_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ⚠️ 이 목록과 msg/*.msg 의 내용은 vision 쪽 원본과 **정확히** 같아야 한다.
# 필드 이름·타입·순서 중 하나라도 다르면 ROS 2 타입 해시가 달라지고, 토픽은
# `ros2 topic list` 에 보이는데 메시지는 한 개도 안 들어온다. README.md 참조.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PixelPoint.msg"
  "msg/Stroke.msg"
)

ament_package()
