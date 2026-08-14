cmake_minimum_required(VERSION 3.22)
project(hello_moveit)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)


find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_interfaces REQUIRED)   # 커스텀 메시지 (Stroke, PixelPoint, MaskImage)
find_package(moveit_visual_tools REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(hello_moveit
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs)

# ============================================================
# draw_strokes : MoveIt 컨테이너에서 실행.
#   OpenCV / json 필요 없음. vision_interfaces + moveit만 있으면 됨.
#   /vision/strokes(Stroke) 구독 -> MoveIt으로 실제 그리기
# ============================================================

add_executable(draw_strokes src/draw_strokes_node.cpp)
target_include_directories(draw_strokes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(draw_strokes
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  vision_interfaces
  std_srvs
  visualization_msgs
  tf2_geometry_msgs # <-- 추가 확인
)

install(TARGETS draw_strokes
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME})

# Launch 파일들을 install 폴더로 복사 설정
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
