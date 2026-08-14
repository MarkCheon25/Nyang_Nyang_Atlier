cmake_minimum_required(VERSION 3.22)
project(vision_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(moveit_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_interfaces REQUIRED)   # 커스텀 메시지 (Stroke, PixelPoint, MaskImage)

# ============================================================
# contour_pixel : 비전 컨테이너에서 실행. OpenCV만 있으면 됨 (json 불필요)
#   /vision/parts(MaskImage) 구독 -> /vision/strokes(Stroke) 발행
# ============================================================
find_package(OpenCV REQUIRED)

add_executable(contour_pixel src/contour_pixel_node.cpp)
ament_target_dependencies(contour_pixel
  rclcpp
  vision_interfaces
  OpenCV
)
target_link_libraries(contour_pixel
  ${OpenCV_LIBS}
)
install(TARGETS contour_pixel
  DESTINATION lib/${PROJECT_NAME})
# ============================================================
# draw_strokes : MoveIt 컨테이너에서 실행.
#   OpenCV / json 필요 없음. vision_interfaces + moveit만 있으면 됨.
#   /vision/strokes(Stroke) 구독 -> MoveIt으로 실제 그리기
# ============================================================
# find_package(moveit_ros_planning_interface REQUIRED)
# find_package(moveit_visual_tools REQUIRED)

# add_executable(draw_strokes src/draw_strokes_node.cpp)
# ament_target_dependencies(draw_strokes
#   rclcpp
#   std_msgs
#   geometry_msgs
#   moveit_msgs
#   std_srvs
#   visualization_msgs
#   vision_interfaces
#   moveit_ros_planning_interface
#   moveit_visual_tools
# )

# install(TARGETS contour_pixel draw_strokes
#   DESTINATION lib/${PROJECT_NAME})

# ============================================================
# sam : 호스트 venv에서 실행되는 파이썬 노드 (참고용 설치 규칙)
# ============================================================



install(PROGRAMS
  src/sam3_extract_ros_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sam
)



install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
