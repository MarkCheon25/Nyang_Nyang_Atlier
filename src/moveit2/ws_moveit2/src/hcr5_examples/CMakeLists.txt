cmake_minimum_required(VERSION 3.22)
project(hcr5_examples)

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
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 예제 실행파일. 파일명 앞의 번호는 학습 순서이고 타겟명은 번호를 뺀다.
set(EXAMPLES
  joint_goal:01_joint_goal
  pose_goal:02_pose_goal
  cartesian_square:03_cartesian_square
  4_cartesian_square:04_cartesian_square
  draw_contour:05_draw_contour
  draw_movel:06_draw_movel
)

foreach(pair IN LISTS EXAMPLES)
  string(REPLACE ":" ";" parts "${pair}")
  list(GET parts 0 target)
  list(GET parts 1 srcfile)

  add_executable(${target} src/${srcfile}.cpp)
  # include/ 는 종이·도형 좌표 정본(a4_paper.hpp). 05·06 이 같은 것을 본다.
  target_include_directories(${target} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(${target}
    rclcpp
    moveit_ros_planning_interface
    moveit_core
    geometry_msgs
    moveit_msgs
    visualization_msgs
    std_srvs)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
