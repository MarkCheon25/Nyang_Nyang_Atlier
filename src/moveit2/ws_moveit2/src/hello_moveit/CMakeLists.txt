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


find_package(moveit_visual_tools REQUIRED)


add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(box_test src/box_test.cpp)
add_executable(cartesian src/cartesian.cpp)

target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(hello_moveit
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs)

target_include_directories(box_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(box_test
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs)

target_include_directories(cartesian PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cartesian
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_visual_tools)


install(TARGETS
  hello_moveit
  box_test
  cartesian
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
