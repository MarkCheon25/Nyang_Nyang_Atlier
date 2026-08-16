cmake_minimum_required(VERSION 3.16)
project(hcr5_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# JTC(FollowJointTrajectory) 경로용. 2026-08-16 movej 가 MQTT 로 옮겨 가면서 **지금은 어떤
# 타깃도 이것을 안 쓴다** — include/move_common.hpp 만 참조하고 그 헤더도 고아가 됐다(movej.md §4).
# 지우면 move_common.hpp 가 컴파일 불가가 되므로, 그 헤더의 폐기 여부가 정해질 때까지 남긴다.
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# libmosquitto / libuuid 는 ament 패키지가 아니라 pkg-config 로 찾는다.
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
pkg_check_modules(UUID REQUIRED uuid)

add_library(hcr_mqtt src/mqtt_client.cpp)
target_include_directories(hcr_mqtt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MOSQUITTO_INCLUDE_DIRS} ${UUID_INCLUDE_DIRS})
target_link_libraries(hcr_mqtt ${MOSQUITTO_LIBRARIES} ${UUID_LIBRARIES} nlohmann_json::nlohmann_json)
# 아래 플러그인이 공유 라이브러리라 정적으로 묶이는 hcr_mqtt 도 PIC 여야 한다.
set_property(TARGET hcr_mqtt PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(state_bridge_node src/state_bridge_node.cpp)
target_link_libraries(state_bridge_node hcr_mqtt)
ament_target_dependencies(state_bridge_node rclcpp sensor_msgs std_srvs)

# ── ros2_control 하드웨어 플러그인 ────────────────────────────────────────
# 같은 패키지 안에 두어 hcr_mqtt · joint_convention.hpp 를 배선 없이 재사용한다.
# (hcr5_bridge 에 ament_export_targets 가 없어 외부 패키지는 hcr_mqtt 를 링크 못 한다)
add_library(hcr_system_interface SHARED src/hcr_system_interface.cpp)
target_link_libraries(hcr_system_interface hcr_mqtt)
ament_target_dependencies(hcr_system_interface
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface hcr5_bridge_plugins.xml)

# ── 운전 실행기 ───────────────────────────────────────────────────────────
# ros2 run hcr5_bridge {movej|movel|servo}. **셋 다 rclcpp 를 안 쓴다** — program/plan 경로가
# ros2_control 스택과 배타이기 때문이다. 스택이 떠 있으면 애초에 같이 못 돌고, 안 떠 있어도
# 혼자 돌아야 한다 (movel.md §1.3 · movej.md §1.3). 현재 자세도 /joint_states 가 아니라
# MQTT get/command/pos 로 읽는다. 그래서 셋 다 hcr_mqtt 만 링크하면 된다.
#
# movej 는 2026-08-16(260816-호크니)에 JTC 경로에서 옮겨 왔다 — 종전에는 유일하게 rclcpp 를
# 쓰는 실행기였다. 경위는 movej.md §3.3.
add_executable(movej src/movej.cpp)
target_link_libraries(movej hcr_mqtt)

add_executable(movel src/movel.cpp)
target_link_libraries(movel hcr_mqtt)

# servo 는 rclcpp 를 안 쓴다 — 스택이 죽은 뒤에도 단독으로 돌아야 한다(정지 사다리 3순위).
add_executable(servo src/servo.cpp)
target_link_libraries(servo hcr_mqtt)

install(TARGETS hcr_mqtt hcr_system_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS state_bridge_node movej movel servo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

ament_package()
