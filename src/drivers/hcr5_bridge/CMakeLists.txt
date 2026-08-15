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
# movej · movel 실행기용
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
# ros2 run hcr5_bridge {movej|movel|servo}. 액션을 직접 쏘는 것과 도달하는 곳은 같고,
# 다른 것은 발행 전에 단위·한계·속도를 검사한다는 점뿐이다 (include/move_common.hpp).
# movej 는 MQTT 를 안 쓴다 — 현재 자세를 /joint_states 에서 읽어 ros2_control 경로 안에서
# 완결한다. 그래서 hcr_mqtt 를 링크하지 않고, include 경로를 직접 준다.
add_executable(movej src/movej.cpp)
target_include_directories(movej PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(movej rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs)

# movel 은 IK 를 로봇 컨트롤러에 물어야 해서 MQTT 도 링크한다.
add_executable(movel src/movel.cpp)
target_link_libraries(movel hcr_mqtt)
ament_target_dependencies(movel rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs)

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
