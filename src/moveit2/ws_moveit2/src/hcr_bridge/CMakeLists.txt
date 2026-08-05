cmake_minimum_required(VERSION 3.16)
project(hcr_bridge)

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

add_executable(state_bridge_node src/state_bridge_node.cpp)
target_link_libraries(state_bridge_node hcr_mqtt)
ament_target_dependencies(state_bridge_node rclcpp sensor_msgs std_srvs)

install(TARGETS hcr_mqtt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS state_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
