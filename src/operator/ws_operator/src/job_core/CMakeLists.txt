cmake_minimum_required(VERSION 3.22)
project(job_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ── 코어 라이브러리 ─────────────────────────────────────────────────────────
# ROS·웹·DB 어디에도 의존하지 않는다. rclcpp 를 여기 붙이지 말 것 — ROS 없이
# 시나리오를 관통시켜 전이를 검증할 수 있는 것이 2층 구조의 이유다
# (ROS 래핑은 job_manager, DB 는 웹 백엔드 담당).
add_library(${PROJECT_NAME} SHARED
  src/job.cpp
  src/job_machine.cpp
  src/job_json.cpp
  src/job_timeline.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ── 확인용 CLI ──────────────────────────────────────────────────────────────
add_executable(job_cli src/job_cli.cpp)
target_link_libraries(job_cli PRIVATE ${PROJECT_NAME})

# ── 설치 ────────────────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS job_cli DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
