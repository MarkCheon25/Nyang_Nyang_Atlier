cmake_minimum_required(VERSION 3.22)
project(sim_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ── 코어 라이브러리 ─────────────────────────────────────────────────────────
# ROS 에도 MuJoCo 에도 의존하지 않는다. 궤적을 기록하는 쪽(시뮬 노드)과 읽는
# 쪽(확인 하네스)이 타입만 공유하면 되므로, 시뮬 연동이 막혀도 확인 경로는 선다.
add_library(${PROJECT_NAME} SHARED
  src/contact_trace.cpp
  src/planned_map.cpp
  src/trace_io.cpp
  src/trace_svg.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ── 확인용 CLI ──────────────────────────────────────────────────────────────
add_executable(trace_cli src/trace_cli.cpp)
target_link_libraries(trace_cli PRIVATE ${PROJECT_NAME})

# ── 설치 ────────────────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS trace_cli DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
