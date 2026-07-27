cmake_minimum_required(VERSION 3.22)
project(vision_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# ximgproc 를 COMPONENTS 로 명시해 두면 contrib 가 빠진 환경에서 configure 단계에
# 바로 걸린다 — 링크 시점까지 끌고 가지 않는다.
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs ximgproc)

# ── 코어 라이브러리 ─────────────────────────────────────────────────────────
# ROS 에 의존하지 않는다. rclcpp 를 여기 붙이지 말 것 — ROS 없이 단위 테스트가
# 되는 것이 2층 구조의 이유다 (ROS 래핑은 vision_node 담당).
add_library(${PROJECT_NAME} SHARED
  src/stroke_map.cpp
  src/image_to_map.cpp
  src/render_svg.cpp
  src/debug_dump.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})

# ── 확인용 CLI ──────────────────────────────────────────────────────────────
add_executable(stroke_map_cli src/stroke_map_cli.cpp)
target_link_libraries(stroke_map_cli PRIVATE ${PROJECT_NAME})

# ── 설치 ────────────────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS stroke_map_cli DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(OpenCV)

ament_package()
