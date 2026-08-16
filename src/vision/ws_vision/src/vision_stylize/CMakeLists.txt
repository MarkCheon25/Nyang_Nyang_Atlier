cmake_minimum_required(VERSION 3.22)
project(vision_stylize)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# 전처리(축소·흑백·여백)와 후처리(이진화)에만 쓴다. contrib 는 필요 없다 —
# thinning 경로는 이 흐름에 들어오지 않는다.
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
# 설치 경로 조회용. **CLI 에만 링크한다** — 라이브러리에 붙이면
# "ROS 없이도 빌드·실행된다" 가 깨진다.
find_package(ament_index_cpp REQUIRED)

# ── 코어 라이브러리 ─────────────────────────────────────────────────────────
# ROS 에 의존하지 않는다. rclcpp 를 여기 붙이지 말 것 — ROS 없이 빌드·실행되는
# 것이 2층 구조의 이유다 (vision_core 가 이미 선 방식).
add_library(${PROJECT_NAME} SHARED
  src/types.cpp
  src/image_io.cpp
  src/image_ops.cpp
  src/prompt_preset.cpp
  src/api_client.cpp
  src/cost_guard.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  PUBLIC ${OpenCV_LIBS}
  PRIVATE CURL::libcurl nlohmann_json::nlohmann_json)

# ── 확인용 CLI ──────────────────────────────────────────────────────────────
add_executable(stylize_cli src/stylize_cli.cpp)
target_link_libraries(stylize_cli PRIVATE ${PROJECT_NAME} ament_index_cpp::ament_index_cpp)

# ── 설치 ────────────────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS stylize_cli DESTINATION lib/${PROJECT_NAME})

# 프롬프트는 소스가 아니라 데이터다 — 고쳐도 다시 빌드하지 않는다.
# CLI 는 실행 파일 위치에서 lib/vision_stylize → share/vision_stylize/prompts 로 찾는다.
install(DIRECTORY prompts/ DESTINATION share/${PROJECT_NAME}/prompts)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(OpenCV)

ament_package()
