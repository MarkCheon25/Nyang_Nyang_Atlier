cmake_minimum_required(VERSION 3.8)
project(drawing_cat)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_interfaces REQUIRED)

add_executable(draw_cat src/draw_cat.cpp)
target_include_directories(draw_cat PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(draw_cat
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  visualization_msgs
  vision_interfaces
)

install(TARGETS
  draw_cat
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  data
  DESTINATION share/${PROJECT_NAME}
)

# ── test_tools ──────────────────────────────────────────────────────────────
# ⚠️ 연결 확인 전용이다. 실제 vision 이 붙어 검증이 끝나면 **이 블록과
#    test_tools/ 폴더를 함께 지운다.** 제품 코드가 아니다.
#    자세한 것은 test_tools/README.md
install(PROGRAMS
  test_tools/fake_vision_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_vision_publisher
)
install(DIRECTORY test_tools DESTINATION share/${PROJECT_NAME})
# ── test_tools 끝 ───────────────────────────────────────────────────────────


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # ⚠️ 아래 셋도 끈다 — **이 패키지는 처음부터 이 린터들을 통과한 적이 없다.**
  #    (2026-08-14 에 처음 colcon test 를 돌려 보고 알았다: 38 개 실패이고 그 대상이
  #     src/draw_cat.cpp · launch/*.py · test_tools/*.py 등 기존 파일들이다.)
  #
  #    이 저장소는 ROS 2 기본 스타일을 따르지 않는다 — C++ 은 4 칸 들여쓰기에 Allman
  #    중괄호, Python 은 큰따옴표, 주석은 한국어다. copyright·cpplint 를 이미 끈 것과
  #    같은 이유다.
  #
  #    끄지 않으면 colcon test 가 **항상 빨간불**이라 아래 단위시험의 실패가 묻힌다.
  #    스타일을 맞추기로 한다면 이 세 줄을 지우고 전체를 정리하면 된다.
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # ── F3.1 단위시험 ─────────────────────────────────────────────────────────
  # include/drawing_cat/optimizer.hpp 가 ROS 에 의존하지 않아서 로봇 없이 돈다.
  #   colcon test --packages-select drawing_cat
  # 지울 일이 생기면 이 블록과 test/ 폴더, package.xml 의 ament_cmake_gtest
  # test_depend 세 군데다 (CLEANUP.md 1절).
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_optimizer test/test_optimizer.cpp)
  target_include_directories(test_optimizer PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  # ── F3.1 단위시험 끝 ──────────────────────────────────────────────────────
endif()
ament_package()
