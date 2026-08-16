cmake_minimum_required(VERSION 3.8)
project(nyang_pen)

find_package(ament_cmake REQUIRED)

# $(find nyang_pen)/urdf/... 가 듣게 하는 유일한 줄이다. 빠뜨리면 본체 xacro 의
# include 가 조용히 깨진다 (briefs/P1_nyang_pen_URDF.md §5-2).
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
