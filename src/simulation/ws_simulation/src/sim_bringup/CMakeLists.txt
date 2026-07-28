cmake_minimum_required(VERSION 3.22)
project(sim_bringup)

find_package(ament_cmake REQUIRED)

# 자산만 설치한다 — 빌드할 코드가 없다.
# 로봇 정의(urdf)·씬(mjcf)·설정(config)·기동(launch)이 전부다.
install(DIRECTORY
  urdf
  mjcf
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
