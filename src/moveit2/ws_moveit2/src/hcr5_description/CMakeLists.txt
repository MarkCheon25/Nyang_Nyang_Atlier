cmake_minimum_required(VERSION 3.8)
project(hcr5_description)

# 이 패키지는 데이터만 담는다 (URDF·메쉬·설정). 컴파일 대상이 없다.
find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
