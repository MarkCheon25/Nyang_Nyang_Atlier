# 세션 학습 기록

각 세션(다알리아 → 라... 순)의 진행 내용을 시간 순서로 기록.
새 세션 시작 시 이 파일의 마지막 엔트리부터 확인.

---

## 세션 #3 — 다알리아 (Dahlia) · 2026-07-18

### 목표
- ROS 2 Jazzy + MoveIt2 Docker 환경 구축
- 공식 튜토리얼(Getting Started ~ Move Group Interface) 진행
- Panda arm 시뮬레이션 조작 확인

### 진행 사항
- [x] 디렉토리 구조 설계 및 `DIRECTORY_STRUCTURE.md` 작성
- [x] Dockerfile 작성 (`ros:jazzy-ros-base` + moveit + resources)
- [x] docker-compose.yml, run_container.sh 작성
- [x] 이미지 빌드 완료 (`moveit2_tutorial:jazzy`, 6.13GB)
- [x] RViz 실행 검증 (`xdpyinfo` OK, DISPLAY=:1)
- [x] Panda demo 실행 (`ros2 launch moveit_resources_panda_moveit_config demo.launch.py`) — 인터랙티브 마커로 Plan/Execute 동작 확인
- [x] `hello_moveit` C++ 패키지 작성 및 빌드
- [x] `ros2 run hello_moveit hello_moveit` 실행 → target pose `(0.28, -0.2, 0.5)`로 이동 성공

### 결정 사항
- **ROS 2 Jazzy 채택** (Ubuntu 24.04 네이티브 일치)
- **Docker 컨테이너로 격리** (호스트 오염 방지, GPU/X11은 마운트)
- **Panda robot 로 튜토리얼 진행** (공식 기본, moveit_resources 제공)
- **fake_controller 우선**, 이후 필요 시 Gazebo 확장
- **스크립트 제어는 C++ 우선** (공식 튜토리얼 경로, MoveGroupInterface API 성숙도 높음)

### 이슈 / 메모
- Jazzy에서 `moveit/move_group_interface/move_group_interface.h` → `.hpp` 로 이전됨. 컴파일 경고만 나오고 동작에는 문제 없음. 다음 단계에서 헤더 정리.
- `rosdep update` 미실행 상태 (컨테이너 사용자). apt로 이미 다 설치되어 있어 지금은 무관하지만, 향후 소스 패키지 vcstool로 가져오면 필요.
- MoveGroup 실행 시 `panda_link8`이 endeffector로 잡힘 (그리퍼 없이 마지막 링크 기준).
