# ROS 2 / MoveIt2 치트시트

자주 쓰는 명령을 축적. 사용하며 계속 추가.

## 컨테이너 (호스트에서 실행)

```bash
# 프로젝트 루트: ~/Workspace_260417/Nyang_Nyang_Atlier_260704/src/moveit2_tutorial

./scripts/run_container.sh build   # 이미지 빌드
./scripts/run_container.sh up      # 컨테이너 백그라운드 기동
./scripts/run_container.sh shell   # 컨테이너 진입 (bash)
./scripts/run_container.sh down    # 정지 및 제거
```

## 컨테이너 내부

```bash
# ROS 2 환경 로드 (entrypoint에서 자동 실행됨)
source /opt/ros/jazzy/setup.bash

# 워크스페이스 빌드
cd ~/ws_moveit
colcon build --symlink-install
source install/setup.bash

# 의존성 자동 설치
rosdep install --from-paths src --ignore-src -r -y
```

## MoveIt2 튜토리얼 실행

```bash
# Panda demo (RViz + MoveGroup + fake controller)
ros2 launch moveit2_tutorials demo.launch.py

# 자체 노드 실행 (예: hello_moveit)
ros2 run hello_moveit hello_moveit
```

## 디버깅

```bash
ros2 node list                     # 실행 중 노드
ros2 topic list                    # 토픽 목록
ros2 topic echo /joint_states      # 관절 상태 스트림
ros2 action list                   # 액션 서버 목록
ros2 param list <node>             # 노드 파라미터
```
