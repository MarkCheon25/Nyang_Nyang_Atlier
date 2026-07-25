# ROS 2 / MoveIt2 치트시트

자주 쓰는 명령을 축적. 사용하며 계속 추가.

## 컨테이너 (호스트에서 실행)

```bash
# 위치: <리포 디렉터리>/src/moveit2

./run_container.sh build   # 이미지 빌드 (최초 1회, 20~40분)
./run_container.sh up      # 컨테이너 백그라운드 기동
./run_container.sh shell   # 컨테이너 진입 (bash) — 안 떠 있으면 자동 기동
./run_container.sh down    # 정지 및 제거
./run_container.sh logs    # 로그 확인
```

## 컨테이너 내부

```bash
# ROS 2 환경 로드 (entrypoint에서 자동 실행됨)
source /opt/ros/jazzy/setup.bash

# 워크스페이스 빌드
cd ~/ws_moveit2
colcon build --symlink-install
source install/setup.bash

# 특정 패키지만 빌드
colcon build --symlink-install --packages-select hello_moveit

# 의존성 자동 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드 산출물 정리 (지워도 재빌드로 복구됨)
rm -rf build install log
```

## 실행

```bash
# Panda demo (RViz + MoveGroup + fake controller) — 이미지에 포함됨
ros2 launch moveit_resources_panda_moveit_config demo.launch.py

# 자체 노드
ros2 run hello_moveit hello_moveit
```

> `moveit2_tutorials` 패키지는 이미지에 없다. 소스로 받는 방법은
> `moveit2_튜토리얼_실행_가이드.md` 참고.

## 디버깅

```bash
ros2 node list                     # 실행 중 노드
ros2 topic list                    # 토픽 목록
ros2 topic echo /joint_states      # 관절 상태 스트림
ros2 action list                   # 액션 서버 목록
ros2 param list <node>             # 노드 파라미터
```

## GUI가 안 뜰 때

```bash
# 호스트에서
xhost +local:root
echo $DISPLAY                      # compose.yml이 이 값을 그대로 넘긴다

# 컨테이너 안에서
xdpyinfo | head -3                 # X 서버 접속 확인
glxinfo -B                         # 렌더러 확인 (llvmpipe면 소프트웨어 렌더링)
```
