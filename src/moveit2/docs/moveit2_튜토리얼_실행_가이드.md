# MoveIt2 실행 명령어

> 환경 세팅(이미지 빌드, GPU, X11)은 상위 폴더의 `README.md`를 먼저 완료할 것.
> 아래는 세팅이 끝난 상태에서 **실행**하는 명령어입니다.

## 컨테이너 진입

### 터미널 1
```bash
cd moveit2
./run_container.sh shell
```

- 컨테이너가 안 떠 있으면 자동 기동 후 진입합니다.
- ROS 2 환경은 진입 시 자동 로드됩니다 (`entrypoint.sh`가 `/opt/ros/jazzy/setup.bash`와,
  빌드되어 있다면 `~/ws_moveit2/install/setup.bash`까지 source).

### 터미널 2 이상 (추가 진입)
```bash
./run_container.sh shell     # 같은 명령을 그대로 다시 실행
```

---

## 동작 확인 — Panda demo

터미널 1에서:
```bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```
→ RViz2가 뜨고 인터랙티브 마커로 Plan/Execute가 되면 정상.

`moveit_resources_*`는 이미지에 이미 들어 있습니다 (`ros-jazzy-moveit-resources`).

---

## 자체 노드 실행 — hello_moveit

터미널 1에서 위 demo를 띄워둔 상태로, 터미널 2에서:

```bash
cd ~/ws_moveit2
colcon build --symlink-install
source install/setup.bash
ros2 run hello_moveit hello_moveit
```
→ target pose `(0.28, -0.2, 0.5)`로 팔이 이동하면 성공.

---

## 공식 튜토리얼 예제 (moveit2_tutorials)

> ⚠️ **`moveit2_tutorials` 패키지는 이미지에 들어 있지 않습니다.**
> apt로 배포되지 않아 워크스페이스에 소스로 받아야 합니다.
> Panda demo와 자체 노드만 쓸 거면 이 절은 건너뛰어도 됩니다.

```bash
cd ~/ws_moveit2/src
git clone -b main https://github.com/moveit/moveit2_tutorials.git

cd ~/ws_moveit2
rosdep install --from-paths src --ignore-src -r -y    # 의존성 설치
colcon build --symlink-install
source install/setup.bash
```
> 소스 빌드라 시간이 꽤 걸립니다.

받은 뒤 실행할 수 있는 예제 (터미널 1에서 Panda demo를 먼저 띄운 상태):

```bash
# 다중 목표 시퀀싱 (MoveGroupInterface)
ros2 launch moveit2_tutorials move_group_interface_tutorial.launch.py

# 동적 장애물 회피 (PlanningScene)
ros2 launch moveit2_tutorials planning_scene_ros_api_tutorial.launch.py

# 제약 조건 플래닝 (Constraints)
ros2 run moveit2_tutorials ompl_constrained_planning
```

---

## 워크스페이스 경로

| 호스트 | 컨테이너 |
|---|---|
| `src/moveit2/ws_moveit2/` | `/home/rosuser/ws_moveit2/` |

`ws_moveit2/src/` 아래에 패키지를 두고 컨테이너 안에서 빌드하면, 결과물이 호스트에 그대로 남습니다.
`build/`·`install/`·`log/`는 git에 올리지 않으며, 지워도 `colcon build`로 다시 생성됩니다.
