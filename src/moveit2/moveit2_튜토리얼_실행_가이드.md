# MoveIt2 튜토리얼 실행 명령어

> 환경 세팅(이미지 pull, GPU, X11)은 `README_동료용.md` 먼저 완료할 것.
> 아래는 세팅이 끝난 상태에서 **튜토리얼을 실행**하는 명령어입니다.

## 컨테이너 진입

### 터미널 1 (MoveIt2 스택 + RViz2)
```bash
cd ~/moveit2      # compose.yml 이 있는 폴더 (동료 PC 경로에 맞게)
docker compose run --name moveit2_dev --rm moveit2
```
```bash
# 컨테이너 안에서 — ROS 2 환경 source (첫 진입 시 자동 적용됨)
source /opt/ros/jazzy/setup.bash
source /root/ws_moveit/install/setup.bash
```

> **주의**: `--name moveit2_dev` 없이 실행하면 컨테이너 이름이 자동 생성됨.
> 터미널 2의 `docker exec -it moveit2_dev bash`가 동작하지 않게 되므로 반드시 포함.
> 이름 누락 시 `docker ps`로 실제 이름 확인 후 exec에 대입.

### 터미널 2 (추가 진입 — 필요한 만큼 반복 가능)
```bash
docker exec -it moveit2_dev bash
```
```bash
# 컨테이너 안에서 — ROS 2 환경 source (exec 진입 시 수동 필요)
source /opt/ros/jazzy/setup.bash && source /root/ws_moveit/install/setup.bash
```

---

## 튜토리얼 실행

### 공통: 터미널 1에서 먼저 실행 (RViz2가 뜰 때까지 대기)
```bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

---

### 프로젝트 1 — 다중 목표 시퀀싱 (MoveGroupInterface)
```bash
# 터미널 2에서 실행
ros2 launch moveit2_tutorials move_group_interface_tutorial.launch.py
```

### 프로젝트 2 — 동적 장애물 회피 (PlanningScene)
```bash
# 터미널 2에서 실행
ros2 launch moveit2_tutorials planning_scene_ros_api_tutorial.launch.py
```

### 프로젝트 3 — 제약 조건 플래닝 (Constraints)
```bash
# 터미널 2에서 실행
ros2 run moveit2_tutorials ompl_constrained_planning
```

---

## 내 소스 컴파일해서 실행하기

`./src` (호스트) = `/root/ws_user/src` (컨테이너)로 마운트됩니다.
여기에 패키지를 두고 컨테이너 안에서 빌드하면 됩니다.

```bash
# 컨테이너 안에서
cd /root/ws_user
colcon build
source install/setup.bash
ros2 run <패키지명> <노드명>
```

> 예: `hello_moveit.cpp` 같은 튜토리얼 실습 코드를 `./src/<패키지>/` 아래에 두고 빌드.
