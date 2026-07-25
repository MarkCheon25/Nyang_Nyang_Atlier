# moveit2 — 디렉터리 구조

`src/moveit2/`의 구조와 각 파일의 목적을 정의한다. 구조 변경 시 이 문서를 먼저 갱신한다.

## 트리

```
Nyang_Nyang_Atlier/                 ← 리포 디렉터리 (GitHub)
├── docs/                           설계 문서·다이어그램 (원본)
├── hanwha_robot_arm/               외부 자산 — 커뮤니티 ROS1 패키지의 ROS2 Jazzy 포팅
│   ├── LICENSE  README.md          출처·라이선스 (건드리지 않는다)
│   ├── HCR_5/
│   │   ├── hcr_robot_description/  colcon 패키지 — URDF·xacro·STL·USD
│   │   └── hcr_moveit_config/      colcon 패키지 — SRDF·kinematics·ompl·demo.launch.py
│   └── docs/ros1_to_ros2_migration/
└── src/                            자체 개발 코드
    ├── moveit2/                    ← 본 문서 범위
    │   ├── Dockerfile              ROS 2 Jazzy + MoveIt2 이미지 정의 (환경 중립)
    │   ├── compose.yml             실행 정의 — GPU·X11·볼륨 (PC마다 갈리는 곳)
    │   ├── entrypoint.sh           컨테이너 진입 시 ROS 환경 로드
    │   ├── run_container.sh        호스트 헬퍼 (build/up/shell/down/logs)
    │   ├── docs/                   (본 문서 포함) 문서
    │   └── ws_moveit2/             ROS 2 워크스페이스 — 컨테이너가 연결하는 루트
    │       └── src/
    │           └── hello_moveit/   colcon 패키지 (C++ 노드)
    ├── vision/
    ├── simulation/
    └── operator/
```

**외부 자산과 자체 코드의 경계** — `hanwha_robot_arm/`은 외부에서 가져와 포팅한 것이라 `src/`(자체 개발)와 섞지 않고 최상위에 둔다. 대신 colcon 이 잡을 수 있도록 컨테이너 안에서만 워크스페이스로 끌어온다(아래 매핑).

## 컨테이너 매핑

| 호스트 | 컨테이너 |
|---|---|
| `src/moveit2/ws_moveit2/` | `/home/rosuser/ws_moveit2/` (rw) |
| **`hanwha_robot_arm/`** | **`/home/rosuser/ws_moveit2/src/hanwha_robot_arm/`** (rw) |
| `src/moveit2/docs/` | 마운트 안 함 — 호스트 전용 |

> **호스트와 컨테이너의 배치가 다르다.** `hanwha_robot_arm/`은 호스트에서 리포 최상위지만 컨테이너 안에서는 워크스페이스 `src/` 아래로 보인다. 호스트의 `ws_moveit2/src/`에는 그 디렉터리가 없는 것이 정상이다.
> `colcon build`는 세 패키지를 함께 빌드한다 — `hello_moveit` · `hcr_robot_description` · `hcr_moveit_config`.

- 컨테이너 사용자 `rosuser` — 호스트와 같은 UID/GID로 생성 (`run_container.sh`가 전달)
- 이미지 `moveit2_dev:jazzy` / 컨테이너 `moveit2_dev`
- `build/`·`install/`·`log/`는 컨테이너 안에서 생성되며 `.gitignore` 대상

## 환경 의존 지점

**Dockerfile은 어느 PC에서든 같다.** PC마다 갈리는 것은 compose.yml에 모여 있다.

| 갈리는 것 | 처리 |
|---|---|
| GPU 벤더 | compose.yml `[GPU]` 섹션 — (A) AMD/Intel · (B) NVIDIA · (C) 소프트웨어 렌더링 |
| 렌더 노드 권한 | `/dev/dri` 통째 마운트 (cardN 번호 무관). 권한 오류 시 `group_add`로 호스트 GID 전달 |
| DISPLAY / X11 | `${DISPLAY}` 참조, 실행 전 `xhost +local:root` |
| UID/GID | `run_container.sh`가 `HOST_UID`/`HOST_GID`로 전달 |
| CPU 아키텍처 | x86_64 가정 (유일하게 이미지 층이 갈리는 지점) |

## 유지 규칙

1. **구조 변경 시 본 문서 우선 갱신** — 실제 폴더 이동보다 문서를 먼저 고친다.
2. `moveit2/` 하위 폴더에는 README를 두지 않는다 — 본 문서로 통합 (파일 산개 방지).
   `vision/`·`simulation/`·`operator/`는 설계 문서 참조용 README만 갖는다.
3. **세션 인수인계** — 새 세션은 `SESSION_LOG.md` 마지막 엔트리와 본 문서를 먼저 읽는다.
