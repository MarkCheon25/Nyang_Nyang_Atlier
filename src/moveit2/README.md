# MoveIt2 개발 환경

**ROS 2 Jazzy + MoveIt2 실행 환경**입니다. 같은 폴더의 `Dockerfile`로 **각자 PC에서 이미지를 직접 빌드**합니다 — 큰 이미지 파일을 주고받을 필요가 없습니다.

- 베이스: `ros:jazzy-ros-base` → `ros-jazzy-desktop` + `ros-jazzy-moveit` 설치
- 구성: Docker Compose 하나로 컨테이너 실행 + RViz2 GUI + ROS 2 DDS 통신

```
moveit2/
├── README.md            # (이 파일) 세팅 절차
├── Dockerfile           # 이미지 정의 — 어느 PC에서든 동일
├── compose.yml          # 실행 정의 — ⚠️ GPU 부분만 PC에 맞게 확인
├── entrypoint.sh        # 컨테이너 진입 시 ROS 환경 로드
├── run_container.sh     # 헬퍼 (build/up/shell/down/logs)
├── docs/                # 구조·치트시트·튜토리얼 실행 가이드
└── ws_moveit2/          # 작업 워크스페이스 — 컨테이너가 연결하는 루트
    └── src/             # 여기에 ROS 2 패키지를 두고 빌드
```

## 설계 책임 (구현 착수 전 — 설계 세션에서 정리된 내용)

이 모듈은 vision(`src/vision`, F2)이 만든 **지도**(정렬 안 된 스트로크 집합 — 구조는 `src/vision/README.md` 참조)를 받아서 다음을 담당한다.

| 단계 | 내용 |
|---|---|
| F3.1 스트로크 순서 최적화 | 로봇의 실제 이동거리 기준으로 스트로크 그리는 순서 결정 (AC2: 무최적화 대비 이동시간 20% 이상 단축) |
| F3.2 펜업/다운 결정 | 정해진 순서에 따라 스트로크 사이 펜업(이동) 구간 결정 |
| F4.1 좌표 변환 | 종이 mm 좌표 → 로봇 좌표 (calibration 결과 사용) |
| F4.2 경로 계획 | MoveIt2 데카르트 궤적 계획 |

F3이 vision에서 이 모듈로 옮겨진 경위·근거는 `src/vision/README.md`(A. 이미지 → 지도)에 기록.

---

## 0. 전제 조건 — 동료 PC에서 먼저 확인

**아래를 통째로 복사해 실행하면 필요한 항목이 한 번에 나옵니다.**

```bash
echo "아키텍처   : $(uname -m)"
echo "OS         : $(. /etc/os-release; echo "$PRETTY_NAME")"
echo "Docker     : $(docker --version 2>/dev/null || echo '✗ 없음')"
echo "Compose    : $(docker compose version --short 2>/dev/null || echo '✗ 없음 (v2 필요)')"
echo "docker 그룹: $(groups | grep -qw docker && echo '✓ 소속' || echo '✗ 없음')"
echo "디스크 여유: $(df -h / | awk 'NR==2{print $4}')"
echo "GPU        : $(lspci | grep -Ei 'vga|3d' | sed 's/.*: //' | head -1)"
echo "렌더 노드  : $(ls /dev/dri/ 2>/dev/null | grep -v by-path | tr '\n' ' ')"
echo "세션       : ${XDG_SESSION_TYPE:-?} / DISPLAY=${DISPLAY:-✗ 미설정}"
```

| 항목 | 필요한 값 | 안 맞으면 |
|---|---|---|
| 아키텍처 | `x86_64` | ARM(Apple Silicon 등)은 **미검증** |
| OS | Ubuntu 22.04+ | 다른 배포판도 Docker만 되면 대개 동작 |
| Docker Engine | 설치됨 | 아래 설치 절차 |
| Docker Compose | **v2** (`docker compose`) | `docker-compose`(v1, 하이픈)는 안 됩니다 |
| **docker 그룹** | 소속 | `sudo usermod -aG docker $USER` → **재로그인** |
| **디스크 여유** | **10GB 이상** | 이미지 6.1GB + 빌드 산출물·apt 캐시 |
| GPU | 확인만 | §2에서 A/B/C로 분기 |
| 렌더 노드 | `/dev/dri/`에 `card*`·`renderD*` | 없으면 §2의 **(C) 소프트웨어 렌더링** |
| **세션 타입** | `x11` 또는 `wayland` | Wayland면 XWayland 경유 — 대개 자동이지만 RViz가 안 뜨면 §3 확인 |
| 네트워크 | — | 첫 빌드에 apt로 **수 GB** 내려받습니다 |

> **자동으로 맞춰지는 것들** — 아래는 `run_container.sh`가 각 PC 값을 읽어 처리하므로 신경 쓰지 않아도 됩니다.
> 컨테이너 사용자 UID/GID · `XAUTHORITY` 경로(없으면 생성) · `/dev/dri` 렌더 그룹 GID(`video`·`render`).

<details>
<summary>Docker 미설치 시 (Ubuntu)</summary>

```bash
# Docker 공식 설치 스크립트
curl -fsSL https://get.docker.com | sudo sh
# 현재 사용자를 docker 그룹에 추가 (재로그인 필요)
sudo usermod -aG docker $USER
newgrp docker
docker run --rm hello-world   # 동작 확인
```
</details>

---

## 1. 이 폴더 받기

리포를 clone 하거나 `moveit2/` 폴더를 통째로 복사합니다. 텍스트 파일 몇 KB뿐입니다.

---

## 2. ⚠️ GPU 설정 확인 (PC마다 다른 유일한 부분)

`compose.yml`의 `[GPU]` 섹션에 **(A)/(B)/(C) 세 경우**가 정리되어 있습니다.

| GPU | 할 일 |
|---|---|
| **AMD / Intel** | **그대로 두면 됩니다.** `/dev/dri`를 통째로 넘기므로 `cardN` 번호를 맞출 필요가 없습니다 |
| **NVIDIA** | `devices:` 블록을 (B) 블록으로 교체 + `nvidia-container-toolkit` 설치 |
| **없음 / 문제 발생** | (C) `LIBGL_ALWAYS_SOFTWARE=1` 로 소프트웨어 렌더링 (느리지만 동작 확인 가능) |

---

## 3. X11 GUI 허용 (RViz2 창 띄우기)

```bash
xhost +local:root
```

> 재부팅하면 초기화됩니다. `run_container.sh`가 실행 시 자동으로 한 번 걸어주지만, GUI가 안 뜨면 직접 실행해 보세요.
> Wayland 세션이면 X11 앱 호환을 위해 XWayland가 필요합니다(대개 기본 설치).

---

## 4. 이미지 빌드 (최초 1회)

```bash
cd moveit2
./run_container.sh build
```

> **20~40분 걸립니다** (apt로 ROS 2 desktop + MoveIt2 내려받음). 네트워크 속도에 따라 더 걸릴 수 있습니다.
> 빌드는 처음 한 번만 하면 되고, 이후에는 `shell`로 바로 들어갑니다.

---

## 5. 컨테이너 진입

```bash
./run_container.sh shell
```

- 컨테이너가 안 떠 있으면 자동으로 기동한 뒤 들어갑니다.
- 두 번째 터미널이 필요하면 같은 명령을 다시 실행하면 됩니다.
- ROS 2 환경은 진입 시 자동 로드됩니다 (`entrypoint.sh`).

```bash
./run_container.sh down    # 정지 및 제거
./run_container.sh logs    # 로그 확인
```

---

## 6. 동작 확인

컨테이너 안에서 — **HCR-5** (이 프로젝트의 로봇):
```bash
cd ~/ws_moveit2 && colcon build --symlink-install && source install/setup.bash
ros2 launch hcr_moveit_config demo.launch.py
```
→ RViz2에 HCR-5가 뜨고 인터랙티브 마커로 Plan/Execute가 되면 성공.

MoveIt 기본 예제(**Panda**)로 환경만 확인하려면:
```bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

이후 실행 명령은 [`docs/moveit2_튜토리얼_실행_가이드.md`](docs/moveit2_%ED%8A%9C%ED%86%A0%EB%A6%AC%EC%96%BC_%EC%8B%A4%ED%96%89_%EA%B0%80%EC%9D%B4%EB%93%9C.md)를 참고하세요.

---

## 7. 내 소스 빌드하기

호스트의 `ws_moveit2/`가 컨테이너의 `/home/rosuser/ws_moveit2/`로 연결됩니다.
**컨테이너 안에서 만든 결과물이 호스트에 그대로 남습니다.**

```bash
# 컨테이너 안에서
cd ~/ws_moveit2
colcon build --symlink-install
source install/setup.bash
ros2 run hello_moveit hello_moveit
```

> `build/`·`install/`·`log/`는 빌드 산출물이라 git에 올리지 않습니다 (`.gitignore` 등록).
> 지워도 `colcon build`로 다시 만들어집니다.

### ⚠️ 호스트와 컨테이너의 디렉터리 배치가 다릅니다

HCR-5 로봇 모델(`hcr_robot_description` · `hcr_moveit_config`)은 **호스트에서는 리포 최상위**에 있지만, **컨테이너 안에서는 워크스페이스 `src/` 아래**에 나타납니다.

```
호스트                                    컨테이너
Nyang_Nyang_Atlier/
├── hanwha_robot_arm/          ─────┐
│   └── HCR_5/                      │
│       ├── hcr_robot_description/  │    /home/rosuser/ws_moveit2/
│       └── hcr_moveit_config/      ├──►   ├── src/
└── src/moveit2/                    │      │   ├── hello_moveit/
    └── ws_moveit2/            ─────┘      │   └── hanwha_robot_arm/HCR_5/…
        └── src/hello_moveit/                └── build· install· log
```

**왜 이렇게 했나** — `hanwha_robot_arm/`은 외부(커뮤니티 ROS1 패키지)에서 가져와 포팅한 자산이라, 자체 개발 코드(`src/`)와 섞이지 않도록 리포 최상위에 두었습니다. 그런데 colcon은 워크스페이스 `src/` 아래에 있는 패키지만 빌드하므로, 호스트 배치를 그대로 두면서 컨테이너 안에서만 워크스페이스로 들어오도록 `compose.yml`에 볼륨을 하나 더 두었습니다.

```yaml
- ../../hanwha_robot_arm:/home/rosuser/ws_moveit2/src/hanwha_robot_arm:rw
```

실무상 알아둘 점:
- **`colcon build`는 세 패키지를 한꺼번에 빌드합니다** — `hello_moveit` · `hcr_robot_description` · `hcr_moveit_config`
- 컨테이너 안에서 `~/ws_moveit2/src/hanwha_robot_arm/`을 고치면 **호스트 최상위 `hanwha_robot_arm/`이 바뀝니다** (같은 디렉터리)
- 호스트에서 `ws_moveit2/src/`를 봐도 `hanwha_robot_arm`은 **없습니다** — 컨테이너 안에서만 보이는 게 정상입니다

---

## 8. 트러블슈팅

| 증상 | 원인 / 해결 |
|---|---|
| `docker: permission denied` | docker 그룹 미소속. `sudo usermod -aG docker $USER` 후 **재로그인** |
| `docker compose` 명령이 없음 | Compose v1(`docker-compose`) 설치됨. v2 플러그인 필요 |
| 빌드가 apt 단계에서 실패 | 네트워크/미러 문제. 잠시 후 `./run_container.sh build` 재시도 |
| 빌드 중 `no space left on device` | 디스크 부족. `docker system prune -a`로 정리 후 재시도 |
| RViz2 창이 안 뜸 | `xhost +local:root` 재실행 / `echo $DISPLAY` 확인 / Wayland면 XWayland 설치 확인 |
| `cannot open /dev/dri/renderD128` | 렌더 그룹 권한. `run_container.sh`가 자동 처리하지만, 직접 `docker compose`를 호출했다면 헬퍼로 실행하세요 |
| RViz는 뜨는데 3D가 검거나 느림 | GPU passthrough 실패 → (C) `LIBGL_ALWAYS_SOFTWARE=1`로 임시 확인 |
| 컨테이너가 만든 파일이 root 소유 | `run_container.sh`를 거치지 않은 경우. 헬퍼로 실행하면 UID/GID가 맞춰집니다 |
| 다른 ROS 2 노드와 통신 안 됨 | `ROS_DOMAIN_ID`를 상대와 동일하게 (`ROS_DOMAIN_ID=7 ./run_container.sh shell`) |

**GUI가 안 뜰 때 진단 순서**

```bash
# 호스트에서
xhost +local:root
echo $DISPLAY                      # compose.yml이 이 값을 그대로 넘긴다

# 컨테이너 안에서
xdpyinfo | head -3                 # X 서버 접속 확인
glxinfo -B                         # 렌더러 확인 (llvmpipe면 소프트웨어 렌더링)
```

---

## 참고

- 컨테이너 매핑 상세: [`docs/DIRECTORY_STRUCTURE.md`](docs/DIRECTORY_STRUCTURE.md)
- 자주 쓰는 명령: [`docs/CHEATSHEET.md`](docs/CHEATSHEET.md)
- 튜토리얼 실행: [`docs/moveit2_튜토리얼_실행_가이드.md`](docs/moveit2_%ED%8A%9C%ED%86%A0%EB%A6%AC%EC%96%BC_%EC%8B%A4%ED%96%89_%EA%B0%80%EC%9D%B4%EB%93%9C.md)
- MoveIt2 공식 튜토리얼: https://moveit.picknik.ai/main/index.html
