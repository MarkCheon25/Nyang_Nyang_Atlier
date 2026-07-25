# MoveIt2 개발 환경 세팅 (동료용)

이 폴더는 **ROS 2 Jazzy + MoveIt2 실행 환경**을 담고 있습니다.
같은 폴더의 `Dockerfile`로 **각자 PC에서 이미지를 직접 빌드**합니다. 큰 이미지 파일을 주고받을 필요가 없습니다.

- 베이스: `ros:jazzy-ros-base` → `ros-jazzy-desktop` + `ros-jazzy-moveit` 설치
- 구성: Docker Compose 하나로 컨테이너 실행 + RViz2 GUI + ROS 2 DDS 통신

```
moveit2/
├── Dockerfile           # 이미지 정의 — 어느 PC에서든 동일
├── compose.yml          # 실행 정의 — ⚠️ GPU 부분만 PC에 맞게 확인
├── entrypoint.sh        # 컨테이너 진입 시 ROS 환경 로드
├── run_container.sh     # 헬퍼 (build/up/shell/down/logs)
├── docs/                # (이 파일 포함) 문서
└── ws_moveit2/          # 작업 워크스페이스 — 컨테이너가 연결하는 루트
    └── src/             # 여기에 ROS 2 패키지를 두고 빌드
```

---

## 0. 전제 조건 (호스트 PC)

| 항목 | 확인 명령 | 비고 |
|---|---|---|
| OS | `lsb_release -a` | Ubuntu 22.04+ 권장 |
| Docker Engine | `docker --version` | 없으면 아래 설치 |
| Docker Compose v2 | `docker compose version` | `docker-compose`(구버전) 아님 |
| GPU 드라이버 | `ls -l /dev/dri/` 또는 `nvidia-smi` | RViz2 3D 렌더링용 |
| X 서버(GUI) | `echo $DISPLAY` | 보통 `:0` 또는 `:1` |
| 아키텍처 | `uname -m` | `x86_64` 기준 |

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

```bash
lspci | grep -Ei 'vga|3d'    # GPU 제조사 확인
ls -l /dev/dri/              # 렌더 디바이스 확인
```

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
> (Wayland 세션이면 X11 앱 호환을 위해 XWayland가 필요할 수 있습니다.)

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

## 6. 동작 확인 (RViz2 데모)

컨테이너 안에서:
```bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```
→ **RViz2 창이 뜨고 Panda 로봇 팔이 보이면 환경 세팅 성공.**

이후 실행 명령은 `moveit2_튜토리얼_실행_가이드.md`를 참고하세요.

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

---

## 8. 트러블슈팅

| 증상 | 원인 / 해결 |
|---|---|
| `docker: permission denied` | `sudo usermod -aG docker $USER` 후 재로그인 |
| 빌드가 apt 단계에서 실패 | 네트워크/미러 문제. 잠시 후 `./run_container.sh build` 재시도 |
| RViz2 창이 안 뜸 | `xhost +local:root` 재실행 / `echo $DISPLAY` 값 확인 |
| `cannot open /dev/dri/renderD128` | 렌더 그룹 권한. `getent group video render`로 GID 확인 후 `compose.yml`의 `group_add`에 숫자로 추가 |
| RViz는 뜨는데 3D가 검거나 느림 | GPU passthrough 실패 → (C) `LIBGL_ALWAYS_SOFTWARE=1`로 임시 확인 |
| 컨테이너가 만든 파일이 root 소유 | `run_container.sh`를 거치지 않고 `docker compose`를 직접 호출한 경우. 헬퍼로 실행하면 UID/GID가 맞춰집니다 |
| 다른 ROS 2 노드와 통신 안 됨 | `ROS_DOMAIN_ID`를 상대와 동일하게 (`ROS_DOMAIN_ID=7 ./run_container.sh shell`) |

---

## 참고

- 구조·경로 매핑 상세: `DIRECTORY_STRUCTURE.md`
- 자주 쓰는 명령: `CHEATSHEET.md`
- MoveIt2 공식 튜토리얼: https://moveit.picknik.ai/main/index.html
