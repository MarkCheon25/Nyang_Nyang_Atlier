# MoveIt2 튜토리얼 환경 세팅 (동료용)

이 폴더는 **MoveIt2 튜토리얼 실행 환경**을 담고 있습니다.
직접 빌드할 필요 없이 **MoveIt 공식 사전 빌드 이미지**를 받아서 바로 실행합니다.

- 이미지: `moveit/moveit2:main-jazzy-tutorial-source` (ROS 2 **Jazzy** 기반, 약 9.7GB)
- 구성: Docker Compose 하나로 컨테이너 실행 + RViz2 GUI + ROS 2 DDS 통신

```
moveit2/
├── compose.yml                    # Docker 실행 정의 (GPU 부분만 PC에 맞게 수정)
├── README_동료용.md                # (이 파일) 세팅 절차
├── moveit2_튜토리얼_실행_가이드.md   # 컨테이너 안에서 튜토리얼 돌리는 명령어
└── src/                           # 컨테이너에 마운트되는 작업 폴더 (여기에 소스 두면 됨)
```

---

## 0. 전제 조건 (호스트 PC)

| 항목 | 확인 명령 | 비고 |
|---|---|---|
| OS | `lsb_release -a` | Ubuntu 22.04+ 권장 |
| Docker Engine | `docker --version` | 없으면 아래 설치 |
| Docker Compose v2 | `docker compose version` | `docker-compose`(구버전) 아님 |
| GPU 드라이버 | `ls -l /dev/dri/` 또는 `nvidia-smi` | RViz2 3D 렌더링용 |
| X 서버(GUI) | `echo $DISPLAY` | 보통 `:0` |

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

## 1. 이 폴더를 동료 PC로 복사

`moveit2/` 폴더 전체를 동료 PC의 원하는 경로에 복사합니다. (예: `~/moveit2/`)

```bash
# 예시: USB/scp/git 등으로 폴더 전달 후
cd ~/moveit2
```

---

## 2. 이미지 받기 (pull)

```bash
docker pull moveit/moveit2:main-jazzy-tutorial-source
```

> 약 9.7GB라 시간이 걸립니다. 인터넷이 느리면 Mark가 이 PC에서 `docker save`로 tar를 떠서
> 직접 전달할 수도 있습니다 (필요 시 요청).

---

## 3. ⚠️ GPU 설정 확인 (가장 중요 — PC마다 다름)

`compose.yml`의 `devices:` 부분은 **이 기준 PC(LenV15) 전용 경로**로 되어 있습니다.
동료 PC의 GPU에 맞게 **반드시 확인·수정**하세요.

```bash
# GPU 제조사 확인
lspci | grep -Ei 'vga|3d'
# 렌더 디바이스 확인
ls -l /dev/dri/
```

- **AMD / Intel GPU** → `/dev/dri/`에 보이는 실제 `cardN` 번호로 `compose.yml` 수정
  - 대부분 `card0` (기준 PC는 예외적으로 `card1`)
- **NVIDIA GPU** → `compose.yml`의 (B) NVIDIA 블록 사용 + `nvidia-container-toolkit` 설치
- **GPU 문제로 막히면** → (C) 소프트웨어 렌더링(`LIBGL_ALWAYS_SOFTWARE=1`)으로 우선 동작 확인

> 자세한 분기 방법은 `compose.yml` 안의 `[GPU]` 주석에 A/B/C로 정리되어 있습니다.

---

## 4. X11 GUI 허용 (RViz2 창 띄우기)

컨테이너가 호스트 화면에 창을 띄울 수 있도록 **실행 전 1회** 허용합니다.

```bash
xhost +local:docker
```

> 재부팅하면 초기화되므로, GUI가 안 뜨면 이 명령을 다시 실행하세요.
> (Wayland 세션이면 X11 앱 호환을 위해 XWayland가 필요할 수 있습니다.)

---

## 5. 컨테이너 실행

```bash
cd ~/moveit2      # compose.yml 이 있는 폴더
docker compose run --name moveit2_dev --rm moveit2
```

- `--name moveit2_dev` : 두 번째 터미널에서 `docker exec`로 붙기 위해 **반드시 지정**
- `--rm` : 종료 시 컨테이너 자동 삭제 (작업물은 `./src`에 남으므로 안전)

컨테이너 안에서 ROS 2 환경을 source (첫 진입 시 보통 자동 적용):
```bash
source /opt/ros/jazzy/setup.bash
source /root/ws_moveit/install/setup.bash
```

### 두 번째 터미널 붙기 (튜토리얼 노드 실행용)
```bash
docker exec -it moveit2_dev bash
# 안에서:
source /opt/ros/jazzy/setup.bash && source /root/ws_moveit/install/setup.bash
```

---

## 6. 동작 확인 (RViz2 데모)

터미널 1(컨테이너 안)에서:
```bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```
→ **RViz2 창이 뜨고 Panda 로봇 팔이 보이면 환경 세팅 성공.**

이후 튜토리얼 실행 명령어는 `moveit2_튜토리얼_실행_가이드.md` 참고.

---

## 7. 트러블슈팅

| 증상 | 원인 / 해결 |
|---|---|
| `docker: permission denied` | `sudo usermod -aG docker $USER` 후 재로그인 |
| RViz2 창이 안 뜸 | `xhost +local:docker` 재실행 / `DISPLAY` 값 확인 (`echo $DISPLAY`) |
| `cannot open /dev/dri/cardN` | `ls -l /dev/dri/`로 실제 번호 확인 후 `compose.yml` 수정 |
| RViz는 뜨는데 3D가 검거나 느림 | GPU passthrough 실패 → (C) `LIBGL_ALWAYS_SOFTWARE=1`로 임시 확인 |
| `docker exec ... moveit2_dev` 안 됨 | 실행 시 `--name moveit2_dev` 빠짐. `docker ps`로 실제 이름 확인 |
| 다른 ROS 2 노드와 통신 안 됨 | `ROS_DOMAIN_ID`를 상대와 동일하게 맞춤 (기본 0) |

---

## 참고

- 원본 세팅 위치: `harnessing_260603/cpp_project_i/` (이 폴더는 그 사본 + GPU 이식 주석 추가본)
- MoveIt2 공식 튜토리얼: https://moveit.picknik.ai/main/index.html
