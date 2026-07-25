# Nyang_Nyang_Atlier 🐾

> **고양이 이미지 한 장을 넣으면, 로봇암이 A4 종이에 연필로 선화를 그린다.**
> 로봇이 그리고, AI가 채점한다.

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)
![MoveIt 2](https://img.shields.io/badge/MoveIt-2-0A7BBB)
![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420?logo=ubuntu&logoColor=white)
![Status](https://img.shields.io/badge/status-구현_진행중-yellow)

---

## 개요

6-DoF 협동로봇암이 **고양이 이미지를 받아 사람 개입 없이 A4 종이에 연필 선화를 완성**하는 시스템입니다.
전시회·캣카페에서 방문객의 반려묘 사진을 즉석 선화로 그려주는 시나리오를 목표로 합니다.

경쟁재("AI 그림 + 프린터")와의 차별점은 **로봇이 그리는 광경 자체가 관람 상품**이라는 점과 **진짜 연필로 그린 실물 원본**이라는 두 가지입니다. 기술 우선순위는 이 기준으로 판단합니다.

현재 **MVP(sim First Stroke)** 구현 단계입니다 — MoveIt2 mock hardware + RViz2 환경에서 선화를 완주하는 것이 목표입니다.

| 항목 | 내용 |
|---|---|
| 개발 기간 | **2026-07-22 ~ 2026-08-15** |
| 인원 | **4명** |

> 요구사항·설계 근거·수락 기준·로드맵은 [`docs/`](docs/)에 있습니다.

---

## 1. 주요 기능

| # | 기능 그룹 | 내용 |
|---|---|---|
| **F1** | 입력 | 이미지 업로드(PNG/JPG) · 입력 적합성 검사 |
| **F2** | 이미지 처리 | 전처리(이진화·노이즈 제거) · 윤곽 추출 · 스트로크 변환 · A4 좌표 스케일링 |
| **F3** | 스트로크 계획 | 실행 순서 최적화 · pen-up/pen-down 이동 계획 · 최적화 전후 시간 로그 |
| **F4** | 로봇 제어·실행 | 종이→로봇 좌표 변환 · MoveIt2 궤적 계획 · 실행 및 진행 보고 |
| **F5** | 캘리브레이션 | hand-eye 캘리브레이션 · A4 종이 위치·자세 인식 · 결과 저장·재사용 |
| **F6** | 운영 | 업로드 UI · 진행률·상태 모니터링 · 작업 이력 · **긴급정지** · 일시정지/재개 |
| **F7** | 검증 환경 | MuJoCo 파이프라인 실행 · IsaacSim(선택) · sim/real 스위칭 |

상세 요구사항 명세: [`docs/Business Requirements.md`](docs/Business%20Requirements.md)

---

## 2. 시스템 설계 및 플로우차트

| 문서 | 다이어그램 원본 |
|---|---|
| [시스템 구조 (System Architecture)](docs/System%20Architecture.md) | [`System_Architecture.drawio`](docs/System_Architecture.drawio) |
| [프로세스 흐름 (Process Flow)](docs/Process%20Flow.md) | [`Process_Flow.drawio`](docs/Process_Flow.drawio) |

> `.drawio` 파일은 GitHub에서 바로 미리보기되지 않습니다. [draw.io](https://app.diagrams.net/)에서 열어 보세요.
> 각 마크다운 문서에는 같은 내용이 mermaid 다이어그램으로 들어 있어 GitHub에서 바로 볼 수 있습니다.

### 파이프라인

이미지 한 장이 로봇 관절값이 되기까지:

```mermaid
flowchart LR
    A["입력 이미지<br/>(라인아트)"]
    B["스트로크 폴리라인<br/>(픽셀 좌표)"]
    C["종이 평면 경로<br/>(x, y mm)"]
    D["관절 궤적<br/>(q1~q6)"]
    E["A4에 연필 스트로크"]
    A -->|"① 이진화 · 윤곽 · centerline"| B
    B -->|"② 픽셀→종이 스케일링 · 순서 최적화"| C
    C -->|"③ 좌표 변환 · IK 계획 (MoveIt2)"| D
    D -->|"④ ros2_control → mock / real"| E
```

**하드웨어 추상화 경계**를 두어 같은 제어 코드가 시뮬레이션과 실물에서 동일하게 동작합니다.

### 디렉터리 구조

```
Nyang_Nyang_Atlier/
├── README.md
├── docs/                          요구사항·설계 문서 + 다이어그램
│   ├── Business Requirements.md       BRD
│   ├── System Architecture.md         시스템 구조
│   ├── Process Flow.md                프로세스 흐름
│   ├── System_Architecture.drawio
│   └── Process_Flow.drawio
└── src/
    ├── moveit2/                   MoveIt2 실행 환경(Docker) + ROS 2 워크스페이스
    │   ├── Dockerfile  compose.yml  run_container.sh
    │   ├── docs/                      환경 세팅·실행 가이드
    │   └── ws_moveit2/                컨테이너가 연결하는 워크스페이스
    ├── vision/                    (예정) 이미지 처리
    ├── simulation/                (예정) 검증 환경
    └── operator/                  (예정) 웹 운영 레이어 · DB
```

---

## 3. 운영체제 및 개발환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04.4 LTS (커널 6.8) |
| 미들웨어 | ROS 2 Jazzy (`rmw-cyclonedds` 권장) |
| 언어 | C++17 (제품 코드 전 구간) · Python (검증·프로토타입 도구) |
| 빌드 | colcon (`ament_cmake`) |
| 실행 환경 | Docker + Compose v2 — 호스트 오염 없이 격리 실행 |
| GUI | X11 (RViz2) — GPU 렌더링 패스스루 |

MoveIt2 개발 환경은 컨테이너로 제공됩니다. 이미지는 각 PC에서 직접 빌드하며, GPU(AMD·Intel / NVIDIA / 소프트웨어 렌더링) 분기와 UID·X11 설정은 실행 스크립트가 자동 처리합니다.

세팅 절차: [`src/moveit2/docs/README_동료용.md`](src/moveit2/docs/README_%EB%8F%99%EB%A3%8C%EC%9A%A9.md)

---

## 4. 사용 장비 목록

| 구성 | 사양 |
|---|---|
| 로봇암 | **Hanwha Techwin HCR-5** — 6-DoF 협동로봇, reach 915mm / payload 5kg / 반복정밀도 ±0.1mm |
| 엔드이펙터 | 스프링 내장 펜홀더 (자체 설계 · 3D 프린팅) — 플랜지 직결, 그리퍼 미사용 |
| 카메라 | Intel RealSense — **모델 미정** · eye-to-hand, 외부 기둥 고정 |
| 작업대 | 철제 책상 상판 수평 고정 · A4 종이 지그 (자체 제작) |
| 필기구 | 연필 (A4 선화) |

---

## 5. 기술 스택

| 구분 | 기술 |
|---|---|
| **Robotics — SW** | **MoveIt 2** (Jazzy) · `ros2_control` · `joint_trajectory_controller` · MuJoCo → IsaacSim 5.1(선택) |
| **Robotics — HW** | **Hanwha HCR-5** — 사양은 [§4](#4-사용-장비-목록) |
| **Computer Vision** | OpenCV 4.6 (+contrib `ximgproc`) — 이진화·윤곽·스트로크 변환 / ORB+Homography·SSIM — 작화 검증 / ChArUco — hand-eye 캘리브레이션 |
| **AI / Voice** | CLIP (HuggingFace `transformers`, zero-shot) — 결과물 품질 판정 · **Voice 미정** |
| **Sensor** | Intel RealSense — **모델·드라이버 버전 미정** |
| **Communication** | **ROS 2 Jazzy** (`rmw-cyclonedds` 권장) · `rosbridge_suite` (websocket :9090) · React + `roslibjs` · FastAPI (REST) + `rclpy` 노드 |
| **Database** | SQLite + SQLAlchemy ORM — 이미지는 파일시스템, DB엔 경로·메타 |
| **Tool** | **Git** · Docker + Compose v2 · colcon (`ament_cmake`) · draw.io |
| **OS** | Ubuntu 24.04 LTS (커널 6.8) |
| **Programming** | **C++17** (제품 코드 전 구간) · Python (검증·프로토타입 도구) |

**함정 메모** — 설계 검증에서 확인된 제약입니다.
- `realsense-ros`는 **DKMS 금지**, ROS apt로 설치
- `rosbridge_suite`는 토픽·서비스만 지원 — **액션 미지원**이라 MoveIt2 제어는 서버측에서 수행
- `moveit_calibration`은 Jazzy 바이너리 미제공 → `easy_handeye2` 등으로 대체

버전 선정 근거와 검증 결과: [`docs/System Architecture.md`](docs/System%20Architecture.md) §5

---

## 6. 실행 순서

> 🚧 **추후 업데이트 예정** — 파이프라인 구현이 진행되는 대로 전체 실행 순서를 채웁니다.

현재 실행 가능한 것은 MoveIt2 개발 환경입니다:

```bash
cd src/moveit2
./run_container.sh build    # 최초 1회 (20~40분)
./run_container.sh shell    # 컨테이너 진입
```

컨테이너 안에서 동작 확인:
```bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

상세: [`src/moveit2/docs/README_동료용.md`](src/moveit2/docs/README_%EB%8F%99%EB%A3%8C%EC%9A%A9.md) · [실행 가이드](src/moveit2/docs/moveit2_%ED%8A%9C%ED%86%A0%EB%A6%AC%EC%96%BC_%EC%8B%A4%ED%96%89_%EA%B0%80%EC%9D%B4%EB%93%9C.md)

---

<sub>Phase 1은 포트폴리오 / 학습용 개인 프로젝트입니다.</sub>
