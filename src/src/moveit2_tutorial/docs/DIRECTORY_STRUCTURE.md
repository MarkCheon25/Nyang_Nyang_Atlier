# moveit2_tutorial — 디렉토리 구조

본 문서는 `Nyang_Nyang_Atlier_260704/src/moveit2_tutorial` 프로젝트 구조와 각 폴더의 목적을 정의한다.
반복적인 구조 재탐색을 방지하고, 이후 세션에서 바로 참조할 수 있도록 유지한다.

## 전체 트리

```
Nyang_Nyang_Atlier_260704/
└── src/
    └── moveit2_tutorial/
        ├── docs/                       # 문서 및 학습 기록
        │   ├── DIRECTORY_STRUCTURE.md  # (본 문서) 폴더 구조 정의
        │   ├── SESSION_LOG.md          # 세션별 학습·작업 기록
        │   └── CHEATSHEET.md           # 자주 쓰는 ROS2/MoveIt2 명령 정리
        │
        ├── docker/                     # 컨테이너 정의
        │   ├── Dockerfile              # ROS 2 Jazzy + MoveIt2 이미지
        │   ├── docker-compose.yml      # 실행 서비스 정의 (X11, GPU, 볼륨)
        │   └── entrypoint.sh           # 컨테이너 진입 시 환경 셋업
        │
        ├── ws_moveit/                  # 컨테이너에 마운트되는 ROS 2 워크스페이스
        │   └── src/                    # colcon build 대상 패키지 배치
        │
        └── scripts/                    # 호스트 측 실행 헬퍼
            └── run_container.sh        # 컨테이너 build/up/exec 진입 헬퍼
```

## 폴더별 역할

### `docs/`
학습·의사결정 기록. 코드가 아닌 지식과 절차를 저장.
- `DIRECTORY_STRUCTURE.md` — 본 문서. 구조 변경 시 반드시 동기화.
- `SESSION_LOG.md` — 각 세션(다알리아 → 라... 순)별 진행 내용, 실행 명령, 스크린샷 경로, 발생한 에러/해결 방법.
- `CHEATSHEET.md` — 반복 사용하는 명령(`colcon build`, `ros2 launch`, `source install/setup.bash` 등) 축적.

### `docker/`
호스트(Ubuntu 24.04)를 오염시키지 않기 위한 격리 환경 정의.
- ROS 2 Jazzy는 24.04 네이티브지만, 튜토리얼 전용 격리를 위해 컨테이너 사용.
- RTX 5080 GPU 접근 필요 시 `--gpus all` (RViz3 렌더링 가속).

### `ws_moveit/`
ROS 2 워크스페이스. 컨테이너의 `/root/ws_moveit`(또는 사용자 홈 하위)에 볼륨 마운트.
- `src/` 하위에 튜토리얼 패키지(`hello_moveit`, `moveit2_tutorials` 등) 배치.
- `build/`, `install/`, `log/`은 `.gitignore` 대상(컨테이너 내부에서만 생성).

### `scripts/`
호스트에서 실행하는 헬퍼 스크립트.
- `run_container.sh` — build/up/exec/down 서브커맨드로 반복 작업 단순화.

## 유지 규칙

1. **구조 변경 시 본 문서 우선 수정** — 실제 폴더 생성/이동보다 문서를 먼저 갱신.
2. **각 폴더에 README 없음** — 본 문서로 통합 관리 (파일 산개 방지).
3. **컨테이너 내부 경로 매핑**
   - 호스트 `ws_moveit/` ↔ 컨테이너 `/home/rosuser/ws_moveit/`
   - 호스트 `docs/` ↔ (마운트 안 함, 호스트 전용)
4. **세션 인수인계** — 새 세션 시작 시 `SESSION_LOG.md` 마지막 엔트리와 본 문서를 먼저 읽음.

## 호스트 환경 (2026-07-18 확인)

| 항목 | 값 |
|------|-----|
| OS | Ubuntu 24.04.4 LTS |
| Docker | 29.6.2 |
| Docker Compose | v5.3.1 |
| NVIDIA Container Toolkit | 1.19.1 |
| GPU | RTX 5080 Laptop (driver 595.71.05) |
| DISPLAY | `:1` (소켓 `/tmp/.X11-unix/X1`) |
| 사용자 UID/GID | 1000/1000 (markch03) |
