# operator

미구현 — 디렉터리만 확보한 상태. 아래 구성은 **방침이며 실제 폴더는 아직 만들지 않았다.**

## 역할

웹 운영 레이어. 이미지 업로드(입력) · 진행 모니터링 · 결과 표시(출력) · 티칭펜던트 조작 · 작업 이력 관리.

## 구성 (착수 시)

```
src/operator/
├── backend/                FastAPI + rclpy 노드 (별도 스레드 spin)
│   ├── app/
│   │   ├── api/            REST 라우터 — 업로드 · 이력 조회 · 티칭펜던트 명령
│   │   ├── models/         SQLAlchemy ORM
│   │   └── ros/            rclpy 노드 — MoveIt2 구동, 상태 발행
│   └── migrations/         Alembic
├── frontend/               React + roslibjs
└── data/                   ⛔ git 제외 — atlier.db, uploads/, results/
```

## DB

- **SQLite + SQLAlchemy ORM** (SA §5.1 확정)
- **DB에 직접 붙는 것은 FastAPI 하나** — 다른 모듈은 REST를 경유한다 (SA §5.2). DB는 공유 저장소가 아니라 operator가 소유하는 내부 자산이므로 이 디렉터리 아래 둔다.
- **이미지는 파일시스템에 두고 DB에는 경로·메타만** 저장 (SA §5.1)
- 저장 대상 = 작업 이력: 입력 이미지 · 계획 · 실행 결과 · 소요시간 (BRD **F6.3**)

`data/`는 런타임 산출물이라 git에 올리지 않는다. 구현 착수 시 `.gitignore`에 등록하고, 경로는 환경변수(`ATLIER_DATA_DIR`)로 빼서 나중에 리포 밖(루트 디렉터리)으로도 옮길 수 있게 둔다.

## 설계 제약 — MoveIt2 제어 경로

SA §5.2: **MoveIt2 제어를 웹에서 직접 호출하지 않는다.** rosbridge 액션이 불안정하다는 검증 결과에 따라, 서버측(rclpy 노드)이 MoveIt을 구동하고 웹은 상태만 구독한다.

```
명령      React --(REST)--> FastAPI --(rclpy)--> MoveIt2
텔레메트리 React <--(rosbridge 구독)-- 로봇 상태
```

**티칭펜던트 UI도 이 경로를 따른다** — roslibjs로 MoveIt 액션을 직접 호출하지 않는다.

## 설계 참조

| 문서 | 경로 |
|---|---|
| 시스템 구조 | `docs/System Architecture.md` — §4 구성 블록, §5.1 기술 스택, §5.2 웹↔ROS2 연동 |
| 요구사항 | `docs/Business Requirements.md` — F6.3 작업 이력 관리 |
| 처리 흐름 | `docs/Process Flow.md` |
| 다이어그램 | `docs/System_Architecture.drawio`, `docs/Process_Flow.drawio` |
