# backend — 웹 백엔드 (Node.js)

**미착수 — 자리만 잡아 둔 상태.** 아래는 방침이며 코드는 아직 없다.

## 역할

사람과 파일·기록을 다루는 쪽만 맡는다. **ROS 를 몰라도 되는 일만 여기 있다.**

| 하는 일 | 근거 |
|---|---|
| 이미지 업로드 수신 → 파일시스템 저장 | F1.1 |
| 작업 이력 저장·조회 (SQLite) | F6.3 |
| 완성작 이미지·정적 파일 서빙 | F8.2 · F6.1 |
| AC3 채점기(CLIP, Python) 호출·결과 보관 | AC3 · BRD N1 예외 |

## 하지 않는 일

- **작업 상태를 소유하지 않는다.** 어디까지 그렸는지, 지금 멈출 수 있는지는
  `job_manager`(C++ ROS 노드)가 안다. 백엔드가 상태를 따로 들면 두 곳이 어긋난다.
- **MoveIt2 를 직접 부르지 않는다.** SA §5.2 — 액션 호출은 서버측(job_manager) 몫이다.

## ROS 와 만나는 지점

**rosbridge websocket(:9090)을 구독만 한다.** `roslib` 를 Node 에서 그대로 쓸 수 있다.

```
job_manager ──(상태·레코드 발행)── rosbridge ──ws── backend ──→ SQLite
                                        └──ws── React (실시간 화면)
```

- 브라우저와 **별개로** 붙는다 — 브라우저를 닫아도 이력이 남아야 하기 때문이다.
- 명령(정지·일시정지·계속)은 브라우저가 rosbridge 로 직접 보낸다. 백엔드를 거치지 않는다.

## DB

- **SQLite** (SA §5.1 확정). ORM 은 Node 쪽에서 고른다 (SQLAlchemy 는 Python 것이라 폐기)
- **DB 에 직접 붙는 것은 이 백엔드 하나뿐**이다 (SA §5.2). C++ 쪽에 DB 드라이버를 붙이지 말 것
- **이미지는 파일시스템, DB 에는 경로·메타만** (SA §5.1)
- 저장할 레코드의 형식은 **`job_core` 가 발행하는 JSON 이 원본**이다 —
  `ws_operator/src/job_core/include/atlier/job/job.hpp` 의 `Record` 와
  `job_json.hpp` 를 보고 스키마를 짠다. 수락 기준 판정(`acceptance`)도 그 JSON 에
  실려 오므로 **판정 규칙을 여기서 다시 구현하지 말 것** — 두 언어에 중복되면 어긋난다

`data/` 는 런타임 산출물이라 git 에 올리지 않는다. 경로는 환경변수(`ATLIER_DATA_DIR`)로
빼서 나중에 리포 밖으로도 옮길 수 있게 둔다.

## 확인 방법 (착수 전)

백엔드가 받게 될 레코드는 지금도 볼 수 있다:

```bash
ros2 run job_core job_cli --scenario all --out-dir ~/data/out
cat ~/data/out/happy.job.json
```

## 설계 참조

| 문서 | 경로 |
|---|---|
| 이 모듈의 인터페이스 원본 | `../README.md` |
| 시스템 구조 | `../../../docs/System Architecture.md` — §5.1 기술 스택, §5.2 웹↔ROS2 연동 |
| 요구사항 | `../../../docs/Business Requirements.md` — F1.1 · F6.1 · F6.3 |
