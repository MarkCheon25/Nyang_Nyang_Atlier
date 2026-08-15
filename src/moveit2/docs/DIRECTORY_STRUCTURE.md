# moveit2 — 컨테이너 매핑

`src/moveit2/` 환경에서 호스트와 컨테이너의 디렉터리가 어떻게 대응되는지를 정의한다.

> **디렉터리 트리와 GPU 분기는 이 문서에 두지 않는다** — README가 원본이다.
> 리포 전체 트리는 리포 최상위 `README.md`, `moveit2/` 하위 트리와 GPU (A)/(B)/(C) 분기는 `../README.md` §2 참조.

## 컨테이너 매핑

| 호스트 | 컨테이너 |
|---|---|
| `src/moveit2/ws_moveit2/` | `/home/rosuser/ws_moveit2/` (rw) |
| **`hanwha_robot_arm/`** | **`/home/rosuser/ws_moveit2/src/hanwha_robot_arm/`** (rw) |
| **`src/drivers/`** | **`/home/rosuser/ws_moveit2/src/drivers/`** (rw) |
| `src/moveit2/docs/` | 마운트 안 함 — 호스트 전용 |

> **호스트와 컨테이너의 배치가 다르다.** `hanwha_robot_arm/`(리포 최상위)과 `src/drivers/`(모듈 트리)는 호스트에서 워크스페이스 밖에 있지만 컨테이너 안에서는 워크스페이스 `src/` 아래로 보인다. 호스트의 `ws_moveit2/src/`에 그 둘이 **비어 있는 마운트 지점으로만** 있는 것이 정상이다.
> `colcon build`는 다섯 패키지를 함께 빌드한다 — `hello_moveit` · `hcr_robot_description` · `hcr_moveit_config` · **`hcr5_bridge`**(실기 브릿지·ros2_control 플러그인) · **`nyang_pen`**(펜홀더 URDF). 뒤의 둘은 `src/drivers/` 하위다.
>
> ⚠️ **`src/drivers` 마운트는 컨테이너 재생성으로만 붙는다** — 재시작으로는 안 바뀐다(`compose.yml:72` 주석). 이 마운트가 없는 컨테이너에서는 `hcr5_bridge` 가 통째로 안 보인다.

- 컨테이너 사용자 `rosuser` — 호스트와 같은 UID/GID로 생성 (`run_container.sh`가 전달)
- 이미지 `moveit2_dev:jazzy` / 컨테이너 `moveit2_dev`
- `build/`·`install/`·`log/`는 컨테이너 안에서 생성되며 `.gitignore` 대상

## 환경 의존 지점

**Dockerfile 도 compose.yml 도 어느 PC에서든 같다.** PC마다 갈리는 것은 두 곳으로 나뉜다.

| 갈리는 것 | 어디서 처리 |
|---|---|
| **GPU 벤더·렌더 노드 권한** | 각 PC의 **`compose.override.yml`** (git 제외). `compose.yml`에 GPU 설정을 두지 않는 이유 |
| DISPLAY/X11 · UID/GID · 렌더 그룹 GID | `run_container.sh`가 호스트에서 읽어 자동 전달 |
| CPU 아키텍처 | x86_64 가정 — 유일하게 이미지 층이 갈리는 지점 |

확인·대응 절차는 `../README.md` §0(전제 조건)·§2(GPU)·§3(X11)이 원본이다.

## 유지 규칙

1. **매핑 변경 시 본 문서 우선 갱신** — 실제 볼륨 변경보다 문서를 먼저 고친다. 단 디렉터리 트리·GPU 분기·실행 절차는 README가 원본이므로 그쪽을 고친다.
2. **README는 `moveit2/` 바로 아래 하나만** — 세팅 진입점(전제조건·빌드·실행). GitHub에서 폴더를 열면 바로 렌더링되어 발견성이 높다.
   `docs/` 하위와 그 밖의 폴더에는 README를 두지 않는다 (파일 산개 방지).
   `vision/`·`simulation/`·`operator/`는 설계 문서 참조용 README만 갖는다.
3. **세션 인수인계** — 새 세션은 **리포 상위(루트) 디렉터리의 `작업일지.md`·`업무목록.md`** 마지막 엔트리와 본 문서를 먼저 읽는다.
   단 이 장부는 **리포에 포함되지 않는다** — 각 개발 PC에서 각자가 작성·관리하는 PC 로컬 자산이다 (03pc 적용 방식). 리포를 clone한 PC에는 이 파일이 없을 수 있다.
