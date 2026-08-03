# HANDOFF — vision_LLM 브랜치 작업 인수인계

> **임시 문서.** 이 작업이 끝나면 삭제한다.
> 프로젝트 규칙상 세션 기록·협업 문서는 리포 밖(루트 디렉터리)에 두지만, **다른 PC로 넘기는 채널이 git 리포뿐이라 1회 예외로 리포에 넣었다** (Mark 승인 2026-08-03).
>
> 작성: 2026-08-03, 09pc 세션 `260803-페튜니아`
> 사유: **09pc 네트워크 단절 예정** — 네트워크가 되는 다른 PC로 작업 이관

---

## 0. 한 줄 요약

고양이 **사진**을 OpenAI 이미지 생성 API에 넣어 **라인아트**로 바꾸고, 그 라인아트에서 스트로크를 뽑아 HCR-5가 A4에 그린다. 이번 작업은 **"실제로 이미지가 나오는 데까지"** 가 목표다.

---

## 1. 목표 파이프라인

```
고양이 사진(JPG/PNG)
   │
   ├─ ① OpenAI 이미지 생성 API (image-to-image)   ← 이번 작업의 신규 구간
   │      "흰 배경 · 검은 단선 · 채색/해칭 없음" 라인아트로 변환
   ↓
라인아트 PNG
   │
   ├─ ② 기존 vision_core 파이프라인                 ← 이미 스켈레톤 있음
   │      이진화 → thinning(centerline) → 폴리라인 → mm 스케일링
   ↓
StrokeMap (종이 평면 mm 좌표, 순서 없음)
   │
   └─ ③ moveit2 (순서 최적화 → 궤적 → 실행)        ← 별도 작업, 이번 범위 밖
```

**①이 기존 설계의 빈틈을 메운다.** `src/vision/README.md`의 A(이미지→지도)는 입력 계약이 *"흰 배경 고양이 라인아트/실루엣이 들어온다"* 인데, 그 라인아트를 **누가 만드는지**가 지금까지 비어 있었다. ①이 그 자리다.

---

## 2. 확정 사실 (재조사 불필요)

| 사실 | 근거 |
|---|---|
| **Claude API는 이미지를 생성하지 못한다** | 이미지 **입력**(vision)은 되지만 이미지를 **출력**하는 엔드포인트가 없다. 스타일 변환은 OpenAI 등 별도 API가 필요하다 |
| **OpenAI API로 확정** | Mark 결정 (2026-08-03) |
| **09pc에 실증 환경 없음** | `OPENAI_API_KEY` 미설정 · python `openai` 패키지 미설치 · 리포·루트 어디에도 샘플 이미지 없음 |
| **원격 `markch/vision_LLM` 존재** | `b025651` (= `main`과 동일). 동료 병렬 세션이 푸시함. 코드 변경 아직 없음 |
| **기존 vision 스켈레톤이 서 있다** | 08-02 `skeleton_integration` 머지 (PR #4) |

### ⚠️ 조사 미완 — 다른 PC에서 먼저 할 것

**OpenAI 이미지 API의 현행 스펙을 확인하지 못했다.** 09pc에서 웹 조사를 시작했으나 네트워크 사정으로 중단했다. 다음을 **웹 문서(platform.openai.com/docs)로 직접 확인**하고 시작할 것 — 기억이나 오래된 예제를 믿지 말 것:

- 현행 이미지 생성 **model id 문자열** (정확히)
- **image-to-image 엔드포인트** — `/v1/images/edits` 인지 다른 것인지, 마스크가 필수인지 선택인지
- 요청 형식 (multipart/form-data vs JSON), 이미지 입력 방식 (파일 업로드 vs base64)
- 응답 형식 (b64_json vs URL), 지원 해상도·출력 포맷·투명배경 옵션
- **과금 단위** — 장당 고정가인지 토큰 기반인지, 입력 이미지도 과금되는지
- API 키 발급에 선불 크레딧이 필요한지

---

## 3. 현재 상태

- 브랜치 `markch/vision_LLM` = `origin/markch/vision_LLM` = `main` (`b025651`). **코드 변경 0**
- 09pc 세션에서 만든 파일 없음. git 이력 건드리지 않음 (이 문서가 첫 커밋)

### 기존 자산 — 재사용할 것

`src/vision/ws_vision/src/`

| 패키지 | 상태 |
|---|---|
| `vision_core` | ROS 무관 C++ 라이브러리. **인터페이스·빌드·확인 하네스 완성**, 알고리즘은 TODO |
| `vision_node` | rclcpp 래퍼. 경계 인터페이스(토픽/서비스) 미정 |

`vision_core` 구현 상태 (원본: `src/vision/README.md` §구현 상태):

| 단계 | 상태 |
|---|---|
| `Preprocess` (F2.1) | ⬜ TODO — 그레이스케일·이진화·노이즈 제거 |
| `ExtractContours` (F2.2) | ⬜ TODO — **함정**: `findContours`는 폐곡선 경계를 주므로 1픽셀 centerline에 그대로 쓰면 선 양쪽을 돌아 같은 선을 두 번 긋는다. 그래프 기반 선 추적 필요 |
| `ToPolylines` (F2.3) | ⬜ TODO — **함정**: `resample_step_mm`은 mm인데 이 단계 좌표는 픽셀 |
| `ScaleToPaper` (F2.4) | ✅ 구현됨 (letterbox 중앙정렬·여백·closed 판정) |
| 하네스 (SVG·PNG·CSV·CLI) | ✅ 구현됨 |

**`stroke_map_cli --dump-stages`가 이번 작업의 핵심 도구다.** 이진화·윤곽·재샘플링 중간 결과를 PNG로 떨궈줘서, 생성된 라인아트가 실제로 쓸 만한지 **눈으로 판정**할 수 있다.

```bash
cd src/vision
./run_container.sh build          # 최초 1회, 5~15분
./run_container.sh shell
colcon build --symlink-install
source install/setup.bash
ros2 run vision_core stroke_map_cli --in ~/data/cat_lineart.png \
    --out-dir ~/data/out --dump-stages
```

---

## 4. 작업 순서

### ① 라인아트 1장 뽑기 `최우선`

준비물:
- **OpenAI API 키** — `platform.openai.com`에서 발급. 선불 크레딧 필요 여부는 §2 조사 항목
- **테스트용 고양이 사진 1장** — 리포에 없으니 새로 구할 것
- python `openai` 패키지 (또는 `curl`)

스크래치 영역(`/tmp/claude-*/.../scratchpad`)에서 실험한다. **리포에 실험 스크립트를 커밋하지 말 것** — 프롬프트가 확정된 뒤에 정리해서 넣는다.

프롬프트는 "흰 배경 · 검은 단선 · 채색 없음 · 해칭 없음 · 그라데이션 없음"을 강하게 요구해야 한다. 보수적/균형/미니멀 3단계로 만들어 비교할 것.

### ② 생성 결과를 기존 파이프라인에 태우기

①의 PNG를 `stroke_map_cli --dump-stages`에 넣고 단계별 PNG를 본다.

**예상 함정 (반드시 확인할 것):**
- 생성 이미지가 육안으로 라인아트여도 실제로는 **안티앨리어싱된 회색 그라데이션**이라 이진화 임계값에 민감하다
- **이중 윤곽선** — 선을 면으로 그리면 thinning이 선 양쪽에 centerline을 만든다
- **해칭·스티플링** — 스트로크 수가 폭발해서 N2(15분) 예산을 넘긴다
- **닫히지 않은 선** — 폴리라인이 잘게 쪼개진다

### ③ 판단 — 새 스켈레톤이 필요한가 `분기점`

②의 결과를 보고 정한다. **Mark가 이 판단을 위해 ①②를 먼저 하라고 지시했다.**

- 기존 `vision_core`(이진화 → thinning → 폴리라인)로 충분한가?
- 아니면 생성 이미지 전용 전처리(색 양자화·선 병합·노이즈 제거)가 별도로 필요한가?
- 아니면 래스터를 거치지 않고 벡터 추출(potrace/vtracer 계열 centerline trace)로 가야 하는가?

### ④ 스켈레톤 생성 + 배치·언어 확정

③의 결론에 따라 코드를 만든다. **이 두 가지는 ③ 이후에 정한다 — 지금 정하지 말 것 (Mark 지시):**

**배치 후보**
| 안 | 근거 |
|---|---|
| `src/vision/ws_vision/src/vision_stylize/` | 이미지→지도 파이프라인의 앞단이므로 vision 도메인. 기존 Dockerfile·compose·CLI 하네스를 그대로 재사용 |
| `src/stylize/` (최상위 신규 모듈) | 외부 API 의존·네트워크·과금이라는 성격이 vision(순수 이미지처리)과 다르다. 경계를 최상위에서 드러냄 |
| `src/drivers/llm_comm/` | 외부 시스템 연동이라는 점에서 `hcr_comm`·`cam_comm`과 같은 계층 |

**언어 후보** — 프로젝트 규칙 N1은 제품 코드 C++
| 안 | 근거 |
|---|---|
| C++ (libcurl + nlohmann/json) | N1 유지. base64·멀티파트·재시도를 직접 구현해야 함 |
| Python 실험 → C++ 이관 | 프롬프트 튜닝은 반복 실험이 필수라 Python이 빠르다. `src/drivers/hcr_comm/tools/mqtt_cmd.py`가 이미 쓰는 패턴 |
| Python 유지 (이 모듈만 예외) | 네트워크 I/O가 본질이고 실시간 제약이 없다. N1에 명시적 예외를 둔다 |

---

## 5. 주의사항

- 🔑 **API 키를 절대 커밋하지 말 것.** 환경변수로만 다룬다. `.gitignore`에 키 파일 패턴이 아직 **없으니** 키를 파일로 둘 거면 `.gitignore`부터 등록할 것
- `src/vision/data/`는 이미 `.gitignore`에 등록되어 있다 → 입력 사진·생성 결과는 여기에 두면 안전하다 (디렉터리는 아직 실재하지 않으니 만들어야 함)
- **`main`에 직접 push 금지.** 작업 브랜치는 `markch/vision_LLM`
- 원격 상태로 판단·보고하기 전에 **반드시 `git fetch` 먼저** — `git branch -avv`의 `remotes/origin/*`은 마지막 fetch 시점의 캐시다
- 동료가 이 브랜치에 병렬로 작업 중일 수 있다. 커밋 전 `git fetch` + `git log origin/markch/vision_LLM` 확인

---

## 6. 참조

| 무엇 | 원본 |
|---|---|
| vision 인터페이스 계약·구현 상태·CLI 사용법 | `src/vision/README.md` ← **먼저 읽을 것** |
| 컨테이너 세팅 전제조건 | `src/moveit2/README.md` §0 |
| 시스템 구조·데이터 흐름 | `docs/System Architecture.md` §3.1 |
| 요구사항 (F1 이미지 입력, F2 이미지 분석) | `docs/Business Requirements.md` |
| 처리 흐름 | `docs/Process Flow.md` |
| 디렉터리 용어·배치·git 규칙 | 루트 디렉터리의 `CLAUDE.md` (리포 밖) |
| 미결 장부 | 루트 디렉터리의 `업무목록.md` (리포 밖) |

---

## 7. 이 문서 정리

작업이 끝나면:
1. 확정된 설계는 `src/vision/README.md` 또는 새 모듈의 README로 옮긴다
2. 세션 경과는 09pc 루트 디렉터리의 `작업일지.md`에 기록한다
3. **이 파일은 삭제하고 커밋한다** — 리포에 세션 기록을 남기지 않는다는 원칙으로 복귀
