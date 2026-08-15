# llm — 사진을 캐리커쳐로 바꾸는 앞단

> 작성 2026-08-16 · 04pc 세션 `260816-원터치피팅`
> 짝 문서 — 설계 의도는 [`llm_script.md`](llm_script.md), 비전 실물 코드는 [`README.md`](README.md)(dy)

---

## 1. 개요

### 한 줄

사용자가 올린 **실물 고양이 사진**을 OpenAI `gpt-image-2` 로 **캐리커쳐 PNG** 로 바꾼다.
그 PNG 를 dy 의 SAM3 노드가 그대로 받아 부위를 딴다.

### 파이프라인에서의 자리

```
실물 고양이 사진 (사용자 업로드)
   │
   ├─ ① llm        (이 모듈 · 신규)   gpt-image-2 /v1/images/edits
   ↓
캐리커쳐 PNG        ← cat_char.png 형식
   │
   ├─ ② sam3_extract_ros_node.py (dy · 무수정)   → /vision_parts  (MaskImage)
   ├─ ③ contour_pixel_node.cpp   (dy · 무수정)   → /vision/strokes (Stroke)
   ↓
   └─ ④ moveit2 (g0ranii)  →  HCR-5  →  A4
```

**동료 코드는 한 줄도 고치지 않는다.** ①의 출력이 ②의 입력 형식을 맞추는 쪽으로 설계했다.

### 왜 이 칸이 필요한가 — 실측 근거

dy 가 올린 산출물을 열어 확인했다 (2026-08-16, 커밋 `0c8dddd`):

| 확인한 것 | 값 |
|---|---|
| `sam3_output/metadata.json` 의 `image_path` | `/home/dorong/cat_char.png` |
| 이미지 크기 | 480 × 538 |
| `sam3_extract_ros_node.py:215` 의 입력 | `CURRENT_DIR / "cat_char.png"` 하드코딩 |
| `cat_char.png` 의 정체 | **두꺼운 균일 검은 외곽선 + 단색 채움**의 벡터풍 고양이 일러스트. 실물 사진이 아니다 |

즉 **dy 의 파이프라인은 이미 "캐리커쳐를 받아 부위를 딴다"로 만들어져 있다.**
비어 있는 것은 *그 캐리커쳐를 사용자 사진에서 만들어내는 칸* 하나다 — dy 는 그림을 손으로
구해다 놓았다. 이 모듈이 그 손을 대신한다.

> 이 사실이 설계를 크게 줄였다. SAM3 가 캐리커쳐 위에서 도는 이상 **좌표계가 하나뿐**이라,
> "사진 좌표의 마스크를 캐리커쳐에 얹는" 정합 문제가 아예 생기지 않는다.
> (검토 과정에서 그 병렬 구조를 후보로 놓았다가, 위 실측으로 폐기했다.)

### 목표 출력의 정의

"라인아트"라는 말은 너무 넓다. **`cat_char.png` 가 기준이다:**

- 두껍고 굵기가 일정한 검은 외곽선
- 단색(또는 매우 단순한) 채움 — 그라데이션·해칭·스티플링 없음
- 밝은 단색 배경
- 얼굴 · 몸 · 다리 · 꼬리 · 눈 · 코 · 입이 **닫힌 선으로 서로 구분**됨
- 원본 고양이의 자세·비율·특징을 알아볼 수 있을 것

마지막 두 줄이 이 모듈의 진짜 제약이다. **예쁜 그림이 아니라 SAM3 가 부위를 나눌 수 있는
그림**이어야 한다.

### 성공 판정

**사람이 눈으로 판정한다** (Rokey6 결정 2026-08-16). 자동 지표를 게이트로 쓰지 않는다.

다만 눈 판정을 돕는 참고 수치가 하나 있다 — 같은 사진을 프롬프트 3종으로 돌린 뒤
②를 태워 **SAM3 confidence** 를 비교한다. `cat_char.png` 의 실측값이 대조군이 된다:

| 부위 | cat_char.png 기준값 |
|---|---|
| cat | 0.9723 |
| eye of cat (좌 / 우) | 0.9077 / 0.9138 |
| nose of cat | 0.8621 |
| mouth of cat | 0.8323 |

> ⚠️ **이 수치는 통과 기준이 아니라 대조군이다.** 낮아도 그림이 좋으면 채택할 수 있고,
> 높아도 그림이 못 쓸 것이면 버린다. 판정 주체는 사람이다.

### 확정 사실 — OpenAI 이미지 API

developers.openai.com 에서 **2026-08-16 직접 확인**했다. 기억이나 예제를 믿지 말 것.

| 항목 | 값 |
|---|---|
| 모델 | `gpt-image-2` (기본 스냅샷 `gpt-image-2-2026-04-21`, 2026-04-21 출시) |
| 엔드포인트 | **POST** `https://api.openai.com/v1/images/edits` |
| 형식 | `multipart/form-data` |
| 필수 | `model` · `image` · `prompt` |
| 선택 | `mask` · `size` · `quality` · `background` · `output_format` · `output_compression` · `n` |
| 마스크 | **선택이다.** 전체를 다시 그리므로 우리는 보내지 않는다 |
| 입력 이미지 | PNG / JPEG / WebP, **50MB 미만** |
| `size` 제약 | `auto` 또는 `<W>x<H>` — 양변이 **16의 배수**, 최대변 ≤ 3840, 종횡비 ≤ 3:1, 총 픽셀 655,360 ~ 8,294,400 |
| `quality` | `low` · `medium` · `high` · `auto` |
| 응답 | **base64** (`b64_json`), 기본 PNG |
| 과금 | 장당 정가 없음 — **토큰 과금**. 이미지 출력 $30/1M · 이미지 입력 $8/1M · 캐시 입력 $2/1M · 텍스트 입력 $5/1M. Batch 는 전부 50% |
| 장당 추정 | 1024×1024 기준 low **$0.006** · medium **$0.053** · high **$0.211** |
| ⚠️ 전제 | **조직 인증(API Organization Verification)이 필요할 수 있다** |

> `input_fidelity` 는 gpt-image-2 에 **해당 없음**으로 문서에 명시돼 있다. 구도 보존을 이
> 파라미터로 제어할 수 없다.

### 확정된 결정 (2026-08-16, Rokey6)

| 항목 | 결정 | 비고 |
|---|---|---|
| 제공자 | OpenAI `gpt-image-2` | Claude API 는 이미지를 **출력**하지 못한다 (입력은 됨) |
| 배치 | `src/vision` 하위 | 이미지 파이프라인의 앞단이므로 vision 도메인 |
| 언어 | **C++** | 프로젝트 규칙 N1 유지 |
| 소비자 | **dy 의 SAM3** (`vision_core` 아님) | `vision_core` 의 thinning 경로는 이 흐름에 들어오지 않는다 |
| 구조 | 직렬 (사진 → 캐리커쳐 → SAM3) | 좌표계가 하나뿐이라 정합 문제가 없다 |
| 작업 브랜치 | `markch/vision_LLM` | `dy/vision` 을 **fast-forward 머지**해 한 트리에 모았다 |
| 성공 판정 | 사람이 눈으로 | 자동 지표는 참고용 |

> **브랜치 메모** — `dy/vision` 은 `markch/vision_LLM` 의 직계 후손이라 병합이 fast-forward 다
> (0 앞 / 4 뒤, 충돌 없음, 실측). 체리픽은 같은 변경을 다른 해시로 복제해 나중 병합을
> 지저분하게 만들므로 **쓰지 않는다.**

### 범위 밖

- SAM3 프롬프트 확장 (`cat`/`eye`/`nose`/`mouth` → 얼굴·몸·다리·꼬리) — **dy 몫**
- `sam3_extract_ros_node.py` 의 입력 경로 인자화 — dy 몫 (지금 하드코딩)
- 컨투어 파라미터(`epsilon_ratio`) · 스트로크 순서 · 궤적 — dy · g0ranii 몫
- 업로드 UI · 작업 관리 — operator 몫

### 미결

- **구도 보존 여부 미실측** — `/v1/images/edits` 가 입력 사진의 구도를 어느 정도 지키는지
  확인하지 않았다. 지금 설계는 이것에 **의존하지 않지만**, 보존이 잘 되면 원본 사진과
  나란히 놓고 비교하기 쉬워진다
- **프롬프트 미확정** — 3종 프리셋을 만들어 두었으나 어느 것도 실사진으로 검증하지 않았다
- **`size` 선택 미확정** — 정사각형이 가장 빠르다고 문서가 말하지만, A4 세로에 그리므로
  세로가 긴 쪽이 여백을 덜 남긴다. 실측 후 정한다
- **API 키·크레딧·조직 인증 미확보** — 아래 §2 참조

---

## 2. 설치

> 이 절은 **아직 실행으로 검증되지 않았다.** 코드가 서면 실측값으로 갱신한다.

### (1) API 키

**키를 리포에 커밋하지 않는다.** 환경변수로만 다룬다. 파일로 두려면 **작업 트리 밖**에
두고 셸이 읽어 넘긴다:

```bash
# 최초 1회 — 리포 밖에 둔다
install -m 600 /dev/null ~/.openai_key
# (에디터로 키를 붙여넣는다. 이 명령을 셸 히스토리에 남기지 말 것)

# 사용할 때
export OPENAI_API_KEY="$(cat ~/.openai_key)"
```

- 명령행 인자로 키를 받는 경로는 **두지 않는다** — `ps` 와 셸 히스토리에 그대로 남는다
- 입력 사진과 생성 결과는 `src/vision/data/` 에 둔다 — 이미 `.gitignore` 에 있다
  (디렉터리는 아직 실재하지 않으니 만들어야 한다)

### (2) 조직 인증 · 크레딧

`platform.openai.com` 에서:
- API 키 발급
- **API Organization Verification** — gpt-image-2 계열은 이것이 필요할 수 있다
- 선불 크레딧 필요 여부 확인

> ⬜ **미확인.** 04pc 에 `OPENAI_API_KEY` 가 설정돼 있지 않고 python `openai` 패키지도 없다
> (2026-08-16 실측).

### (3) 빌드 의존

| 의존 | 왜 | 상태 |
|---|---|---|
| libcurl | HTTPS multipart 전송 | ⬜ vision `Dockerfile` 에 없음 — 추가 필요 |
| nlohmann/json | 응답 JSON 파싱 | ⬜ 없음 — 추가 필요 |

둘 다 apt 로 들어오지만 **이미지 재빌드(5~15분)가 따른다.**

```bash
cd src/vision
./run_container.sh build
./run_container.sh shell
colcon build --symlink-install
source install/setup.bash
```

---

## 3. 실행

> ⬜ **미작성.** 코드가 서고 실제로 한 번 돌린 뒤에 실측 절차로 채운다.
> 아래는 예정 형태이며 아직 동작하지 않는다.

```bash
export OPENAI_API_KEY="$(cat ~/.openai_key)"

# 프롬프트 3종을 같은 사진에 돌려 나란히 본다
ros2 run vision_stylize stylize_cli \
    --in  ~/data/my_cat.jpg \
    --out-dir ~/data/out \
    --preset all \
    --quality low
```

이어서 dy 파이프라인에 태운다 (dy `README.md` §실행 방법 참조):

```bash
# 비전 컨테이너
ros2 run vision_node contour_pixel
# 가상환경에서
python3 sam3_extract_ros_node.py     # ← 입력 경로가 하드코딩이라 지금은 파일을 바꿔 넣어야 한다
```
