"""설정과 결과물 입출력.

설정값은 코드에 박지 않고 전부 여기를 거친다.
결과물(왜곡·기준·호모그래피)은 사람이 읽고 손으로 고칠 수 있는 YAML 로 남긴다.

설계 근거: cal_script.md §3.1
🚧 부분 구현 (2026-08-16, 세션 `260816-인서트너트`)
   ✅ 설정 계층 3개 — 블록 A 인쇄물(cal_board.py)이 실제로 돌아가는 데 필요한 만큼
   🚧 결과물·원자료 6개 — 블록 B~E 가 미구현이라 스키마를 지금 정하면 추측이 섞인다
"""

from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any

import yaml


# 설정 파일 기본 경로 — 이 파일 옆의 cal_config.yaml
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "cal_config.yaml"


# ── 설정 ────────────────────────────────────────────────

def default_config() -> dict[str, Any]:
    """설정 기본값. 파일이 없을 때의 출발점이자 키 목록의 정본.

    🔴 여기 적힌 치수는 **설계 명목값이지 실측값이 아니다.** 인쇄물을 뽑은 뒤
       자로 재서 `markers.size_mm` 를 실측값으로 갈아끼워야 한다 — 인쇄 배율 오차가
       그대로 H 의 스케일 오차가 된다 (cal_readme.md §2 인쇄 함정).
    """
    return {
        # ── 마커 규격 — 블록 A(인쇄) 와 B·C·D(검출) 가 공유하는 정본 ──
        "markers": {
            # DICT_4X4_50: 테두리 포함 6셀이라 셀이 크다. 45도 경사에서 유효
            # 해상도가 cos45=0.71 배로 깎이는 이 셋업에 가장 유리 (cal_markers.py 머리주석)
            "dictionary": "DICT_4X4_50",
            "size_mm": 50.0,          # 마커 변 길이(검은 테두리 포함)
            "quiet_zone_mm": 10.0,    # 마커 둘레 흰 여백. 최소 1셀(=size/6) 이상
            "reference_ids": [0, 1, 2, 3],   # 기준 마커 — 작업대 영구 부착
            "paper_id_base": 40,             # 종이 지그용 대역 시작 (블록 D, 도입 시)
        },

        # ── 블록 A 인쇄물 ──
        "board": {
            "dpi": 600,               # 600 미만이면 4X4 셀 경계가 인쇄에서 뭉갠다
            "page_mm": [210.0, 297.0],  # A4 세로
            "margin_mm": 12.0,        # 대부분의 가정용 프린터 인쇄 가능 영역 밖 여유
            "columns": 2,             # 기준 마커 4개 → 2×2
            "tile_gap_mm": 10.0,      # 타일 사이 간격 = 가위가 들어갈 폭
            "label_strip_mm": 9.0,    # 타일 하단 이름표 띠 (재단선 안쪽)
            "cut_marks": True,
            "scale_bar_mm": 100.0,    # 인쇄 배율 검증자. 100mm 를 자로 재서 확인한다
            "output_name": "marker_sheet",
        },

        # ── ChArUco — 조건부. 블록 A ④ 정식 intrinsic 으로 갈 때만 ──
        "charuco": {
            # 🔴 기준 마커와 **딕셔너리를 가른다.** ChArUco 보드는 ID 0 부터 채워
            #    쓰므로 같은 딕셔너리면 기준 마커 0~3 과 ID 가 충돌한다.
            #    (cal_markers.py 의 ID 대역 분리 규칙은 같은 딕셔너리 안에서만 유효)
            "dictionary": "DICT_5X5_100",
            "squares_x": 5,
            "squares_y": 7,
            "square_mm": 35.0,        # 보드 실치 175×245mm — A4 여백 17.5·26mm
            "marker_mm": 26.0,
            "output_name": "charuco_sheet",
        },

        # ── 경로 ──
        "paths": {
            # 산출물·원자료 루트. 환경변수 CAL_DATA_DIR 로 덮을 수 있다.
            # 리포 .gitignore 에 등록돼 있다 (2026-08-16) — 여기 아래는 커밋되지 않는다
            "data_dir": os.environ.get(
                "CAL_DATA_DIR", str(Path(__file__).resolve().parent / "data")),
        },
    }


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    """설정 파일을 읽어 기본값 위에 덮는다. path 가 None 이면 기본 경로.

    파일이 없어도 예외가 아니다 — 기본값만으로 블록 A 가 돈다.
    다만 **명시적으로 준 경로가 없으면** 그건 오타일 확률이 높아 예외로 올린다.
    """
    cfg = default_config()

    explicit = path is not None
    p = Path(path) if explicit else DEFAULT_CONFIG_PATH
    if not p.exists():
        if explicit:
            raise FileNotFoundError(f"설정 파일이 없다: {p}")
        validate_config(cfg)
        return cfg

    with p.open("r", encoding="utf-8") as f:
        loaded = yaml.safe_load(f) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"설정 파일 최상위가 매핑이 아니다: {p}")

    cfg = _deep_merge(cfg, loaded)
    cfg["_source"] = str(p)
    validate_config(cfg)
    return cfg


def validate_config(cfg: dict[str, Any]) -> None:
    """필수 키·값 범위 검사. 어긋나면 즉시 예외 — 반쯤 맞는 설정으로 진행하지 않는다.

    특히 인쇄물 쪽은 **틀린 채로 통과하면 종이가 낭비되고, 더 나쁘게는 틀린 치수의
    마커가 작업대에 붙는다.** 그 오차는 H 에 스케일 오차로 박혀 뒤늦게 드러난다.
    """
    for section in ("markers", "board", "charuco", "paths"):
        if not isinstance(cfg.get(section), dict):
            raise ValueError(f"설정에 '{section}' 절이 없다")

    m, b, c = cfg["markers"], cfg["board"], cfg["charuco"]

    # ── 마커 ──
    size = _need_positive(m, "size_mm", "markers")
    quiet = _need_number(m, "quiet_zone_mm", "markers")
    if quiet < 0:
        raise ValueError("markers.quiet_zone_mm 은 음수일 수 없다")

    cells = _dictionary_cells(m["dictionary"])          # 4X4 → 테두리 포함 6
    min_quiet = size / cells
    if quiet < min_quiet:
        raise ValueError(
            f"markers.quiet_zone_mm={quiet} 가 1셀({min_quiet:.2f}mm)보다 작다. "
            "여백이 부족하면 검출률이 급격히 떨어진다")

    ids = m.get("reference_ids")
    if not isinstance(ids, list) or len(ids) < 4:
        raise ValueError("markers.reference_ids 는 4개 이상의 리스트여야 한다 "
                         "(H 를 풀려면 대응점 4쌍 이상)")
    if len(set(ids)) != len(ids):
        raise ValueError(f"markers.reference_ids 에 중복이 있다: {ids}")

    n_marks = _dictionary_size(m["dictionary"])
    for i in ids:
        if not isinstance(i, int) or not 0 <= i < n_marks:
            raise ValueError(f"markers.reference_ids 의 {i} 가 "
                             f"{m['dictionary']}(0~{n_marks - 1}) 범위 밖이다")

    base = m.get("paper_id_base")
    if not isinstance(base, int) or base <= max(ids):
        raise ValueError("markers.paper_id_base 는 reference_ids 최댓값보다 커야 한다 "
                         "— 대역이 겹치면 같은 화면에서 섞인다")

    # ── 인쇄물 ──
    dpi = _need_positive(b, "dpi", "board")
    if dpi < 300:
        raise ValueError(f"board.dpi={dpi} 는 너무 낮다. 4X4 셀 경계가 인쇄에서 뭉갠다")

    page = b.get("page_mm")
    if not (isinstance(page, (list, tuple)) and len(page) == 2
            and all(isinstance(v, (int, float)) and v > 0 for v in page)):
        raise ValueError("board.page_mm 은 [폭, 높이] 양수 2개여야 한다")

    margin = _need_number(b, "margin_mm", "board")
    cols = b.get("columns")
    if not isinstance(cols, int) or cols < 1:
        raise ValueError("board.columns 는 1 이상의 정수여야 한다")
    gap = _need_number(b, "tile_gap_mm", "board")
    strip = _need_number(b, "label_strip_mm", "board")
    for name, v in (("margin_mm", margin), ("tile_gap_mm", gap),
                    ("label_strip_mm", strip)):
        if v < 0:
            raise ValueError(f"board.{name} 은 음수일 수 없다")

    # 타일이 실제로 페이지에 들어가는가 — 여기서 못 잡으면 그리다 잘린다
    tile_w = size + 2 * quiet
    rows = -(-len(ids) // cols)
    need_w = cols * tile_w + (cols - 1) * gap + 2 * margin
    if need_w > page[0]:
        raise ValueError(
            f"타일 {cols}열이 페이지 폭을 넘는다: 필요 {need_w:.1f}mm > {page[0]}mm. "
            "columns 를 줄이거나 margin·quiet_zone 을 줄일 것")
    need_h = rows * (tile_w + strip) + (rows - 1) * gap + 2 * margin
    if need_h > page[1]:
        raise ValueError(
            f"타일 {rows}행이 페이지 높이를 넘는다: 필요 {need_h:.1f}mm > {page[1]}mm "
            "(머리글·스케일 바 제외한 값이다)")

    bar = _need_positive(b, "scale_bar_mm", "board")
    if bar > page[0] - 2 * margin:
        raise ValueError(f"board.scale_bar_mm={bar} 가 인쇄 폭을 넘는다")

    # ── ChArUco ──
    sq = _need_positive(c, "square_mm", "charuco")
    mk = _need_positive(c, "marker_mm", "charuco")
    if mk >= sq:
        raise ValueError(f"charuco.marker_mm({mk}) 는 square_mm({sq}) 보다 작아야 한다")
    sx, sy = c.get("squares_x"), c.get("squares_y")
    if not (isinstance(sx, int) and isinstance(sy, int) and sx > 1 and sy > 1):
        raise ValueError("charuco.squares_x·squares_y 는 2 이상의 정수여야 한다")

    need_marks = (sx * sy) // 2
    have = _dictionary_size(c["dictionary"])
    if need_marks > have:
        raise ValueError(
            f"ChArUco {sx}×{sy} 는 마커 {need_marks}개가 필요한데 "
            f"{c['dictionary']} 에는 {have}개뿐이다")
    if c["dictionary"] == m["dictionary"]:
        raise ValueError(
            "charuco.dictionary 가 markers.dictionary 와 같다. ChArUco 보드는 ID 0 부터 "
            "채워 쓰므로 기준 마커와 충돌한다 — 딕셔너리를 가를 것")

    # ── 경로 ──
    if not cfg["paths"].get("data_dir"):
        raise ValueError("paths.data_dir 이 비어 있다")


# ── 설정 보조 ───────────────────────────────────────────

def _deep_merge(base: dict[str, Any], over: dict[str, Any]) -> dict[str, Any]:
    """중첩 매핑을 절 단위로 덮는다. 절 하나를 통째로 갈아치우지 않는다."""
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def _need_number(section: dict[str, Any], key: str, name: str) -> float:
    v = section.get(key)
    if not isinstance(v, (int, float)) or isinstance(v, bool):
        raise ValueError(f"{name}.{key} 가 숫자가 아니다: {v!r}")
    return float(v)


def _need_positive(section: dict[str, Any], key: str, name: str) -> float:
    v = _need_number(section, key, name)
    if v <= 0:
        raise ValueError(f"{name}.{key} 는 양수여야 한다: {v}")
    return v


def _dictionary_cells(name: str) -> int:
    """딕셔너리 이름 → 마커 한 변의 셀 수(검은 테두리 1셀씩 포함).

    DICT_4X4_50 → 4 + 2 = 6. quiet zone 하한과 렌더 픽셀 정수화의 기준이 된다.
    """
    bits = _dictionary_field(name, 0)
    return bits + 2


def _dictionary_size(name: str) -> int:
    """딕셔너리 이름 → 수록 마커 개수. DICT_4X4_50 → 50."""
    return _dictionary_field(name, 1)


def _dictionary_field(name: str, which: int) -> int:
    """`DICT_4X4_50` 을 (변 비트수, 개수) 로 쪼갠다. which=0 이면 비트수, 1 이면 개수."""
    if not isinstance(name, str):
        raise ValueError(f"딕셔너리 이름이 문자열이 아니다: {name!r}")
    parts = name.split("_")
    # DICT / 4X4 / 50
    if len(parts) != 3 or parts[0] != "DICT":
        raise ValueError(
            f"지원하지 않는 딕셔너리 이름: {name}. DICT_<n>X<n>_<개수> 형식만 쓴다 "
            "(APRILTAG·ARUCO_ORIGINAL 계열은 이 스크립트가 다루지 않는다)")
    try:
        if which == 0:
            return int(parts[1].split("X")[0])
        return int(parts[2])
    except ValueError:
        raise ValueError(f"딕셔너리 이름을 해석하지 못했다: {name}") from None


# ── 결과물 ──────────────────────────────────────────────
# 🚧 블록 B~E 미구현. 결과물 스키마가 확정되기 전에 여기를 채우면 추측이 섞인다.

def result_path(cfg: dict[str, Any], name: str) -> Path:
    """결과물 이름(distortion/reference/homography/paper)을 실제 경로로."""
    raise NotImplementedError


def save_result(cfg: dict[str, Any], name: str, data: dict[str, Any]) -> Path:
    """결과를 YAML 로 저장. 생성 시각·설정 해시를 함께 박아 출처를 남긴다."""
    raise NotImplementedError


def load_result(cfg: dict[str, Any], name: str) -> dict[str, Any]:
    """저장된 결과를 읽는다. 없으면 무엇을 먼저 돌려야 하는지 알려주는 예외."""
    raise NotImplementedError


def result_is_stale(cfg: dict[str, Any], name: str) -> bool:
    """결과가 현재 설정과 어긋나는지(설정 해시 불일치·유효기간 초과) 판정."""
    raise NotImplementedError


# ── 원자료 ──────────────────────────────────────────────

def raw_dir(cfg: dict[str, Any], block: str) -> Path:
    """블록별 원자료 디렉터리. 수집과 계산을 가르는 자리 (cal_script.md §3.6)."""
    raise NotImplementedError


def save_raw(cfg: dict[str, Any], block: str, tag: str, data: Any) -> Path:
    """수집 원자료를 떨군다. 임계가 바뀌어도 재수집 없이 재계산만 하도록."""
    raise NotImplementedError


def load_raw(cfg: dict[str, Any], block: str) -> list[Any]:
    """저장된 원자료 전량을 읽는다."""
    raise NotImplementedError
