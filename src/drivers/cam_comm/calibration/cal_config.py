"""설정과 결과물 입출력.

설정값은 코드에 박지 않고 전부 여기를 거친다.
결과물(왜곡·기준·호모그래피)은 사람이 읽고 손으로 고칠 수 있는 YAML 로 남긴다.

설계 근거: cal_script.md §3.1
✅ 구현 완료
   설정 계층 3개 (2026-08-16, 세션 `260816-인서트너트`)
   결과물·원자료 7개 + `pen`·`thresholds` 절 (2026-08-16, 세션 `260816-브론치노`)
"""

from __future__ import annotations

import copy
import datetime as _dt
import hashlib
import os
from pathlib import Path
from typing import Any

import numpy as np
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

        # ── 펜 — 블록 B 가 평면 z 에서 펜다운/펜업을 만든다 ──
        "pen": {
            # 🔴 스프링 **자유 길이** 기준으로 TCP 가 등록돼 있다. 그래서
            #    "지령면 아래로 파고든 mm = 스프링 압축량 = 필압" 이 그대로 성립한다.
            #    (실기 4점법 실측, 2026-08-16 세션 `260816-알트도르퍼`)
            "delta_mm": 0.5,          # 펜다운 파고듦 δ — 재는 값이 아니라 고르는 값
            "up_mm": 4.0,             # 펜업 들어올림 h
            # 🔴 홀더 스프링이 실제로 눌릴 수 있는 전체 행정. 여유의 상한이다.
            #    δ + 작업평면 오차 가 이 값을 넘으면 스프링이 바닥나 펜이 종이를 긁는다.
            "spring_travel_mm": 1.069,
        },

        # ── 판정 임계 — cal_script.md §3.4 N4 ──
        "thresholds": {
            # ✅ 물리에서 확정된 유일한 값. 스프링 행정 1.069mm 에서 역산했다.
            #    선화 품질이 아니라 **펜이 종이에 닿느냐 긁느냐**의 조건이라 협상 대상이 아니다.
            "plane_residual_mm": 0.4,
            # 🔴 **미정 (N4)** — 선화 정밀도 요구에서 역산해야 한다. null 이면 판정을
            #    하지 않고 **값만 보고**한다. 임의의 숫자를 넣어 통과시키는 것보다,
            #    임계가 없다는 사실이 보이는 편이 안전하다.
            "homography_residual_mm": None,
            "drift_warn_mm": None,
            "drift_stop_mm": None,
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
    for section in ("markers", "board", "charuco", "pen", "thresholds", "paths"):
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

    # ── 펜 ──
    p = cfg["pen"]
    delta = _need_positive(p, "delta_mm", "pen")
    _need_positive(p, "up_mm", "pen")
    travel = _need_positive(p, "spring_travel_mm", "pen")

    # 🔴 δ 하나만으로 스프링을 다 써 버리면 평면 오차를 흡수할 여유가 0 이 된다.
    #    아래 plane_residual_mm 과의 합으로 다시 한 번 본다.
    if delta >= travel:
        raise ValueError(
            f"pen.delta_mm={delta} 가 스프링 행정 {travel}mm 이상이다 — "
            "스프링이 바닥나 펜이 종이를 긁는다")

    # ── 임계 ──
    t = cfg["thresholds"]
    plane_th = t.get("plane_residual_mm")
    if plane_th is not None:
        if not isinstance(plane_th, (int, float)) or plane_th <= 0:
            raise ValueError("thresholds.plane_residual_mm 은 양수이거나 null 이어야 한다")
        if delta + plane_th >= travel:
            raise ValueError(
                f"pen.delta_mm({delta}) + thresholds.plane_residual_mm({plane_th}) "
                f"= {delta + plane_th}mm 가 스프링 행정 {travel}mm 이상이다. "
                "평면이 임계까지 어긋난 지점에서 스프링이 바닥난다 — "
                "δ 를 줄이거나 평면 임계를 조이거나 홀더를 바꿀 것")

    for key in ("homography_residual_mm", "drift_warn_mm", "drift_stop_mm"):
        v = t.get(key, "__missing__")
        if v == "__missing__":
            raise ValueError(f"thresholds.{key} 가 없다 (미정이면 null 로 명시할 것)")
        if v is not None and (not isinstance(v, (int, float)) or v <= 0):
            raise ValueError(f"thresholds.{key} 는 양수이거나 null 이어야 한다")

    warn, stop = t.get("drift_warn_mm"), t.get("drift_stop_mm")
    if warn is not None and stop is not None and warn > stop:
        raise ValueError(f"drift_warn_mm({warn}) 이 drift_stop_mm({stop}) 보다 크다 — "
                         "경고가 정지보다 늦게 울린다")

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
# 사람이 읽고 손으로 고칠 수 있는 YAML 로 남긴다. 블록 B~E 가 이걸 주고받는다.
#
# 🔴 **numpy 를 그대로 담그지 않는다.** yaml.safe_dump 는 ndarray·np.float64 를
#    !!python/object 태그로 뱉거나 아예 실패한다. 저장 직전에 순수 파이썬으로 내리고
#    (`_plain`), 읽을 때는 **리스트 그대로** 돌려준다 — 자동으로 ndarray 로 되돌리면
#    "어떤 키가 배열인가"를 이 파일이 알아야 해서 결과물 스키마와 결합된다.
#    배열이 필요한 쪽에서 np.asarray 하면 된다.

# 결과물 이름과 그것을 만드는 명령. 없을 때 무엇을 먼저 돌리라고 할지의 근거다.
_RESULT_OWNER = {
    "distortion": "cal.py distortion",
    "reference": "cal.py reference",
    "homography": "cal.py update",
    "paper": "cal.py paper",
}


def _plain(obj: Any) -> Any:
    """numpy·Path 를 YAML 이 아는 순수 타입으로 내린다. 중첩 구조를 그대로 훑는다."""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, np.generic):          # np.float64 등 스칼라
        return obj.item()
    if isinstance(obj, Path):
        return str(obj)
    if isinstance(obj, dict):
        return {(_plain(k) if not isinstance(k, str) else k): _plain(v)
                for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_plain(v) for v in obj]
    return obj


def config_fingerprint(cfg: dict[str, Any]) -> str:
    """결과물에 박을 설정 지문.

    🔴 **결과에 영향을 주는 절만 넣는다.** `paths` 는 결과값을 바꾸지 않으므로 뺀다 —
       데이터 디렉터리를 옮겼다는 이유로 멀쩡한 캘리브레이션이 무효가 되면,
       사람은 그 경고를 무시하는 법부터 배운다.
    """
    material = {k: cfg[k] for k in ("markers", "board", "charuco", "pen")
                if k in cfg}
    blob = yaml.safe_dump(_plain(material), sort_keys=True, allow_unicode=True)
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:16]


def result_path(cfg: dict[str, Any], name: str) -> Path:
    """결과물 이름(distortion/reference/homography/paper)을 실제 경로로."""
    if name not in _RESULT_OWNER:
        raise ValueError(f"모르는 결과물 이름이다: {name!r} "
                         f"(아는 것: {sorted(_RESULT_OWNER)})")
    return Path(cfg["paths"]["data_dir"]) / "results" / f"{name}.yaml"


def save_result(cfg: dict[str, Any], name: str, data: dict[str, Any]) -> Path:
    """결과를 YAML 로 저장. 생성 시각·설정 해시를 함께 박아 출처를 남긴다.

    🔴 **임시 파일에 쓰고 마지막에 바꿔 끼운다.** 저장 도중에 죽으면 반쯤 쓰인 YAML 이
       남는데, 그것이 다음 실행에서 "있긴 있는 결과"로 읽혀 조용히 틀린다.
    """
    path = result_path(cfg, name)
    path.parent.mkdir(parents=True, exist_ok=True)

    payload = {
        "_meta": {
            "name": name,
            "saved_at": _dt.datetime.now().astimezone().isoformat(timespec="seconds"),
            "config_fingerprint": config_fingerprint(cfg),
            "config_source": cfg.get("_source", "(기본값)"),
        },
        **_plain(data),
    }

    tmp = path.with_suffix(".yaml.tmp")
    with tmp.open("w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True,
                       default_flow_style=False)
    tmp.replace(path)
    return path


def load_result(cfg: dict[str, Any], name: str) -> dict[str, Any]:
    """저장된 결과를 읽는다. 없으면 무엇을 먼저 돌려야 하는지 알려주는 예외."""
    path = result_path(cfg, name)
    if not path.exists():
        raise FileNotFoundError(
            f"{name} 결과가 없다: {path}\n"
            f"  → 먼저 `{_RESULT_OWNER[name]}` 를 돌릴 것")

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path} 의 최상위가 매핑이 아니다 — 파일이 깨졌다")
    return data


def result_is_stale(cfg: dict[str, Any], name: str) -> bool:
    """결과가 현재 설정과 어긋나는지(설정 해시 불일치) 판정.

    없는 결과는 stale 이 아니라 **없는 것**이다 — 여기서 True 를 주면 호출부가
    "낡았으니 다시 돌려라"라고 말하는데, 사실은 한 번도 안 돌린 것이라 메시지가 틀린다.
    """
    path = result_path(cfg, name)
    if not path.exists():
        return False

    try:
        meta = load_result(cfg, name).get("_meta", {})
    except (ValueError, yaml.YAMLError):
        return True                       # 못 읽는 결과는 낡은 것으로 친다
    return meta.get("config_fingerprint") != config_fingerprint(cfg)


# ── 원자료 ──────────────────────────────────────────────
# 🔴 수집과 계산을 가른다 (cal_script.md §3.5). 임계(N4)가 아직 미정이라, 임계를 바꿀
#    때마다 로봇·카메라를 다시 돌려야 한다면 임계를 못 고친다. 원자료를 남겨 두면
#    `--recompute` 로 계산만 다시 돈다.

def raw_dir(cfg: dict[str, Any], block: str) -> Path:
    """블록별 원자료 디렉터리. 수집과 계산을 가르는 자리 (cal_script.md §3.5)."""
    b = str(block).upper()
    if b not in ("A", "B", "C", "D", "E"):
        raise ValueError(f"블록은 A~E 다: {block!r}")
    return Path(cfg["paths"]["data_dir"]) / "raw" / b


def save_raw(cfg: dict[str, Any], block: str, tag: str, data: Any) -> Path:
    """수집 원자료를 떨군다. 임계가 바뀌어도 재수집 없이 재계산만 하도록.

    🔴 **덮어쓰지 않는다.** 파일명에 수집 시각을 박는다 — 같은 tag 로 두 번 수집하는
       일은 흔하고(짚다 실수해서 다시 짚는다), 그때 앞의 것이 사라지면 무엇이 튀었는지
       나중에 대조할 수 없다. 원자료는 지우지 않는 것이 이 계층의 존재 이유다.
    """
    d = raw_dir(cfg, block)
    d.mkdir(parents=True, exist_ok=True)

    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    safe_tag = "".join(c if c.isalnum() or c in "-_." else "_" for c in str(tag))
    path = d / f"{stamp}__{safe_tag}.yaml"

    payload = {
        "_meta": {
            "block": str(block).upper(),
            "tag": str(tag),
            "saved_at": _dt.datetime.now().astimezone().isoformat(timespec="seconds"),
            "config_fingerprint": config_fingerprint(cfg),
        },
        "data": _plain(data),
    }
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True,
                       default_flow_style=False)
    return path


def load_raw(cfg: dict[str, Any], block: str) -> list[Any]:
    """저장된 원자료 전량을 **수집 시각 순**으로 읽는다. 없으면 빈 리스트."""
    d = raw_dir(cfg, block)
    if not d.is_dir():
        return []

    out = []
    for path in sorted(d.glob("*.yaml")):          # 파일명이 시각으로 시작한다
        with path.open("r", encoding="utf-8") as f:
            item = yaml.safe_load(f)
        if isinstance(item, dict):
            item.setdefault("_meta", {})["path"] = str(path)
            out.append(item)
    return out
