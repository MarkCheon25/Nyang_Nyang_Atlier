"""인쇄물 생성 — 블록 A.

카메라도 로봇도 없이 도는 유일한 블록이다. 인쇄해 두면 카메라가 오는 즉시 블록 B 로 들어간다.

기본 산출물 = **기준 마커 시트**. A4 한 장에 마커 4개(ID 0~3) + 재단선.
잘라서 작업대에 붙인다 — 종이 자리를 둘러싸도록 네 귀퉁이에.

  ┌─────────────────────────────┐
  │  [M0]                 [M1]  │   마커는 종이 바깥
  │      ┌───────────────┐      │   → 종이가 절대 안 가림
  │      │   A4 종이     │      │   → 종이 영역이 내삽 구간에 들어가 정확도가 가장 좋다
  │      └───────────────┘      │
  │  [M2]                 [M3]  │
  └─────────────────────────────┘

🔴 인쇄 함정 — 프린터의 "페이지에 맞춤"이 켜져 있으면 실제 치수가 몇 % 달라지고,
   그 오차가 H 에 스케일 오차로 그대로 박힌다. **100% 배율로 인쇄하고, 인쇄물을 자로 재서
   설정에 실측값을 넣을 것.** 시트에 검증용 자(스케일 바)를 함께 찍는다.
🔴 작업면에 평평하게 붙일 것. 들뜨면 그 오차가 그대로 들어간다.

🔴 **렌더 치수는 명목 치수와 정확히 같지 않다.** 마커 한 변은 셀 수(4X4 → 테두리 포함 6)의
   배수 픽셀이어야 셀 경계가 정수로 떨어지고, 그래서 목표 픽셀을 셀 수 배수로 **반올림**한다.
   600dpi·50mm 면 1181.10px → 1182px = **50.0380mm** (+0.076%).
   이 값은 추정이 아니라 확정값이라 **시트 머리글·이름표·로그에 실제 렌더 치수를 찍는다.**
   설정에 넣을 것은 그래도 **인쇄물 실측값**이다 — 렌더 오차 위에 프린터 배율 오차가 또 얹힌다.

설계 근거: cal_script.md §3.4 · cal_readme.md §1 블록 A
✅ 구현 완료 (2026-08-16, 세션 `260816-인서트너트`) — 카메라·로봇 없이 검증됨
"""

from __future__ import annotations

import datetime as _dt
from pathlib import Path
from typing import Any

import cv2
import numpy as np

try:                                      # 스크립트로도, 패키지로도 부를 수 있게
    from . import cal_markers
except ImportError:                       # pragma: no cover
    import cal_markers                    # type: ignore[no-redef]


# 이 모듈이 다루는 이미지는 전부 **이진 그레이스케일**(uint8, 0=검정 / 255=흰) 이다.
# 반색조를 섞지 않는 이유 — PDF 를 1비트로 저장해 용량을 100배 줄이고, 무엇보다
# 마커·재단선·눈금이 인쇄에서 뭉개지지 않는다. 그래서 글자도 안티앨리어싱 없이(LINE_8) 찍는다.
#
# 🔴 **LINE_8 만으로는 안 된다.** cv2 5.0 의 putText 는 LINE_8 을 무시하고 안티앨리어싱
#    하므로 _text() 가 그린 자리를 되이진화한다. 4.6/5.0 실측으로 확인했고,
#    rectangle·fillPoly 는 양쪽 다 이진이라 그대로 둔다.
_BLACK, _WHITE = 0, 255
_FONT = cv2.FONT_HERSHEY_SIMPLEX


# ── mm ↔ px ─────────────────────────────────────────────

def _px(mm: float, dpi: float) -> int:
    """mm → 픽셀(반올림). 이 스크립트의 길이 단위는 처음부터 끝까지 mm 다."""
    return int(round(mm / 25.4 * dpi))


def _mm(px: float, dpi: float) -> float:
    """픽셀 → mm. 렌더 실치수를 되돌려 보고할 때 쓴다."""
    return px / dpi * 25.4


def _blank(w_mm: float, h_mm: float, dpi: float) -> np.ndarray:
    """흰 캔버스."""
    return np.full((_px(h_mm, dpi), _px(w_mm, dpi)), _WHITE, dtype=np.uint8)


def _paste(canvas: np.ndarray, img: np.ndarray, x_mm: float, y_mm: float,
           dpi: float) -> None:
    """캔버스 위 (x, y) mm 지점에 좌상단을 맞춰 붙인다. 넘치면 조용히 자르지 않고 예외."""
    x, y = _px(x_mm, dpi), _px(y_mm, dpi)
    h, w = img.shape[:2]
    if y < 0 or x < 0 or y + h > canvas.shape[0] or x + w > canvas.shape[1]:
        raise ValueError(
            f"배치가 캔버스를 벗어난다: ({x_mm:.1f}, {y_mm:.1f})mm 에 "
            f"{_mm(w, dpi):.1f}×{_mm(h, dpi):.1f}mm — 캔버스 "
            f"{_mm(canvas.shape[1], dpi):.1f}×{_mm(canvas.shape[0], dpi):.1f}mm")
    canvas[y:y + h, x:x + w] = img


#: cv2 의 Hershey 폰트는 **ASCII 32~126 만** 그린다. 한글은 물론이고 em dash 같은
#: 문장부호도 물음표로 찍힌다 — 그것도 조용히. 인쇄물의 경고문이 "???" 로 나가는 사고를
#: 막으려고, 자주 쓰는 것들을 ASCII 로 미리 눕히고 나머지는 '?' 로 바꾼다.
#: (한국어가 필요하면 cv2 가 아니라 PIL + 폰트 파일로 가야 한다 — 시스템 폰트 의존이 생긴다)
_ASCII_MAP = str.maketrans({
    "—": "-", "–": "-", "−": "-", "·": ".", "×": "x",
    "“": '"', "”": '"', "‘": "'", "’": "'", "…": "...",
    "→": "->", "±": "+/-", "℃": "degC", "㎜": "mm", " ": " ",
})


def _ascii(s: str) -> str:
    """Hershey 로 찍을 수 있는 문자만 남긴다."""
    return "".join(c if 32 <= ord(c) < 127 else "?"
                   for c in s.translate(_ASCII_MAP))


def _text_metrics(s: str, h_mm: float, dpi: float) -> tuple[float, int, float]:
    """글자 높이 h_mm 로 s 를 찍을 때의 (폭 mm, 두께 px, cv2 스케일)."""
    s = _ascii(s)
    thick = max(1, _px(h_mm * 0.11, dpi))
    (_, base_h), _ = cv2.getTextSize(s, _FONT, 1.0, thick)
    scale = _px(h_mm, dpi) / max(base_h, 1)
    (w, _), _ = cv2.getTextSize(s, _FONT, scale, thick)
    return _mm(w, dpi), thick, scale


def _text(img: np.ndarray, s: str, x_mm: float, y_mm: float, h_mm: float,
          dpi: float, max_w_mm: float | None = None) -> float:
    """(x, y)mm 에 글자 높이 h_mm 로 텍스트. y 는 **베이스라인**. 반환은 실제 폭(mm).

    max_w_mm 을 주면 넘칠 때 자동으로 줄여 넣는다 — 주의문이 페이지 밖으로
    잘려 나가면 그 경고를 아무도 못 읽는다.
    """
    s = _ascii(s)
    w_mm, thick, scale = _text_metrics(s, h_mm, dpi)

    if max_w_mm is not None and w_mm > max_w_mm:
        shrink = max_w_mm / w_mm
        scale *= shrink
        thick = max(1, int(thick * shrink))
        (w, _), _ = cv2.getTextSize(s, _FONT, scale, thick)
        w_mm = _mm(w, dpi)

    x0, y0 = _px(x_mm, dpi), _px(y_mm, dpi)
    cv2.putText(img, s, (x0, y0), _FONT, scale, _BLACK, thick, cv2.LINE_8)

    # 🔴 **cv2 5.0 의 putText 는 LINE_8 을 줘도 안티앨리어싱을 한다** (4.6 은 안 했다).
    #    rectangle·fillPoly 는 양쪽 다 이진이고 putText 만 그렇다 — 실측으로 갈랐다.
    #    반색조가 섞이면 1비트 PDF 에서 획이 뭉개지므로 그린 자리만 다시 이진으로 못 박는다.
    (tw, th), base = cv2.getTextSize(s, _FONT, scale, thick)
    pad = thick + 2
    ya, yb = max(0, y0 - th - pad), min(img.shape[0], y0 + base + pad)
    xa, xb = max(0, x0 - pad), min(img.shape[1], x0 + tw + pad)
    if yb > ya and xb > xa:
        roi = img[ya:yb, xa:xb]
        np.copyto(roi, np.where(roi >= 128, _WHITE, _BLACK).astype(np.uint8))
    return w_mm


# ── 마커 낱장 ───────────────────────────────────────────

def render_marker(dictionary: Any, marker_id: int, size_mm: float,
                  dpi: int) -> np.ndarray:
    """마커 하나를 실치수(mm)대로 래스터화. 기본 50mm.

    🔴 목표 픽셀을 **셀 수의 배수로 반올림**한다. 안 그러면 OpenCV 가 셀을 불균등하게
       나눠(어떤 셀 197px, 어떤 셀 196px) 코너 위치가 최대 0.5px 어긋난다.
       대신 실치수가 목표에서 최대 반 셀만큼 벗어나므로, 그 실제 값을 호출부가
       `rendered_marker_mm()` 로 되물어 이름표·머리글에 찍는다.
    """
    if size_mm <= 0 or dpi <= 0:
        raise ValueError(f"size_mm·dpi 는 양수여야 한다: {size_mm}, {dpi}")

    cells = int(getattr(dictionary, "markerSize", 4)) + 2   # 검은 테두리 1셀씩
    n = dictionary.bytesList.shape[0]
    if not 0 <= marker_id < n:
        raise ValueError(f"마커 ID {marker_id} 가 딕셔너리 범위(0~{n - 1}) 밖이다")

    side = max(cells, int(round(_px(size_mm, dpi) / cells)) * cells)

    draw = getattr(cv2.aruco, "generateImageMarker", None)   # 4.7+
    if draw is not None:
        img = draw(dictionary, marker_id, side)
    else:                                                    # 4.6 이하
        img = cv2.aruco.drawMarker(dictionary, marker_id, side)
    return np.ascontiguousarray(img, dtype=np.uint8)


def rendered_marker_mm(dictionary: Any, size_mm: float, dpi: int) -> float:
    """render_marker 가 실제로 뽑는 한 변의 mm. 셀 배수 반올림이 반영된 값이다.

    시트에 찍히는 치수·이름표·로그가 전부 이 값을 쓴다 — 명목값을 찍으면
    사람이 그걸 믿고 설정에 넣는다.
    """
    cells = int(getattr(dictionary, "markerSize", 4)) + 2
    side = max(cells, int(round(_px(size_mm, dpi) / cells)) * cells)
    return _mm(side, dpi)


def add_quiet_zone(img: np.ndarray, margin_mm: float, dpi: int) -> np.ndarray:
    """마커 둘레 흰 여백. 없으면 검출이 급격히 나빠진다.

    aruco 는 마커 바깥의 흰 띠로 테두리를 찾는다. 권장 하한은 **1셀**
    (50mm·4X4 면 8.33mm) — 그 검사는 cal_config.validate_config 가 한다.
    """
    if margin_mm < 0:
        raise ValueError(f"margin_mm 은 음수일 수 없다: {margin_mm}")
    m = _px(margin_mm, dpi)
    return cv2.copyMakeBorder(img, m, m, m, m, cv2.BORDER_CONSTANT, value=_WHITE)


def add_label(img: np.ndarray, text: str, cfg: dict[str, Any]) -> np.ndarray:
    """ID·치수·생성일 각인. 인쇄물과 설정이 어긋나는 사고를 막는 이름표.

    이름표는 **타일 아래**에 붙는다. 그게 부착 방향의 기준이기도 하다 —
    붙일 때 이름표가 아래로 오면 마커가 인쇄된 방향 그대로 선다.
    """
    dpi = int(cfg["board"]["dpi"])
    strip_mm = float(cfg["board"]["label_strip_mm"])
    if strip_mm <= 0:
        return img

    strip = np.full((_px(strip_mm, dpi), img.shape[1]), _WHITE, dtype=np.uint8)
    _text(strip, text, 1.0, strip_mm * 0.72, strip_mm * 0.48, dpi,
          max_w_mm=_mm(img.shape[1], dpi) - 2.0)
    return np.vstack([img, strip])


def add_cut_marks(img: np.ndarray, cfg: dict[str, Any]) -> np.ndarray:
    """재단선. 잘라 붙일 때 마커 방향이 헷갈리지 않게 기준변을 표시한다.

    기준변 표시는 **상단 재단선 위에 걸쳐** 그린다. 선 중심에 놓아 자를 때 절반이
    잘려 나가고 절반이 조각에 남는다 — quiet zone 을 파고드는 깊이가 표시 높이의
    절반이라 검출 여백을 해치지 않는다.
    """
    dpi = int(cfg["board"]["dpi"])
    if not cfg["board"].get("cut_marks", True):
        return img

    out = img.copy()
    h, w = out.shape[:2]
    lw = max(1, _px(0.15, dpi))                     # 재단선 굵기 0.15mm

    cv2.rectangle(out, (lw // 2, lw // 2), (w - 1 - lw // 2, h - 1 - lw // 2),
                  _BLACK, lw, cv2.LINE_8)

    # 상단 = 기준변. 삼각형 3개를 선 위에 얹는다
    tri_w, tri_h = _px(2.4, dpi), _px(1.5, dpi)
    for frac in (0.30, 0.50, 0.70):
        cx = int(w * frac)
        pts = np.array([[cx - tri_w // 2, tri_h // 2],
                        [cx + tri_w // 2, tri_h // 2],
                        [cx, tri_h // 2 + tri_h]], dtype=np.int32)
        cv2.fillPoly(out, [pts], _BLACK, cv2.LINE_8)
    return out


def add_scale_bar(img: np.ndarray, length_mm: float, dpi: int,
                  label_mm: float | None = None) -> np.ndarray:
    """인쇄 배율 검증용 자. 인쇄 후 이걸 재서 100% 인지 확인한다.

    10mm 마다 검정/흰이 교대하는 띠라 자를 대고 읽기 쉽다.
    **이 시트에서 유일하게 "재어서 틀렸는지 알 수 있는" 물건이다** — 마커는 재기 번거롭지만
    이 자가 100.0mm 면 마커도 같은 배율로 나온 것이다.

    `label_mm` — 눈금에 **적을** 값. 기본은 length_mm(그린 길이)와 같다.
    프린터 배율 보정을 걸면 그린 길이와 인쇄 후 길이가 달라지므로, 눈금에는
    **인쇄 후 값**을 적는다. 사람이 자를 대는 것은 종이 위이지 화면이 아니다.
    """
    label_mm = float(length_mm if label_mm is None else label_mm)
    if length_mm <= 0:
        raise ValueError(f"length_mm 은 양수여야 한다: {length_mm}")

    out = img.copy()
    h, w = out.shape[:2]
    bar_w = _px(length_mm, dpi)
    if bar_w > w:
        raise ValueError(
            f"스케일 바 {length_mm}mm 가 대상 폭 {_mm(w, dpi):.1f}mm 를 넘는다")

    x0 = (w - bar_w) // 2
    bar_h = _px(5.0, dpi)
    y0 = _px(1.0, dpi)

    # 🔴 빈 블록의 테두리는 **안쪽으로** 그린다. cv2.rectangle 의 두께는 선 중심
    #    기준이라 그냥 그리면 바깥으로 절반(0.075mm)씩 삐져나가고, 자로 잰 전체 길이가
    #    100.15mm 가 된다 — 배율을 재라고 만든 자가 스스로 0.15% 틀리면 안 된다.
    step_mm = 10.0
    lw = max(1, _px(0.15, dpi))
    n = int(round(length_mm / step_mm))
    for i in range(n):
        xa = x0 + _px(i * step_mm, dpi)
        xb = x0 + _px((i + 1) * step_mm, dpi)
        if i % 2 == 0:
            cv2.rectangle(out, (xa, y0), (xb, y0 + bar_h), _BLACK, -1, cv2.LINE_8)
        else:
            h = lw // 2
            cv2.rectangle(out, (xa + h, y0 + h), (xb - h, y0 + bar_h - h),
                          _BLACK, lw, cv2.LINE_8)

    # 눈금 숫자 — 양 끝과 가운데만. 촘촘하면 오히려 못 읽는다.
    # 🔴 자리는 **그린 길이**로 잡고 글자는 **인쇄 후 값**으로 찍는다. 둘을 같은 변수로
    #    쓰면 보정이 걸린 순간 눈금이 바 밖으로 밀려 나간다.
    ty = y0 + bar_h + _px(3.4, dpi)
    for frac in (0.0, 0.5, 1.0):
        s = f"{label_mm * frac:g}"
        wmm, _, _ = _text_metrics(s, 2.8, dpi)          # 가운데 맞추려면 폭이 먼저 필요
        _text(out, s, _mm(x0, dpi) + length_mm * frac - wmm / 2,
              _mm(ty, dpi), 2.8, dpi)
    _text(out, "mm", _mm(x0 + bar_w, dpi) + 5.0, _mm(ty, dpi), 2.8, dpi)

    if abs(label_mm - length_mm) < 1e-9:
        cap = (f"PRINT AT 100% - this bar must measure exactly {label_mm:g} mm. "
               "If not, do NOT use this sheet.")
    else:
        cap = (f"SCALE-COMPENSATED SHEET - after printing this bar must measure "
               f"{label_mm:g} mm. If it does not, re-run cal.py board "
               f"--measured-bar <what you measured>.")
    _text(out, cap, 0.0, _mm(ty, dpi) + 4.6, 2.8, dpi, max_w_mm=_mm(w, dpi))
    return out


# ── 시트 ────────────────────────────────────────────────

def build_marker_sheet(cfg: dict[str, Any]) -> np.ndarray:
    """A4 한 장에 기준 마커 4개 + 재단선 + 스케일 바를 앉힌다.

    🔴 **프린터 배율 보정** (`board.print_scale`) — 1.0 이 아니면 렌더 치수를 1/scale 로
       키워서 인쇄 후 제 치수가 되게 한다. 종이 위 결과가 정본이므로,
       **이름표·스케일 바 눈금에는 "인쇄 후 예상 치수"를 찍는다.** 렌더 치수를 찍으면
       사람이 그 값을 설정에 넣는데, 그건 종이 위 어디에도 없는 숫자다.
    """
    m, b = cfg["markers"], cfg["board"]
    dpi = int(b["dpi"])
    page_w, page_h = (float(v) for v in b["page_mm"])
    margin = float(b["margin_mm"])
    cols = int(b["columns"])
    ids = list(m["reference_ids"])

    # ── 보정: 물리 치수는 전부 1/scale 로 키운다 (페이지·여백은 그대로) ──
    # 페이지를 안 키우는 이유 — 프린터는 **페이지째** 축소하므로, 페이지 안에서
    # 내용만 키우면 축소 후 제 치수가 된다. 페이지까지 키우면 축소가 한 번 더 걸린다.
    scale = float(b.get("print_scale", 1.0))
    want_size = float(m["size_mm"])            # 종이 위에서 원하는 마커 치수
    want_bar = float(b["scale_bar_mm"])        # 종이 위에서 원하는 눈금 길이

    size_mm = want_size / scale                # 실제로 그릴 치수
    quiet = float(m["quiet_zone_mm"]) / scale
    gap = float(b["tile_gap_mm"]) / scale

    dictionary = cal_markers.make_dictionary(m["dictionary"])
    rendered_mm = rendered_marker_mm(dictionary, size_mm, dpi)
    # 사람이 자로 잴 값 = 그린 치수 × 프린터 배율
    actual_mm = rendered_mm * scale
    today = _dt.date.today().isoformat()

    # ── 타일 조립 ──
    tiles = []
    for mid in ids:
        t = render_marker(dictionary, mid, size_mm, dpi)
        t = add_quiet_zone(t, quiet, dpi)
        t = add_label(t, f"M{mid}  {actual_mm:.3f}mm  {m['dictionary']}  "
                         f"{dpi}dpi  {today}", cfg)
        t = add_cut_marks(t, cfg)
        tiles.append(t)

    tile_w_mm = _mm(tiles[0].shape[1], dpi)
    tile_h_mm = _mm(tiles[0].shape[0], dpi)
    rows = -(-len(ids) // cols)
    grid_w = cols * tile_w_mm + (cols - 1) * gap
    grid_h = rows * tile_h_mm + (rows - 1) * gap

    canvas = _blank(page_w, page_h, dpi)
    text_w = page_w - 2 * margin

    # ── 머리글 ──
    y = margin + 4.0
    _text(canvas, "HCR-5 / Nyang_Nyang_Atlier  -  reference marker sheet (block A)",
          margin, y, 4.0, dpi, max_w_mm=text_w)
    y += 5.4
    _text(canvas, f"{m['dictionary']}   ids {', '.join(str(i) for i in ids)}   "
                  f"target {want_size:.3f} mm  ->  expected on paper "
                  f"{actual_mm:.3f} mm   @ {dpi} dpi   "
                  f"quiet zone {quiet * scale:.1f} mm",
          margin, y, 3.0, dpi, max_w_mm=text_w)
    y += 4.2
    if abs(scale - 1.0) > 1e-9:
        _text(canvas,
              f"*** PRINT SCALE COMPENSATED x{1 / scale:.4f} "
              f"(printer measured at {scale * 100:.1f}%) - drawn {rendered_mm:.3f} mm "
              f"so it lands at {actual_mm:.3f} mm. THIS SHEET IS FOR THAT PRINTER ONLY.",
              margin, y, 3.0, dpi, max_w_mm=text_w)
        y += 4.2
    _text(canvas, f"generated {today}   cal.py board --kind markers", margin, y,
          3.0, dpi, max_w_mm=text_w)

    # ── 타일 격자 ──
    y += 6.0
    gx0 = (page_w - grid_w) / 2
    for i, t in enumerate(tiles):
        r, c = divmod(i, cols)
        _paste(canvas, t, gx0 + c * (tile_w_mm + gap),
               y + r * (tile_h_mm + gap), dpi)
    y += grid_h + 8.0

    # ── 스케일 바 ──
    # 눈금 글자는 want_bar(=100)로 찍히고 실제 길이는 want_bar/scale 로 그려진다.
    # 보정이 맞았다면 인쇄물에서 자로 재어 정확히 want_bar 가 나온다 — 그게 검증 신호다.
    bar_mm = want_bar / scale
    strip = np.full((_px(16.0, dpi), _px(text_w, dpi)), _WHITE, dtype=np.uint8)
    strip = add_scale_bar(strip, bar_mm, dpi, label_mm=want_bar)
    _paste(canvas, strip, margin, y, dpi)
    y += 16.0 + 5.0

    # ── 주의문 ──
    first = ("1. Print at 100% scale. Turn OFF 'fit to page' / "
             "'shrink oversized pages'.") if abs(scale - 1.0) < 1e-9 else (
        f"1. Print with the SAME settings used when you measured {scale * 100:.1f}%. "
        "Do NOT 'fix' the printer now - the compensation assumes it stays wrong.")
    notes = [
        first,
        "2. Verify the scale bar with a ruler BEFORE cutting. A wrong print scale "
        "becomes a scale error inside H.",
        "3. Cut along the outer lines. Keep the LABEL STRIP AT THE BOTTOM - "
        "the triangles mark the top edge.",
        "4. Stick FLAT on the work surface. A lifted marker puts its error straight "
        "into H.",
        "5. Layout on the table: M0 M1 on top, M2 M3 below, surrounding the paper "
        "area so the paper never occludes them.",
        f"6. Measure a printed marker edge and put the measured value into "
        f"markers.size_mm (expected on paper: {actual_mm:.3f} mm). "
        "The measured value wins - it is the only one that exists on paper.",
    ]
    # 🔴 줄마다 max_w_mm 에 걸리는 정도가 달라 크기가 제각각이 되면, 자동 축소를
    #    안 당한 줄만 혼자 굵어 보인다. 가장 긴 줄에 맞춘 **공통 높이**로 통일한다
    note_h = 2.8
    widest = max(_text_metrics(n, note_h, dpi)[0] for n in notes)
    if widest > text_w:
        note_h *= text_w / widest
    line_gap = note_h * 1.45

    for note in notes:
        if y > page_h - margin:
            over = y - (page_h - margin)
            # 세로로 넘친 만큼을 마커 치수로 환산해 "그럼 얼마면 되는가"를 알려준다.
            # 격자는 세로 2줄이므로 마커를 d 줄이면 세로가 대략 2d 줄어든다.
            rows_n = max(1, -(-len(ids) // cols))
            hint = want_size - over / rows_n * scale
            raise ValueError(
                f"시트가 A4 를 {over:.1f}mm 넘는다"
                + (f" (프린터 배율 {scale * 100:.1f}% 보정으로 {1 / scale:.3f}배 커졌다)"
                   if abs(scale - 1.0) > 1e-9 else "")
                + f".\n  → markers.size_mm 를 약 {hint:.1f}mm 이하로 낮추거나"
                  f" (지금 {want_size:g}mm),\n"
                  f"    quiet_zone_mm·tile_gap_mm·margin_mm 을 줄일 것.\n"
                  f"    마커를 줄이면 **검출**이 나빠진다(N14) — 짚기 증폭(N15)은 마커"
                  f" 크기와 무관하니 거기까지 걱정할 필요는 없다.\n"
                  f"    프린터 설정을 고칠 수 있으면 그쪽이 먼저다.")
        _text(canvas, note, margin, y, note_h, dpi, max_w_mm=text_w)
        y += line_gap
    return canvas


def build_charuco_sheet(cfg: dict[str, Any]) -> np.ndarray:
    """ChArUco 보드 시트 — **조건부**. 정식 intrinsic(블록 A ④)으로 갈 때만.

    A4 / 5×7 / square 35mm / marker 26mm → 보드 실치 175×245mm, 여백 17.5·26mm.

    🔴 보드가 A4 를 거의 다 먹는다. 그래서 마커 시트와 달리 여백을 설정값이 아니라
       **남는 자리에서 역산**한다 — 머리글 한 줄과 스케일 바만 넣는다.
    🔴 딕셔너리는 기준 마커와 다른 것을 쓴다. 보드는 ID 0 부터 채우므로 같으면 충돌한다
       (검사는 cal_config.validate_config).
    """
    c, b = cfg["charuco"], cfg["board"]
    dpi = int(b["dpi"])
    page_w, page_h = (float(v) for v in b["page_mm"])
    sx, sy = int(c["squares_x"]), int(c["squares_y"])
    sq_mm = float(c["square_mm"])

    board = cal_markers.make_charuco_board(cfg)

    # 사각형 한 변을 정수 픽셀로 고정해야 체커 경계가 흐트러지지 않는다
    sq_px = max(1, _px(sq_mm, dpi))
    bw, bh = sq_px * sx, sq_px * sy
    board_w_mm, board_h_mm = _mm(bw, dpi), _mm(bh, dpi)

    header_mm, bar_mm_h, pad = 6.0, 16.0, 3.0
    free_h = page_h - (board_h_mm + header_mm + bar_mm_h + 2 * pad)
    if free_h < 0 or board_w_mm > page_w:
        raise ValueError(
            f"ChArUco 보드 {board_w_mm:.1f}×{board_h_mm:.1f}mm 가 "
            f"{page_w:g}×{page_h:g}mm 페이지에 머리글·스케일 바와 함께 들어가지 않는다. "
            "squares 나 square_mm 을 줄일 것")
    margin = free_h / 2

    gen = getattr(board, "generateImage", None)          # 4.7+
    img = gen((bw, bh), marginSize=0, borderBits=1) if gen is not None \
        else board.draw((bw, bh), marginSize=0, borderBits=1)
    img = np.ascontiguousarray(img, dtype=np.uint8)

    canvas = _blank(page_w, page_h, dpi)
    text_w = page_w - 2 * pad

    y = margin + 4.0
    _text(canvas, f"ChArUco {sx}x{sy}  square {_mm(sq_px, dpi):.3f} mm  "
                  f"marker {float(c['marker_mm']):g} mm  {c['dictionary']}  "
                  f"{dpi} dpi  {_dt.date.today().isoformat()}   "
                  f"- PRINT AT 100%",
          pad, y, 3.2, dpi, max_w_mm=text_w)

    y = margin + header_mm + pad
    _paste(canvas, img, (page_w - board_w_mm) / 2, y, dpi)
    y += board_h_mm + pad

    strip = np.full((_px(bar_mm_h, dpi), _px(text_w, dpi)), _WHITE, dtype=np.uint8)
    strip = add_scale_bar(strip, float(b["scale_bar_mm"]), dpi)
    _paste(canvas, strip, pad, y, dpi)
    return canvas


def save_pdf(sheet: np.ndarray, path: str | Path, dpi: int) -> Path:
    """실치수가 보존되는 PDF 로 저장. 배율이 흐트러지면 인쇄물이 무의미해진다.

    PIL 로 1비트 PDF 를 쓴다 — `resolution` 이 곧 PDF MediaBox 의 물리 치수가 되므로
    실치수가 보존되고, 이진이라 CCITT G4 로 눌려 용량이 그레이스케일의 1/100 이 된다.
    reportlab 을 안 쓰는 이유이기도 하다(이 PC 에 없다).

    ⚠️ PDF 물리 치수는 픽셀/dpi 라 정수 픽셀화에서 소수점 오차가 남는다
       (600dpi A4 → 595.2 × 841.8 pt, 정확값 595.276 × 841.89 대비 0.03mm).
       인쇄 배율 검증은 이 숫자가 아니라 **시트의 스케일 바**로 한다.
    """
    from PIL import Image                      # 여기서만 쓴다

    if sheet.ndim != 2:
        raise ValueError(f"이진 그레이스케일 2차원 배열이어야 한다: {sheet.shape}")

    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)

    # 혹시 반색조가 섞여 들어와도 여기서 이진으로 못 박는다
    bw = (sheet >= 128).astype(np.uint8) * 255
    Image.fromarray(bw, mode="L").convert("1").save(
        p, "PDF", resolution=float(dpi))
    return p


def run_board(cfg: dict[str, Any], kind: str = "markers") -> Path:
    """블록 A 인쇄물 절차 진입점. kind: markers | charuco."""
    if kind == "markers":
        sheet = build_marker_sheet(cfg)
        name = cfg["board"]["output_name"]
    elif kind == "charuco":
        sheet = build_charuco_sheet(cfg)
        name = cfg["charuco"]["output_name"]
    else:
        raise ValueError(f"kind 는 markers 또는 charuco 다: {kind!r}")

    out = Path(cfg["paths"]["data_dir"]) / "board" / f"{name}.pdf"
    return save_pdf(sheet, out, int(cfg["board"]["dpi"]))
