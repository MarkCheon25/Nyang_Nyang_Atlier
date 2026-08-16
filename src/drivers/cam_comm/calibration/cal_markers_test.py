#!/usr/bin/env python3
"""cal_markers 단위테스트 — 카메라 없이 합성 영상으로 돈다 (N2 우회).

`cal_simulate.py` 가 만드는 45°·400mm 합성 시점을 그대로 빌려 쓴다. 인쇄물을 실제로
렌더해서 찍으므로 "검출이 되는가"는 진짜 검출기를 통과한 결과다.

    python3 cal_markers_test.py         # 단독 실행
    pytest cal_markers_test.py

🔴 **합성에 없는 것** — 렌즈 왜곡 · 롤링셔터 기울어짐 · 조명 불균일 · 마커 들뜸 ·
   IR 프로젝터 점무늬. 여기 통과가 실물 통과를 뜻하지 않는다. 카메라가 오면 다시 잰다.

설계 근거: cal_script.md §3.1 · cal_markers.py 머리주석
✅ 2026-08-16, 세션 `260816-브론치노`
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cal_markers as M          # noqa: E402
import cal_simulate as S         # noqa: E402

import cv2                       # noqa: E402  (검출 계층은 cv2 가 있어야 돈다)


_NOTES: list[tuple[str, str]] = []


def _record(label: str, value: str) -> None:
    _NOTES.append((label, value))


# ── 합성 장면 (한 번만 만들어 돌려 쓴다) ────────────────
_SCENE: dict[str, object] = {}


def _scene(dist=400.0, tilt=45.0, res=(1280, 800), **kw):
    """(view, K, R, t, dictionary, marker_mm). 같은 인자는 캐시한다."""
    key = (dist, tilt, res, tuple(sorted(kw.items())))
    if key not in _SCENE:
        dictionary = M.make_dictionary("DICT_4X4_50")
        table, extent, marker_mm = S.build_table(dictionary, 50.0)
        view, K, R, t = S.synth_view(table, extent, dist, tilt, res, **kw)
        _SCENE[key] = (view, K, R, t, dictionary, marker_mm)
    return _SCENE[key]


def _project(K, R, t, pts_mm):
    """작업면 점(z=0) → 이미지 픽셀."""
    P = np.asarray(pts_mm, float).reshape(-1, 2)
    cam = np.hstack([P, np.zeros((len(P), 1))]) @ R.T + t
    img = cam @ K.T
    return img[:, :2] / img[:, 2:3]


# ── 딕셔너리 ────────────────────────────────────────────

def test_make_dictionary():
    assert M.make_dictionary("DICT_4X4_50") is not None
    assert M.make_dictionary({"markers": {"dictionary": "DICT_5X5_100"}}) is not None


def test_make_dictionary_rejects_unknown():
    for bad in ("DICT_없는거", {"markers": {}}):
        try:
            M.make_dictionary(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"이상한 딕셔너리를 통과시켰다: {bad}")


# ── 검출 ────────────────────────────────────────────────

def test_detect_finds_all_four():
    """기준 조건에서 4개 다 보이는가 — 이게 안 되면 그 뒤가 전부 무의미하다."""
    view, _, _, _, dictionary, _ = _scene()
    ids, corners = M.detect_markers(view, dictionary)
    assert sorted(ids.tolist()) == [0, 1, 2, 3], ids.tolist()
    assert corners.shape == (4, 4, 2)


def test_detect_return_contract():
    """반환 규약 — ids (N,) int32 · corners (N,4,2) float64. 호출부가 이걸 믿는다."""
    view, _, _, _, dictionary, _ = _scene()
    ids, corners = M.detect_markers(view, dictionary)
    assert ids.dtype == np.int32 and ids.ndim == 1
    assert corners.dtype == np.float64 and corners.shape[1:] == (4, 2)
    assert len(ids) == len(corners)


def test_detect_empty_is_a_value_not_an_exception():
    """아무것도 없는 화면 — 예외가 아니라 빈 결과여야 한다."""
    dictionary = M.make_dictionary("DICT_4X4_50")
    ids, corners = M.detect_markers(np.full((480, 640), 255, np.uint8), dictionary)
    assert len(ids) == 0 and corners.shape == (0, 4, 2)


def test_detect_accepts_bgr_and_gray():
    """RealSense 컬러는 BGR 3채널로 들어온다 — 양쪽 다 같은 결과여야 한다."""
    view, _, _, _, dictionary, _ = _scene()
    ids_g, c_g = M.detect_markers(view, dictionary)
    ids_c, c_c = M.detect_markers(cv2.cvtColor(view, cv2.COLOR_GRAY2BGR), dictionary)
    assert sorted(ids_g.tolist()) == sorted(ids_c.tolist())
    assert np.allclose(c_g, c_c)


def test_detect_rejects_empty_image():
    dictionary = M.make_dictionary("DICT_4X4_50")
    for bad in (None, np.empty((0, 0), np.uint8)):
        try:
            M.detect_markers(bad, dictionary)
        except (ValueError, AttributeError):
            pass
        else:
            raise AssertionError("빈 이미지를 통과시켰다")


def test_occlusion_loses_exactly_one():
    """마커 하나를 가리면 3개만 남는다 — N10 의 전제 확인."""
    view, _, _, _, dictionary, _ = _scene(occlude=1)
    ids, _ = M.detect_markers(view, dictionary)
    assert sorted(ids.tolist()) == [0, 2, 3], ids.tolist()


# ── 조회·검사 ───────────────────────────────────────────

def test_corners_of():
    view, _, _, _, dictionary, _ = _scene()
    ids, corners = M.detect_markers(view, dictionary)
    for k, mid in enumerate(ids.tolist()):
        assert np.allclose(M.corners_of(ids, corners, mid), corners[k])
    assert M.corners_of(ids, corners, 99) is None


def test_corners_of_rejects_duplicate_id():
    """같은 ID 가 둘 — 아무거나 고르면 그 뒤로 조용히 전부 틀린다."""
    ids = np.array([0, 1, 1, 3], np.int32)
    corners = np.zeros((4, 4, 2))
    try:
        M.corners_of(ids, corners, 1)
    except ValueError:
        pass
    else:
        raise AssertionError("중복 ID 를 통과시켰다")


def test_require_markers_names_what_is_missing():
    try:
        M.require_markers(np.array([0, 2, 3]), [0, 1, 2, 3])
    except LookupError as e:
        assert "1" in str(e), str(e)
    else:
        raise AssertionError("빠진 마커를 통과시켰다")
    M.require_markers(np.array([3, 1, 0, 2]), [0, 1, 2, 3])      # 순서는 무관


def test_require_markers_checks_ids_not_count():
    """개수만 세면 종이 지그 마커(40~)가 기준 마커 자리를 대신 채운다."""
    try:
        M.require_markers(np.array([0, 1, 2, 40]), [0, 1, 2, 3])
    except LookupError:
        pass
    else:
        raise AssertionError("개수만 맞는 조합을 통과시켰다")


def test_marker_corner_ids_matches_corner_flattening():
    """이름표 순서가 corners.reshape(-1,2) 와 어긋나면 대응쌍이 통째로 틀린다."""
    view, _, _, _, dictionary, _ = _scene()
    ids, corners = M.detect_markers(view, dictionary)
    labels = M.marker_corner_ids(ids.tolist())
    flat = corners.reshape(-1, 2)

    assert len(labels) == len(flat) == 4 * len(ids)
    for i, (mid, k) in enumerate(labels):
        assert np.allclose(flat[i], corners[ids.tolist().index(mid)][k])


# ── 마커 중심 — 사영변환의 함정 ─────────────────────────

def test_marker_center_beats_corner_mean_under_perspective():
    """대각선 교점이 코너 평균보다 정답에 가까운가.

    정답은 마커 중심(작업면 좌표)을 그대로 투영한 픽셀이다. 45° 로 보면 사영변환이
    중점을 보존하지 않으므로 코너 평균은 계통적으로 어긋난다.
    """
    view, K, R, t, dictionary, _ = _scene()
    ids, corners = M.detect_markers(view, dictionary)
    corners = M.refine_corners_subpixel(view, corners)

    d_cross, d_mean = [], []
    for k, mid in enumerate(ids.tolist()):
        truth = _project(K, R, t, [S.MARKER_CENTERS[mid]])[0]
        d_cross.append(np.linalg.norm(M.marker_center(corners[k]) - truth))
        d_mean.append(np.linalg.norm(corners[k].mean(axis=0) - truth))

    cross, mean = float(np.mean(d_cross)), float(np.mean(d_mean))
    assert cross < mean, (cross, mean)
    _record("마커중심 — 대각선 교점 오차", f"{cross:.4f}px")
    _record("마커중심 — 코너 평균 오차", f"{mean:.4f}px")


def test_marker_center_exact_on_axis_aligned_square():
    """정사영(왜곡 없음)이면 교점과 평균이 같아야 한다 — 식의 기준점."""
    sq = np.array([[10.0, 10.0], [110.0, 10.0], [110.0, 110.0], [10.0, 110.0]])
    assert np.allclose(M.marker_center(sq), [60.0, 60.0], atol=1e-9)


# ── 서브픽셀 정련 ───────────────────────────────────────

def _corner_error(view, K, R, t, ids, corners, marker_mm):
    """검출 코너와 정답 코너의 평균 거리(px)."""
    errs = []
    for k, mid in enumerate(ids.tolist()):
        truth = _project(K, R, t,
                         S.marker_corners_mm(*S.MARKER_CENTERS[mid], marker_mm))
        errs.append(np.linalg.norm(corners[k] - truth, axis=1).mean())
    return float(np.mean(errs))


def test_subpixel_refinement_improves_corners():
    """정련이 실제로 코너를 정답 쪽으로 당기는가 — '기본이어야 한다'의 근거."""
    view, K, R, t, dictionary, marker_mm = _scene()
    ids, raw = M.detect_markers(view, dictionary)
    refined = M.refine_corners_subpixel(view, raw)

    e_raw = _corner_error(view, K, R, t, ids, raw, marker_mm)
    e_ref = _corner_error(view, K, R, t, ids, refined, marker_mm)
    assert e_ref < e_raw, (e_raw, e_ref)
    _record("코너 오차 — 원시", f"{e_raw:.4f}px")
    _record("코너 오차 — 서브픽셀", f"{e_ref:.4f}px")
    _record("정련 이득", f"{e_raw / e_ref:.2f}x")


def test_subpixel_shape_and_dtype_preserved():
    view, _, _, _, dictionary, _ = _scene()
    _, corners = M.detect_markers(view, dictionary)
    refined = M.refine_corners_subpixel(view, corners)
    assert refined.shape == corners.shape and refined.dtype == np.float64


def test_subpixel_handles_empty():
    """빈 검출 결과를 그대로 통과시켜야 한다 — 호출부가 분기하지 않게."""
    out = M.refine_corners_subpixel(np.full((100, 100), 255, np.uint8),
                                    np.empty((0, 4, 2)))
    assert out.shape == (0, 4, 2)


def test_subpixel_window_shrinks_for_small_markers():
    """저해상도로 마커가 작게 찍혀도 정련이 코너를 망치지 않는가.

    창이 이웃 코너를 덮으면 서로 끌어당겨 오히려 나빠진다 — 그 방어의 확인.
    """
    view, K, R, t, dictionary, marker_mm = _scene(dist=800.0, res=(640, 480))
    ids, raw = M.detect_markers(view, dictionary)
    if len(ids) < 4:
        _record("작은 마커 정련", f"검출 {len(ids)}/4 — 건너뜀")
        return
    refined = M.refine_corners_subpixel(view, raw)
    e_raw = _corner_error(view, K, R, t, ids, raw, marker_mm)
    e_ref = _corner_error(view, K, R, t, ids, refined, marker_mm)
    assert e_ref <= e_raw * 1.05, (e_raw, e_ref)
    _record("작은 마커(800mm·640x480) 정련", f"{e_raw:.3f} → {e_ref:.3f}px")


# ── ChArUco ─────────────────────────────────────────────

def _charuco_cfg():
    import cal_config
    return cal_config.default_config()


def test_charuco_board_and_detection():
    """보드를 렌더해서 되검출 — 4.6 / 4.7+ 어느 API 갈래든 같은 규약으로 나오는가."""
    import cal_board
    cfg = _charuco_cfg()
    board = M.make_charuco_board(cfg)
    sheet = cal_board.build_charuco_sheet(cfg)

    corners, ids = M.detect_charuco(sheet, board)
    c = cfg["charuco"]
    inner = (int(c["squares_x"]) - 1) * (int(c["squares_y"]) - 1)

    assert corners.dtype == np.float64 and corners.shape[1:] == (2,)
    assert ids.dtype == np.int32 and len(ids) == len(corners)
    assert len(ids) == inner, f"내부 코너 {inner}개 중 {len(ids)}개만 나왔다"
    _record(f"ChArUco 코너 (cv2 {cv2.__version__})", f"{len(ids)}/{inner}")


def test_charuco_empty_on_blank():
    board = M.make_charuco_board(_charuco_cfg())
    corners, ids = M.detect_charuco(np.full((600, 400), 255, np.uint8), board)
    assert len(corners) == 0 and len(ids) == 0


# ── 검출 → H 까지 이어 보기 ─────────────────────────────

def test_detection_feeds_homography():
    """이 파일의 출력이 cal_geometry 입력으로 그대로 들어가는가 — 계층 접합부."""
    import cal_geometry as G

    view, _, _, _, dictionary, marker_mm = _scene()
    ids, corners = M.detect_markers(view, dictionary)
    M.require_markers(ids, [0, 1, 2, 3])
    corners = M.refine_corners_subpixel(view, corners)

    base = np.concatenate([S.marker_corners_mm(*S.MARKER_CENTERS[mid], marker_mm)
                           for mid in ids.tolist()])
    H = G.solve_homography(corners.reshape(-1, 2), base)
    res = G.homography_residual(H, corners.reshape(-1, 2), base)

    assert res.mean() < 1.0, res.mean()
    _record("검출 → H 잔차 (평균/최대)", f"{res.mean():.4f} / {res.max():.4f}mm")


# ── 실행부 ──────────────────────────────────────────────

def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]

    print(f"cal_markers 단위테스트 — {len(tests)}개  (cv2 {cv2.__version__})\n" + "─" * 62)
    failed = []
    for name, fn in tests:
        try:
            fn()
        except Exception as e:                       # noqa: BLE001
            failed.append((name, e))
            print(f"  ✗ {name}\n      {type(e).__name__}: {e}")
        else:
            print(f"  ✓ {name}")

    if _NOTES:
        print("\n합성으로 잰 값 " + "─" * 47)
        for label, value in _NOTES:
            print(f"  {label:<44} {value:>14}")

    print("─" * 62)
    if failed:
        print(f"실패 {len(failed)} / {len(tests)}")
        return 1
    print(f"전부 통과 ({len(tests)}/{len(tests)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
