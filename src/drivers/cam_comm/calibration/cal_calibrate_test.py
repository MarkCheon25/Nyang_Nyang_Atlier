#!/usr/bin/env python3
"""cal_calibrate + cal_config 결과물 계층 단위테스트 — 장비 없이 돈다.

블록 B(터치 → 기준맵 + 평면)와 블록 C(대응 → H → 드리프트)의 **계산부 전체**를
합성 데이터로 통과시킨다. 막힌 것은 수집 세 걸음(observe_markers · run_reference ·
run_update)뿐이고, 그 뒤 계산은 여기서 전부 검증된다.

합성이라 정답을 우리가 정하고 들어간다 — 로봇이 짚은 점을 우리가 만들어 주므로
"짚기가 정확했다면 결과가 맞는가"를 분리해서 볼 수 있다.

    python3 cal_calibrate_test.py
    pytest cal_calibrate_test.py

🔴 여기 통과가 실기 통과를 뜻하지 않는다. 사람이 펜 끝으로 코너를 얼마나 정확히
   짚는가(블록 B 정밀도의 실제 지배 요인)는 합성에 없다.

설계 근거: cal_script.md §3.3 · cal_calibrate.py 머리주석
✅ 2026-08-16, 세션 `260816-브론치노`
"""

from __future__ import annotations

import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cal_calibrate as C        # noqa: E402
import cal_config                # noqa: E402
import cal_geometry as G         # noqa: E402
import cal_simulate as S         # noqa: E402


_NOTES: list[tuple[str, str]] = []


def _record(label: str, value: str) -> None:
    _NOTES.append((label, value))


# ── 합성 작업대 ─────────────────────────────────────────
# 작업면은 살짝 기울어 있다 — 완전 수평이면 평면 계산의 오류가 안 드러난다.
_TILT_N = np.array([0.010, -0.006, 1.0])
_TILT_N /= np.linalg.norm(_TILT_N)
_TABLE_Z = 150.0                     # 원점 부근 작업면 높이(mm)
_PLANE_D = float(_TILT_N @ np.array([0.0, 0.0, _TABLE_Z]))
_MARKER_MM = 50.0


def _cfg(tmp: Path | None = None) -> dict:
    cfg = cal_config.default_config()
    if tmp is not None:
        cfg["paths"]["data_dir"] = str(tmp)
    return cfg


def _z_at(x: float, y: float) -> float:
    return (_PLANE_D - _TILT_N[0] * x - _TILT_N[1] * y) / _TILT_N[2]


# 작업면 안의 정규직교 기저. 마커는 이 기저 위에서 정사각형이다.
#
# 🔴 여기를 xy 격자로 잡으면 안 된다. 기울어진 평면에 놓인 50mm 마커는 **평면 안에서**
#    변이 50mm 지, xy 투영이 50mm 인 게 아니다. 격자로 잡으면 3D 변 길이가
#    50.0025mm(+0.005%)가 되고, 그 미세한 어긋남이 코너 전개 오차(3.4e-3mm)와
#    인쇄배율 검산(1.5% → 1.505%)으로 새어 나온다. 처음에 실제로 그렇게 잡았다가
#    5개 테스트가 실패했고, 구현이 아니라 이 정답이 틀린 것이었다.
_EX = np.array([1.0, 0.0, 0.0]) - _TILT_N * _TILT_N[0]
_EX /= np.linalg.norm(_EX)
_EY = np.cross(_TILT_N, _EX)


def _true_corners(mid: int, size: float = _MARKER_MM) -> dict[str, np.ndarray]:
    """마커 하나의 네 코너 정답(base 3D). 작업면 **안에서** 한 변 size 인 정사각형이다."""
    cx, cy = S.MARKER_CENTERS[mid]
    center = np.array([cx, cy, _z_at(cx, cy)])
    h = size / 2
    return {"TL": center - h * _EX + h * _EY,
            "TR": center + h * _EX + h * _EY,
            "BR": center + h * _EX - h * _EY,
            "BL": center - h * _EX - h * _EY}


def _touches(cfg: dict, noise: float = 0.0, size: float = _MARKER_MM,
             seed: int = 0) -> dict[str, np.ndarray]:
    """사람이 짚은 8점. noise 는 짚기 오차(mm, 등방)."""
    rng = np.random.default_rng(seed)
    out = {}
    for label in C.touch_labels(cfg):
        mid = int(label.split(".")[0][1:])
        corner = label.split(".")[1]
        p = _true_corners(mid, size)[corner].copy()
        if noise:
            p += rng.normal(0, noise, 3)
        out[label] = p
    return out


# ── 설정 ────────────────────────────────────────────────

def test_default_config_is_valid():
    cal_config.validate_config(_cfg())


def test_pen_delta_must_fit_in_spring():
    """δ + 평면 임계가 스프링 행정을 넘으면 설정 단계에서 막는다."""
    cfg = _cfg()
    cfg["pen"]["delta_mm"] = 0.8            # 0.8 + 0.4 = 1.2 > 1.069
    try:
        cal_config.validate_config(cfg)
    except ValueError as e:
        assert "스프링" in str(e), str(e)
    else:
        raise AssertionError("스프링 행정을 넘는 설정을 통과시켰다")


def test_undetermined_threshold_must_be_explicit_null():
    """미정 임계를 통째로 빼는 것과 null 로 두는 것은 다르다 — 빠지면 예외."""
    cfg = _cfg()
    del cfg["thresholds"]["drift_warn_mm"]
    try:
        cal_config.validate_config(cfg)
    except ValueError as e:
        assert "drift_warn_mm" in str(e)
    else:
        raise AssertionError("빠진 임계를 통과시켰다")


def test_drift_warn_must_precede_stop():
    cfg = _cfg()
    cfg["thresholds"]["drift_warn_mm"] = 5.0
    cfg["thresholds"]["drift_stop_mm"] = 2.0
    try:
        cal_config.validate_config(cfg)
    except ValueError as e:
        assert "경고" in str(e)
    else:
        raise AssertionError("경고가 정지보다 늦은 설정을 통과시켰다")


# ── 결과물 입출력 ───────────────────────────────────────

def test_result_roundtrip_with_numpy():
    """ndarray 를 담아도 저장·복원이 되는가 — yaml 이 numpy 를 못 먹는 함정."""
    tmp = Path(tempfile.mkdtemp())
    try:
        cfg = _cfg(tmp)
        data = {"H": np.eye(3), "scalar": np.float64(1.5),
                "nested": {"pts": np.zeros((2, 2))}, "plain": [1, 2]}
        path = cal_config.save_result(cfg, "homography", data)
        assert path.exists()

        back = cal_config.load_result(cfg, "homography")
        assert np.allclose(np.asarray(back["H"]), np.eye(3))
        assert back["scalar"] == 1.5
        assert np.allclose(np.asarray(back["nested"]["pts"]), 0)
        assert back["_meta"]["name"] == "homography"
    finally:
        shutil.rmtree(tmp)


def test_missing_result_says_what_to_run():
    tmp = Path(tempfile.mkdtemp())
    try:
        cal_config.load_result(_cfg(tmp), "reference")
    except FileNotFoundError as e:
        assert "cal.py reference" in str(e), str(e)
    else:
        raise AssertionError("없는 결과를 읽어 왔다")
    finally:
        shutil.rmtree(tmp)


def test_result_path_rejects_unknown_name():
    try:
        cal_config.result_path(_cfg(), "없는것")
    except ValueError:
        pass
    else:
        raise AssertionError("모르는 결과물 이름을 통과시켰다")


def test_stale_detects_config_change_but_not_path_change():
    """설정이 바뀌면 stale, 데이터 경로만 바뀌면 stale 이 아니다."""
    tmp = Path(tempfile.mkdtemp())
    try:
        cfg = _cfg(tmp)
        cal_config.save_result(cfg, "reference", {"x": 1})
        assert cal_config.result_is_stale(cfg, "reference") is False

        moved = _cfg(tmp)
        moved["paths"]["data_dir"] = str(tmp)      # 경로는 지문에 안 들어간다
        assert cal_config.result_is_stale(moved, "reference") is False

        changed = _cfg(tmp)
        changed["markers"]["size_mm"] = 60.0
        assert cal_config.result_is_stale(changed, "reference") is True
    finally:
        shutil.rmtree(tmp)


def test_missing_result_is_not_stale():
    """한 번도 안 돌린 것과 낡은 것은 다르다 — 메시지가 갈린다."""
    tmp = Path(tempfile.mkdtemp())
    try:
        assert cal_config.result_is_stale(_cfg(tmp), "reference") is False
    finally:
        shutil.rmtree(tmp)


def test_raw_is_never_overwritten():
    """같은 tag 로 두 번 수집해도 앞의 것이 남는가 — 원자료 계층의 존재 이유."""
    tmp = Path(tempfile.mkdtemp())
    try:
        cfg = _cfg(tmp)
        p1 = cal_config.save_raw(cfg, "B", "touch", {"v": 1})
        p2 = cal_config.save_raw(cfg, "B", "touch", {"v": 2})
        assert p1 != p2 and p1.exists() and p2.exists()

        items = cal_config.load_raw(cfg, "B")
        assert [i["data"]["v"] for i in items] == [1, 2]      # 시각 순
        assert cal_config.load_raw(cfg, "C") == []
    finally:
        shutil.rmtree(tmp)


def test_raw_survives_same_millisecond_burst():
    """빠르게 연달아 저장해도 하나도 안 잃는가.

    🔴 처음 구현은 파일명이 밀리초까지라 연속 저장이 같은 밀리초에 걸리면 덮어썼다.
       200회에 86건이 사라졌고, 위 테스트는 두 번만 저장해 **가끔만** 실패했다 —
       그 간헐적 실패가 이 결함의 유일한 신호였다. 여기서 확실히 못 박는다.
    """
    tmp = Path(tempfile.mkdtemp())
    try:
        cfg = _cfg(tmp)
        n = 200
        paths = [cal_config.save_raw(cfg, "B", "touch", {"v": i}) for i in range(n)]
        assert len(set(paths)) == n, f"{n - len(set(paths))}건이 덮어써졌다"

        items = cal_config.load_raw(cfg, "B")
        assert len(items) == n
        # 🔴 정렬까지 본다. 파일명 정렬이 곧 수집 순서여야 한다 — 일련번호를 겹칠 때만
        #    붙이면 여기서 순서가 뒤집힌다(실제로 그렇게 만들었다가 잡혔다).
        assert [i["data"]["v"] for i in items] == list(range(n))
    finally:
        shutil.rmtree(tmp)


def test_raw_dir_rejects_unknown_block():
    try:
        cal_config.raw_dir(_cfg(), "Z")
    except ValueError:
        pass
    else:
        raise AssertionError("모르는 블록을 통과시켰다")


# ── 블록 B — 기준 확정 ──────────────────────────────────

def test_touch_labels():
    labels = C.touch_labels(_cfg())
    assert labels == ["M0.TL", "M0.TR", "M1.TL", "M1.TR",
                      "M2.TL", "M2.TR", "M3.TL", "M3.TR"], labels


def test_work_plane_recovers_the_table():
    """짚은 점에서 작업면이 그대로 나오는가."""
    cfg = _cfg()
    plane = C.build_work_plane(_touches(cfg))
    assert np.allclose(plane["normal"], _TILT_N, atol=1e-9)
    assert abs(plane["offset"] - _PLANE_D) < 1e-7
    assert plane["residual_max_mm"] < 1e-9
    _record("작업대 기울기 (합성 설정값)", f"{plane['tilt_deg']:.3f}도")


def test_work_plane_reports_per_point_residual():
    """한 점만 튀었을 때 그 점이 이름으로 짚히는가."""
    cfg = _cfg()
    t = _touches(cfg)
    t["M2.TR"] = t["M2.TR"] + np.array([0, 0, 2.0])
    plane = C.build_work_plane(t)
    worst = max(plane["residuals_mm"], key=lambda k: abs(plane["residuals_mm"][k]))
    assert worst == "M2.TR", worst


def test_reference_map_expands_all_four_corners():
    """TL·TR 두 점만 짚고 네 코너가 정답으로 전개되는가 — 블록 B 의 핵심."""
    cfg = _cfg()
    ref = C.build_reference_map(_touches(cfg), cfg)

    assert len(ref["corners"]) == 16
    worst = 0.0
    for mid in cfg["markers"]["reference_ids"]:
        truth = _true_corners(int(mid))
        for name, p in truth.items():
            got = np.asarray(ref["corners"][f"M{int(mid)}.{name}"])
            worst = max(worst, float(np.linalg.norm(got - p)))
    assert worst < 1e-7, worst
    _record("코너 전개 최대 오차 (오차 없는 짚기)", f"{worst:.2e}mm")


def test_reference_map_detects_print_scale_error():
    """인쇄물이 1.5% 크게 뽑혔을 때 잡아내는가 — '페이지에 맞춤' 함정."""
    cfg = _cfg()
    ref = C.build_reference_map(_touches(cfg, size=_MARKER_MM * 1.015), cfg)
    assert abs(ref["print_scale_error_pct"] - 1.5) < 1e-6, ref["print_scale_error_pct"]
    _record("인쇄 배율 오차 검출", f"{ref['print_scale_error_pct']:+.3f}%")


def test_reference_map_does_not_silently_fix_config():
    """검산은 하되 설정을 몰래 고치지 않는다 — 재는 것과 정하는 것을 섞지 않는다."""
    cfg = _cfg()
    C.build_reference_map(_touches(cfg, size=_MARKER_MM * 1.02), cfg)
    assert cfg["markers"]["size_mm"] == _MARKER_MM


def test_reference_map_needs_every_touch():
    cfg = _cfg()
    t = _touches(cfg)
    del t["M1.TR"]
    try:
        C.build_reference_map(t, cfg)
    except KeyError as e:
        assert "M1.TR" in str(e)
    else:
        raise AssertionError("빠진 터치를 통과시켰다")


def test_reference_map_survives_touch_noise():
    """손 오차 0.2mm 로 짚어도 코너 전개가 무너지지 않는가."""
    cfg = _cfg()
    worst = []
    for seed in range(20):
        ref = C.build_reference_map(_touches(cfg, noise=0.2, seed=seed), cfg)
        e = max(float(np.linalg.norm(np.asarray(ref["corners"][f"M{m}.{n}"])
                                     - _true_corners(m)[n]))
                for m in (0, 1, 2, 3) for n in ("TL", "TR", "BR", "BL"))
        worst.append(e)
    mean_worst = float(np.mean(worst))
    assert mean_worst < 2.0, mean_worst
    _record("짚기 0.2mm 오차 → 코너 최대 오차", f"{mean_worst:.3f}mm")


# ── 펜 높이 ─────────────────────────────────────────────

def test_pen_heights_follow_the_tilted_plane():
    """기울어진 작업면에서 위치마다 다른 z 가 나오는가."""
    cfg = _cfg()
    plane = C.build_work_plane(_touches(cfg))
    d, u = float(cfg["pen"]["delta_mm"]), float(cfg["pen"]["up_mm"])

    for x, y in ((0.0, 0.0), (210.0, 297.0), (105.0, 148.5)):
        z = G.plane_z_at((np.asarray(plane["normal"]), plane["offset"]), x, y)
        assert abs(C.pen_down_z(plane, x, y, cfg) - (z - d)) < 1e-9
        assert abs(C.pen_up_z(plane, x, y, cfg) - (z + u)) < 1e-9

    # 기울어져 있으므로 위치마다 z 가 달라야 한다.
    # 🔴 대각선(210, 297)으로 재면 안 된다 — 이 합성 기울기는 x 성분(+0.010)과
    #    y 성분(−0.006)이 대각 방향에서 서로 상쇄해 0.3mm 밖에 안 벌어진다.
    #    "기울기를 넣었는데 왜 평평해 보이나"로 헷갈리기 딱 좋은 자리라 x 축으로 잰다.
    z0 = C.pen_down_z(plane, 0.0, 0.0, cfg)
    zx = C.pen_down_z(plane, 210.0, 0.0, cfg)
    assert abs(zx - z0) > 1.0, (z0, zx)
    _record("A4 폭 210mm 구간 펜다운 z 차", f"{abs(zx - z0):.3f}mm")


def test_pen_down_blocks_when_spring_would_bottom_out():
    """평면이 나쁘면 펜다운을 아예 막는다 — 긁고 나서 아는 것보다 낫다."""
    cfg = _cfg()
    t = _touches(cfg)
    t["M0.TL"] = t["M0.TL"] + np.array([0, 0, 1.5])       # 크게 헛짚음
    plane = C.build_work_plane(t)
    try:
        C.pen_down_z(plane, 100.0, 100.0, cfg)
    except ValueError as e:
        assert "스프링" in str(e), str(e)
    else:
        raise AssertionError("스프링이 바닥날 평면으로 펜다운을 내줬다")


def test_pen_up_is_not_blocked_by_bad_plane():
    """펜업은 들어올리는 방향이라 스프링과 무관하다."""
    cfg = _cfg()
    t = _touches(cfg)
    t["M0.TL"] = t["M0.TL"] + np.array([0, 0, 1.5])
    C.pen_up_z(C.build_work_plane(t), 100.0, 100.0, cfg)


# ── 블록 C — 대응 · H · 드리프트 ────────────────────────

def _observe(dist=400.0, tilt=45.0, rot_deg=0.0, res=(1280, 800)):
    """카메라가 본 마커 코너(픽셀). observe_markers 자리를 합성으로 대신한다."""
    K, R, t = _camera(dist, tilt, rot_deg, res)
    out = {}
    for mid in S.MARKER_CENTERS:
        truth = _true_corners(mid)
        pts = np.array([truth[n][:2] for n in ("TL", "TR", "BR", "BL")])
        world = np.column_stack([pts, [truth[n][2] for n in
                                       ("TL", "TR", "BR", "BL")]])
        cam = world @ R.T + t
        img = cam @ K.T
        out[mid] = img[:, :2] / img[:, 2:3]
    return out


def _camera(dist, tilt, rot_deg, res):
    W, _ = res
    f = (W / 2) / np.tan(np.radians(45.0))
    K = np.array([[f, 0, res[0] / 2], [0, f, res[1] / 2], [0, 0, 1]], float)

    target = np.array([105.0, 148.5, _TABLE_Z])
    tr = np.radians(tilt)
    eye = target + np.array([0.0, -dist * np.cos(tr), dist * np.sin(tr)])

    fwd = target - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    R = np.stack([right, np.cross(fwd, right), fwd])

    if rot_deg:                                   # 기둥이 z 축으로 살짝 돌아감
        c, s = np.cos(np.radians(rot_deg)), np.sin(np.radians(rot_deg))
        R = R @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]).T
    return K, R, -R @ eye


def _reference(cfg):
    return C.build_reference_map(_touches(cfg), cfg)


def test_match_correspondences_pairs_by_label():
    cfg = _cfg()
    ref, obs = _reference(cfg), _observe()
    img_pts, base_pts = C.match_correspondences(obs, ref)
    assert img_pts.shape == (16, 2) and base_pts.shape == (16, 2)


def test_match_ignores_unknown_ids():
    """종이 지그 마커(40~)가 같은 화면에 들어와도 대응쌍에 안 섞인다."""
    cfg = _cfg()
    obs = _observe()
    obs[40] = np.array([[10.0, 10], [20, 10], [20, 20], [10, 20]])
    img_pts, _ = C.match_correspondences(obs, _reference(cfg))
    assert len(img_pts) == 16


def test_match_raises_when_too_few():
    """기준과 짝지어진 게 4쌍 미만이면 예외 — 아는 마커가 하나도 없는 화면."""
    cfg = _cfg()
    obs = {40: np.array([[10.0, 10], [20, 10], [20, 20], [10, 20]])}
    try:
        C.match_correspondences(obs, _reference(cfg))
    except ValueError as e:
        assert "N10" in str(e)
    else:
        raise AssertionError("4쌍 미만을 통과시켰다")


def test_single_marker_gives_exactly_the_minimum():
    """마커 하나만 보이면 4쌍 — 통과는 하지만 그게 N10 이 말하는 여유 0 이다.

    🔴 4쌍이라 H 는 **풀린다.** 다만 그 4점이 작은 마커 하나에 몰려 있어 종이 전역이
       외삽이 된다. 개수만 보고 통과시키면 안 된다는 근거이고, 그래서 실제 판정은
       `require_markers`(ID 기준)와 잔차가 한다.
    """
    cfg = _cfg()
    obs = {0: _observe()[0]}
    img_pts, _ = C.match_correspondences(obs, _reference(cfg))
    assert len(img_pts) == 4


def test_compute_homography_end_to_end():
    """터치 → 기준맵 → 관측 → H. 계산 경로 전체가 이어지는가."""
    cfg = _cfg()
    out = C.compute_homography(cfg, _reference(cfg), _observe())

    assert out["n_pairs"] == 16
    assert out["residual_max_mm"] < 1e-6, out["residual_max_mm"]
    assert out["verdict"].startswith("미판정")          # 임계 미정 (N4)
    _record("블록C H 잔차 (오차 없는 합성)", f"{out['residual_max_mm']:.2e}mm")
    _record("종이 중앙 1px", f"{out['px_to_mm_at_center']:.4f}mm")


def test_verdict_is_undetermined_not_pass():
    """임계가 없으면 '통과'가 아니라 '미판정' 이어야 한다 (N4)."""
    cfg = _cfg()
    out = C.compute_homography(cfg, _reference(cfg), _observe())
    assert "통과" != out["verdict"]

    cfg["thresholds"]["homography_residual_mm"] = 1.0
    out2 = C.compute_homography(cfg, _reference(cfg), _observe())
    assert out2["verdict"] == "통과"


def test_drift_absent_on_first_run():
    """기준 H 가 없으면 드리프트는 0 이 아니라 '없음' 이다."""
    cfg = _cfg()
    ref = _reference(cfg)
    out = C.detect_drift(np.eye(3), ref, cfg)
    assert out["available"] is False and out["drift_mm"] is None


def test_drift_measures_pillar_rotation():
    """기둥이 0.1도 돌면 작업면에서 mm 급으로 나타나는가."""
    cfg = _cfg()
    ref = _reference(cfg)
    H0 = np.asarray(C.compute_homography(cfg, ref, _observe())["H"])
    ref["H_reference"] = H0

    for deg in (0.0, 0.1, 0.5):
        H = np.asarray(C.compute_homography(cfg, ref, _observe(rot_deg=deg))["H"])
        d = C.detect_drift(H, ref, cfg)
        assert d["available"] is True
        if deg == 0.0:
            assert d["drift_mm"] < 1e-6
        else:
            _record(f"드리프트 — 기둥 {deg}도 회전", f"{d['drift_mm']:.3f}mm")
    assert C.detect_drift(
        np.asarray(C.compute_homography(cfg, ref, _observe(rot_deg=0.5))["H"]),
        ref, cfg)["drift_mm"] > 1.0


def test_drift_verdict_uses_thresholds():
    cfg = _cfg()
    ref = _reference(cfg)
    H0 = np.asarray(C.compute_homography(cfg, ref, _observe())["H"])
    ref["H_reference"] = H0
    H1 = np.asarray(C.compute_homography(cfg, ref, _observe(rot_deg=0.5))["H"])

    assert C.detect_drift(H1, ref, cfg)["verdict"].startswith("미판정")

    cfg["thresholds"]["drift_warn_mm"] = 1.0
    cfg["thresholds"]["drift_stop_mm"] = 100.0
    assert "경고" in C.detect_drift(H1, ref, cfg)["verdict"]

    cfg["thresholds"]["drift_stop_mm"] = 2.0
    assert "정지" in C.detect_drift(H1, ref, cfg)["verdict"]


# ── 막힌 절차는 이유를 말하는가 ─────────────────────────

def test_blocked_procedures_explain_why():
    """장비 대기 중인 세 함수가 무엇이 막혔는지 말하는가."""
    cfg = _cfg()
    for fn, needle in ((lambda: C.run_reference(cfg), "N1"),
                       (lambda: C.observe_markers(cfg), "N2")):
        try:
            fn()
        except NotImplementedError as e:
            assert needle in str(e), str(e)
        else:
            raise AssertionError("막힌 절차가 그냥 돌았다")


# ── 실행부 ──────────────────────────────────────────────

def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]

    print(f"cal_calibrate 단위테스트 — {len(tests)}개\n" + "─" * 62)
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
