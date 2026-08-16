"""기준 확정과 호모그래피 갱신 — 블록 B·C. 캘리브레이션의 본체.

블록 B (최초 1회, 카메라 + 로봇)
    작업대에 붙인 기준 마커를 펜 끝으로 짚어 **base 좌표를 확정**하고,
    같은 터치 데이터에서 작업평면(펜다운 z)까지 함께 뽑는다.
    이 값은 마커가 떨어지지 않는 한 영원히 유효하다.

블록 C (매 작업 전, 카메라만 · 1초)
    마커를 찍어 **H 를 그 자리에서 다시 푼다.**
    H 를 저장값이 아니라 매번 다시 푸는 값으로 두면, 기둥이 밀렸든 돌았든 자동 반영된다.
    진동에 대한 방어가 "잘 조여두기"가 아니라 "움직여도 무관한 구조"가 된다.

    ⚠️ 그래도 드리프트는 보고한다 — 자동 보정되더라도 기둥이 움직였다는 사실 자체가
       점검 신호다. 임계를 넘으면 사람을 부른다.

🔴 **마커당 2코너만 짚는다** (TL·TR). 마커 치수를 알고 있고 작업면이 평면이므로,
   그 둘이 마커의 방향과 크기를 다 정한다 — 나머지 두 코너는 계산으로 전개된다.
   4개 마커 × 2점 = **8번**이면 기준 맵과 작업평면이 동시에 나온다.

설계 근거: cal_script.md §3.4 · cal_readme.md §1 블록 B·C
🚧 부분 구현 (2026-08-16, 세션 `260816-브론치노`)
   ✅ 계산부 8개 — 장비 없이 합성 데이터로 검증됨 (cal_calibrate_test.py)
   🚧 절차부 3개 (observe_markers · run_reference · run_update) — 카메라(N2)·로봇(N1) 대기
"""

from __future__ import annotations

from typing import Any

import numpy as np

try:                                      # 스크립트로도, 패키지로도 부를 수 있게
    from . import cal_config, cal_geometry
except ImportError:                       # pragma: no cover
    import cal_config                     # type: ignore[no-redef]
    import cal_geometry                   # type: ignore[no-redef]

# cal_markers 는 observe_markers 가 서면 여기서 쓰인다. 지금 부르는 곳이 없어
# import 하지 않는다 — 안 쓰는 import 를 남겨 두면 의존 관계를 잘못 읽게 만든다.


# 짚는 코너. aruco 코너 순서 TL, TR, BR, BL 중 앞의 둘이다.
# TL→TR 이 마커의 +x 방향이라 이 둘만으로 방향이 정해진다.
_TOUCH_CORNERS = ("TL", "TR")
_CORNER_INDEX = {"TL": 0, "TR": 1, "BR": 2, "BL": 3}


# ── 블록 B — 기준 확정 (최초 1회) ───────────────────────

def touch_labels(cfg: dict[str, Any]) -> list[str]:
    """짚어야 할 기준점 목록을 만든다. 마커당 어느 코너를 짚을지가 여기서 정해진다.

    반환 예 — `["M0.TL", "M0.TR", "M1.TL", ...]`. 사람에게 읽어 주는 순서이자
    원자료의 키라서, 이 순서가 바뀌면 예전 원자료를 다시 못 읽는다. 함부로 바꾸지 말 것.
    """
    return [f"M{int(mid)}.{c}"
            for mid in cfg["markers"]["reference_ids"]
            for c in _TOUCH_CORNERS]


def build_reference_map(touches: dict[str, np.ndarray],
                        cfg: dict[str, Any]) -> dict[str, Any]:
    """짚은 점 + 인쇄 규격 → 기준 마커 코너 전체의 base 좌표.

    마커 치수를 알므로 마커당 1~2코너만 짚어도 나머지 코너는 계산으로 전개된다.

    전개 방식 — 짚은 TL·TR 이 마커의 +x 축(ex)을 주고, 작업평면 법선 n 과 함께
    +y 축을 만든다(ey = n × ex). 그러면 네 코너가 전부 닫힌 식으로 나온다:

        TR = TL + s·ex        BL = TL − s·ey        BR = TL + s·ex − s·ey

    🔴 **짚은 거리로 인쇄 배율을 검산한다.** |TR − TL| 은 마커 실치수여야 하는데,
       프린터의 "페이지에 맞춤"이 켜져 있으면 몇 % 어긋나고 그 오차가 H 에 스케일
       오차로 그대로 박힌다. 여기서 안 잡으면 나중에 아무 데서도 안 잡힌다.
       설정값을 조용히 고치지는 않는다 — **재는 것과 정하는 것을 섞지 않는다.**
       보고만 하고, 설정에 넣을지는 사람이 판단한다(인쇄물 실측이 정본).

    🔴 전개에는 **설정 치수가 아니라 짚어서 잰 치수**를 쓴다. 인쇄물이 실제로 그 크기이기
       때문이다. 설정값은 어디까지나 검산의 기준선이다.
    """
    plane = build_work_plane(touches)
    n = np.asarray(plane["normal"], np.float64)
    nominal = float(cfg["markers"]["size_mm"])

    corners: dict[str, list[float]] = {}
    sizes: dict[str, float] = {}

    for mid in cfg["markers"]["reference_ids"]:
        mid = int(mid)
        tl_key, tr_key = f"M{mid}.TL", f"M{mid}.TR"
        for k in (tl_key, tr_key):
            if k not in touches:
                raise KeyError(f"기준점 {k} 가 안 짚혔다 — touch_labels() 전부가 필요하다")

        # 짚은 점은 손 오차로 평면에서 조금 떠 있다. 평면 위로 내려서 전개한다 —
        # 안 그러면 ex 가 평면 밖으로 기울고 그 기울기가 네 코너에 증폭돼 퍼진다.
        tl = cal_geometry.project_to_plane(np.asarray(touches[tl_key], np.float64), _plane_tuple(plane))
        tr = cal_geometry.project_to_plane(np.asarray(touches[tr_key], np.float64), _plane_tuple(plane))

        edge = tr - tl
        s = float(np.linalg.norm(edge))
        if s < 1e-6:
            raise ValueError(f"M{mid} 의 TL·TR 이 같은 점이다 — 짚기를 다시 할 것")

        ex = edge / s
        ey = np.cross(n, ex)               # 오른손 규약: n × ex 가 마커의 +y

        for name, p in (("TL", tl),
                        ("TR", tl + s * ex),
                        ("BR", tl + s * ex - s * ey),
                        ("BL", tl - s * ey)):
            corners[f"M{mid}.{name}"] = [float(v) for v in p]
        sizes[f"M{mid}"] = s

    measured = np.array(list(sizes.values()))
    scale_ratio = float(measured.mean() / nominal)

    return {
        "corners": corners,                       # 라벨 → base [x, y, z] (평면 위)
        "marker_size_measured_mm": {k: float(v) for k, v in sizes.items()},
        "marker_size_nominal_mm": nominal,
        "print_scale_ratio": scale_ratio,         # 1.0 이면 배율 오차 없음
        "print_scale_error_pct": (scale_ratio - 1.0) * 100.0,
        "size_spread_mm": float(measured.max() - measured.min()),
    }


def _plane_tuple(plane: dict[str, Any]) -> tuple[np.ndarray, float]:
    """평면 dict → cal_geometry 가 받는 (n, d) 꼴."""
    return np.asarray(plane["normal"], np.float64), float(plane["offset"])


def build_work_plane(touches: dict[str, np.ndarray]) -> dict[str, Any]:
    """같은 터치 데이터에서 작업평면을 뽑는다. 펜다운 z 의 출처.

    반환에 **점별 잔차를 그대로 싣는다.** 평균만 남기면 "한 점만 튀었다"와
    "종이가 전체적으로 휘었다"를 나중에 구별할 수 없다.

    🔴 판정은 여기서 하지 않는다 — 임계는 설정(thresholds.plane_residual_mm) 몫이고,
       이 함수는 **재기만** 한다. 수집과 판정을 섞으면 임계를 바꿀 때마다 다시 짚어야 한다.
    """
    labels = sorted(touches)
    if len(labels) < 3:
        raise ValueError(f"작업평면을 풀려면 터치가 3점 이상 필요하다: {len(labels)}점")

    P = np.array([np.asarray(touches[k], np.float64).reshape(3) for k in labels])
    n, d = cal_geometry.fit_plane(P)
    res = cal_geometry.plane_residuals(P, (n, d))

    return {
        "normal": [float(v) for v in n],
        "offset": float(d),
        "residuals_mm": {k: float(r) for k, r in zip(labels, res)},
        "residual_max_mm": float(np.abs(res).max()),
        "residual_rms_mm": float(np.sqrt(np.mean(res ** 2))),
        "n_points": len(labels),
        # 기울기 — 작업대가 얼마나 기울어 있는가. 사람이 눈으로 검산할 값이다.
        "tilt_deg": float(np.degrees(np.arccos(np.clip(abs(n[2]), 0.0, 1.0)))),
    }


def _pen_z(plane: dict[str, Any], x: float, y: float,
           cfg: dict[str, Any], sign: int, key: str) -> float:
    z = cal_geometry.plane_z_at(_plane_tuple(plane), x, y)
    return z + sign * float(cfg["pen"][key])


def pen_down_z(plane: dict[str, Any], x: float, y: float,
               cfg: dict[str, Any]) -> float:
    """(x, y) 에서의 펜다운 높이 = 평면 z − δ.

    δ 는 볼펜이 파고드는 깊이다. TCP 가 스프링 **자유 길이** 기준으로 등록돼 있어
    (2026-08-16 실기 4점법) *파고든 mm = 스프링 압축량 = 필압* 이 그대로 성립한다.
    그래서 δ 는 재는 값이 아니라 **고르는 값**이다.

    🔴 여유가 거의 없다 — 스프링 **행정이 1.069mm 뿐**이다. δ 에 평면 오차가 더해져
       행정을 넘기면 스프링이 바닥나고 펜이 종이를 긁는다. 그 지점이 실제로 어디인지
       평면 잔차로 알 수 있으므로, 넘길 것 같으면 **미리 막는다.**
    """
    plane_err = float(plane.get("residual_max_mm", 0.0))
    delta = float(cfg["pen"]["delta_mm"])
    travel = float(cfg["pen"]["spring_travel_mm"])

    if delta + plane_err >= travel:
        raise ValueError(
            f"δ({delta}mm) + 평면 최대잔차({plane_err:.3f}mm) = "
            f"{delta + plane_err:.3f}mm 가 스프링 행정 {travel}mm 이상이다. "
            "가장 어긋난 지점에서 스프링이 바닥나 펜이 종이를 긁는다 — "
            "다시 짚어 평면을 개선하거나 δ 를 줄일 것")

    return _pen_z(plane, x, y, cfg, -1, "delta_mm")


def pen_up_z(plane: dict[str, Any], x: float, y: float,
             cfg: dict[str, Any]) -> float:
    """(x, y) 에서의 펜업 높이 = 평면 z + h. 깊이 센서와 무관하다."""
    return _pen_z(plane, x, y, cfg, +1, "up_mm")


def run_reference(cfg: dict[str, Any]) -> dict[str, Any]:
    """블록 B 절차 진입점 — 터치 수집 → 기준 맵 + 평면 → reference.yaml.

    🔴 **미구현 — 로봇이 필요하다(N1).** L21(`move.joint.velocity` 단위 미확정)과
       movel 사용 정지 때문에 자동 이동이 봉인돼 있어, 사람이 조그로 짚고 스크립트는
       읽기만 하는 모드로만 돌 수 있다. 그 대화형 절차는 cal_robot 이 서야 붙는다.

    계산부는 이미 서 있다 — 원자료(터치 8점)만 있으면
    `build_work_plane` → `build_reference_map` 으로 결과가 나온다. 그래서
    수집만 되면 `--recompute` 로 언제든 다시 계산할 수 있다.
    """
    raise NotImplementedError(
        "블록 B 는 로봇 터치가 필요하다 (N1: L21 + movel 정지). "
        "cal_robot.collect_touch_points 가 서면 연결된다")


# ── 블록 C — H 갱신 (매 작업 전) ────────────────────────

def observe_markers(cfg: dict[str, Any]) -> dict[int, np.ndarray]:
    """기준 마커를 찍어 지금 이미지에서의 코너 좌표를 얻는다.

    🔴 **미구현 — 카메라가 필요하다(N2).** 검출 자체는 `cal_markers` 가 이미 서 있고
       합성 영상으로 검증됐다. 여기서 막힌 것은 프레임을 가져오는 부분(cal_camera)뿐이다.

    그래서 `compute_homography` 는 관측을 **주입받을 수 있게** 열어 두었다 —
    카메라 없이도 계산 경로 전체를 시험할 수 있다.
    """
    raise NotImplementedError(
        "블록 C 촬영은 카메라가 필요하다 (N2). "
        "cal_camera.grab_stable_frame 이 서면 연결된다")


def match_correspondences(observed: dict[int, np.ndarray],
                          reference: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """관측 코너 ↔ 기준 base 좌표를 짝지어 (img_pts, base_pts) 로.

    🔴 **짝은 라벨로 맞춘다.** 검출 순서는 프레임마다 달라지므로 배열 순서로 맞추면
       조용히 뒤섞인다 — 그래도 H 는 풀리고 잔차만 커져서 "정밀도가 원래 이런가 보다"가 된다.

    🔴 기준에 없는 ID 는 **말없이 버린다.** 종이 지그 마커(40~)가 같은 화면에 들어오는
       것은 정상이고, 그것까지 대응쌍에 넣으면 안 된다. 대신 몇 쌍이 남았는지는
       호출부가 세서 판단한다.
    """
    ref_corners = reference["corners"]
    img_pts: list[np.ndarray] = []
    base_pts: list[np.ndarray] = []

    for mid in sorted(observed):
        quad = np.asarray(observed[mid], np.float64).reshape(4, 2)
        for name, k in _CORNER_INDEX.items():
            label = f"M{int(mid)}.{name}"
            if label not in ref_corners:
                continue
            img_pts.append(quad[k])
            base_pts.append(np.asarray(ref_corners[label], np.float64)[:2])

    if len(img_pts) < 4:
        raise ValueError(
            f"기준과 짝지어진 대응쌍이 {len(img_pts)}개뿐이다 — H 를 풀려면 4개 이상이다(N10). "
            f"관측 ID {sorted(observed)} · 기준 마커 "
            f"{sorted({k.split('.')[0] for k in ref_corners})}")

    return np.array(img_pts), np.array(base_pts)


def compute_homography(cfg: dict[str, Any],
                       reference: dict[str, Any],
                       observed: dict[int, np.ndarray] | None = None) -> dict[str, Any]:
    """대응쌍 → H + 잔차. 잔차가 임계를 넘으면 마커가 들떴거나 왜곡이 남은 것이다.

    `observed` 를 주면 그것을 쓰고, 안 주면 카메라로 찍는다(observe_markers).
    주입을 열어 둔 이유는 **카메라 없이도 계산 경로를 통째로 시험**하기 위해서다.

    🔴 임계(thresholds.homography_residual_mm)가 **null 이면 판정하지 않고 값만 싣는다**(N4).
       임의의 숫자를 넣어 통과시키는 것보다, 임계가 없다는 사실이 결과에 보이는 편이 안전하다.
    """
    if observed is None:
        observed = observe_markers(cfg)

    img_pts, base_pts = match_correspondences(observed, reference)
    H = cal_geometry.solve_homography(img_pts, base_pts)
    res = cal_geometry.homography_residual(H, img_pts, base_pts)

    # 종이가 놓이는 자리에서 1px 이 몇 mm 인가 — 정밀도 보고용
    center_px = img_pts.mean(axis=0)
    scale = cal_geometry.px_to_mm_scale(H, center_px)

    th = cfg["thresholds"].get("homography_residual_mm")
    result = {
        "H": H.tolist(),
        "n_pairs": int(len(img_pts)),
        "observed_ids": sorted(int(i) for i in observed),
        "residual_mean_mm": float(res.mean()),
        "residual_max_mm": float(res.max()),
        "residual_rms_mm": float(np.sqrt(np.mean(res ** 2))),
        "px_to_mm_at_center": float(scale),
        "threshold_mm": th,
        "verdict": _verdict(float(res.max()), th),
    }
    return result


def _verdict(value: float, threshold: float | None) -> str:
    """임계가 없으면 판정하지 않는다 — 통과도 실패도 아닌 '미정'이 정직하다 (N4)."""
    if threshold is None:
        return "미판정 (임계 미정 — N4)"
    return "통과" if value <= threshold else "초과"


def detect_drift(H_now: np.ndarray, reference: dict[str, Any],
                 cfg: dict[str, Any]) -> dict[str, Any]:
    """직전 H 대비 변위(mm)를 재고 임계와 견준다.

    이미지 1px ≈ 작업면 0.6mm(정면) / 0.9mm(45도)이고,
    기둥이 0.1° 돌면 400mm 거리에서 **0.932mm** 밀린다(합성 실측, N12) —
    병진보다 회전에 훨씬 민감하다.

    🔴 **재는 자리가 결과를 바꾼다.** 시야 구석에서 재면 실제로 그림을 그리는 곳과
       무관한 값이 나온다. 기준 마커가 종이 자리를 둘러싸고 있으므로, 그 네 마커의
       **중심이 이루는 사각형**을 탐침으로 쓴다 — 종이 위치를 아직 몰라도(블록 D 이전)
       그림이 그려질 영역을 대표한다.

    🔴 기준 H 가 없으면(최초 실행) 드리프트는 **0 이 아니라 '없음'** 이다.
       0 으로 두면 "안 움직였다"로 읽혀 최초 실행이 항상 통과한다.
    """
    H_ref = reference.get("H_reference")
    if H_ref is None:
        return {"available": False,
                "reason": "기준 H 가 없다 (블록 B 최초 실행) — 이번 H 가 기준이 된다",
                "drift_mm": None, "verdict": "미판정"}

    H_ref = np.asarray(H_ref, np.float64).reshape(3, 3)
    H_now = np.asarray(H_now, np.float64).reshape(3, 3)

    probe_base = _probe_points(reference)
    # 탐침은 이미지 좌표로 줘야 한다 — base 사각형을 기준 H 로 되쏘아 픽셀로 옮긴다
    probe_px = cal_geometry.apply_homography(np.linalg.inv(H_ref), probe_base)
    drift = cal_geometry.homography_drift(H_ref, H_now, probe_px)

    warn = cfg["thresholds"].get("drift_warn_mm")
    stop = cfg["thresholds"].get("drift_stop_mm")

    if stop is not None and drift > stop:
        verdict = "정지 — 기둥을 점검할 것"
    elif warn is not None and drift > warn:
        verdict = "경고 — 기둥이 움직였다"
    elif warn is None and stop is None:
        verdict = "미판정 (임계 미정 — N4)"
    else:
        verdict = "통과"

    return {"available": True, "drift_mm": float(drift),
            "warn_mm": warn, "stop_mm": stop,
            "n_probe": int(len(probe_base)), "verdict": verdict}


def _probe_points(reference: dict[str, Any]) -> np.ndarray:
    """기준 마커 중심이 이루는 사각형 — 그림이 그려질 영역의 대표점."""
    centers: dict[str, list[np.ndarray]] = {}
    for label, xyz in reference["corners"].items():
        centers.setdefault(label.split(".")[0], []).append(
            np.asarray(xyz, np.float64)[:2])

    pts = np.array([np.mean(v, axis=0) for v in centers.values()])
    if len(pts) < 3:
        raise ValueError(f"탐침을 만들 기준 마커가 부족하다: {len(pts)}개")
    return pts


def run_update(cfg: dict[str, Any]) -> dict[str, Any]:
    """블록 C 절차 진입점 — 촬영 → H 재계산 → 드리프트 판정 → homography.yaml.

    🔴 **미구현 — 카메라가 필요하다(N2).** 막힌 곳은 촬영 한 걸음뿐이고,
       그 뒤(대응·H·드리프트·저장)는 전부 서 있다.
    """
    reference = cal_config.load_result(cfg, "reference")
    observed = observe_markers(cfg)                 # ← 여기서 N2 로 막힌다

    homography = compute_homography(cfg, reference, observed)
    homography["drift"] = detect_drift(np.asarray(homography["H"]), reference, cfg)
    cal_config.save_result(cfg, "homography", homography)
    return homography
