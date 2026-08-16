#!/usr/bin/env python3
"""기준 마커 시트 합성 검증 — 카메라 없이 "정상작동"을 확인한다.

PDF 를 되렌더해 다시 검출하는 방식은 **정면·완벽 조건**만 본다. 실제 셋업은
기둥에 고정된 카메라가 A4 작업면을 **45도로 400mm 거리**에서 내려다보는 구성이다.
그 조건을 기하로 재현해서 두 가지를 잰다:

  ① 마커가 검출되는가            (검출 개수)
  ② 검출 코너로 푼 H 가 정확한가  (알려진 작업면 좌표와의 재투영 오차, mm)

②가 이 스크립트의 핵심이다. 검출만 되고 좌표가 틀리면 로봇이 엉뚱한 곳에 그린다.
정답(ground truth)을 우리가 알고 있다는 게 합성의 이점이다 — 실기에서는
"H 가 맞는지"를 재려면 로봇으로 다시 짚어야 한다(블록 B·E).

    python3 cal_simulate.py [--config 경로]

카메라·로봇 없이 돈다. 산출물 PNG 2장은 data_dir/sim/ 에 떨어진다.

🔴 여기서 나오는 수치는 **인쇄물과 검출 알고리즘의 상한**이지 실기 정확도가 아니다.
   렌즈 왜곡·마커 들뜸·조명·로봇 반복도가 빠져 있다. 실기 값은 블록 E 가 잰다.
"""
import argparse
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cal_board          # noqa: E402
import cal_config         # noqa: E402
import cal_markers        # noqa: E402


# ── 작업대 배치 (mm, z=0 평면) ─────────────────────────
# A4 종이가 (0,0)~(210,297) 에 놓이고 마커 4개는 그 바깥 네 귀퉁이.
# cal_readme.md §1 "기준 마커 배치" 를 좌표로 옮긴 것이다.
# 🔴 실제 배치는 블록 B 가 로봇으로 짚어 확정한다 — 여기 값은 그때까지의 대역이다.
PAPER_W, PAPER_H = 210.0, 297.0
MARKER_CENTERS = {
    0: (-35.0, PAPER_H + 35.0),            # M0 좌상
    1: (PAPER_W + 35.0, PAPER_H + 35.0),   # M1 우상
    2: (-35.0, -35.0),                     # M2 좌하
    3: (PAPER_W + 35.0, -35.0),            # M3 우하
}


def marker_corners_mm(cx: float, cy: float, s: float) -> np.ndarray:
    """작업면에서 본 마커 코너 4점. aruco 순서(TL, TR, BR, BL) 에 맞춘다.

    위에서 내려다보는 카메라라 이미지 위쪽 = 작업면 +y 다.
    """
    h = s / 2
    return np.array([[cx - h, cy + h], [cx + h, cy + h],
                     [cx + h, cy - h], [cx - h, cy - h]], dtype=np.float64)


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """world→camera 회전·병진 (OpenCV 규약: x 우, y 아래, z 앞)."""
    eye, target, up = map(lambda v: np.asarray(v, float), (eye, target, up))
    fwd = target - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    R = np.stack([right, down, fwd])           # world→cam
    return R, -R @ eye


def detect(img: np.ndarray, dictionary: Any):
    """마커 검출. cv2 버전 분기는 cal_markers 와 같은 이유다(§2 버전 함정)."""
    det = getattr(cv2.aruco, "ArucoDetector", None)
    if det is not None:                        # 4.7+
        params = cv2.aruco.DetectorParameters()
        corners, ids, _ = det(dictionary, params).detectMarkers(img)
    else:                                      # 4.6 이하
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(img, dictionary,
                                                  parameters=params)
    return corners, (ids.ravel().tolist() if ids is not None else [])


def build_table(dictionary: Any, nominal_mm: float, px_per_mm: float = 12.0):
    """작업대를 위에서 똑바로 본 정사영 이미지. 반환 (이미지, extent, 실제치수mm).

    🔴 반환하는 마커 치수는 **이 캔버스에 실제로 그려진** 값이다.
       render_marker 는 셀 배수로 반올림하므로 캔버스 해상도마다 값이 다르다 —
       정답 좌표를 인쇄 시트(600dpi)의 치수로 쓰면 캔버스와 어긋나고,
       그 불일치가 H 잔차로 나와 성능인 척한다. 실제로 처음에 그렇게 재고 있었다.
    """
    x0, x1 = -80.0, PAPER_W + 80.0
    y0, y1 = -80.0, PAPER_H + 80.0
    W = int((x1 - x0) * px_per_mm)
    H = int((y1 - y0) * px_per_mm)
    table = np.full((H, W), 255, np.uint8)

    dpi = int(round(px_per_mm * 25.4))               # px/mm → dpi
    marker_mm = cal_board.rendered_marker_mm(dictionary, nominal_mm, dpi)

    for mid, (cx, cy) in MARKER_CENTERS.items():
        m = cal_board.render_marker(dictionary, mid, nominal_mm, dpi)
        s = m.shape[0]
        # 작업면 → 정사영 픽셀 (y 뒤집기)
        px = int(round((cx - x0) * px_per_mm)) - s // 2
        py = int(round((y1 - cy) * px_per_mm)) - s // 2
        table[py:py + s, px:px + s] = m

    # A4 종이 자리 — 검출과 무관하지만 배치를 눈으로 볼 때 쓴다
    cv2.rectangle(table,
                  (int(-x0 * px_per_mm), int((y1 - PAPER_H) * px_per_mm)),
                  (int((PAPER_W - x0) * px_per_mm), int(y1 * px_per_mm)),
                  200, max(1, int(px_per_mm * 0.4)))
    return table, (x0, x1, y0, y1, px_per_mm), marker_mm


def synth_view(table, extent, cam_dist, tilt_deg, res, fov_deg=90.0,
               blur=0, noise=0.0, occlude=None):
    """작업대를 가상 카메라로 찍는다. 반환 (이미지, K, R, t)."""
    x0, x1, y0, y1, _ = extent
    W, H = res
    f = (W / 2) / np.tan(np.radians(fov_deg / 2))
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1]], float)

    # 작업면 중심을 본다. 기둥은 -y 쪽에 서서 tilt 만큼 내려다본다
    target = np.array([PAPER_W / 2, PAPER_H / 2, 0.0])
    t_rad = np.radians(tilt_deg)
    eye = target + np.array([0.0,
                             -cam_dist * np.cos(t_rad),
                             cam_dist * np.sin(t_rad)])
    R, tvec = look_at(eye, target)

    # 정사영 이미지의 네 귀퉁이(작업면 좌표)를 투영해 워핑 행렬을 얻는다
    src = np.array([[0, 0], [table.shape[1], 0],
                    [table.shape[1], table.shape[0]], [0, table.shape[0]]],
                   np.float32)
    world = np.array([[x0, y1, 0], [x1, y1, 0], [x1, y0, 0], [x0, y0, 0]], float)
    proj, _ = cv2.projectPoints(world, cv2.Rodrigues(R)[0], tvec, K, None)
    dst = proj.reshape(-1, 2).astype(np.float32)

    Hwarp = cv2.getPerspectiveTransform(src, dst)
    view = cv2.warpPerspective(table, Hwarp, (W, H), flags=cv2.INTER_AREA,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=255)

    if occlude is not None:                     # 마커 하나를 가린다
        pts = cv2.projectPoints(
            np.hstack([marker_corners_mm(*MARKER_CENTERS[occlude], 70.0),
                       np.zeros((4, 1))]),
            cv2.Rodrigues(R)[0], tvec, K, None)[0].reshape(-1, 2)
        cv2.fillPoly(view, [pts.astype(np.int32)], 128)

    if blur:
        view = cv2.GaussianBlur(view, (blur | 1, blur | 1), 0)
    if noise:
        view = np.clip(view.astype(np.float32)
                       + np.random.default_rng(0).normal(0, noise, view.shape),
                       0, 255).astype(np.uint8)
    return view, K, R, tvec


def refine(view: np.ndarray, corners):
    """서브픽셀 코너 정련. cal_markers.refine_corners_subpixel 이 할 일을 미리 재본다."""
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    out = []
    for c in corners:
        p = np.ascontiguousarray(c.reshape(-1, 1, 2).astype(np.float32))
        cv2.cornerSubPix(view, p, (5, 5), (-1, -1), crit)
        out.append(p)
    return out


def evaluate(view, dictionary, size_mm, subpix=False):
    """검출 → H 추정 → 재투영 오차(mm). H 는 이미지→작업면 방향이다.

    반환 (검출개수, 코너별 오차 배열 mm, 종이 위 1px 이 몇 mm 인가).
    마커가 4개 미만이면 오차는 None — **4점이 최소이고 여유가 0 이다**(N10).
    """
    corners, ids = detect(view, dictionary)
    if len(ids) < 4:
        return len(ids), None, None
    if subpix:
        corners = refine(view, corners)

    img_pts, base_pts = [], []
    for c, mid in zip(corners, ids):
        if mid not in MARKER_CENTERS:
            continue
        img_pts.append(c.reshape(4, 2))
        base_pts.append(marker_corners_mm(*MARKER_CENTERS[mid], size_mm))
    img_pts = np.concatenate(img_pts).astype(np.float64)
    base_pts = np.concatenate(base_pts).astype(np.float64)

    Hm, _ = cv2.findHomography(img_pts, base_pts, 0)
    if Hm is None:
        return len(ids), None, None

    got = cv2.perspectiveTransform(img_pts.reshape(-1, 1, 2), Hm).reshape(-1, 2)
    err = np.linalg.norm(got - base_pts, axis=1)

    # 종이 네 귀퉁이가 이미지 어디로 가는지 → 거기서 1px 이 작업면 몇 mm 인가
    paper = np.array([[0, 0], [PAPER_W, 0], [PAPER_W, PAPER_H], [0, PAPER_H]],
                     np.float64)
    pin = cv2.perspectiveTransform(paper.reshape(-1, 1, 2),
                                   np.linalg.inv(Hm)).reshape(-1, 2)
    d = []
    for p in pin:
        a = cv2.perspectiveTransform(np.array([[p]]), Hm).reshape(2)
        b = cv2.perspectiveTransform(np.array([[p + [1, 0]]]), Hm).reshape(2)
        d.append(np.linalg.norm(b - a))
    return len(ids), err, float(np.mean(d))


# 조건표 — 이름, 거리mm, 경사deg, 해상도, 블러px, 노이즈sigma, 가릴 마커
CASES = [
    ("기준 45deg 400mm 1280x800", 400, 45, (1280, 800),  0,  0.0, None),
    ("얕은 각도 20deg (더 비스듬)", 400, 20, (1280, 800),  0,  0.0, None),
    ("가파른 65deg (위에서)",      400, 65, (1280, 800),  0,  0.0, None),
    ("먼 거리 600mm (문서 경고)",  600, 45, (1280, 800),  0,  0.0, None),
    ("아주 먼 800mm",             800, 45, (1280, 800),  0,  0.0, None),
    ("저해상도 640x480",          400, 45, (640, 480),   0,  0.0, None),
    ("블러 5px",                  400, 45, (1280, 800),  5,  0.0, None),
    ("블러 11px (심한 초점흐림)",  400, 45, (1280, 800), 11,  0.0, None),
    ("노이즈 sigma=12",           400, 45, (1280, 800),  0, 12.0, None),
    ("블러5 + 노이즈8 (현실적)",    400, 45, (1280, 800),  5,  8.0, None),
    ("마커 1개 가림 (M1)",         400, 45, (1280, 800),  0,  0.0, 1),
]


def run_simulation(cfg: dict[str, Any]) -> None:
    m = cfg["markers"]
    nominal = float(m["size_mm"])
    dictionary = cal_markers.make_dictionary(m["dictionary"])
    sheet_mm = cal_board.rendered_marker_mm(
        dictionary, nominal, int(cfg["board"]["dpi"]))

    table, extent, size_mm = build_table(dictionary, nominal)
    out_dir = Path(cfg["paths"]["data_dir"]) / "sim"
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"cv2 {cv2.__version__}   마커 {m['dictionary']}  명목 {nominal:g}mm")
    print(f"  인쇄 시트 렌더 치수 {sheet_mm:.4f}mm @ {cfg['board']['dpi']}dpi")
    print(f"  합성 캔버스 렌더 치수 {size_mm:.4f}mm  ← 정답 좌표는 이 값을 쓴다")
    print(f"작업대: A4 {PAPER_W:g}x{PAPER_H:g}mm + 마커 4개 (종이 바깥 네 귀퉁이)")
    print(f"마커 중심 간격: {MARKER_CENTERS[1][0] - MARKER_CENTERS[0][0]:g} x "
          f"{MARKER_CENTERS[0][1] - MARKER_CENTERS[2][1]:g} mm\n")
    cv2.imwrite(str(out_dir / "sim_table.png"),
                cv2.resize(table, None, fx=0.35, fy=0.35))

    print(f"{'조건':<28} {'검출':>5} {'H잔차(원시)':>12} {'H잔차(서브픽셀)':>15} "
          f"{'1px=?mm':>9}")
    print("-" * 76)
    saved = False
    for name, dist, tilt, res, blur, noise, occ in CASES:
        view, *_ = synth_view(table, extent, dist, tilt, res,
                              blur=blur, noise=noise, occlude=occ)
        n, err, mmpp = evaluate(view, dictionary, size_mm)
        _, err_s, _ = evaluate(view, dictionary, size_mm, subpix=True)
        if not saved:
            cv2.imwrite(str(out_dir / "sim_view.png"), view)
            saved = True
        if err is None:
            print(f"{name:<28} {n:>3}/4  {'-- H 못 품':>12}")
        else:
            gain = f"{np.mean(err) / np.mean(err_s):.1f}x" if err_s is not None else "-"
            print(f"{name:<28} {n:>3}/4  {np.mean(err):>10.4f}mm "
                  f"{np.mean(err_s):>12.4f}mm {mmpp:>8.3f}   ({gain})")

    # 반복 촬영 흔들림 — 같은 장면에 노이즈만 다르게
    print("\n같은 장면 반복 촬영 — 센서 노이즈만 다르게 (45deg/400mm, sigma=10):")
    base_view, *_ = synth_view(table, extent, 400, 45, (1280, 800))
    _, _, mmpp = evaluate(base_view, dictionary, size_mm)
    for tag, use_sub in (("원시 코너", False), ("서브픽셀", True)):
        cen = []
        for seed in range(8):
            rng = np.random.default_rng(seed)
            v = np.clip(base_view.astype(np.float32)
                        + rng.normal(0, 10, base_view.shape),
                        0, 255).astype(np.uint8)
            corners, ids = detect(v, dictionary)
            if len(ids) != 4:
                continue
            if use_sub:
                corners = refine(v, corners)
            cen.append(np.concatenate(
                [corners[ids.index(i)].reshape(4, 2) for i in sorted(ids)]))
        if len(cen) > 1:
            sd = np.stack(cen).std(axis=0)
            print(f"  {tag:<9} 코너 표준편차 평균 {sd.mean():.4f}px  "
                  f"최대 {sd.max():.4f}px  → 작업면 약 {sd.mean() * mmpp:.4f}mm")

    print(f"\n산출물: {out_dir}/sim_table.png (위에서 본 배치) · sim_view.png (카메라 시점)")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", help="설정 파일 경로 (기본: cal_config 기본값)")
    args = ap.parse_args(argv)

    cfg = cal_config.load_config(args.config)
    cal_config.validate_config(cfg)
    run_simulation(cfg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
