#!/usr/bin/env python3
"""cal_geometry 단위테스트 — 카메라도 로봇도 없이 돈다.

이 프로젝트에서 **장비 없이 정답을 아는 상태로 검증할 수 있는 유일한 계층**이다.
합성이라 정답(ground truth)을 우리가 정하고 들어가므로, 실기처럼 "맞는지 확인하려면
로봇으로 다시 짚어야 하는" 문제가 없다.

    python3 cal_geometry_test.py        # 단독 실행 (pytest 없어도 된다)
    pytest cal_geometry_test.py         # 파일명이 *_test.py 라 pytest 도 걷어간다

🔴 여기서 통과한다고 실기가 맞는 것은 아니다. 검증되는 것은 **수학뿐**이고,
   렌즈 왜곡·마커 들뜸·조명·로봇 반복도는 전부 빠져 있다. 실기 값은 블록 E 가 잰다.

설계 근거: cal_script.md §3.3 · cal_geometry.py 머리주석
✅ 2026-08-16, 세션 `260816-브론치노`
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cal_geometry as G          # noqa: E402


# ── 합성 셋업 — cal_simulate.py 와 같은 배치 ────────────
PAPER_W, PAPER_H = 210.0, 297.0
MARKER_CENTERS = {
    0: (-35.0, PAPER_H + 35.0), 1: (PAPER_W + 35.0, PAPER_H + 35.0),
    2: (-35.0, -35.0),          3: (PAPER_W + 35.0, -35.0),
}
CAM_DIST, CAM_TILT_DEG = 400.0, 45.0
RES = (1280, 800)
FOV_DEG = 90.0


def _look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """world→camera 회전·병진 (OpenCV 규약: x 우, y 아래, z 앞)."""
    eye, target, up = map(lambda v: np.asarray(v, float), (eye, target, up))
    fwd = target - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    return np.stack([right, down, fwd]), None


def _synthetic_view():
    """기둥에 고정된 카메라가 작업면을 45°로 내려다보는 배치. 반환 (K, R, t)."""
    W, _ = RES
    f = (W / 2) / np.tan(np.radians(FOV_DEG / 2))          # = 640px
    K = np.array([[f, 0, RES[0] / 2], [0, f, RES[1] / 2], [0, 0, 1]], float)

    target = np.array([PAPER_W / 2, PAPER_H / 2, 0.0])
    tr = np.radians(CAM_TILT_DEG)
    eye = target + np.array([0.0, -CAM_DIST * np.cos(tr), CAM_DIST * np.sin(tr)])
    R, _ = _look_at(eye, target)
    return K, R, -R @ eye


def _project(K, R, t, pts_mm):
    """작업면 점(z=0) → 이미지 픽셀. 왜곡 없는 핀홀."""
    P = np.asarray(pts_mm, float).reshape(-1, 2)
    world = np.hstack([P, np.zeros((len(P), 1))])
    cam = world @ R.T + t
    img = cam @ K.T
    return img[:, :2] / img[:, 2:3]


def _marker_corners(cx, cy, s=50.0):
    h = s / 2
    return np.array([[cx - h, cy + h], [cx + h, cy + h],
                     [cx + h, cy - h], [cx - h, cy - h]])


def _reference_pairs():
    """기준 마커 4개 × 코너 4점 = 16 대응쌍. 실제 블록 C 의 표준 입력이다."""
    base = np.concatenate([_marker_corners(*c) for c in MARKER_CENTERS.values()])
    K, R, t = _synthetic_view()
    return _project(K, R, t, base), base


# ── 로봇 자세 ↔ 동차행렬 ────────────────────────────────

def test_pose_matrix_is_rigid():
    """회전부는 직교이고 det = +1 이어야 한다. 아니면 길이가 안 보존된다."""
    for p in [(0, 0, 0, 0, 0, 0), (100, -50, 300, -180, 0, 45),
              (1, 2, 3, 12.5, -33.0, 170.0), (0, 0, 0, 91.0, 89.0, -179.0)]:
        R = G.pose_to_matrix(*p)[:3, :3]
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12), p
        assert abs(np.linalg.det(R) - 1.0) < 1e-12, p


def test_pose_matrix_matches_cpp_zyx():
    """R = Rz(rz)·Ry(ry)·Rx(rx) — pose_convention.hpp 와 같은 합성 순서인가.

    단축 회전 셋을 곱해서 대조한다. 순서가 틀리면 (예: XYZ) 여기서 갈린다.
    """
    rx, ry, rz = 23.0, -41.0, 77.0
    Rx = G.pose_to_matrix(0, 0, 0, rx, 0, 0)[:3, :3]
    Ry = G.pose_to_matrix(0, 0, 0, 0, ry, 0)[:3, :3]
    Rz = G.pose_to_matrix(0, 0, 0, 0, 0, rz)[:3, :3]
    assert np.allclose(G.pose_to_matrix(0, 0, 0, rx, ry, rz)[:3, :3],
                       Rz @ Ry @ Rx, atol=1e-12)


def test_pose_roundtrip():
    """6값 → 행렬 → 6값 이 항등인가. 규약이 어긋나면 여기서 잡힌다."""
    rng = np.random.default_rng(0)
    for _ in range(300):
        p = np.array([*rng.uniform(-500, 500, 3),
                      rng.uniform(-180, 180), rng.uniform(-85, 85),
                      rng.uniform(-180, 180)])
        back = np.array(G.matrix_to_pose(G.pose_to_matrix(*p)))
        assert np.allclose(back[:3], p[:3], atol=1e-9)
        # 각도는 ±180 이 같은 값이라 행렬로 되돌려 비교한다
        assert np.allclose(G.pose_to_matrix(*back), G.pose_to_matrix(*p), atol=1e-9)


def test_gimbal_lock_is_consistent_as_matrix():
    """ry=±90° 에서 6값 왕복은 깨지지만 **행렬 왕복은 성립**해야 한다."""
    for ry in (90.0, -90.0):
        T = G.pose_to_matrix(10, 20, 30, 35.0, ry, -60.0)
        back = G.matrix_to_pose(T)
        assert abs(back[4] - ry) < 1e-6            # ry 는 살아남는다
        assert abs(back[5]) < 1e-9                 # rz 는 0 으로 몰아준다
        assert np.allclose(G.pose_to_matrix(*back), T, atol=1e-9)


def test_home_pose_rx_minus_180():
    """홈 자세 rx=-180 — 플랜지 z축이 아래를 본다(hcr5_comm/README §7.2).

    손목 수직 고정이 이 자세를 전제하므로, 규약이 이걸 못 재현하면 전제가 무너진다.
    """
    R = G.pose_to_matrix(0, 0, 0, -180.0, 0, 0)[:3, :3]
    assert np.allclose(R @ np.array([0, 0, 1.0]), [0, 0, -1.0], atol=1e-12)


def test_invert():
    T = G.pose_to_matrix(123.4, -56.7, 890.1, -170.0, 12.0, 33.0)
    assert np.allclose(T @ G.invert(T), np.eye(4), atol=1e-12)
    assert np.allclose(G.invert(G.invert(T)), T, atol=1e-12)


def test_rvec_zero_is_identity_not_nan():
    """θ=0 에서 0/0 이 안 터지는가. 마커가 정면일 때 하필 걸리는 자리다."""
    for scale in (0.0, 1e-15, 1e-10, 1e-6):
        T = G.rvec_tvec_to_matrix(np.array([scale, 0, 0]), np.zeros(3))
        assert np.isfinite(T).all(), scale
        assert np.allclose(T[:3, :3], np.eye(3), atol=1e-5), scale


def test_rvec_matches_axis_rotation():
    """rvec = (0,0,θ) 는 Rz(θ) 여야 한다. 로봇 규약과 만나는 지점의 기준점."""
    for deg in (5.0, 30.0, 90.0, 179.0):
        T = G.rvec_tvec_to_matrix([0, 0, np.radians(deg)], [1, 2, 3])
        assert np.allclose(T[:3, :3], G.pose_to_matrix(0, 0, 0, 0, 0, deg)[:3, :3],
                           atol=1e-12), deg
        assert np.allclose(T[:3, 3], [1, 2, 3])


def test_rvec_is_rigid():
    rng = np.random.default_rng(1)
    for _ in range(200):
        R = G.rvec_tvec_to_matrix(rng.uniform(-3, 3, 3), np.zeros(3))[:3, :3]
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
        assert abs(np.linalg.det(R) - 1.0) < 1e-12


# ── 작업평면 ────────────────────────────────────────────

def _tilted_plane_points(n_true, d_true, rng, noise=0.0, count=12):
    """주어진 평면 위의 점들. 터치로 짚은 점을 흉내낸다."""
    xy = rng.uniform([0, 0], [PAPER_W, PAPER_H], size=(count, 2))
    z = [(d_true - n_true[0] * x - n_true[1] * y) / n_true[2] for x, y in xy]
    P = np.column_stack([xy, z])
    if noise:
        P[:, 2] += rng.normal(0, noise, count)
    return P


def test_fit_plane_exact():
    """오차 없는 점들에서 평면이 정확히 나오는가."""
    rng = np.random.default_rng(2)
    n_true = np.array([0.02, -0.015, 1.0])
    n_true /= np.linalg.norm(n_true)
    d_true = n_true @ np.array([0, 0, 150.0])

    n, d = G.fit_plane(_tilted_plane_points(n_true, d_true, rng))
    assert np.allclose(n, n_true, atol=1e-9)
    assert abs(d - d_true) < 1e-7


def test_fit_plane_normal_points_up():
    """법선은 항상 위쪽 — 뒤집히면 펜이 종이를 뚫는 쪽으로 간다."""
    rng = np.random.default_rng(3)
    P = _tilted_plane_points(np.array([0, 0, 1.0]), 150.0, rng)
    for order in (P, P[::-1], P[[2, 0, 5, 1, 9, 3, 7, 4, 11, 6, 10, 8]]):
        assert G.fit_plane(order)[0][2] > 0


def test_fit_plane_rejects_collinear():
    """한 직선 위의 점들 — 평면이 안 정해진다. 조용히 아무 평면이나 주면 안 된다."""
    line = np.column_stack([np.linspace(0, 200, 8), np.linspace(0, 100, 8),
                            np.full(8, 150.0)])
    try:
        G.fit_plane(line)
    except ValueError:
        pass
    else:
        raise AssertionError("한 직선 점들을 통과시켰다")


def test_fit_plane_needs_three():
    try:
        G.fit_plane(np.array([[0, 0, 0.0], [1, 0, 0.0]]))
    except ValueError:
        pass
    else:
        raise AssertionError("점 2개로 평면을 풀었다")


def test_plane_residual_signs():
    """+ 는 위, − 는 아래. 부호가 없으면 '휘었다'와 '한 번 잘못 짚었다'를 못 가른다."""
    plane = (np.array([0, 0, 1.0]), 150.0)
    r = G.plane_residuals(np.array([[0, 0, 151.0], [0, 0, 149.0], [0, 0, 150.0]]),
                          plane)
    assert np.allclose(r, [1.0, -1.0, 0.0])


def test_plane_residual_finds_the_bad_touch():
    """한 점만 튀었을 때 그 점이 골라지는가 — 실제 쓰임새 그대로."""
    rng = np.random.default_rng(4)
    P = _tilted_plane_points(np.array([0.01, 0.01, 1.0]) / np.linalg.norm(
        [0.01, 0.01, 1.0]), 150.0, rng, noise=0.02)
    P[5, 2] += 3.0                                   # 헛짚은 점
    r = np.abs(G.plane_residuals(P, G.fit_plane(P)))
    assert int(np.argmax(r)) == 5
    assert r[5] > 5 * np.median(r)


def test_plane_z_at():
    """펜다운 z 의 출처. 기울어진 평면에서 위치마다 다른 값이 나와야 한다."""
    n = np.array([0.05, 0.0, 1.0])
    n /= np.linalg.norm(n)
    plane = (n, float(n @ np.array([0, 0, 150.0])))
    assert abs(G.plane_z_at(plane, 0, 0) - 150.0) < 1e-9
    # x 가 +100 이면 z 는 0.05*100 만큼 내려간다
    assert abs(G.plane_z_at(plane, 100, 0) - (150.0 - 5.0)) < 1e-9
    assert abs(G.plane_z_at(plane, 0, 200) - 150.0) < 1e-9      # y 는 무관


def test_plane_z_at_rejects_vertical():
    try:
        G.plane_z_at((np.array([1.0, 0, 0]), 5.0), 0, 0)
    except ValueError:
        pass
    else:
        raise AssertionError("수직 평면에서 z 를 내놨다")


def test_project_to_plane():
    rng = np.random.default_rng(5)
    n = np.array([0.03, -0.02, 1.0])
    n /= np.linalg.norm(n)
    plane = (n, float(n @ np.array([0, 0, 150.0])))

    P = rng.uniform([-100, -100, 100], [300, 400, 200], size=(20, 3))
    assert np.allclose(G.plane_residuals(G.project_to_plane(P, plane), plane), 0,
                       atol=1e-9)
    assert G.project_to_plane(np.array([1.0, 2.0, 3.0]), plane).shape == (3,)


# ── 호모그래피 ──────────────────────────────────────────

def test_homography_recovers_known_transform():
    """알려진 H 로 만든 대응쌍에서 그 H 가 그대로 나오는가."""
    rng = np.random.default_rng(6)
    H_true = np.array([[0.62, -0.03, 12.0],
                       [0.04, 0.58, -8.0],
                       [3.0e-4, 1.1e-4, 1.0]])
    img = rng.uniform([0, 0], [1280, 800], size=(12, 2))
    base = G.apply_homography(H_true, img)

    H = G.solve_homography(img, base)
    assert np.allclose(H, H_true, atol=1e-9)
    assert np.max(G.homography_residual(H, img, base)) < 1e-9


def test_homography_exact_with_four():
    """4쌍이면 정확해 — 최소 개수에서 잔차가 0 이어야 한다 (N10 의 경계)."""
    H_true = np.array([[0.6, 0.0, 10.0], [0.0, 0.6, -5.0], [2e-4, 1e-4, 1.0]])
    img = np.array([[100.0, 100.0], [1100.0, 120.0], [1150.0, 700.0], [80.0, 680.0]])
    base = G.apply_homography(H_true, img)
    assert np.max(G.homography_residual(G.solve_homography(img, base),
                                        img, base)) < 1e-9


def test_homography_rejects_too_few():
    img = np.array([[0.0, 0], [1, 0], [0, 1]])
    try:
        G.solve_homography(img, img)
    except ValueError:
        pass
    else:
        raise AssertionError("3쌍으로 H 를 풀었다 (N10)")


def test_homography_rejects_collinear():
    """네 점이 한 직선 위 — 개수는 맞지만 H 는 안 정해진다."""
    img = np.array([[0.0, 0], [100, 50], [200, 100], [300, 150]])
    base = img * 0.5
    try:
        G.solve_homography(img, base)
    except ValueError:
        pass
    else:
        raise AssertionError("퇴화한 배치를 통과시켰다")


def test_apply_homography_shapes():
    H = np.array([[0.6, 0, 10.0], [0, 0.6, -5.0], [0, 0, 1.0]])
    assert G.apply_homography(H, np.array([10.0, 20.0])).shape == (2,)
    assert G.apply_homography(H, np.zeros((7, 2))).shape == (7, 2)


def test_homography_inverse_roundtrip():
    """이미지→base→이미지 가 제자리로 오는가."""
    rng = np.random.default_rng(7)
    img, base = _reference_pairs()
    H = G.solve_homography(img, base)
    pts = rng.uniform([0, 0], [1280, 800], size=(30, 2))
    back = G.apply_homography(np.linalg.inv(H), G.apply_homography(H, pts))
    assert np.allclose(back, pts, atol=1e-7)


# ── 실셋업 재현 — 45° · 400mm · 1280×800 ────────────────

def test_real_setup_homography_is_exact_without_noise():
    """왜곡 없는 핀홀이면 평면-평면 대응은 **정확히** 호모그래피다.

    잔차가 0 이 아니면 우리 DLT 가 틀린 것이다 — 셋업 탓이 아니다.
    """
    img, base = _reference_pairs()
    H = G.solve_homography(img, base)
    assert np.max(G.homography_residual(H, img, base)) < 1e-8


def test_real_setup_pixel_noise_maps_to_expected_mm():
    """0.1px 검출 오차가 작업면에서 얼마가 되는가 — 설계 전제의 확인.

    문서가 든 눈금은 서브픽셀 0.1px → 약 0.1mm 다. 자릿수가 맞는지만 본다
    (정확한 값은 마커 배치·거리에 딸린다).
    """
    rng = np.random.default_rng(8)
    img, base = _reference_pairs()
    errs = []
    for _ in range(50):
        H = G.solve_homography(img + rng.normal(0, 0.1, img.shape), base)
        errs.append(np.mean(G.homography_residual(H, img, base)))
    mean_err = float(np.mean(errs))
    assert 0.01 < mean_err < 1.0, mean_err


def test_px_to_mm_scale_on_pure_scale():
    """배율만 있는 H 에서는 정답이 자명하다 — 야코비안 식의 기준점."""
    s = 0.6
    H = np.array([[s, 0, 10.0], [0, s, -5.0], [0, 0, 1.0]])
    for pt in ([0.0, 0.0], [640.0, 400.0], [1279.0, 799.0]):
        assert abs(G.px_to_mm_scale(H, pt) - s) < 1e-12


def test_px_to_mm_scale_matches_finite_difference():
    """실셋업 H 에서 야코비안이 실제 변위와 맞는가 (수치미분 대조)."""
    img, base = _reference_pairs()
    H = G.solve_homography(img, base)
    for pt in ([400.0, 300.0], [640.0, 400.0], [900.0, 550.0]):
        p = np.array(pt)
        h = 1e-3
        du = (G.apply_homography(H, p + [h, 0]) - G.apply_homography(H, p - [h, 0])) / (2 * h)
        dv = (G.apply_homography(H, p + [0, h]) - G.apply_homography(H, p - [0, h])) / (2 * h)
        expect = np.sqrt(abs(np.linalg.det(np.column_stack([du, dv]))))
        assert abs(G.px_to_mm_scale(H, p) - expect) < 1e-6, pt


def test_px_to_mm_scale_varies_across_the_tilted_view():
    """45° 로 보면 가까운 쪽과 먼 쪽의 배율이 다르다 — 단일 숫자가 없다는 근거."""
    img, base = _reference_pairs()
    H = G.solve_homography(img, base)
    near = G.px_to_mm_scale(H, [640.0, 700.0])     # 이미지 아래 = 카메라에 가까운 쪽
    far = G.px_to_mm_scale(H, [640.0, 120.0])      # 이미지 위 = 먼 쪽
    assert far > near * 1.2, (near, far)


# ── 드리프트 ────────────────────────────────────────────

def _view_with_camera_rotation(deg_about_z):
    """기둥이 살짝 돌아간 상태의 H. 진동 감시의 시험 입력이다."""
    K, R, t = _synthetic_view()
    c, s = np.cos(np.radians(deg_about_z)), np.sin(np.radians(deg_about_z))
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    eye = -R.T @ t
    R2 = R @ Rz.T
    base = np.concatenate([_marker_corners(*c_) for c_ in MARKER_CENTERS.values()])
    return G.solve_homography(_project(K, R2, -R2 @ eye, base), base)


def test_drift_is_zero_for_identical_H():
    img, base = _reference_pairs()
    H = G.solve_homography(img, base)
    probe = np.array([[300.0, 250.0], [900.0, 250.0], [900.0, 600.0], [300.0, 600.0]])
    assert G.homography_drift(H, H, probe) < 1e-9


def test_drift_grows_with_camera_rotation():
    """0.1° 회전이 작업면에서 mm 급으로 나타나는가 — 문서의 0.7mm 눈금 확인."""
    probe = np.array([[300.0, 250.0], [900.0, 250.0], [900.0, 600.0], [300.0, 600.0]])
    H0 = _view_with_camera_rotation(0.0)
    d01 = G.homography_drift(H0, _view_with_camera_rotation(0.1), probe)
    d05 = G.homography_drift(H0, _view_with_camera_rotation(0.5), probe)

    assert 0.1 < d01 < 5.0, d01              # 자릿수 확인 (문서 눈금 ≈ 0.7mm)
    assert d05 > 3 * d01                     # 각도에 대체로 비례
    _record("0.1° 회전 → 작업면 드리프트", f"{d01:.3f}mm")
    _record("0.5° 회전 → 작업면 드리프트", f"{d05:.3f}mm")


def test_drift_rejects_empty_probe():
    H = np.eye(3)
    try:
        G.homography_drift(H, H, np.zeros((0, 2)))
    except ValueError:
        pass
    else:
        raise AssertionError("빈 probe 를 통과시켰다")


# ── cv2 교차 대조 (선택) ────────────────────────────────

def test_cross_check_against_cv2():
    """우리 DLT 와 cv2.findHomography 가 같은 답을 주는가.

    cv2 가 없으면 건너뛴다 — 이 모듈은 cv2 없이 서야 한다.

    🔴 **"성분이 똑같은가"를 묻지 않는다.** 둘 다 정규화 DLT 지만 SVD 구현과 정규화
       세부가 달라 마지막 자리가 갈린다(실측 1e-5mm 급 = 나노미터). 그 차이를 억지로
       0 으로 만들려 하면 임계만 헐거워진다. 물어야 할 것은 **어느 쪽이 더 잘 맞는가**다.

    노이즈를 섞어 최소자승이 실제로 일하게 만든 뒤, **RMS 잔차**를 견준다. RMS 를 고른
    이유는 그것이 양쪽이 실제로 최소화하는 양이기 때문이다 — 평균·최댓값은 어느 쪽이
    이길지 표본마다 갈리므로 대조 기준이 못 된다.

    📌 정련(_refine_homography) 을 붙이기 전에는 여기서 **200회 전부 우리가 열세**였다
       (0.1718mm 대비 +1.0e-3mm). cv2 는 DLT 뒤 LM 을 돌리는데 우리는 DLT 만 했기
       때문이다. 이 테스트가 그 차이를 잡아냈다.
    """
    try:
        import cv2
    except ImportError:                       # pragma: no cover
        _record("cv2 교차 대조", "건너뜀 (cv2 없음)")
        return

    rng = np.random.default_rng(9)
    img, base = _reference_pairs()
    probe = np.array([[300.0, 250.0], [900.0, 250.0], [900.0, 600.0],
                      [300.0, 600.0], [640.0, 400.0]])

    gaps, ours_worse = [], 0
    for _ in range(30):
        noisy = img + rng.normal(0, 0.2, img.shape)      # 최소자승이 일하게 만든다
        ours = G.solve_homography(noisy, base)
        theirs, _ = cv2.findHomography(noisy, base, 0)

        gaps.append(float(np.max(np.linalg.norm(
            G.apply_homography(ours, probe) - G.apply_homography(theirs, probe),
            axis=1))))
        rms_ours = float(np.sqrt(np.mean(G.homography_residual(ours, noisy, base) ** 2)))
        rms_cv2 = float(np.sqrt(np.mean(G.homography_residual(theirs, noisy, base) ** 2)))
        if rms_ours > rms_cv2 + 1e-9:
            ours_worse += 1

    assert ours_worse == 0, f"cv2 보다 RMS 잔차가 큰 경우 {ours_worse}/30"
    assert max(gaps) < 1e-2, max(gaps)     # 구현이 진짜로 갈라졌을 때를 잡는 그물
    _record(f"cv2 {cv2.__version__} 대비 최대 차", f"{max(gaps):.2e}mm")
    _record("cv2 보다 RMS 잔차가 컸던 횟수", f"{ours_worse}/30")


# ── 실행부 ──────────────────────────────────────────────

_NOTES: list[tuple[str, str]] = []


def _record(label: str, value: str) -> None:
    """테스트가 부수적으로 잰 수치. 통과/실패와 별개로 사람이 보는 값이다."""
    _NOTES.append((label, value))


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]

    print(f"cal_geometry 단위테스트 — {len(tests)}개\n" + "─" * 62)
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
