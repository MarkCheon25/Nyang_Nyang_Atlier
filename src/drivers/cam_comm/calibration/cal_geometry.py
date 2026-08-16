"""좌표 변환과 호모그래피 수학 — 순수 함수만.

카메라도 로봇도 안 붙는 계층이라 **합성 데이터로 오프라인 단위테스트가 된다**.
카메라·로봇이 없는 동안 검증할 수 있는 유일한 부분이다.

🔴 회전 규약 — ZYX 오일러 `R = Rz(rz)·Ry(ry)·Rx(rx)`, 각도 단위 = 도.
   실기 잔차법으로 확정. 원본은 hcr5_bridge/include/hcr5_bridge/pose_convention.hpp 머리주석.
🔴 OpenCV 표현(rvec/tvec)과 로봇 규약이 만나는 지점은 이 파일 하나로 몬다.

🔴 **cv2 를 쓰지 않는다.** 의존성을 아끼려는 게 아니라, 이 계층이 검증의 바닥이기
   때문이다. cv2 로 풀고 cv2 로 검증하면 같은 구현을 두 번 보는 것이라 서로의 오류를
   가려 준다. 여기서는 정규화 DLT 를 직접 풀고, 테스트에서 cv2 와 **교차 대조**한다
   (cal_geometry_test.py — cv2 가 있을 때만 도는 선택 테스트).

길이 단위는 처음부터 끝까지 **mm** 다. 로봇이 mm 로 말하고 인쇄물도 mm 로 재기 때문이다.

설계 근거: cal_script.md §3.3
✅ 구현 완료 (2026-08-16, 세션 `260816-브론치노`) — 카메라·로봇 없이 검증됨
"""

from __future__ import annotations

import numpy as np

# 수치 바닥. 이보다 작은 값으로 나누면 결과가 의미를 잃는다.
_EPS = 1e-12


# ── 로봇 자세 ↔ 동차행렬 ────────────────────────────────

def pose_to_matrix(x: float, y: float, z: float,
                   rx: float, ry: float, rz: float) -> np.ndarray:
    """로봇 자세 6값 → 4×4 동차행렬. ZYX 오일러·도 단위.

    pose_convention.hpp 의 rotationFromRealDeg 와 **성분까지 같은 식**이다.
    양쪽이 갈리면 파이썬이 푼 좌표를 C++ 가 다른 자세로 읽는다 — 조용히 틀린다.
    """
    cx, sx = np.cos(np.radians(rx)), np.sin(np.radians(rx))
    cy, sy = np.cos(np.radians(ry)), np.sin(np.radians(ry))
    cz, sz = np.cos(np.radians(rz)), np.sin(np.radians(rz))

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                cy * cx],
    ]
    T[:3, 3] = (x, y, z)
    return T


def matrix_to_pose(T: np.ndarray) -> tuple[float, float, float, float, float, float]:
    """4×4 동차행렬 → 로봇 자세 6값. pose_to_matrix 와 왕복 항등이어야 한다.

    ZYX 를 되푸는 식 (위 행렬에서 그대로 읽는다):
        ry = asin(-R20)   ·   rx = atan2(R21, R22)   ·   rz = atan2(R10, R00)

    🔴 **짐벌락** — ry = ±90° 면 cy = 0 이라 R00·R10·R21·R22 가 전부 0 이 되고
       rx 와 rz 가 한 축으로 겹친다. 무한히 많은 (rx, rz) 조합이 같은 회전을 주므로
       관례대로 rz = 0 으로 고정하고 rx 에 몰아준다. 왕복 항등은 **행렬 기준으로만**
       성립하고 6값 기준으로는 깨진다 — 이건 표현의 한계지 버그가 아니다.
       손목 수직 고정(ry≈0) 셋업에서는 걸릴 일이 없지만, N8(기울임 확장)에서 되살아난다.
    """
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"4×4 동차행렬이어야 한다: shape={T.shape}")
    R = T[:3, :3]

    cy = float(np.hypot(R[0, 0], R[1, 0]))       # = |cos(ry)|
    ry = np.degrees(np.arctan2(-R[2, 0], cy))

    if cy < 1e-9:                                 # 짐벌락
        rx = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
        rz = 0.0
    else:
        rx = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
        rz = np.degrees(np.arctan2(R[1, 0], R[0, 0]))

    return (float(T[0, 3]), float(T[1, 3]), float(T[2, 3]),
            float(rx), float(ry), float(rz))


def invert(T: np.ndarray) -> np.ndarray:
    """동차행렬 역변환. R^T, -R^T·t.

    np.linalg.inv 를 쓰지 않는 이유 — 회전+병진이라는 구조를 알고 있으면 역이 닫힌
    형태로 나온다. 빠르기도 하지만 **수치 오차가 안 쌓인다**는 게 본질이다.
    """
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"4×4 동차행렬이어야 한다: shape={T.shape}")

    Rt = T[:3, :3].T
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = Rt
    out[:3, 3] = -Rt @ T[:3, 3]
    return out


def rvec_tvec_to_matrix(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """OpenCV 로드리게스 표현 → 동차행렬. 규약 경계는 여기서만 넘는다.

    회전축 방향 단위벡터 k, 회전각 θ = |rvec| 일 때
        R = I + sinθ·K + (1−cosθ)·K²      (K = skew(k))

    🔴 θ → 0 에서 sinθ/θ 와 (1−cosθ)/θ² 가 0/0 이 된다. 그대로 나누면 마커가 카메라
       정면에 놓였을 때(회전이 거의 없을 때) 결과가 NaN 으로 튄다 — 하필 가장 좋은
       조건에서 터진다. 테일러 전개로 갈아탄다.
    """
    r = np.asarray(rvec, dtype=np.float64).reshape(3)
    t = np.asarray(tvec, dtype=np.float64).reshape(3)

    theta = float(np.linalg.norm(r))
    K = np.array([[0.0, -r[2], r[1]],
                  [r[2], 0.0, -r[0]],
                  [-r[1], r[0], 0.0]], dtype=np.float64)

    if theta < 1e-8:                    # sinθ/θ, (1−cosθ)/θ² 의 테일러 2항
        a = 1.0 - theta ** 2 / 6.0
        b = 0.5 - theta ** 2 / 24.0
    else:
        a = np.sin(theta) / theta
        b = (1.0 - np.cos(theta)) / theta ** 2

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.eye(3) + a * K + b * (K @ K)
    T[:3, 3] = t
    return T


# ── 작업평면 ────────────────────────────────────────────

def fit_plane(points: np.ndarray) -> tuple[np.ndarray, float]:
    """터치 점 N개(N≥3) → 평면. 최소자승으로 (법선 n, 오프셋 d) 반환.

    평면의 정의는 **n·p = d, |n| = 1** 이다. z = ax+by+c 꼴로 풀지 않는다 —
    그 형태는 평면이 수직에 가까워지면 발산한다. 작업면은 수평이라 당장은
    문제없지만, 발산하는 표현을 굳이 고를 이유가 없다.

    SVD 로 중심을 지나는 최적 평면의 법선(최소 특이벡터)을 뽑는다. 이것은 점들의
    **평면까지 수직거리 제곱합**을 최소화한다 — z 방향 오차만 재는 최소자승과 다르고,
    비스듬히 짚은 점이 섞여도 덜 끌린다.

    🔴 법선 부호를 **위쪽(n_z > 0)으로 고정**한다. 펜다운/펜업이 평면 법선 기준
       ∓ 오프셋이라, 부호가 뒤집히면 펜이 종이를 뚫는 쪽으로 간다.
    """
    P = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(P) < 3:
        raise ValueError(f"평면을 풀려면 점이 3개 이상 필요하다: {len(P)}개")

    centroid = P.mean(axis=0)
    _, s, Vt = np.linalg.svd(P - centroid, full_matrices=False)
    n = Vt[-1]

    # 점들이 한 직선 위에 늘어서 있으면 법선이 한 방향으로 정해지지 않는다.
    # 두 번째 특이값이 죽어 있는 것이 그 신호다 — 조용히 아무 평면이나 주면 안 된다.
    if s[1] < 1e-9 * max(s[0], 1.0):
        raise ValueError("터치 점이 한 직선에 가깝다 — 평면이 정해지지 않는다. "
                         "작업면 위에서 서로 떨어진 세 방향으로 짚을 것")

    if n[2] < 0:                        # 법선은 항상 위쪽
        n = -n
    return n, float(n @ centroid)


def plane_residuals(points: np.ndarray, plane: tuple[np.ndarray, float]) -> np.ndarray:
    """각 점의 평면까지 거리. 터치가 튄 점을 골라내는 근거.

    **부호 있는 거리**(mm)를 준다 — + 는 법선 방향(위), − 는 아래.
    절댓값만 주면 "종이가 휘었다"(한쪽으로 쏠림)와 "한 번 잘못 짚었다"(한 점만 튐)를
    구별할 수 없다. 판정 임계는 아직 미정이다(N4).
    """
    P = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    n, d = plane
    return P @ np.asarray(n, dtype=np.float64).reshape(3) - float(d)


def plane_z_at(plane: tuple[np.ndarray, float], x: float, y: float) -> float:
    """평면 위 (x, y) 에서의 z. 펜다운 높이를 여기서 얻는다.

    n·p = d 를 z 에 대해 푼다:  z = (d − n_x·x − n_y·y) / n_z

    🔴 n_z ≈ 0 이면 평면이 수직이라 (x, y) 위의 z 가 하나로 안 정해진다. 작업면이
       수직일 리 없으니 이건 계산 오류가 아니라 **터치 데이터가 잘못됐다는 신호**다.
    """
    n, d = plane
    n = np.asarray(n, dtype=np.float64).reshape(3)
    if abs(n[2]) < 1e-6:
        raise ValueError(f"작업평면이 수직에 가깝다 (n_z={n[2]:.3e}) — "
                         "터치 점을 다시 볼 것")
    return float((float(d) - n[0] * x - n[1] * y) / n[2])


def project_to_plane(point: np.ndarray, plane: tuple[np.ndarray, float]) -> np.ndarray:
    """점을 평면 위로 정사영.

    p' = p − (n·p − d)·n — 즉 부호 있는 거리만큼 법선 반대로 되민다.
    점 하나든 (N,3) 묶음이든 받는다.
    """
    P = np.asarray(point, dtype=np.float64)
    single = (P.ndim == 1)
    P = P.reshape(-1, 3)

    n, d = plane
    n = np.asarray(n, dtype=np.float64).reshape(3)
    out = P - np.outer(P @ n - float(d), n)
    return out[0] if single else out


# ── 호모그래피 ──────────────────────────────────────────

def _normalize_pts(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Hartley 정규화 — 중심을 원점으로, 평균 거리를 √2 로.

    🔴 이걸 빼면 DLT 가 **말없이 나빠진다.** 픽셀 좌표는 0~1280, 작업면 좌표는
       0~300 이라 A 행렬의 열 크기가 자릿수로 갈리고, 최소 특이벡터가 큰 열에 끌린다.
       터지지 않고 잔차만 커져서 "원래 이 정도인가 보다" 하고 넘어가게 된다.
    """
    c = pts.mean(axis=0)
    d = np.linalg.norm(pts - c, axis=1)
    mean_d = float(d.mean())
    if mean_d < _EPS:
        raise ValueError("대응점이 한 점에 뭉쳐 있다")

    s = np.sqrt(2.0) / mean_d
    T = np.array([[s, 0.0, -s * c[0]],
                  [0.0, s, -s * c[1]],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    return (pts - c) * s, T


def _refine_homography(H: np.ndarray, img: np.ndarray, base: np.ndarray,
                       iters: int = 20) -> np.ndarray:
    """DLT 해를 기하 오차 기준으로 정련한다 — 가우스-뉴턴, 8자유도(h22 = 1 고정).

    🔴 **DLT 가 최소화하는 것은 대수 오차이지 mm 가 아니다.** 대수 오차는 동차좌표의
       스케일 w 가 섞인 양이라, 화면 위치에 따라 가중이 달라진다. 우리가 판정에 쓰는
       값은 mm 단위 재투영 오차(homography_residual)이므로, 판정하는 양을 직접
       최소화해야 앞뒤가 맞는다. cv2.findHomography 도 method=0 에서 같은 이유로
       DLT 뒤에 LM 을 돌린다 — 이걸 빼면 우리 잔차가 계통적으로 0.6% 쯤 크게 나온다
       (합성 실측: 0.1718mm 대비 +1.0e-3mm, 200회 전부).

    🔴 **줄어들 때만 채택한다.** 발산하거나 정체하면 DLT 해를 그대로 돌려주므로
       정련이 결과를 나쁘게 만드는 경우가 없다 — 새 실패 모드를 안 만든다.
    """
    def rms(M: np.ndarray) -> float:
        hom = np.hstack([img, np.ones((len(img), 1))]) @ M.T
        w = hom[:, 2]
        if np.any(np.abs(w) < _EPS) or not np.isfinite(hom).all():
            return np.inf
        return float(np.sqrt(np.mean(np.sum((hom[:, :2] / w[:, None] - base) ** 2,
                                            axis=1))))

    best, best_rms = H.copy(), rms(H)
    u, v = img[:, 0], img[:, 1]

    for _ in range(iters):
        h = best.ravel()
        w = h[6] * u + h[7] * v + h[8]
        if np.any(np.abs(w) < _EPS):
            break
        X = (h[0] * u + h[1] * v + h[2]) / w
        Y = (h[3] * u + h[4] * v + h[5]) / w

        J = np.zeros((2 * len(img), 8), dtype=np.float64)
        J[0::2, 0], J[0::2, 1], J[0::2, 2] = u / w, v / w, 1.0 / w
        J[0::2, 6], J[0::2, 7] = -X * u / w, -X * v / w
        J[1::2, 3], J[1::2, 4], J[1::2, 5] = u / w, v / w, 1.0 / w
        J[1::2, 6], J[1::2, 7] = -Y * u / w, -Y * v / w

        r = np.empty(2 * len(img), dtype=np.float64)
        r[0::2], r[1::2] = X - base[:, 0], Y - base[:, 1]

        try:
            step = np.linalg.lstsq(J, r, rcond=None)[0]
        except np.linalg.LinAlgError:            # pragma: no cover
            break
        if not np.isfinite(step).all():
            break

        cand = best.copy()
        cand.ravel()[:8] -= step
        cand_rms = rms(cand)
        if not (cand_rms < best_rms - 1e-15):    # 더 안 좋아지면 거기서 멈춘다
            break
        best, best_rms = cand, cand_rms

    return best


def solve_homography(img_pts: np.ndarray, base_pts: np.ndarray) -> np.ndarray:
    """이미지 픽셀 ↔ 작업평면 base(x, y) 대응쌍(≥4) → 3×3 H.

    이 H 하나가 intrinsic·T_base→cam·펜 TCP 오프셋을 전부 흡수한다.

    정규화 DLT — 대응쌍마다 두 줄을 세우고 A·h = 0 의 최소 특이벡터를 h 로 잡는다.
    4쌍이면 정확해, 그보다 많으면 최소자승해다. 기준 마커 4개 × 코너 4점 = 16쌍이
    표준 입력이라 늘 과결정이다.

    🔴 **4점이 최소이고 여유가 0 이다**(N10). 마커 하나가 가려지면 3개 × 4코너 = 12쌍이
       남지만, 그 12쌍이 전부 종이 한쪽에 몰려 있어 반대편이 외삽이 된다. 개수만 보고
       통과시키지 말 것 — 판정은 잔차(homography_residual)로 한다.

    반환 H 는 H[2,2] = 1 로 정규화한다 (cv2.findHomography 와 같은 관례).
    """
    img = np.asarray(img_pts, dtype=np.float64).reshape(-1, 2)
    base = np.asarray(base_pts, dtype=np.float64).reshape(-1, 2)
    if len(img) != len(base):
        raise ValueError(f"대응쌍 개수가 다르다: 이미지 {len(img)} vs base {len(base)}")
    if len(img) < 4:
        raise ValueError(f"H 를 풀려면 대응쌍이 4개 이상 필요하다: {len(img)}개 (N10)")
    if not (np.isfinite(img).all() and np.isfinite(base).all()):
        raise ValueError("대응점에 NaN/inf 가 있다")

    src, T1 = _normalize_pts(img)
    dst, T2 = _normalize_pts(base)

    A = np.zeros((2 * len(src), 9), dtype=np.float64)
    for i, ((u, v), (X, Y)) in enumerate(zip(src, dst)):
        A[2 * i] = (u, v, 1.0, 0.0, 0.0, 0.0, -X * u, -X * v, -X)
        A[2 * i + 1] = (0.0, 0.0, 0.0, u, v, 1.0, -Y * u, -Y * v, -Y)

    _, s, Vt = np.linalg.svd(A)
    # 최소 특이값이 그 앞 값과 붙어 있으면 해가 하나로 안 정해진다 — 세 점이
    # 한 직선에 있거나 마커가 한 줄로 늘어선 배치가 그렇다.
    if s[-2] < 1e-9 * s[0]:
        raise ValueError("대응점 배치가 퇴화했다 (한 직선에 가깝다) — H 가 정해지지 않는다")

    H = np.linalg.inv(T2) @ Vt[-1].reshape(3, 3) @ T1

    if abs(H[2, 2]) < _EPS:
        raise ValueError("H[2,2] 가 0 이다 — 대응점 배치를 다시 볼 것")
    H /= H[2, 2]

    if abs(np.linalg.det(H)) < _EPS:
        raise ValueError("H 가 특이행렬이다 — 대응점 배치를 다시 볼 것")

    # DLT 는 대수 오차의 해다. 우리가 판정에 쓰는 mm 잔차로 한 번 더 정련한다.
    H = _refine_homography(H, img, base)
    return H / H[2, 2]


def apply_homography(H: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """H 로 점을 옮긴다. 이미지 → base 방향.

    점 하나든 (N,2) 묶음이든 받는다. 반환 형태는 입력을 따라간다.

    🔴 동차 성분 w 가 0 이면 그 점은 무한원점으로 간다 — 카메라 시선과 평행한
       방향이라는 뜻이다. 실제 시야 안의 점에서는 안 생기지만, 잘못 검출된 코너가
       섞이면 생길 수 있다. 조용히 거대한 좌표를 내놓지 않고 예외로 올린다.
    """
    P = np.asarray(pts, dtype=np.float64)
    single = (P.ndim == 1)
    P = P.reshape(-1, 2)

    H = np.asarray(H, dtype=np.float64)
    if H.shape != (3, 3):
        raise ValueError(f"H 는 3×3 이어야 한다: shape={H.shape}")

    hom = np.hstack([P, np.ones((len(P), 1))]) @ H.T
    w = hom[:, 2]
    if np.any(np.abs(w) < _EPS):
        raise ValueError("H 가 어떤 점을 무한원점으로 보낸다 — 코너 검출을 다시 볼 것")

    out = hom[:, :2] / w[:, None]
    return out[0] if single else out


def homography_residual(H: np.ndarray, img_pts: np.ndarray,
                        base_pts: np.ndarray) -> np.ndarray:
    """대응쌍별 잔차(mm). H 가 믿을 만한지의 1차 근거.

    H 로 옮긴 이미지 점과 로봇으로 짚은 base 점 사이의 거리다. 평균만 보지 말고
    **최댓값도 볼 것** — 마커 하나가 들떠 있으면 그 4점만 크게 튄다.

    참고 눈금: 합성(45°·400mm·1280×800·서브픽셀)에서 평균 0.348mm 가 나왔다.
    그것은 인쇄물과 검출 알고리즘의 상한이고, 실기에는 렌즈 왜곡·마커 들뜸·조명·
    로봇 반복도가 더 얹힌다. 판정 임계는 미정(N4).
    """
    got = apply_homography(H, img_pts)
    base = np.asarray(base_pts, dtype=np.float64).reshape(-1, 2)
    if len(got) != len(base):
        raise ValueError(f"대응쌍 개수가 다르다: {len(got)} vs {len(base)}")
    return np.linalg.norm(got - base, axis=1)


def homography_drift(H_ref: np.ndarray, H_now: np.ndarray,
                     probe_pts: np.ndarray) -> float:
    """두 H 가 같은 점을 얼마나 다른 곳으로 보내는지 — 최대 변위(mm).

    기둥이 밀렸는지 판정하는 값. 카메라 자세각 변화에 특히 민감하다
    (400mm 거리에서 0.1° 회전 ≈ 작업면 0.7mm).

    🔴 **H 의 성분을 직접 빼서 비교하면 안 된다.** H 는 스케일 자유도가 있고 성분마다
       단위가 달라(좌상 2×2 는 mm/px, 3행은 1/px) 성분 차이는 물리적 의미가 없다.
       점을 실제로 옮겨 보고 그 변위를 mm 로 재는 것만이 해석 가능한 값이다.

    🔴 probe_pts 는 **종이가 놓이는 영역의 픽셀 좌표**를 줄 것. 시야 구석에서 재면
       실제로 그림을 그리는 곳과 무관한 값이 나온다.

    평균이 아니라 **최댓값**을 주는 이유 — 회전으로 밀리면 한쪽 끝이 가장 많이
    움직이는데, 평균을 내면 반대쪽이 그걸 희석한다.
    """
    P = np.asarray(probe_pts, dtype=np.float64).reshape(-1, 2)
    if len(P) == 0:
        raise ValueError("probe_pts 가 비어 있다")
    return float(np.max(np.linalg.norm(
        apply_homography(H_ref, P) - apply_homography(H_now, P), axis=1)))


def px_to_mm_scale(H: np.ndarray, at_pt: np.ndarray) -> float:
    """해당 지점에서 이미지 1px 이 작업면 몇 mm 인지. 정밀도 보고용.

    H 는 선형이 아니라 지점마다 배율이 다르다 — 45° 로 내려다보면 가까운 쪽과 먼 쪽이
    자릿수까지는 아니어도 확연히 갈린다. 그래서 "이 카메라는 1px = ?mm" 라는 단일
    숫자는 존재하지 않고, **지점을 찍어서** 물어야 한다.

    H 의 야코비안 J (2×2, 단위 mm/px) 를 구해 √|det J| 를 준다 —
    1px×1px 면적이 작업면에서 몇 mm² 가 되는지의 한 변, 즉 두 주방향 배율의 기하평균이다.

    🔴 **경사 셋업에서는 두 주방향 배율이 다르다.** 45° 면 한 축이 cos45=0.71 배로
       압축돼 최악 방향 배율이 이 값보다 최대 1.2 배쯤 크다. 이 숫자는 평균적 정밀도이지
       보장값이 아니다.
    """
    H = np.asarray(H, dtype=np.float64)
    if H.shape != (3, 3):
        raise ValueError(f"H 는 3×3 이어야 한다: shape={H.shape}")

    u, v = np.asarray(at_pt, dtype=np.float64).reshape(2)
    w = H[2, 0] * u + H[2, 1] * v + H[2, 2]
    if abs(w) < _EPS:
        raise ValueError("H 가 그 점을 무한원점으로 보낸다")

    X = (H[0, 0] * u + H[0, 1] * v + H[0, 2]) / w
    Y = (H[1, 0] * u + H[1, 1] * v + H[1, 2]) / w

    J = np.array([[H[0, 0] - X * H[2, 0], H[0, 1] - X * H[2, 1]],
                  [H[1, 0] - Y * H[2, 0], H[1, 1] - Y * H[2, 1]]]) / w
    return float(np.sqrt(abs(np.linalg.det(J))))
