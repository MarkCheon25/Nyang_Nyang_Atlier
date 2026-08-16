"""마커 검출 — 블록 B·C·D 공통 계층.

딕셔너리 DICT_4X4_50. 4×4 는 테두리 포함 6셀이라 셀이 크고,
45도 경사에서 유효 해상도가 cos45 = 0.71 배로 깎이는 이 셋업에 가장 유리하다.

ID 대역을 갈라 쓴다 — 같은 화면에 둘이 들어와도 안 섞인다.
  0~3   기준 마커 (작업대 영구 부착, 종이 자리를 둘러쌈)
  40~   종이 지그용 (블록 D, 도입 시)

설계 근거: cal_script.md §3.1
✅ 구현 완료
   객체 생성 2개 (2026-08-16, 세션 `260816-인서트너트`)
   검출 계열 7개 (2026-08-16, 세션 `260816-브론치노`) — 합성 영상으로 검증(N2 우회)

🔴 **합성으로 검증했지 실물로는 못 했다**(N2). 합성에 없는 것 — 렌즈 왜곡·롤링셔터
   기울어짐·조명 불균일·마커 들뜸·IR 프로젝터 점무늬. 카메라가 오면 다시 잰다.

🔴 반환 규약을 OpenCV 와 다르게 **고정한다**. cv2 는 코너를 `(1,4,2) float32` 리스트로
   주는데, 여기서는 `ids (N,) int32` · `corners (N,4,2) float64` 배열로 통일한다.
   호출부가 zip 과 리스트 인덱싱을 섞어 쓰다 순서를 잃는 일이 실제로 잦고,
   ids 와 corners 의 i 번째가 같은 마커라는 것을 배열 축으로 못 박아 두는 편이 안전하다.

🔴 OpenCV 버전 분기가 여기 있다. 4.7 에서 aruco API 이름이 통째로 바뀌었고
   이 PC 는 **4.6.0**(구 API)이다. 컨테이너·다른 PC 는 4.7+ 일 수 있어 양쪽을 다 받는다.
     4.6 이하 : CharucoBoard_create(sx, sy, sq, mk, dict) · board.draw(size)
     4.7 이상 : CharucoBoard((sx, sy), sq, mk, dict)     · board.generateImage(size)
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


# ── 딕셔너리·검출 ───────────────────────────────────────

def make_dictionary(cfg_or_name: dict[str, Any] | str) -> Any:
    """설정의 딕셔너리 이름으로 aruco 딕셔너리 객체 생성.

    설정 절(`markers`) 대신 이름 문자열을 바로 줄 수도 있다 —
    ChArUco 는 기준 마커와 **다른 딕셔너리**를 쓰므로 (cal_config.py 참조)
    호출부가 어느 이름인지 고를 수 있어야 한다.
    """
    if isinstance(cfg_or_name, str):
        name = cfg_or_name
    else:
        name = cfg_or_name.get("markers", {}).get("dictionary")
        if name is None:
            raise ValueError("설정에 markers.dictionary 가 없다")

    const = getattr(cv2.aruco, name, None)
    if const is None:
        raise ValueError(f"이 OpenCV({cv2.__version__}) 에 없는 딕셔너리다: {name}")

    # getPredefinedDictionary 는 4.6·4.7+ 양쪽에 다 있다. 더 옛 버전만 Dictionary_get
    getter = getattr(cv2.aruco, "getPredefinedDictionary", None) \
        or getattr(cv2.aruco, "Dictionary_get", None)
    if getter is None:
        raise RuntimeError(
            f"cv2.aruco 에 딕셔너리 생성 함수가 없다 (OpenCV {cv2.__version__}). "
            "opencv-contrib-python 이 맞게 깔렸는지 확인할 것")
    return getter(const)


def _as_gray(img: np.ndarray) -> np.ndarray:
    """검출·정련이 요구하는 8비트 단채널로 맞춘다.

    🔴 RealSense 컬러 스트림은 **BGR 3채널**로 들어온다. aruco 는 컬러도 받아 주지만
       cornerSubPix 는 단채널만 받는다. 여기서 한 번 맞춰 두지 않으면 검출은 되는데
       정련에서만 터지고, 그 조합이 하필 "잘 도는 것처럼 보이다 정밀도만 나쁜" 상태다.
    """
    if img is None or getattr(img, "size", 0) == 0:
        raise ValueError("빈 이미지다")
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    elif img.ndim != 2:
        raise ValueError(f"2차원 또는 3채널 이미지여야 한다: shape={img.shape}")
    if img.dtype != np.uint8:
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    return img


def detect_markers(img: np.ndarray, dictionary: Any) -> tuple[np.ndarray, np.ndarray]:
    """마커 검출 → (ids, corners). 검출 실패는 예외가 아니라 빈 결과로 돌려준다.

    반환 — `ids (N,) int32` · `corners (N,4,2) float64`. i 번째끼리 같은 마커다.
    코너 순서는 aruco 규약 그대로 **TL, TR, BR, BL**(마커 자체 기준, 시계방향).

    🔴 **빈 결과를 예외로 만들지 않는다.** 가림·조명은 정상 운용 중에도 생기는 일이고,
       판단은 호출부(require_markers)가 한다. 여기서 던지면 "몇 개는 보였다"는 정보가
       사라져서 무엇이 안 보이는지 짚을 수 없다.
    """
    gray = _as_gray(img)

    detector_cls = getattr(cv2.aruco, "ArucoDetector", None)
    if detector_cls is not None:                       # 4.7+
        params = cv2.aruco.DetectorParameters()
        corners, ids, _ = detector_cls(dictionary, params).detectMarkers(gray)
    else:                                              # 4.6 이하
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary,
                                                  parameters=params)

    if ids is None or len(ids) == 0:
        return np.empty((0,), np.int32), np.empty((0, 4, 2), np.float64)

    return (np.asarray(ids, np.int32).reshape(-1),
            np.asarray(corners, np.float64).reshape(-1, 4, 2))


def refine_corners_subpixel(img: np.ndarray, corners: np.ndarray) -> np.ndarray:
    """코너를 서브픽셀로 다듬는다.

    드리프트 감시의 감도가 여기서 정해진다 — 0.1px 급이면 작업면 0.1mm 수준까지 잡힌다.

    🔴 **선택이 아니라 기본이다.** 합성 실측에서 정련이 H 잔차를 **1.8~2.9배** 줄인다
       (cal_simulate.py). 빼도 아무 데서도 안 터지고 정밀도만 조용히 나빠진다.

    🔴 탐색창이 이웃 코너까지 덮으면 두 코너가 서로를 끌어당겨 **정련이 오히려 나쁘게**
       만든다. 마커가 작게 찍힌 프레임(먼 거리·저해상도)에서 실제로 생긴다.
       그래서 창 크기를 고정하지 않고 **가장 짧은 변의 1/4**로 잡되 5px 을 넘기지 않는다.
    """
    C = np.asarray(corners, np.float64).reshape(-1, 4, 2)
    if len(C) == 0:
        return C

    gray = _as_gray(img)

    # 모든 마커의 모든 변 중 가장 짧은 것 — 여기에 창을 맞춰야 전부 안전하다
    edges = np.linalg.norm(np.diff(np.concatenate([C, C[:, :1]], axis=1), axis=1),
                           axis=2)
    half = int(np.clip(edges.min() / 4.0, 2, 5))

    pts = np.ascontiguousarray(C.reshape(-1, 1, 2).astype(np.float32))
    cv2.cornerSubPix(
        gray, pts, (half, half), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
    return pts.reshape(-1, 4, 2).astype(np.float64)


# ── 조회·검사 ───────────────────────────────────────────

def corners_of(ids: np.ndarray, corners: np.ndarray, marker_id: int) -> np.ndarray | None:
    """특정 ID 마커의 코너 4점. 없으면 None.

    🔴 같은 ID 가 두 번 검출되면 **예외로 올린다.** 정상 셋업에서는 있을 수 없고,
       생겼다면 인쇄물이 두 장 붙어 있거나 화면에 반사가 잡힌 것이다. 아무거나 하나를
       고르면 그 뒤로 전부 틀리는데 아무 데서도 안 터진다.
    """
    ids = np.asarray(ids).reshape(-1)
    hit = np.flatnonzero(ids == marker_id)
    if len(hit) == 0:
        return None
    if len(hit) > 1:
        raise ValueError(f"마커 ID {marker_id} 가 {len(hit)}번 검출됐다 — "
                         "인쇄물이 중복 부착됐거나 반사가 잡혔는지 확인할 것")
    return np.asarray(corners, np.float64).reshape(-1, 4, 2)[hit[0]]


def marker_center(corners_one: np.ndarray) -> np.ndarray:
    """마커 하나의 중심 픽셀.

    🔴 **네 코너의 평균이 아니다.** 사영변환은 중점을 보존하지 않는다 — 비스듬히 보면
       먼 쪽이 압축되므로, 정사각형 중심이 찍히는 자리는 코너 평균이 아니라
       **두 대각선의 교점**이다. 45도로 내려다보는 이 셋업에서 둘의 차이는 마커 크기의
       1% 안팎이고, 0.1px 을 다투는 판에서는 무시할 수 없다.
       (교점이 정확한 이유 — 대각선은 직선이고 사영변환은 직선과 교차를 보존한다.)
    """
    P = np.asarray(corners_one, np.float64).reshape(4, 2)

    # 대각선 P0-P2 와 P1-P3 의 교점. 동차좌표에서 직선은 두 점의 외적, 교점은 두 직선의 외적.
    h = np.hstack([P, np.ones((4, 1))])
    x = np.cross(np.cross(h[0], h[2]), np.cross(h[1], h[3]))
    if abs(x[2]) < 1e-12:                       # 대각선이 평행 — 코너가 퇴화했다
        return P.mean(axis=0)
    return x[:2] / x[2]


def require_markers(ids: np.ndarray, expected: list[int]) -> None:
    """기대한 마커가 다 보이는지 확인. 하나라도 없으면 무엇이 안 보이는지 짚어 예외.

    가림·조명·시야 밖을 조용히 넘기면 H 가 적은 점으로 풀려 정밀도가 말없이 떨어진다.

    🔴 **개수가 아니라 ID 로 본다.** 기준 마커 4개는 여유가 0 이라(N10) 하나만 빠져도
       H 가 안 풀리는데, 개수만 세면 종이 지그 마커(40~)가 대신 채워 통과할 수 있다.
    """
    seen = set(np.asarray(ids).reshape(-1).tolist())
    missing = [int(i) for i in expected if int(i) not in seen]
    if not missing:
        return
    raise LookupError(
        f"기준 마커 {missing} 가 안 보인다 (기대 {list(expected)}, 검출 {sorted(seen)}). "
        "가림·조명·시야 밖을 확인할 것 — 4개 중 하나만 빠져도 H 를 못 푼다(N10)")


def marker_corner_ids(marker_ids: list[int]) -> list[tuple[int, int]]:
    """(마커ID, 코너번호 0~3) 쌍 목록. 대응점의 이름표 노릇을 한다.

    이 순서가 `detect_markers` 의 `corners.reshape(-1, 2)` 순서와 **같아야** 대응쌍이
    맞는다. 기준 마커 4개면 16쌍이 나오고, 그게 블록 C 의 표준 입력이다.
    """
    return [(int(mid), k) for mid in marker_ids for k in range(4)]


# ── ChArUco (조건부 — 블록 A ④ 에서만) ──────────────────

def make_charuco_board(cfg: dict[str, Any]) -> Any:
    """ChArUco 보드 객체. 정식 intrinsic 캘리브레이션으로 갈 때만 쓴다.

    🔴 길이 단위는 **mm 로 통일한다.** OpenCV 예제는 보통 m 를 쓰지만, 이 스크립트는
       설정·인쇄물·로봇 좌표가 전부 mm 라 경계를 하나 더 만들지 않는다.
       보드를 그릴 때는 비율만 쓰이므로 단위와 무관하고, 검출·pose 로 넘어갈 때
       tvec 이 mm 로 나온다는 것만 기억하면 된다.
    """
    c = cfg["charuco"]
    dictionary = make_dictionary(c["dictionary"])
    sx, sy = int(c["squares_x"]), int(c["squares_y"])
    sq, mk = float(c["square_mm"]), float(c["marker_mm"])

    new_api = getattr(cv2.aruco, "CharucoBoard", None)
    # 4.6 에도 CharucoBoard 라는 이름이 있지만 그건 생성자가 아니라 반환 타입이다.
    # 실제로 만들 수 있는지는 CharucoBoard_create 유무로 가른다.
    legacy = getattr(cv2.aruco, "CharucoBoard_create", None)
    if legacy is not None:
        return legacy(sx, sy, sq, mk, dictionary)
    if new_api is not None:
        return new_api((sx, sy), sq, mk, dictionary)
    raise RuntimeError(
        f"cv2.aruco 에 ChArUco 보드 생성 경로가 없다 (OpenCV {cv2.__version__})")


def detect_charuco(img: np.ndarray, board: Any) -> tuple[np.ndarray, np.ndarray]:
    """체커 코너·ID 검출 → (charuco_corners, charuco_ids).

    반환 — `corners (M,2) float64` · `ids (M,) int32`. 검출 실패는 빈 배열이다.

    🔴 이름과 달리 여기서 쓰는 정밀 코너는 **아루코 코너가 아니라 체커 코너**다.
       아루코는 어느 칸인지(ID)만 알려주고, 좌표 정밀도는 흑백 체커 교차점에서 나온다.
       ChArUco 를 쓰는 이유가 이 분업이다.

    🔴 API 가 세 갈래다 — 4.6 은 `interpolateCornersCharuco`, 4.7+ 는 `CharucoDetector`,
       5.0 은 구 함수를 **삭제**했다. 셋 다 받는다. (cal_readme.md §2 버전 함정)
    """
    gray = _as_gray(img)
    empty = (np.empty((0, 2), np.float64), np.empty((0,), np.int32))

    detector_cls = getattr(cv2.aruco, "CharucoDetector", None)
    if detector_cls is not None:                       # 4.7+
        c_corners, c_ids, _, _ = detector_cls(board).detectBoard(gray)
        if c_ids is None or len(c_ids) == 0:
            return empty
        return (np.asarray(c_corners, np.float64).reshape(-1, 2),
                np.asarray(c_ids, np.int32).reshape(-1))

    interpolate = getattr(cv2.aruco, "interpolateCornersCharuco", None)
    if interpolate is None:                            # pragma: no cover
        raise RuntimeError(
            f"cv2.aruco 에 ChArUco 검출 경로가 없다 (OpenCV {cv2.__version__})")

    # 4.6 — 아루코를 먼저 찾고, 그 배치로 체커 교차점을 보간한다
    dictionary = getattr(board, "dictionary", None) or board.getDictionary()
    ids, corners = detect_markers(gray, dictionary)
    if len(ids) == 0:
        return empty

    n, c_corners, c_ids = interpolate(
        [c.reshape(1, 4, 2).astype(np.float32) for c in corners],
        ids.reshape(-1, 1), gray, board)
    if not n or c_ids is None:
        return empty
    return (np.asarray(c_corners, np.float64).reshape(-1, 2),
            np.asarray(c_ids, np.int32).reshape(-1))
