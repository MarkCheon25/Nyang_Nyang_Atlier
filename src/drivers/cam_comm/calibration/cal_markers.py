"""마커 검출 — 블록 B·C·D 공통 계층.

딕셔너리 DICT_4X4_50. 4×4 는 테두리 포함 6셀이라 셀이 크고,
45도 경사에서 유효 해상도가 cos45 = 0.71 배로 깎이는 이 셋업에 가장 유리하다.

ID 대역을 갈라 쓴다 — 같은 화면에 둘이 들어와도 안 섞인다.
  0~3   기준 마커 (작업대 영구 부착, 종이 자리를 둘러쌈)
  40~   종이 지그용 (블록 D, 도입 시)

설계 근거: cal_script.md §3.1
🚧 부분 구현 (2026-08-16, 세션 `260816-인서트너트`)
   ✅ make_dictionary · make_charuco_board — 객체 생성 2개. 블록 A 인쇄물이 이걸 쓴다
   🚧 검출 계열 8개 — 카메라가 없어 실물로 검증할 수 없다 (cal_script.md §3.4 N2)

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


def detect_markers(img: np.ndarray, dictionary: Any) -> tuple[np.ndarray, np.ndarray]:
    """마커 검출 → (ids, corners). 검출 실패는 예외가 아니라 빈 결과로 돌려준다."""
    raise NotImplementedError


def refine_corners_subpixel(img: np.ndarray, corners: np.ndarray) -> np.ndarray:
    """코너를 서브픽셀로 다듬는다.

    드리프트 감시의 감도가 여기서 정해진다 — 0.1px 급이면 작업면 0.1mm 수준까지 잡힌다.
    """
    raise NotImplementedError


# ── 조회·검사 ───────────────────────────────────────────

def corners_of(ids: np.ndarray, corners: np.ndarray, marker_id: int) -> np.ndarray | None:
    """특정 ID 마커의 코너 4점. 없으면 None."""
    raise NotImplementedError


def marker_center(corners_one: np.ndarray) -> np.ndarray:
    """마커 하나의 중심 픽셀."""
    raise NotImplementedError


def require_markers(ids: np.ndarray, expected: list[int]) -> None:
    """기대한 마커가 다 보이는지 확인. 하나라도 없으면 무엇이 안 보이는지 짚어 예외.

    가림·조명·시야 밖을 조용히 넘기면 H 가 적은 점으로 풀려 정밀도가 말없이 떨어진다.
    """
    raise NotImplementedError


def marker_corner_ids(marker_ids: list[int]) -> list[tuple[int, int]]:
    """(마커ID, 코너번호 0~3) 쌍 목록. 대응점의 이름표 노릇을 한다."""
    raise NotImplementedError


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
    """체커 코너·ID 검출 → (charuco_corners, charuco_ids)."""
    raise NotImplementedError
