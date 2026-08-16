"""마커 검출 — 블록 B·C·D 공통 계층.

딕셔너리 DICT_4X4_50. 4×4 는 테두리 포함 6셀이라 셀이 크고,
45도 경사에서 유효 해상도가 cos45 = 0.71 배로 깎이는 이 셋업에 가장 유리하다.

ID 대역을 갈라 쓴다 — 같은 화면에 둘이 들어와도 안 섞인다.
  0~3   기준 마커 (작업대 영구 부착, 종이 자리를 둘러쌈)
  40~   종이 지그용 (블록 D, 도입 시)

설계 근거: cal_script.md §3.1
🚧 스켈레톤 — 함수 본문 미구현
"""

from __future__ import annotations

from typing import Any

import numpy as np


# ── 딕셔너리·검출 ───────────────────────────────────────

def make_dictionary(cfg: dict[str, Any]) -> Any:
    """설정의 딕셔너리 이름으로 aruco 딕셔너리 객체 생성."""
    raise NotImplementedError


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
    """ChArUco 보드 객체. 정식 intrinsic 캘리브레이션으로 갈 때만 쓴다."""
    raise NotImplementedError


def detect_charuco(img: np.ndarray, board: Any) -> tuple[np.ndarray, np.ndarray]:
    """체커 코너·ID 검출 → (charuco_corners, charuco_ids)."""
    raise NotImplementedError
