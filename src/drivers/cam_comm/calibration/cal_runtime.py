"""종이 위치 인식과 검증 — 블록 D·E. 캘리브레이션 결과를 실제로 쓰는 쪽.

블록 D (매 작업)
    종이를 대충 놓아도 되게 만드는 부분. 카메라가 종이를 찾아 base 좌표로 옮긴다.
    H 가 이미 이미지 ↔ 작업평면을 잇고 있으므로, 종이 인식은 그 평면 위 2D 문제로 끝난다.

블록 E (수시)
    알려진 점이 재현되는지 재서 파이프라인이 이 값을 믿어도 되는지의 근거를 만든다.

🔴 미정 — 종이를 마커로 잡을지(ID 40~), 외곽선으로 잡을지. 지그 제작과 함께 정한다.
   외곽선은 부착물이 없어 편하지만 흰 종이 / 흰 작업대면 대비가 안 나온다.

설계 근거: cal_script.md §3.4 · cal_readme.md §1 블록 D·E
🚧 스켈레톤 — 함수 본문 미구현
"""

from __future__ import annotations

from typing import Any

import numpy as np


# ── 블록 D — 종이 위치 인식 ─────────────────────────────

def detect_paper_by_markers(img: np.ndarray, cfg: dict[str, Any]) -> np.ndarray | None:
    """지그 마커(ID 40~)로 종이 네 모서리의 이미지 좌표를 찾는다."""
    raise NotImplementedError


def detect_paper_by_contour(img: np.ndarray, cfg: dict[str, Any]) -> np.ndarray | None:
    """외곽선으로 종이를 찾는다. 대비가 안 나오면 실패를 값으로 돌려준다."""
    raise NotImplementedError


def paper_corners_to_base(corners_px: np.ndarray, H: np.ndarray) -> np.ndarray:
    """종이 모서리 픽셀 → 작업평면 base 좌표. H 한 번 적용으로 끝난다."""
    raise NotImplementedError


def paper_frame(corners_base: np.ndarray) -> dict[str, Any]:
    """종이 좌표계 — 원점·회전·실측 치수. 스트로크 계획(F2·F3)이 쓰는 값."""
    raise NotImplementedError


def check_paper_sanity(frame: dict[str, Any], cfg: dict[str, Any]) -> dict[str, Any]:
    """검출된 종이가 A4(210×297mm)와 맞는지 대조. 어긋나면 오검출이다."""
    raise NotImplementedError


def paper_to_base(pt_paper: np.ndarray, frame: dict[str, Any]) -> np.ndarray:
    """종이 좌표(mm) → 로봇 base 좌표. 그림 그릴 때 매 점이 거치는 변환."""
    raise NotImplementedError


def run_paper(cfg: dict[str, Any]) -> dict[str, Any]:
    """블록 D 절차 진입점 — 촬영 → 종이 검출 → 좌표계 산출 → paper.yaml."""
    raise NotImplementedError


# ── 블록 E — 검증 ───────────────────────────────────────

def verify_known_points(cfg: dict[str, Any]) -> dict[str, Any]:
    """알려진 점을 이미지에서 base 로 옮겨 실측과 견준다. 오차 분포를 낸다."""
    raise NotImplementedError


def verify_round_trip(cfg: dict[str, Any]) -> dict[str, Any]:
    """base → 이미지 → base 왕복 오차. 순수 계산이라 장비 없이도 돈다."""
    raise NotImplementedError


def verify_repeatability(cfg: dict[str, Any], n: int = 5) -> dict[str, Any]:
    """같은 장면을 반복 촬영해 H 가 얼마나 흔들리는지 — 진동의 실측치."""
    raise NotImplementedError


def report(results: dict[str, Any]) -> str:
    """사람이 읽는 오차 리포트. 파이프라인이 이 값을 믿어도 되는지의 근거."""
    raise NotImplementedError


def run_verify(cfg: dict[str, Any]) -> dict[str, Any]:
    """블록 E 절차 진입점."""
    raise NotImplementedError
