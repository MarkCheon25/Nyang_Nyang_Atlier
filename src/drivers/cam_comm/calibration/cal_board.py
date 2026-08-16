"""인쇄물 생성 — 블록 A.

카메라도 로봇도 없이 도는 유일한 블록이다. 인쇄해 두면 카메라가 오는 즉시 블록 B 로 들어간다.

기본 산출물 = **기준 마커 시트**. A4 한 장에 마커 4개(ID 0~3) + 재단선.
잘라서 작업대에 붙인다 — 종이 자리를 둘러싸도록 네 귀퉁이에.

  ┌─────────────────────────────┐
  │  [M0]                 [M1]  │   마커는 종이 바깥
  │      ┌───────────────┐      │   → 종이가 절대 안 가림
  │      │   A4 종이     │      │   → 종이 영역이 내삽 구간에 들어가 정확도가 가장 좋다
  │      └───────────────┘      │
  │  [M2]                 [M3]  │
  └─────────────────────────────┘

🔴 인쇄 함정 — 프린터의 "페이지에 맞춤"이 켜져 있으면 실제 치수가 몇 % 달라지고,
   그 오차가 H 에 스케일 오차로 그대로 박힌다. **100% 배율로 인쇄하고, 인쇄물을 자로 재서
   설정에 실측값을 넣을 것.** 시트에 검증용 자(스케일 바)를 함께 찍는다.
🔴 작업면에 평평하게 붙일 것. 들뜨면 그 오차가 그대로 들어간다.

설계 근거: cal_script.md §3.4 · cal_readme.md §1 블록 A
🚧 스켈레톤 — 함수 본문 미구현
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np


# ── 마커 낱장 ───────────────────────────────────────────

def render_marker(dictionary: Any, marker_id: int, size_mm: float,
                  dpi: int) -> np.ndarray:
    """마커 하나를 실치수(mm)대로 래스터화. 기본 50mm."""
    raise NotImplementedError


def add_quiet_zone(img: np.ndarray, margin_mm: float, dpi: int) -> np.ndarray:
    """마커 둘레 흰 여백. 없으면 검출이 급격히 나빠진다."""
    raise NotImplementedError


def add_cut_marks(img: np.ndarray, cfg: dict[str, Any]) -> np.ndarray:
    """재단선. 잘라 붙일 때 마커 방향이 헷갈리지 않게 기준변을 표시한다."""
    raise NotImplementedError


def add_label(img: np.ndarray, text: str, cfg: dict[str, Any]) -> np.ndarray:
    """ID·치수·생성일 각인. 인쇄물과 설정이 어긋나는 사고를 막는 이름표."""
    raise NotImplementedError


def add_scale_bar(img: np.ndarray, length_mm: float, dpi: int) -> np.ndarray:
    """인쇄 배율 검증용 자. 인쇄 후 이걸 재서 100% 인지 확인한다."""
    raise NotImplementedError


# ── 시트 ────────────────────────────────────────────────

def build_marker_sheet(cfg: dict[str, Any]) -> np.ndarray:
    """A4 한 장에 기준 마커 4개 + 재단선 + 스케일 바를 앉힌다."""
    raise NotImplementedError


def build_charuco_sheet(cfg: dict[str, Any]) -> np.ndarray:
    """ChArUco 보드 시트 — **조건부**. 정식 intrinsic(블록 A ④)으로 갈 때만.

    A4 / 5×7 / square 35mm / marker 26mm → 보드 실치 175×245mm, 여백 17.5·26mm.
    """
    raise NotImplementedError


def save_pdf(sheet: np.ndarray, path: str | Path, dpi: int) -> Path:
    """실치수가 보존되는 PDF 로 저장. 배율이 흐트러지면 인쇄물이 무의미해진다."""
    raise NotImplementedError


def run_board(cfg: dict[str, Any], kind: str = "markers") -> Path:
    """블록 A 인쇄물 절차 진입점. kind: markers | charuco."""
    raise NotImplementedError
