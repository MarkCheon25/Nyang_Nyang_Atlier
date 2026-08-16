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

설계 근거: cal_script.md §3.4 · cal_readme.md §1 블록 B·C
🚧 스켈레톤 — 함수 본문 미구현
"""

from __future__ import annotations

from typing import Any

import numpy as np


# ── 블록 B — 기준 확정 (최초 1회) ───────────────────────

def touch_labels(cfg: dict[str, Any]) -> list[str]:
    """짚어야 할 기준점 목록을 만든다. 마커당 어느 코너를 짚을지가 여기서 정해진다."""
    raise NotImplementedError


def build_reference_map(touches: dict[str, np.ndarray],
                        cfg: dict[str, Any]) -> dict[str, Any]:
    """짚은 점 + 인쇄 규격 → 기준 마커 코너 전체의 base 좌표.

    마커 치수를 알므로 마커당 1~2코너만 짚어도 나머지 코너는 계산으로 전개된다.
    """
    raise NotImplementedError


def build_work_plane(touches: dict[str, np.ndarray]) -> dict[str, Any]:
    """같은 터치 데이터에서 작업평면을 뽑는다. 펜다운 z 의 출처."""
    raise NotImplementedError


def pen_down_z(plane: dict[str, Any], x: float, y: float,
               cfg: dict[str, Any]) -> float:
    """(x, y) 에서의 펜다운 높이 = 평면 z - δ.

    δ 는 볼펜이 파고드는 깊이(≈0.5~1mm). 스프링 홀더가 평면 오차를 일부 흡수하지만,
    이 셋업의 스프링은 약해 여유가 크지 않다 — 평면 정밀도가 여전히 중요하다.
    """
    raise NotImplementedError


def pen_up_z(plane: dict[str, Any], x: float, y: float,
             cfg: dict[str, Any]) -> float:
    """(x, y) 에서의 펜업 높이 = 평면 z + h. 깊이 센서와 무관하다."""
    raise NotImplementedError


def run_reference(cfg: dict[str, Any]) -> dict[str, Any]:
    """블록 B 절차 진입점 — 터치 수집 → 기준 맵 + 평면 → reference.yaml."""
    raise NotImplementedError


# ── 블록 C — H 갱신 (매 작업 전) ────────────────────────

def observe_markers(cfg: dict[str, Any]) -> dict[int, np.ndarray]:
    """기준 마커를 찍어 지금 이미지에서의 코너 좌표를 얻는다."""
    raise NotImplementedError


def match_correspondences(observed: dict[int, np.ndarray],
                          reference: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """관측 코너 ↔ 기준 base 좌표를 짝지어 (img_pts, base_pts) 로."""
    raise NotImplementedError


def compute_homography(cfg: dict[str, Any],
                       reference: dict[str, Any]) -> dict[str, Any]:
    """대응쌍 → H + 잔차. 잔차가 임계를 넘으면 마커가 들떴거나 왜곡이 남은 것이다."""
    raise NotImplementedError


def detect_drift(H_now: np.ndarray, reference: dict[str, Any],
                 cfg: dict[str, Any]) -> dict[str, Any]:
    """직전 H 대비 변위(mm)를 재고 임계와 견준다.

    이미지 1px ≈ 작업면 0.6mm(정면) / 0.9mm(45도)이고,
    기둥이 0.1° 돌면 400mm 거리에서 0.7mm 밀린다 — 병진보다 회전에 훨씬 민감하다.
    """
    raise NotImplementedError


def run_update(cfg: dict[str, Any]) -> dict[str, Any]:
    """블록 C 절차 진입점 — 촬영 → H 재계산 → 드리프트 판정 → homography.yaml."""
    raise NotImplementedError
