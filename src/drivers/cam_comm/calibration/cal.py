#!/usr/bin/env python3
"""캘리브레이션 CLI 진입점.

    cal.py board      [--kind markers|charuco]   # [A] 인쇄물 생성   — 장비 불필요
    cal.py distortion                            # [A] 왜곡 점검     — 카메라
    cal.py reference                             # [B] 기준 확정+평면 — 카메라+로봇, 최초 1회
    cal.py update                                # [C] H 갱신        — 카메라, 매 작업 전
    cal.py paper                                 # [D] 종이 위치 인식 — 카메라, 매 작업
    cal.py verify                                # [E] 검증

🔴 수집과 계산을 한 명령에 묶지 않는다. 판정 임계가 아직 미정이라, 붙여 놓으면
   임계를 바꿀 때마다 로봇을 다시 돌려야 한다. 갈라두면 원자료 한 벌로 계산만 다시 한다.
   → 각 절차는 원자료를 먼저 떨구고, --recompute 로 계산만 다시 돌 수 있어야 한다.

설계 근거: cal_script.md · cal_readme.md
🚧 스켈레톤 — 함수 본문 미구현
"""

from __future__ import annotations

import argparse
from typing import Any


# ── 인자 ────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    """서브커맨드 파서. 공통 옵션 --config / --recompute / --dry-run."""
    raise NotImplementedError


def resolve_config(args: argparse.Namespace) -> dict[str, Any]:
    """인자와 설정 파일을 합쳐 최종 설정으로. 인자가 파일을 이긴다."""
    raise NotImplementedError


# ── 서브커맨드 ──────────────────────────────────────────

def cmd_board(args: argparse.Namespace, cfg: dict[str, Any]) -> int:
    """[A] 기준 마커 시트(또는 ChArUco) PDF 생성."""
    raise NotImplementedError


def cmd_distortion(args: argparse.Namespace, cfg: dict[str, Any]) -> int:
    """[A] 공장값 읽기 → 직선 테스트 → (필요시) 보정 후 재검사."""
    raise NotImplementedError


def cmd_reference(args: argparse.Namespace, cfg: dict[str, Any]) -> int:
    """[B] 펜 터치로 기준 마커 base 좌표와 작업평면을 확정. 최초 1회."""
    raise NotImplementedError


def cmd_update(args: argparse.Namespace, cfg: dict[str, Any]) -> int:
    """[C] 마커 촬영 → H 재계산 → 드리프트 판정."""
    raise NotImplementedError


def cmd_paper(args: argparse.Namespace, cfg: dict[str, Any]) -> int:
    """[D] 종이 위치·자세 인식."""
    raise NotImplementedError


def cmd_verify(args: argparse.Namespace, cfg: dict[str, Any]) -> int:
    """[E] 검증 리포트."""
    raise NotImplementedError


# ── 공통 ────────────────────────────────────────────────

def check_prerequisites(cfg: dict[str, Any], command: str) -> None:
    """그 명령이 요구하는 선행 결과가 있는지 본다.

    없으면 무엇을 먼저 돌려야 하는지 짚어 준다 (update 는 reference 를 요구하는 식).
    """
    raise NotImplementedError


def main(argv: list[str] | None = None) -> int:
    """진입점."""
    raise NotImplementedError


if __name__ == "__main__":
    raise SystemExit(main())
