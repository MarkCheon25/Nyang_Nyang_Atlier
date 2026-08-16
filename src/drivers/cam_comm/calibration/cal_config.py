"""설정과 결과물 입출력.

설정값은 코드에 박지 않고 전부 여기를 거친다.
결과물(왜곡·기준·호모그래피)은 사람이 읽고 손으로 고칠 수 있는 YAML 로 남긴다.

설계 근거: cal_script.md §3.1
🚧 스켈레톤 — 함수 본문 미구현
"""

from __future__ import annotations

from pathlib import Path
from typing import Any


# ── 설정 ────────────────────────────────────────────────

def default_config() -> dict[str, Any]:
    """설정 기본값. 파일이 없을 때의 출발점이자 키 목록의 정본."""
    raise NotImplementedError


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    """설정 파일을 읽어 기본값 위에 덮는다. path 가 None 이면 기본 경로."""
    raise NotImplementedError


def validate_config(cfg: dict[str, Any]) -> None:
    """필수 키·값 범위 검사. 어긋나면 즉시 예외 — 반쯤 맞는 설정으로 진행하지 않는다."""
    raise NotImplementedError


# ── 결과물 ──────────────────────────────────────────────

def result_path(cfg: dict[str, Any], name: str) -> Path:
    """결과물 이름(distortion/reference/homography/paper)을 실제 경로로."""
    raise NotImplementedError


def save_result(cfg: dict[str, Any], name: str, data: dict[str, Any]) -> Path:
    """결과를 YAML 로 저장. 생성 시각·설정 해시를 함께 박아 출처를 남긴다."""
    raise NotImplementedError


def load_result(cfg: dict[str, Any], name: str) -> dict[str, Any]:
    """저장된 결과를 읽는다. 없으면 무엇을 먼저 돌려야 하는지 알려주는 예외."""
    raise NotImplementedError


def result_is_stale(cfg: dict[str, Any], name: str) -> bool:
    """결과가 현재 설정과 어긋나는지(설정 해시 불일치·유효기간 초과) 판정."""
    raise NotImplementedError


# ── 원자료 ──────────────────────────────────────────────

def raw_dir(cfg: dict[str, Any], block: str) -> Path:
    """블록별 원자료 디렉터리. 수집과 계산을 가르는 자리 (cal_script.md §3.6)."""
    raise NotImplementedError


def save_raw(cfg: dict[str, Any], block: str, tag: str, data: Any) -> Path:
    """수집 원자료를 떨군다. 임계가 바뀌어도 재수집 없이 재계산만 하도록."""
    raise NotImplementedError


def load_raw(cfg: dict[str, Any], block: str) -> list[Any]:
    """저장된 원자료 전량을 읽는다."""
    raise NotImplementedError
