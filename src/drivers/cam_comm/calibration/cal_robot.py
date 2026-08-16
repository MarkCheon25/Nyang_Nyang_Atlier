"""로봇 자세 취득과 터치 — 블록 B.

⚠️ **flange 기준으로만 일한다.** tcp 는 활성 툴이 세션 사이에 바뀌어 못 믿는다
   (업무목록 L14 — 같은 날 128.05 / 10 / 0mm 세 값이 관측됐다).
   손목 자세를 수직으로 고정하면 펜 끝과 flange 는 상수 오프셋만 다르고,
   그 오프셋은 호모그래피 H 가 흡수한다. 그래서 펜 TCP 캘리브레이션이 필요 없다.

🔴 로봇을 자동으로 움직이는 함수는 아직 없다 — 업무목록 L21(`move.joint.velocity`
   단위 미확정)이 닫히기 전까지 자동 이동 금지. 지금은 사람이 조그로 짚고
   스크립트는 **자세를 읽기만** 한다 (prompt_touch).

통신은 hcr5_comm/tools/mqtt_cmd.py 의 query_pos() 경로를 재사용한다.

설계 근거: cal_script.md §3.3
🚧 스켈레톤 — 함수 본문 미구현
"""

from __future__ import annotations

from typing import Any

import numpy as np


# ── 연결 ────────────────────────────────────────────────

def connect_robot(cfg: dict[str, Any]) -> Any:
    """컨트롤러 MQTT 연결. ⚠️ 다른 세션이 버스를 쓰는 중인지 먼저 확인할 것."""
    raise NotImplementedError


def disconnect_robot(bus: Any) -> None:
    """정리."""
    raise NotImplementedError


# ── 읽기 ────────────────────────────────────────────────

def get_flange_pose(bus: Any) -> tuple[float, float, float, float, float, float]:
    """현재 flange 자세 6값 (x,y,z,rx,ry,rz). tcp 가 아니다."""
    raise NotImplementedError


def wait_settled(bus: Any, timeout: float = 5.0) -> None:
    """정지 확인. 움직이는 중 찍은 쌍은 전부 오염된다 (롤링 셔터)."""
    raise NotImplementedError


def check_wrist_vertical(pose: tuple, cfg: dict[str, Any]) -> bool:
    """손목이 수직 자세인지 검사.

    호모그래피 방식이 서는 전제다. 자세가 틀어진 채 짚은 점은 오프셋이 달라져
    H 를 오염시키므로, 터치 기록 전에 반드시 통과해야 한다.
    """
    raise NotImplementedError


# ── 터치 (수동) ─────────────────────────────────────────

def prompt_touch(label: str) -> None:
    """사람에게 '지금 이 점을 펜 끝으로 짚고 엔터'를 알린다.

    L21 이 닫힐 때까지의 수집 방식. 닫히면 move_to 자동 순회로 바꾼다.
    """
    raise NotImplementedError


def record_touch_point(bus: Any, label: str, cfg: dict[str, Any]) -> np.ndarray:
    """짚은 상태의 flange (x, y, z) 를 기록. 자세 검사·정지 확인을 통과해야 남긴다."""
    raise NotImplementedError


def collect_touch_points(bus: Any, labels: list[str],
                         cfg: dict[str, Any]) -> dict[str, np.ndarray]:
    """지정된 기준점들을 차례로 짚어 모은다. 원자료로 즉시 떨군다."""
    raise NotImplementedError


# ── 이동 (봉인) ─────────────────────────────────────────

def move_to(bus: Any, pose: tuple, cfg: dict[str, Any], dry_run: bool = True) -> None:
    """캘리브레이션 자세로 자동 이동.

    🔴 L21 이 닫히기 전까지 실기 호출 금지. dry_run 을 먼저 통과시킨다
       (movej.md §2.3 관례).
    """
    raise NotImplementedError
