"""로봇 자세 취득과 터치 — 블록 B.

⚠️ **flange 기준으로만 일한다.** 손목 자세를 수직으로 고정하면 펜 끝과 flange 는 상수
   오프셋만 다르고, 그 오프셋은 호모그래피 H 가 흡수한다. 그래서 이 스크립트에는
   펜 TCP 캘리브레이션이 없다.

   📌 **2026-08-16 갱신 — L14 가 닫혔다.** 병렬 세션(`260816-알트도르퍼`)이 펜홀더 TCP 를
      실기 4점법으로 실측해 컨트롤러에 등록했다: **(-3.675, -1.725, 120.560) mm**.
      "tcp 를 못 믿는다"는 이유(같은 날 128.05 / 10 / 0mm 세 값)는 사라졌다.
      그래도 **flange 를 계속 기준으로 쓴다** — 이유가 바뀌었을 뿐 결론은 같다:
        · H 가 상수 오프셋을 흡수하므로 tcp 를 알아도 식이 더 간단해지지 않는다
        · 등록값은 컨트롤러(펜던트) 소유라 세션 밖에서 바뀔 수 있다. flange 는 안 바뀐다
        · 편심 xy(-3.675, -1.725) 는 **좌표계 회전이 미확정**이라 아직 방향을 못 믿는다
      단, 되읽기 경로는 생겼다 — `robot/convertPose` 가 poseType "tcp"/"flange" 를 둘 다
      받고 `hcr5_comm/tools/tool_probe.py` 가 **로봇을 안 움직이고** 현재 툴을 확인한다.
      블록 B 진입 전 점검에 쓸 것.

🔴 로봇을 자동으로 움직이는 함수는 아직 없다.
     · L21 — `move.joint.velocity` 단위 미확정. `movej` 자동 이동 금지
     · 📌 2026-08-16 추가 — **`movel` 은 실기 사용 정지**다. `poseTriple()` 이 tcp·flange
       슬롯에 같은 포즈를 복사하는데, 오프셋 0 이던 시절의 전제라 이제 120.628mm
       어긋난 모순 조합을 싣는다. (`hcr5_bridge/movel.md`)
   지금은 사람이 조그로 짚고 스크립트는 **자세를 읽기만** 한다 (prompt_touch).

🔴 **정밀도 요구가 실측으로 세졌다** — 펜홀더 스프링 행정이 **1.069mm** 뿐이다
   (같은 세션 실측). 완충이 1mm 라 종이 높이(작업평면 z)가 **±0.4mm 급**으로 맞아야
   한다. 이것이 build_work_plane 의 평면 잔차 임계를 정하는 첫 실물 근거다 (N4·N6).

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
    """현재 flange 자세 6값 (x,y,z,rx,ry,rz). tcp 가 아니다 — 머리주석 참조."""
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
