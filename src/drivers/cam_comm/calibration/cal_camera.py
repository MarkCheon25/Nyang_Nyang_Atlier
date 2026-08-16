"""카메라 입출력과 왜곡 점검 — 블록 A.

RealSense D456, **RGB 스트림 하나만** 쓴다. depth 는 안 받는다
(그림은 평면 위에서만 일어나고, 평면 높이는 로봇 터치로 잡는 쪽이 훨씬 정확하다).

🔴 /dev/videoN 번호를 하드코딩하면 안 된다 — 열거 순서라 재연결마다 밀린다.
   판별법은 RealSense_D456.md §2 (인터페이스 문자열 **마지막 토큰**으로 가른다).
🟡 RGB 는 롤링 셔터다. 움직이는 중 찍으면 검출은 성공하고 자세만 조용히 틀어진다.
   반드시 로봇 정지 후 촬영할 것.

설계 근거: cal_script.md §3.1 · cal_readme.md §1 블록 A
🚧 스켈레톤 — 함수 본문 미구현
"""

from __future__ import annotations

from typing import Any

import numpy as np


# ── 스트림 ──────────────────────────────────────────────

def find_color_node(cfg: dict[str, Any]) -> str:
    """컬러 스트림의 /dev/videoN 을 sysfs 로 찾는다. 번호 추측 금지."""
    raise NotImplementedError


def open_camera(cfg: dict[str, Any]) -> Any:
    """카메라를 연다. 해상도는 1280×800 권장 (45도 경사에서 해상도가 깎이므로)."""
    raise NotImplementedError


def close_camera(cam: Any) -> None:
    """정리. 예외 경로에서도 반드시 불린다."""
    raise NotImplementedError


def grab_frame(cam: Any) -> np.ndarray:
    """컬러 프레임 1장."""
    raise NotImplementedError


def grab_stable_frame(cam: Any, warmup: int = 10) -> np.ndarray:
    """노출·화이트밸런스가 안정된 뒤의 프레임 1장. 첫 프레임은 못 믿는다."""
    raise NotImplementedError


def save_snapshot(img: np.ndarray, path: str) -> None:
    """원자료로 프레임을 떨군다. 재계산 때 다시 찍지 않으려고."""
    raise NotImplementedError


# ── 내부 파라미터·왜곡 (블록 A) ─────────────────────────

def read_factory_intrinsics(cam: Any) -> dict[str, Any]:
    """펌웨어의 공장 intrinsic 을 읽는다 (pyrealsense2 경유).

    ⚠️ D400 계열은 컬러 왜곡계수가 전부 0으로 보고되는 경우가 흔하다.
       렌즈가 완벽해서가 아니므로 이 값만 믿지 말고 check_straightness 로 눈검사할 것.
    """
    raise NotImplementedError


def check_straightness(img: np.ndarray, cfg: dict[str, Any]) -> dict[str, Any]:
    """직선 테스트 — 화면 **가장자리**에 걸친 곧은 물체가 휘는지 잰다.

    호모그래피는 직선을 직선으로 보내는 변환이라, 왜곡이 남아 있으면
    가운데는 맞고 가장자리에서 어긋난다. 그 여부를 여기서 가른다.
    """
    raise NotImplementedError


def undistort(img: np.ndarray, K: np.ndarray, D: np.ndarray) -> np.ndarray:
    """왜곡 보정. check_straightness 가 통과하면 이 단계 자체를 건너뛴다."""
    raise NotImplementedError


def run_distortion_check(cfg: dict[str, Any]) -> dict[str, Any]:
    """블록 A 절차 — ①공장값 읽기 → ②직선 테스트 → ③보정 후 재검사.

    ②나 ③에서 통과하면 거기서 끝낸다. 둘 다 실패할 때만 정식 intrinsic(④)으로 간다.
    결과는 distortion.yaml.
    """
    raise NotImplementedError


def calibrate_intrinsic_charuco(cfg: dict[str, Any]) -> dict[str, Any]:
    """④ 정식 intrinsic 캘리브레이션 — **조건부**. ③까지 실패했을 때만 부른다.

    ChArUco 보드 15~20장. 화면 네 귀퉁이를 채운 표본이 있어야 왜곡계수가 풀린다.
    """
    raise NotImplementedError
