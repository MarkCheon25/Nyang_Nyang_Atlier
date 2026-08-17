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
🚧 부분 구현 (2026-08-16, 세션 `260816-인서트너트`)
   ✅ board 경로만 — 장비가 필요 없는 유일한 블록이다 (카메라 미연결, cal_script.md §3.4 N2)
   🚧 distortion·reference·update·paper·verify 5개는 스켈레톤 그대로
"""

from __future__ import annotations

import argparse
import sys
from typing import Any

try:                                      # 스크립트로도, 패키지로도 부를 수 있게
    from . import cal_board, cal_config, cal_markers
except ImportError:                       # pragma: no cover
    import cal_board                      # type: ignore[no-redef]
    import cal_config                     # type: ignore[no-redef]
    import cal_markers                    # type: ignore[no-redef]


# ── 인자 ────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    """서브커맨드 파서. 공통 옵션 --config / --recompute / --dry-run."""
    # 공통 옵션은 부모 파서로 상속시킨다 — 안 그러면 서브커맨드 **앞**에서만 먹고
    # `cal.py board --dry-run` 이 "unrecognized arguments" 로 죽는다
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--config", metavar="PATH",
                        help="설정 YAML 경로 (기본: cal_config.yaml, 없으면 기본값)")
    common.add_argument("--recompute", action="store_true",
                        help="수집 없이 저장된 원자료로 계산만 다시 한다")
    common.add_argument("--dry-run", action="store_true",
                        help="파일을 쓰지 않고 무엇을 할지만 보고한다")

    p = argparse.ArgumentParser(
        prog="cal.py", parents=[common],
        description="HCR-5 카메라 캘리브레이션 — 평면 호모그래피 방식",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="블록별 선행 조건은 cal_script.md §2 참조.")

    sub = p.add_subparsers(dest="command", metavar="COMMAND", required=True)

    b = sub.add_parser("board", parents=[common],
                       help="[A] 인쇄물 생성 — 장비 불필요")
    b.add_argument("--kind", choices=("markers", "charuco"), default="markers",
                   help="markers = 기준 마커 시트(기본) / charuco = 정식 intrinsic 용")
    b.add_argument("-o", "--output", metavar="PATH",
                   help="출력 PDF 경로. 생략하면 paths.data_dir/board/ 아래")
    b.add_argument("--measured-bar", metavar="MM", type=float,
                   help="앞서 뽑은 시트의 스케일 바를 자로 잰 값(mm). "
                        "board.scale_bar_mm(기본 100) 과의 비로 프린터 배율을 계산해 "
                        "보정한다. 예: 85 를 재었으면 --measured-bar 85")
    b.add_argument("--print-scale", metavar="RATIO", type=float,
                   help="프린터 배율을 직접 준다(0.85 = 85%%). --measured-bar 와 택일")

    sub.add_parser("distortion", parents=[common], help="[A] 왜곡 점검 — 카메라")
    sub.add_parser("reference", parents=[common],
                   help="[B] 기준 확정+평면 — 카메라+로봇, 최초 1회")
    sub.add_parser("update", parents=[common], help="[C] H 갱신 — 카메라, 매 작업 전")
    sub.add_parser("paper", parents=[common], help="[D] 종이 위치 인식 — 카메라, 매 작업")
    sub.add_parser("verify", parents=[common], help="[E] 검증")
    return p


def resolve_config(args: argparse.Namespace) -> dict[str, Any]:
    """인자와 설정 파일을 합쳐 최종 설정으로. 인자가 파일을 이긴다."""
    cfg = cal_config.load_config(getattr(args, "config", None))

    # 🔴 인자로 들어온 값도 파일과 똑같이 validate 를 통과해야 한다.
    measured = getattr(args, "measured_bar", None)
    direct = getattr(args, "print_scale", None)
    if measured is not None and direct is not None:
        raise SystemExit("--measured-bar 와 --print-scale 은 함께 쓸 수 없다 — 하나만 줄 것")

    if measured is not None:
        nominal = float(cfg["board"]["scale_bar_mm"])
        if measured <= 0:
            raise SystemExit(f"--measured-bar 는 양수여야 한다: {measured}")
        cfg["board"]["print_scale"] = measured / nominal
    elif direct is not None:
        cfg["board"]["print_scale"] = float(direct)

    if measured is not None or direct is not None:
        cal_config.validate_config(cfg)
    return cfg


# ── 서브커맨드 ──────────────────────────────────────────

def cmd_board(args: argparse.Namespace, cfg: dict[str, Any]) -> int:
    """[A] 기준 마커 시트(또는 ChArUco) PDF 생성."""
    from pathlib import Path

    kind = getattr(args, "kind", "markers")
    dpi = int(cfg["board"]["dpi"])

    scale = float(cfg["board"].get("print_scale", 1.0))

    if kind == "markers":
        m = cfg["markers"]
        want = float(m["size_mm"])
        dictionary = cal_markers.make_dictionary(m["dictionary"])
        drawn = cal_board.rendered_marker_mm(dictionary, want / scale, dpi)
        on_paper = drawn * scale
        print(f"[A] 기준 마커 시트  {m['dictionary']}  "
              f"ids {', '.join(str(i) for i in m['reference_ids'])}")
        if abs(scale - 1.0) > 1e-9:
            print(f"    🖨  프린터 배율 보정 {scale * 100:.1f}% → 렌더를 "
                  f"{1 / scale:.4f}배로 키운다")
            print(f"    목표 {want:.3f}mm  →  그리는 치수 {drawn:.3f}mm  "
                  f"→  인쇄 후 예상 {on_paper:.3f}mm")
            print(f"    ⚠️ 이 시트는 **그 프린터 전용**이다 — 설정을 고치면 다시 뽑을 것")
        else:
            print(f"    명목 {want:.3f}mm → 렌더 {on_paper:.3f}mm @ {dpi}dpi "
                  f"(셀 배수 반올림, {(on_paper / want - 1) * 100:+.3f}%)")
        print(f"    quiet zone {float(m['quiet_zone_mm']):g}mm")
    else:
        c = cfg["charuco"]
        print(f"[A] ChArUco 시트  {c['squares_x']}x{c['squares_y']}  "
              f"square {float(c['square_mm']):g}mm  marker {float(c['marker_mm']):g}mm  "
              f"{c['dictionary']} @ {dpi}dpi")

    out = getattr(args, "output", None)
    if out is None:
        name = (cfg["board"]["output_name"] if kind == "markers"
                else cfg["charuco"]["output_name"])
        out = Path(cfg["paths"]["data_dir"]) / "board" / f"{name}.pdf"
    out = Path(out)

    if getattr(args, "dry_run", False):
        print(f"    --dry-run: {out} 를 쓰지 않고 끝낸다")
        return 0

    sheet = (cal_board.build_marker_sheet(cfg) if kind == "markers"
             else cal_board.build_charuco_sheet(cfg))
    path = cal_board.save_pdf(sheet, out, dpi)
    print(f"    → {path}  ({path.stat().st_size / 1024:.0f} KB)")
    if abs(scale - 1.0) > 1e-9:
        print(f"    🔴 **앞서 {scale * 100:.1f}% 를 재던 그 설정 그대로** 인쇄할 것 — "
              "지금 프린터를 고치면 보정이 반대로 작용한다.")
        print(f"    🔴 인쇄 후 스케일 바가 {float(cfg['board']['scale_bar_mm']):g}mm 로 "
              "나오면 성공. 아니면 그 값으로 --measured-bar 를 다시 줄 것.")
    else:
        print("    🔴 100% 배율로 인쇄하고, 시트의 스케일 바를 자로 재서 확인할 것.")
    return 0


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

    🚧 board 만 판정한다 — **선행 조건이 없는 유일한 명령**이고, 나머지는 요구할
       결과물의 스키마 자체가 아직 없다(cal_config 의 result 계열이 미구현).
    """
    if command == "board":
        return                              # 설정만 있으면 된다. 장비도 선행 결과도 없다
    raise NotImplementedError(
        f"'{command}' 의 선행 조건 판정은 아직 없다 — 블록 B~E 미구현")


_COMMANDS = {
    "board": cmd_board,
    "distortion": cmd_distortion,
    "reference": cmd_reference,
    "update": cmd_update,
    "paper": cmd_paper,
    "verify": cmd_verify,
}


def main(argv: list[str] | None = None) -> int:
    """진입점."""
    args = build_parser().parse_args(argv)

    try:
        cfg = resolve_config(args)
        check_prerequisites(cfg, args.command)
        return _COMMANDS[args.command](args, cfg)
    except NotImplementedError as e:
        # 미구현은 사고가 아니다. 무엇이 없는지만 한 줄로 알리고 스택은 안 뱉는다
        print(f"미구현: {e}", file=sys.stderr)
        return 2
    except (ValueError, FileNotFoundError, RuntimeError) as e:
        print(f"오류: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
