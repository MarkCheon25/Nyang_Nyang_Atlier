#!/usr/bin/env python3
"""HCR-5 MoveIt2 설정 검증 — 손보정 값이 살아 있는지 확인한다.

왜 필요한가
-----------
MoveIt Setup Assistant 를 재실행하면 config/ 아래 파일이 **전부 새로 생성**되면서
손으로 넣은 값 두 가지가 조용히 초기화된다. 둘 다 증상이 "계획은 되는데 실행이 안 됨"
이라 원인을 찾기까지 시간이 걸린다.

  1) joint_limits.yaml     has_acceleration_limits:false / max_acceleration:0 로 되돌아감
     → "No acceleration limit was defined for joint joint_1!"
       AddTimeOptimalParameterization 실패 (URDF <limit> 에는 가속도 항목이 아예 없다)

  2) ros2_controllers.yaml command_interfaces / state_interfaces 가 빈 [] 로 되돌아감
     → "Action client not connected to action server:
        hcr_arm_controller/follow_joint_trajectory"
       컨트롤러가 configure 에 실패해 아예 뜨지 않는다

이 스크립트는 moveit_config **패키지 바깥**(src/moveit2/tools/)에 둔다. Setup Assistant 가
건드리지 않는 위치라 재생성에도 살아남는다.

사용법
------
  python3 src/moveit2/tools/check_moveit_config.py
  echo $?        # 0 = 전부 통과, 1 = 문제 있음

Setup Assistant 를 다시 돌린 직후에는 **반드시** 한 번 실행할 것.
"""
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("PyYAML 이 필요합니다:  pip3 install pyyaml  (또는 apt install python3-yaml)")

# ── 기대값 ────────────────────────────────────────────────────────────────────
# 출처: HCR-5 User Manual v2.0 Appendix F(제원) · G(정지거리)
MAX_VELOCITY = 3.1416       # rad/s = 180°/s  (공식값)
MAX_ACCEL = 3.5             # rad/s² ≈ 200°/s²  (정지거리 역산 추정치 — 벤더 확인 대상)
JOINT3_UPPER = 2.879793     # ±165°  (J3 만 별도 제한이 실제로 존재)
FULL_TURN = 6.283185        # ±360°
TIP_LINK = "pen_tip"
CMD_IF = ["position"]
STATE_IF = ["position", "velocity"]
JOINTS = [f"joint_{i}" for i in range(1, 7)]

WS = Path(__file__).resolve().parent.parent / "ws_moveit2" / "src"
DESC = WS / "hcr5_description"
MCFG = WS / "hcr5_moveit_config"

_fail = []
_warn = []


def ok(msg):
    print(f"  \033[32m✓\033[0m {msg}")


def bad(msg, fix=""):
    print(f"  \033[31m✗\033[0m {msg}")
    _fail.append((msg, fix))


def warn(msg):
    print(f"  \033[33m!\033[0m {msg}")
    _warn.append(msg)


def near(a, b, tol=1e-3):
    return abs(float(a) - float(b)) < tol


def load_yaml(p):
    if not p.exists():
        bad(f"파일 없음: {p}")
        return None
    return yaml.safe_load(p.read_text())


# ── A. URDF 관절 한계 (hcr5_description) ──────────────────────────────────────
def check_urdf_limits():
    print("\n[A] URDF 관절 한계  hcr5_description/urdf/hcr5_arm.xacro")
    f = DESC / "urdf" / "hcr5_arm.xacro"
    if not f.exists():
        bad(f"파일 없음: {f}")
        return
    text = f.read_text()

    # <joint name="joint_N" ...> ... <limit upper=".." lower=".." velocity=".."/>
    blocks = re.findall(
        r'<joint name="(joint_\d)"[^>]*>(.*?)</joint>', text, re.S)
    found = {name: body for name, body in blocks}

    missing = [j for j in JOINTS if j not in found]
    if missing:
        bad(f"조인트 누락: {missing}")
        return

    for j in JOINTS:
        m = re.search(r'<limit\s+upper="([-\d.]+)"\s+lower="([-\d.]+)"[^>]*velocity="([-\d.]+)"',
                      found[j])
        if not m:
            bad(f"{j}: <limit> 파싱 실패")
            continue
        upper, lower, vel = float(m.group(1)), float(m.group(2)), float(m.group(3))

        if not near(vel, MAX_VELOCITY):
            bad(f"{j}: velocity={vel} (기대 {MAX_VELOCITY})",
                f'{j} 의 <limit ... velocity="{MAX_VELOCITY}"/> 로 수정')

        # joint_1 · joint_6 은 음수 방향이 열려 있어야 한다.
        # 막혀 있으면 11° 이동이 348° 대회전으로 계획된다.
        if j in ("joint_1", "joint_6"):
            if lower >= 0:
                bad(f"{j}: lower={lower} — 음수 방향이 막혀 있다",
                    f'{j} 의 lower 를 -{FULL_TURN} 로 수정')
            elif not near(lower, -FULL_TURN):
                warn(f"{j}: lower={lower} (기대 -{FULL_TURN})")

        if j == "joint_3" and not near(upper, JOINT3_UPPER):
            warn(f"joint_3: upper={upper} (실기 공식값 {JOINT3_UPPER} = ±165°)")

    if not _fail:
        ok(f"6개 관절 velocity={MAX_VELOCITY}, joint_1·6 음수 방향 열림")


# ── B. joint_limits.yaml (MoveIt 이 실제로 읽는 파일) ─────────────────────────
def check_joint_limits():
    print("\n[B] 관절 한계 오버라이드  hcr5_moveit_config/config/joint_limits.yaml")
    d = load_yaml(MCFG / "config" / "joint_limits.yaml")
    if d is None:
        return
    jl = d.get("joint_limits", {})

    n_ok = 0
    for j in JOINTS:
        e = jl.get(j)
        if e is None:
            bad(f"{j} 항목 없음")
            continue
        if not e.get("has_acceleration_limits"):
            bad(f"{j}: has_acceleration_limits=false — TOTG 가 실패한다",
                f"{j} 에 has_acceleration_limits:true / max_acceleration:{MAX_ACCEL}")
            continue
        if not e.get("max_acceleration"):
            bad(f"{j}: max_acceleration={e.get('max_acceleration')}",
                f"{j} 의 max_acceleration 을 {MAX_ACCEL} 로")
            continue
        if not near(e["max_acceleration"], MAX_ACCEL):
            warn(f"{j}: max_acceleration={e['max_acceleration']} (기준 {MAX_ACCEL})")
        if not near(e.get("max_velocity", 0), MAX_VELOCITY):
            bad(f"{j}: max_velocity={e.get('max_velocity')} (기대 {MAX_VELOCITY})",
                f"{j} 의 max_velocity 를 {MAX_VELOCITY} 로")
            continue
        n_ok += 1

    if n_ok == len(JOINTS):
        ok(f"6개 관절 max_velocity={MAX_VELOCITY} · max_acceleration={MAX_ACCEL}")

    # description 쪽 원본 근거와 값이 어긋나면 알린다
    src = DESC / "config" / "joint_limits.yaml"
    if src.exists():
        s = yaml.safe_load(src.read_text()).get("joint_limits", {}).get("joint_1", {})
        t = jl.get("joint_1", {})
        if s and t and not near(s.get("max_acceleration", -1), t.get("max_acceleration", -2)):
            warn("hcr5_description/config/joint_limits.yaml 과 값이 다르다 — 정본을 맞출 것")


# ── C. ros2_controllers.yaml ──────────────────────────────────────────────────
def check_controllers():
    print("\n[C] 컨트롤러 인터페이스  hcr5_moveit_config/config/ros2_controllers.yaml")
    d = load_yaml(MCFG / "config" / "ros2_controllers.yaml")
    if d is None:
        return
    c = d.get("hcr_arm_controller", {}).get("ros__parameters", {})
    if not c:
        bad("hcr_arm_controller.ros__parameters 없음")
        return

    cmd = c.get("command_interfaces") or []
    st = c.get("state_interfaces") or []

    if not cmd:
        bad("command_interfaces 가 비어 있다 — 컨트롤러가 configure 에 실패한다",
            f"command_interfaces: {CMD_IF}")
    elif list(cmd) != CMD_IF:
        warn(f"command_interfaces={list(cmd)} (기대 {CMD_IF})")

    if not st:
        bad("state_interfaces 가 비어 있다 — 컨트롤러가 configure 에 실패한다",
            f"state_interfaces: {STATE_IF}")
    elif list(st) != STATE_IF:
        warn(f"state_interfaces={list(st)} (기대 {STATE_IF})")

    joints = c.get("joints") or []
    if list(joints) != JOINTS:
        bad(f"joints={list(joints)} (기대 {JOINTS})")

    if cmd and st and list(joints) == JOINTS:
        ok(f"command={list(cmd)} · state={list(st)} · joints 6개")


# ── D. SRDF 플래닝 그룹 ───────────────────────────────────────────────────────
def check_srdf():
    print("\n[D] 플래닝 그룹  hcr5_moveit_config/config/hcr5.srdf")
    f = MCFG / "config" / "hcr5.srdf"
    if not f.exists():
        bad(f"파일 없음: {f}")
        return
    root = ET.parse(f).getroot()
    g = root.find("./group[@name='hcr_arm']")
    if g is None:
        bad("group 'hcr_arm' 없음")
        return
    ch = g.find("chain")
    if ch is None:
        bad("hcr_arm 이 chain 이 아니다 — 조인트 나열로 잡혀 있다",
            "Setup Assistant Planning Groups 에서 'Add Kin. Chain' 으로 재설정")
        return
    tip = ch.get("tip_link")
    if tip != TIP_LINK:
        bad(f"tip_link={tip} (기대 {TIP_LINK}) — IK 기준점이 펜 끝이 아니다",
            f"Setup Assistant 에서 Tip Link 를 {TIP_LINK} 로")
        return
    ok(f"chain base_link → {TIP_LINK}  (펜 끝 기준 IK)")


# ── E. 툴 체인 ────────────────────────────────────────────────────────────────
def check_tool():
    print("\n[E] 툴 체인  hcr5_description/urdf/hcr5_tool.xacro")
    f = DESC / "urdf" / "hcr5_tool.xacro"
    if not f.exists():
        bad(f"파일 없음: {f}")
        return
    t = f.read_text()
    for name in ("tool0", TIP_LINK):
        if f'<link name="{name}"' not in t:
            bad(f"링크 {name} 정의 없음")
            return
    m = re.search(r'flange_p:=([-\d.]+)', t)
    ok(f"tool0 · {TIP_LINK} 정의됨" + (f" (flange_p={m.group(1)})" if m else ""))


def main():
    print("═" * 72)
    print(" HCR-5 MoveIt2 설정 검증")
    print(f" 워크스페이스: {WS}")
    print("═" * 72)

    if not DESC.exists() or not MCFG.exists():
        sys.exit(f"\n패키지를 찾을 수 없습니다. WS 경로 확인: {WS}")

    check_urdf_limits()
    check_joint_limits()
    check_controllers()
    check_srdf()
    check_tool()

    print("\n" + "═" * 72)
    if _fail:
        print(f"\033[31m문제 {len(_fail)}건\033[0m — 고칠 것:")
        for msg, fix in _fail:
            print(f"  · {msg}")
            if fix:
                print(f"      → {fix}")
        print("\n상세: src/moveit2/docs/HCR5_ros2_control_구축_상세.md §7")
        return 1
    if _warn:
        print(f"\033[33m경고 {len(_warn)}건\033[0m (동작은 하지만 확인 권장)")
    print("\033[32m전부 통과\033[0m — Setup Assistant 손보정 값이 살아 있습니다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
