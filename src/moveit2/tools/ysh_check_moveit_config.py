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

  3) hcr5.srdf 의 group_state 'home' 이 임의 자세로 되돌아감
     → 실기 `move/joint/home` 이 가는 자세와 달라진다

그리고 재생성과 무관하게, **더 조용히 틀리는** 것이 하나 더 있다:

  4) URDF 관절 origin 이 교정 전 CAD 값으로 되돌아감  ← [F]
     → 계획도 실행도 에러 없이 성공하고 **펜만 43mm 옆에 그린다.**
       A4 폭이 210mm 이므로 이 상태로는 그림이 성립하지 않는다.
       [F] 는 값 대조에 그치지 않고 **순기구학을 직접 풀어 실측 flange 좌표와 대조**한다.

이 스크립트는 moveit_config **패키지 바깥**(src/moveit2/tools/)에 둔다. Setup Assistant 가
건드리지 않는 위치라 재생성에도 살아남는다.

사용법
------
  python3 src/moveit2/tools/ysh_check_moveit_config.py
  echo $?        # 0 = 전부 통과, 1 = 문제 있음

Setup Assistant 를 다시 돌린 직후에는 **반드시** 한 번 실행할 것.
"""
import math
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

# ── 기구학 정본 ───────────────────────────────────────────────────────────────
# 출처: 2026-08-05 실기 실측 교정 (FK RPC 108 샘플 스윕 → 최소제곱).
#       109개 자세 위치오차 RMS 43.4mm → 0.0060mm.
# ⚠️ 이 값이 CAD 원본으로 되돌아가면 아무 에러 없이 펜이 43mm 옆에 그린다.
CANON_ORIGIN = {
    "joint_1": (0.0,       0.0,     0.03),
    "joint_2": (-0.059138, 0.0,     0.119001),
    "joint_3": (-0.019348, 0.0,     0.425001),
    "joint_4": (-0.030016, -8e-06,  0.338499),
    "joint_5": (-0.062,    0.0,     0.089504),
    "joint_6": (-0.132498, 0.0,     0.062),
}
# 공식 가동범위 (User Manual v2.0 Appendix F): ±360°, J3 만 ±165°.
CANON_LIMIT = {j: (-FULL_TURN, FULL_TURN) for j in JOINTS}
CANON_LIMIT["joint_3"] = (-JOINT3_UPPER, JOINT3_UPPER)

# 실기 홈 [0,−90,−90,−90,90,0]° 를 SIGN/DELTA 규약으로 환산한 URDF 값(rad).
HOME_URDF = [1.570796, 0.0, 1.570796, 0.0, 1.570796, 0.0]
# 그 자세에서 펜던트가 읽은 flange 좌표 [m]. FK 결과가 여기 0.1mm 안으로 들어와야 한다.
HOME_FLANGE_M = (0.490, -0.1705, 0.4415)
FK_TOL_M = 1e-4             # 0.1mm — 로봇 반복정밀도와 같은 자릿수

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


# ── URDF 파싱 (A·F 공용) ──────────────────────────────────────────────────────
def parse_arm():
    """hcr5_arm.xacro 에서 관절별 origin·axis·limit 을 뽑는다. 실패하면 None."""
    f = DESC / "urdf" / "hcr5_arm.xacro"
    if not f.exists():
        bad(f"파일 없음: {f}")
        return None
    text = f.read_text()
    blocks = dict(re.findall(r'<joint name="(joint_\d)"[^>]*>(.*?)</joint>', text, re.S))

    missing = [j for j in JOINTS if j not in blocks]
    if missing:
        bad(f"조인트 누락: {missing}")
        return None

    out = {}
    for j in JOINTS:
        body = blocks[j]
        o = re.search(r'<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"', body)
        a = re.search(r'<axis\s+xyz="([^"]+)"', body)
        m = re.search(r'<limit\s+upper="([-\d.eE+]+)"\s+lower="([-\d.eE+]+)"'
                      r'[^>]*velocity="([-\d.eE+]+)"', body)
        if not (o and a and m):
            bad(f"{j}: origin/axis/limit 파싱 실패")
            return None
        out[j] = {
            "origin": tuple(float(v) for v in o.group(1).split()),
            "rpy": tuple(float(v) for v in o.group(2).split()),
            "axis": tuple(float(v) for v in a.group(1).split()),
            "upper": float(m.group(1)),
            "lower": float(m.group(2)),
            "velocity": float(m.group(3)),
        }
    return out


# ── A. URDF 관절 한계 ─────────────────────────────────────────────────────────
def check_urdf_limits(arm):
    print("\n[A] URDF 관절 한계  hcr5_description/urdf/hcr5_arm.xacro")
    if arm is None:
        return
    n_ok = 0
    for j in JOINTS:
        e = arm[j]
        lo, hi = CANON_LIMIT[j]
        good = True

        if not near(e["velocity"], MAX_VELOCITY):
            bad(f"{j}: velocity={e['velocity']} (기대 {MAX_VELOCITY} = 180°/s)",
                f'{j} 의 <limit ... velocity="{MAX_VELOCITY}"/> 로 수정')
            good = False

        # 가동범위가 실기보다 **좁으면** IK 해가 있어도 계획이 실패한다.
        # 특히 joint_5 는 실기 wrist2 가 −267° 로 관측되므로 ±120° 로는
        # 실기의 현재 자세조차 URDF 로 표현할 수 없다 (상태 브릿지가 깨진다).
        if e["lower"] > lo + 1e-3 or e["upper"] < hi - 1e-3:
            bad(f"{j}: 가동범위 [{e['lower']}, {e['upper']}] 가 공식값 "
                f"[{lo:.6f}, {hi:.6f}] 보다 좁다",
                f'{j} 의 <limit upper="{hi:.6f}" lower="{lo:.6f}" .../> 로 수정')
            good = False
        elif e["lower"] < lo - 1e-3 or e["upper"] > hi + 1e-3:
            # 넓으면 로봇이 자체 정지를 건다 (매뉴얼 8.5)
            warn(f"{j}: 가동범위 [{e['lower']}, {e['upper']}] 가 공식값보다 넓다 "
                 f"— 로봇이 자체 정지를 걸 수 있다")

        n_ok += good

    if n_ok == len(JOINTS):
        ok(f"6개 관절 velocity={MAX_VELOCITY} · 가동범위 ±360° (J3 ±165°)")


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


# ── F. 기구학 정본 대조 + 순기구학 실측 검증 ─────────────────────────────────
def _rot(axis, th):
    """축-각 회전행렬 (3×3, 리스트의 리스트). numpy 없이 돌아야 한다."""
    n = math.sqrt(sum(v * v for v in axis))
    x, y, z = (v / n for v in axis)
    c, s, C = math.cos(th), math.sin(th), 1 - math.cos(th)
    return [
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


def _fk_flange(arm, q):
    """base_link → link6_1 위치 [m]. 모든 관절 rpy 가 0 인 것을 전제한다."""
    R = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
    p = [0.0, 0.0, 0.0]
    for i, j in enumerate(JOINTS):
        d = arm[j]["origin"]
        p = [p[k] + sum(R[k][m] * d[m] for m in range(3)) for k in range(3)]
        Rj = _rot(arm[j]["axis"], q[i])
        R = [[sum(R[a][k] * Rj[k][b] for k in range(3)) for b in range(3)] for a in range(3)]
    return p


def check_kinematics(arm):
    print("\n[F] 기구학 정본  hcr5_description/urdf/hcr5_arm.xacro")
    if arm is None:
        return

    # F-1. origin 값 대조
    drift = []
    for j in JOINTS:
        got, want = arm[j]["origin"], CANON_ORIGIN[j]
        d = math.dist(got, want)
        if d > 1e-6:
            drift.append((j, got, want, d))
    if drift:
        for j, got, want, d in drift:
            bad(f"{j}: origin {got} ≠ 정본 {want}  (차이 {d * 1000:.2f}mm)",
                f'{j} 의 <origin xyz="{want[0]} {want[1]} {want[2]}" rpy="0 0 0"/>')
        print("      ↑ 값이 되돌아가면 **에러 없이 펜이 43mm 옆에 그린다.** "
              "근거: 2026-08-05 실기 FK 스윕 교정")
        return
    ok("6개 관절 origin 이 실측 교정 정본과 일치")

    # F-2. 순기구학 → 실측 flange 대조 (값 대조보다 강한 검증)
    nonzero_rpy = [j for j in JOINTS if any(abs(v) > 1e-9 for v in arm[j]["rpy"])]
    if nonzero_rpy:
        warn(f"관절 rpy 가 0 이 아니다 {nonzero_rpy} — FK 검증을 건너뛴다")
        return
    p = _fk_flange(arm, HOME_URDF)
    d = math.dist(p, HOME_FLANGE_M)
    got = f"({p[0] * 1000:.2f}, {p[1] * 1000:.2f}, {p[2] * 1000:.2f})"
    want = f"({HOME_FLANGE_M[0] * 1000:.2f}, {HOME_FLANGE_M[1] * 1000:.2f}, " \
           f"{HOME_FLANGE_M[2] * 1000:.2f})"
    if d > FK_TOL_M:
        bad(f"홈 자세 FK flange={got} ≠ 실측 {want} mm  (차이 {d * 1000:.2f}mm)",
            "관절 origin 을 정본으로 되돌릴 것 (F-1)")
    else:
        ok(f"홈 자세 FK flange={got} mm ≡ 실측 (차이 {d * 1000:.2f}mm)")


# ── G. SRDF 홈 자세 ───────────────────────────────────────────────────────────
def check_home_state():
    print("\n[G] 홈 자세  hcr5_moveit_config/config/hcr5.srdf")
    f = MCFG / "config" / "hcr5.srdf"
    if not f.exists():
        bad(f"파일 없음: {f}")
        return
    gs = ET.parse(f).getroot().find("./group_state[@name='home']")
    if gs is None:
        bad("group_state 'home' 없음",
            "Setup Assistant Robot Poses 에서 home 을 다시 정의")
        return
    vals = {e.get("name"): float(e.get("value")) for e in gs.findall("joint")}
    off = [j for i, j in enumerate(JOINTS) if not near(vals.get(j, 1e9), HOME_URDF[i], 1e-3)]
    if off:
        bad(f"home 자세가 실기와 다르다 — 어긋난 관절 {off}",
            "URDF [1.570796, 0, 1.570796, 0, 1.570796, 0] "
            "(= 실기 [0,−90,−90,−90,90,0]°) 로 수정")
        return
    ok("home = 실기 `move/joint/home` 자세 (URDF 90,0,90,0,90,0°)")


def main():
    print("═" * 72)
    print(" HCR-5 MoveIt2 설정 검증")
    print(f" 워크스페이스: {WS}")
    print("═" * 72)

    if not DESC.exists() or not MCFG.exists():
        sys.exit(f"\n패키지를 찾을 수 없습니다. WS 경로 확인: {WS}")

    arm = parse_arm()
    check_urdf_limits(arm)
    check_joint_limits()
    check_controllers()
    check_srdf()
    check_tool()
    check_kinematics(arm)
    check_home_state()

    print("\n" + "═" * 72)
    if _fail:
        print(f"\033[31m문제 {len(_fail)}건\033[0m — 고칠 것:")
        for msg, fix in _fail:
            print(f"  · {msg}")
            if fix:
                print(f"      → {fix}")
        print("\n상세: src/moveit2/docs/ysh_HCR5_ros2_control_구축_상세.md §7")
        return 1
    if _warn:
        print(f"\033[33m경고 {len(_warn)}건\033[0m (동작은 하지만 확인 권장)")
    print("\033[32m전부 통과\033[0m — Setup Assistant 손보정 값이 살아 있습니다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
