#!/usr/bin/env python3
"""V-5 상태 정합 스냅샷 — 펜던트 판독 시각의 **버스 원값**과 **`/joint_states`** 를 한 자리에 뜬다.

왜 스냅샷이 필요한가 — V-5 는 *실기 관절값(펜던트) ↔ `/joint_states`* 대조다. 세 값이 **같은 자세**에서
나와야 판정이 된다. 펜던트는 사람이 읽으므로(Rokey6), 그 사이 로봇이 안 움직이는 **서보 OFF** 상태에서
버스와 ROS 를 동시에 떠 놓는다. 이 스크립트는 **아무것도 발행하지 않는다**.

규약 변환은 코드가 이미 하고 있으므로(`joint_convention.hpp`) 여기서는 **역변환**으로 `/joint_states`(rad)를
실기 도(度)로 되돌려 버스 원값과 직접 비교한다 — 펜던트 열과 같은 단위가 돼 사람이 바로 대조할 수 있다.

    q_URDF[i](도) = s[i]*q_실기[i] + δ[i],   s = (+1,+1,−1,+1,+1,+1),  δ = (90,90,0,90,0,0)

사용: c2_v5_snap.py <라벨> [수집초]      예) c2_v5_snap.py pose_a 4
출력: logs/c2_v5_<라벨>.json + stdout 마크다운 표(펜던트 열은 비워 둔다 — 사람이 채운다)
근거: `검증.md` V-5 · 중간결과물 '대조 정답지' · 계획서 §5 검C2
"""
import json
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(HERE)
TOOLS = os.path.normpath(os.path.join(BASE, "..", "..", "tools"))


def moveit_container():
    """컨테이너명은 PC마다 다르다 — 04pc=markch_moveit2_dev · 03pc=moveit2_dev.
    떠 있는 쪽을 잡고, 환경변수 CONTAINER 로 덮어쓸 수 있다 (2026-08-15 이식)."""
    if os.environ.get("CONTAINER"):
        return os.environ["CONTAINER"]
    names = subprocess.run(["docker", "ps", "--format", "{{.Names}}"],
                           capture_output=True, text=True, timeout=20).stdout.split()
    for n in names:
        if n in ("markch_moveit2_dev", "moveit2_dev"):
            return n
    sys.exit("❌ moveit2 컨테이너가 안 떠 있다 — run_container.sh up 먼저")

LABEL = sys.argv[1] if len(sys.argv) > 1 else "snap"
SEC = sys.argv[2] if len(sys.argv) > 2 else "4"

BUS_KEYS = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
SIGN = [1, 1, -1, 1, 1, 1]
DELTA = [90.0, 90.0, 0.0, 90.0, 0.0, 0.0]

tmp = f"/tmp/c2_v5_{LABEL}.jsonl"
subprocess.run(["python3", os.path.join(TOOLS, "mqtt_trace.py"), "192.168.0.20", SEC, tmp],
               capture_output=True, text=True, timeout=float(SEC) + 30)

bus = None
with open(tmp) as f:
    for line in f:
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if r.get("topic", "").endswith("motion/joint/position"):
            d = r["d"]
            while isinstance(d, dict) and "data" in d:
                d = d["data"]
            if isinstance(d, dict) and all(k in d for k in BUS_KEYS):
                bus = d
if bus is None:
    sys.exit("❌ 버스에서 motion/joint/position 을 못 받았다 — 링크·브로커 확인")

out = subprocess.run(
    ["docker", "exec", moveit_container(), "bash", "-lc",
     "cd ~/ws_moveit2 && source install/setup.bash && export ROS_DOMAIN_ID=0 && "
     "timeout 6 ros2 topic echo /joint_states --once --field position"],
    capture_output=True, text=True, timeout=40).stdout
def parse_positions(text):
    """`--field position` 출력에서 6축을 뽑는다.

    ⚠️ 2026-08-15 03pc 에서 **5/5 재현**된 두 함정을 피한다:
      ① `ros2 topic echo --once` 가 "A message was lost!!!" 경고를 **stdout 에 섞는다**
         (100Hz 토픽 + --once 면 매번 난다).
      ② 출력이 `array('d', [ ... ])` 형식이다 — YAML `- 1.57` 이 아니다. 종전의 `- ` 파싱은
         이 형식에서 **항상 0개**를 받아 여기서 죽었다.
    대괄호 안을 먼저 노리고, 없으면 YAML '- ' 줄만 본다.
    """
    text = text.replace("\r", "")
    m = re.search(r"\[([^\]]*)\]", text)
    if m:
        return [float(x) for x in m.group(1).split(",") if x.strip()][:6]
    return [float(x[2:]) for x in text.split("\n") if x.startswith("- ")][:6]


js = parse_positions(out)
if len(js) != 6:
    sys.exit(f"❌ /joint_states 를 못 읽었다 (받은 값 {len(js)}개) — 스택이 떠 있는지 확인")

rows, worst = [], 0.0
for i, (bk, jn) in enumerate(zip(BUS_KEYS, JOINTS)):
    b = float(bus[bk])                                   # 실기 도
    j = js[i]                                            # URDF rad
    back = (math.degrees(j) - DELTA[i]) / SIGN[i]        # → 실기 도 역변환
    dev = back - b
    worst = max(worst, abs(dev))
    rows.append({"joint": jn, "bus_deg": b, "js_rad": j, "js_back_deg": back, "dev_deg": dev})

res = {"label": LABEL, "bus_deg": {k: float(bus[k]) for k in BUS_KEYS},
       "joint_states_rad": js, "rows": rows, "max_abs_dev_deg": worst}
with open(os.path.join(BASE, "c2", "logs", f"c2_v5_{LABEL}.json"), "w") as f:
    json.dump(res, f, indent=1, ensure_ascii=False)

print(f"\n### V-5 스냅샷 — `{LABEL}`\n")
print("| 관절 | 펜던트 (도) | 버스 값 (도) | `/joint_states` (rad) | 역변환 (도) | 버스−역변환 |")
print("|---|---|---|---|---|---|")
for bk, r in zip(BUS_KEYS, rows):
    print(f"| {bk} / `{r['joint']}` | **(판독)** | `{r['bus_deg']:.7f}` | `{r['js_rad']!r}` | "
          f"`{r['js_back_deg']:.7f}` | {r['dev_deg']:+.3e}° |")
print(f"\n- 버스 ↔ `/joint_states` 최대 편차 **{worst:.3e}°** (기준 V-5 ≤0.05°)")
print("- **펜던트 열은 Rokey6 가 채운다** — 이 표를 읽는 시각의 실기 표시값")
