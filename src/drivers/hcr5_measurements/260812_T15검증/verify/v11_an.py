#!/usr/bin/env python3
"""V-11 재산출 — on_configure 소요시간과 실패 메시지를 로그에서 뽑는다 (260812-감압밸브, 검B).

측정량 = `'configure' hardware 'hcr_robot_system'` 로그줄 → 그 뒤 첫 ERROR 줄까지의 시간.
프로세스 wall time 이 아니다 — 노드 기동 오버헤드가 섞이기 때문.

기준(확정 문면, 세션분할계획 §3):
  · on_configure 가 **≤3.1s** 안에 ERROR — OS 타임아웃(수십 초)에 잡히면 실패
  · **실패 사유마다** 다른 메시지 (TCP 미도달 / 표본 미수신)

사용: python3 v11_an.py <로그디렉터리>
"""
import re
import sys
import pathlib

LOG_RE = re.compile(r"^\[(\w+)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)$")
CASES = [
    ("noresp", "① 무응답 호스트 192.168.0.99:1883"),
    ("refused", "② 접속 거부 127.0.0.1:18830"),
    ("stubonly", "③ 브로커만 (스텁) 127.0.0.1:1884"),
]


def parse(path):
    """configure 시작 시각과 그 뒤 첫 ERROR(시각, 본문)를 돌려준다."""
    t_cfg, err = None, None
    for line in path.read_text(errors="replace").splitlines():
        m = LOG_RE.match(line)
        if not m:
            continue
        level, ts, _node, msg = m.group(1), float(m.group(2)), m.group(3), m.group(4)
        if t_cfg is None and msg.startswith("'configure' hardware"):
            t_cfg = ts
        elif t_cfg is not None and err is None and level == "ERROR":
            err = (ts, msg)
    return t_cfg, err


def main():
    d = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    rows, msgs = [], {}
    for tag, label in CASES:
        f = d / f"v11_{tag}.log"
        if not f.exists():
            print(f"  {label}: 로그 없음 ({f})")
            continue
        t_cfg, err = parse(f)
        if t_cfg is None or err is None:
            print(f"  {label}: configure/ERROR 를 못 찾았다")
            continue
        ms = (err[0] - t_cfg) * 1000.0
        rows.append((label, ms, err[1]))
        # 주소만 다른 것은 같은 메시지로 묶는다 — 문면 종류를 세기 위해서다
        msgs.setdefault(re.sub(r"[\d.]+:\d+", "<addr>", err[1]), []).append(tag)

    print("=== V-11 on_configure 소요 · 메시지 ===")
    for label, ms, msg in rows:
        ok = "OK" if ms <= 3100 else "초과"
        print(f"  {label}\n      소요 {ms:9.3f} ms  [{ok}, 기준 ≤3100]\n      «{msg}»")

    print(f"\n=== 메시지 문면 {len(msgs)}종 (주소는 <addr> 로 정규화) ===")
    for i, (m, tags) in enumerate(msgs.items(), 1):
        print(f"  {i}. {'·'.join(tags)} → «{m}»")

    within = all(ms <= 3100 for _, ms, _ in rows)
    print(f"\n소요 기준(≤3.1s) : {'전 경로 충족' if within else '미충족'}")
    print(f"문면 기준(사유별): 사유 2종(TCP 미도달 / 표본 미수신) ↔ 실측 {len(msgs)}종")


if __name__ == "__main__":
    main()
