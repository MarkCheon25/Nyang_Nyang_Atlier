#!/usr/bin/env python3
"""TCP 검증용 점 기록기 — 로봇을 **전혀 움직이지 않는다**. `get/command/pos` 읽기만 한다.

무엇에 쓰나
  펜 끝을 어딘가에 맞춰 놓고 이 도구로 한 점씩 찍어 쌓는다. 쌓인 점으로 세 가지를 판정한다:

    ① 검산 A — 다자세 일관성
       같은 물리적 한 점을 **서로 다른 자세**로 찍는다. TCP 등록이 맞다면 tcp 좌표가
       자세와 무관하게 같아야 한다. 흩어지는 폭이 곧 등록 오차다.
       ⚠️ 4점법에 쓴 점·자세를 그대로 다시 쓰면 순환논리다. **새 점, 새 자세**로 찍을 것.

    ② 검산 B — 알려진 길이
       치수가 표준으로 정해진 물건의 두 끝을 찍어 거리를 잰다.
         카드(ISO/IEC 7810 ID-1)  85.60 × 53.98 mm   ← 가장 정확
         A4 (ISO 216)             210 × 297 mm  (±2mm)
         100원 / 500원 동전 지름   24.00 / 26.50 mm

    ③ 행정(stroke) 측정
       같은 표면을 압축 상태 / 자유 상태로 각각 찍는다. tcp.z 차이가 행정이다.
       차이값이라 등록 기준이 무엇이든 상쇄된다.

사용법
  python3 tcp_point.py add <라벨>        점 하나 기록 (지금 자세를 읽어 저장)
  python3 tcp_point.py list              쌓인 점 보기
  python3 tcp_point.py spread <라벨...>  검산 A — 그 라벨들의 산포 (같은 점을 여러 자세로 찍은 것)
  python3 tcp_point.py dist <라벨A> <라벨B>   검산 B — 두 점 거리
  python3 tcp_point.py stroke <압축라벨> <자유라벨>   행정 = z(자유) − z(압축)
  python3 tcp_point.py clear             전부 지움

  라벨은 아무 문자열이나 된다. 같은 라벨을 여러 번 쓰면 그 라벨의 표본이 쌓인다
  (검산 A 는 그렇게 쓴다 — 같은 점을 자세만 바꿔 여러 번).
"""
import sys, os, json, math, time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
import mqtt_cmd as M

STORE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tcp_points.json")


def load():
    if not os.path.exists(STORE):
        return []
    with open(STORE) as f:
        return json.load(f)


def save(rows):
    with open(STORE, "w") as f:
        json.dump(rows, f, indent=2, ensure_ascii=False)


def read_pose():
    """읽기 전용 RPC. 로봇은 움직이지 않는다."""
    m = M.Mqtt(M.HOST, M.PORT)
    d = M.query_pos(m, "점 기록")
    return d


def cmd_add(label):
    d = read_pose()
    tcp, fl = d["tcp"], d["flange"]
    row = {
        "label": label,
        "t": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "joint": d["joint"],
        "tcp_pos": [tcp["position"]["x"], tcp["position"]["y"], tcp["position"]["z"]],
        "tcp_ori": [tcp["orientation"]["x"], tcp["orientation"]["y"], tcp["orientation"]["z"]],
        "flange_pos": [fl["position"]["x"], fl["position"]["y"], fl["position"]["z"]],
        "flange_ori": [fl["orientation"]["x"], fl["orientation"]["y"], fl["orientation"]["z"]],
    }
    rows = load()
    rows.append(row)
    save(rows)

    off = math.dist(row["tcp_pos"], row["flange_pos"])
    n = sum(1 for r in rows if r["label"] == label)
    print(f"기록 [{label}] #{n}")
    print(f"  tcp    x={row['tcp_pos'][0]:9.3f} y={row['tcp_pos'][1]:9.3f} z={row['tcp_pos'][2]:9.3f}")
    print(f"  flange x={row['flange_pos'][0]:9.3f} y={row['flange_pos'][1]:9.3f} z={row['flange_pos'][2]:9.3f}")
    print(f"  |tcp−flange| = {off:.3f} mm   ← 등록된 툴 오프셋 크기")
    print(f"  관절 " + " ".join(f"{v:7.2f}" for v in row["joint"]))
    if off < 1e-6:
        print("  ⚠️ 오프셋이 0 이다 — 아직 TCP 가 등록되지 않았거나 활성 툴이 없다")


def cmd_list():
    rows = load()
    if not rows:
        print("기록된 점이 없다.")
        return
    print(f"{'#':>3} {'라벨':<14} {'시각':<20} {'tcp x':>9} {'tcp y':>9} {'tcp z':>9} {'오프셋':>8}")
    print("-" * 82)
    for i, r in enumerate(rows):
        off = math.dist(r["tcp_pos"], r["flange_pos"])
        print(f"{i:>3} {r['label']:<14} {r['t']:<20} "
              f"{r['tcp_pos'][0]:9.3f} {r['tcp_pos'][1]:9.3f} {r['tcp_pos'][2]:9.3f} {off:8.3f}")


def cmd_spread(labels):
    rows = [r for r in load() if r["label"] in labels]
    if len(rows) < 2:
        raise SystemExit(f"표본이 {len(rows)}개다 — 검산 A 는 최소 2개, 권장 4개 이상이다")

    print(f"검산 A — 같은 점을 {len(rows)}개 자세로 찍은 결과\n")
    print(f"{'라벨':<14} {'tcp x':>9} {'tcp y':>9} {'tcp z':>9}   자세(관절각)")
    print("-" * 96)
    for r in rows:
        j = " ".join(f"{v:6.1f}" for v in r["joint"])
        print(f"{r['label']:<14} {r['tcp_pos'][0]:9.3f} {r['tcp_pos'][1]:9.3f} {r['tcp_pos'][2]:9.3f}   {j}")

    mean = [sum(r["tcp_pos"][i] for r in rows) / len(rows) for i in range(3)]
    spread = [max(r["tcp_pos"][i] for r in rows) - min(r["tcp_pos"][i] for r in rows) for i in range(3)]
    rms = math.sqrt(sum(math.dist(r["tcp_pos"], mean) ** 2 for r in rows) / len(rows))
    worst = max(math.dist(r["tcp_pos"], mean) for r in rows)

    print("-" * 96)
    print(f"평균  x={mean[0]:9.3f} y={mean[1]:9.3f} z={mean[2]:9.3f}")
    print(f"산포(최대−최소)  x={spread[0]:.3f} y={spread[1]:.3f} z={spread[2]:.3f} mm")
    print(f"★ 중심에서 RMS = {rms:.3f} mm · 최악 = {worst:.3f} mm")
    print()
    if worst < 0.5:
        print("판정: ✅ 등록이 좋다. 로봇 반복정밀도(±0.1mm) 수준에 근접.")
    elif worst < 2.0:
        print("판정: 🟡 쓸 만하다. 소묘 정밀도 요구에 따라 재티칭 검토.")
    else:
        print("판정: 🔴 흩어진다. 4점법 조준 오차이거나 펜이 접촉 방향에 따라 휜다.")
        print("      펜 고정(테이프)이 느슨하지 않은지 먼저 보라.")


def cmd_dist(a, b):
    rows = load()
    pa = [r for r in rows if r["label"] == a]
    pb = [r for r in rows if r["label"] == b]
    if not pa or not pb:
        raise SystemExit(f"라벨을 못 찾았다 (a={len(pa)}개, b={len(pb)}개)")
    # 같은 라벨이 여럿이면 평균을 쓴다
    ca = [sum(r["tcp_pos"][i] for r in pa) / len(pa) for i in range(3)]
    cb = [sum(r["tcp_pos"][i] for r in pb) / len(pb) for i in range(3)]
    d = math.dist(ca, cb)
    print(f"검산 B — 두 점 거리\n")
    print(f"  [{a}] x={ca[0]:9.3f} y={ca[1]:9.3f} z={ca[2]:9.3f}   ({len(pa)}개 평균)")
    print(f"  [{b}] x={cb[0]:9.3f} y={cb[1]:9.3f} z={cb[2]:9.3f}   ({len(pb)}개 평균)")
    print(f"\n★ 거리 = {d:.3f} mm")
    print(f"   Δx={cb[0]-ca[0]:9.3f}  Δy={cb[1]-ca[1]:9.3f}  Δz={cb[2]-ca[2]:9.3f}")
    print()
    for name, ref in (("카드 장변", 85.60), ("카드 단변", 53.98),
                      ("A4 장변", 297.0), ("A4 단변", 210.0),
                      ("500원 지름", 26.50), ("100원 지름", 24.00)):
        err = d - ref
        if abs(err) < max(3.0, ref * 0.05):
            print(f"   {name} {ref}mm 대비  오차 {err:+.3f} mm  ({err/ref*100:+.2f}%)")


def cmd_stroke(compressed, free):
    rows = load()
    pc = [r for r in rows if r["label"] == compressed]
    pf = [r for r in rows if r["label"] == free]
    if not pc or not pf:
        raise SystemExit(f"라벨을 못 찾았다 (압축={len(pc)}개, 자유={len(pf)}개)")
    zc = sum(r["tcp_pos"][2] for r in pc) / len(pc)
    zf = sum(r["tcp_pos"][2] for r in pf) / len(pf)
    stroke = zf - zc
    print(f"행정(stroke) 측정\n")
    print(f"  압축 [{compressed}] tcp.z = {zc:9.3f} mm   ({len(pc)}개 평균)  ← 이것이 표면 높이 z_s")
    print(f"  자유 [{free}] tcp.z = {zf:9.3f} mm   ({len(pf)}개 평균)")
    print(f"\n★ 행정 = {stroke:.3f} mm")
    print(f"   자유길이 TCP z = 압축 TCP z + {stroke:.3f} mm  ← 최종 등록값")
    print()
    if stroke <= 0:
        print("판정: 🔴 부호가 이상하다. 자유 상태가 압축보다 낮게 나왔다 —")
        print("      라벨을 바꿔 넣었거나, 접촉 순간 판정이 어긋났다.")
    elif stroke < 1.0:
        print(f"판정: 🔴 행정 {stroke:.2f}mm 는 매우 짧다. 종이 높이 캘리브레이션이")
        print("      ±0.3mm 급으로 정밀해야 한다 — BRD R2 리스크가 거의 안 줄었다.")
    elif stroke < 5.0:
        print(f"판정: 🟡 행정 {stroke:.2f}mm. 종이 높이 오차 ±{stroke/3:.1f}mm 정도까지 흡수된다.")
    else:
        print(f"판정: ✅ 행정 {stroke:.2f}mm. 종이 높이 오차 ±{stroke/3:.1f}mm 를 흡수한다 —")
        print("      캘리브레이션 요구정밀도가 크게 완화된다.")


def main():
    a = sys.argv[1:]
    if not a:
        print(__doc__)
        return
    c = a[0]
    if c == "add":
        if len(a) < 2:
            raise SystemExit("사용법: tcp_point.py add <라벨>")
        cmd_add(a[1])
    elif c == "list":
        cmd_list()
    elif c == "spread":
        cmd_spread(a[1:] if len(a) > 1 else [r["label"] for r in load()])
    elif c == "dist":
        if len(a) < 3:
            raise SystemExit("사용법: tcp_point.py dist <라벨A> <라벨B>")
        cmd_dist(a[1], a[2])
    elif c == "stroke":
        if len(a) < 3:
            raise SystemExit("사용법: tcp_point.py stroke <압축라벨> <자유라벨>")
        cmd_stroke(a[1], a[2])
    elif c == "clear":
        save([])
        print("전부 지웠다.")
    else:
        raise SystemExit(f"모르는 명령: {c}\n{__doc__}")


if __name__ == "__main__":
    main()
