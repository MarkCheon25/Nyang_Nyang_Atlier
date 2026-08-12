#!/usr/bin/env python3
"""축온 감시견 — 서보 ON 창 안에서 실시간으로 축온을 보고 임계에서 **서보를 끊는다**.

왜 있나 — 검C1 착수 시 축온이 **55°C** 로 T20 재개 게이트(≤50°C)를 넘겼다. Rokey6 판정은
*"진행하되 창 ≤20초 + 58°C 즉시중단"*(2026-08-12). 그 '즉시중단'을 사람 눈이 아니라 이 스크립트가 건다.

  - `monitor/robot` 구독 → 6축 temp 를 본다. 임계 이상이거나 `-1` 이면 **`mqtt_cmd.py servo off`** 를 쏘고 죽는다
  - 실패 방향이 안전하다 — 오발동하면 창이 일찍 닫힐 뿐이고, 미발동해도 사람 감시보다 나쁘지 않다
  - 판정용 원자료가 아니다. 축온의 **기록**은 `mqtt_trace.py` 의 `monitor/robot` 줄이 원본이다

사용: c1_tempwatch.py [host] [sec] [임계°C] [out.jsonl]   기본 192.168.0.20 · 60초 · 58 · /tmp/c1_temp.jsonl
종료: 0 정상 · 3 **임계 초과로 서보를 끊었다** · 1 브로커 접속 실패
근거: T20(업무목록) · `briefs/검증_재현_세션분할.md` §2
"""
import socket, sys, struct, time, json, subprocess, os

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.20"
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
LIMIT = float(sys.argv[3]) if len(sys.argv) > 3 else 58.0
OUT = sys.argv[4] if len(sys.argv) > 4 else "/tmp/c1_temp.jsonl"
PORT = 1883

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "..", "..", "tools")
CMD = os.path.normpath(os.path.join(TOOLS, "mqtt_cmd.py"))


def enc_len(n):
    out = b""
    while True:
        d = n % 128; n //= 128
        if n > 0: d |= 0x80
        out += bytes([d])
        if n == 0: break
    return out


def enc_str(s):
    b = s.encode()
    return struct.pack("!H", len(b)) + b


def read_len(sock):
    mult, val = 1, 0
    while True:
        b = sock.recv(1)
        if not b: return None
        d = b[0]; val += (d & 0x7f) * mult
        if not (d & 0x80): break
        mult *= 128
    return val


def abort(reason):
    """서보를 끊는다. 이 경로는 되도록 짧아야 한다."""
    print(f"\n🚨 {reason} — 서보 OFF 를 쏜다", flush=True)
    try:
        r = subprocess.run([sys.executable, CMD, "servo", "off"],
                           capture_output=True, text=True, timeout=15)
        print((r.stdout or "")[-400:], flush=True)
        print((r.stderr or "")[-200:], flush=True)
    except Exception as e:
        print(f"⚠️ servo off 송신 실패: {e} — 사람이 비상정지를 친다", flush=True)
    sys.exit(3)


s = socket.create_connection((HOST, PORT), timeout=5)
s.sendall(bytes([0x10]) + enc_len(len(
    enc_str("MQTT") + bytes([4, 0x02]) + struct.pack("!H", 60) + enc_str("tempwatch01"))) +
    enc_str("MQTT") + bytes([4, 0x02]) + struct.pack("!H", 60) + enc_str("tempwatch01"))
hdr = s.recv(1)
if not hdr or hdr[0] >> 4 != 2:
    print(f"CONNACK 아님: {hdr!r}"); sys.exit(1)
rest = s.recv(read_len(s))
if (rest[1] if len(rest) > 1 else 255) != 0:
    print("[CONNACK] 거부"); sys.exit(1)

body = struct.pack("!H", 1) + enc_str("monitor/robot") + bytes([0])
s.sendall(bytes([0x82]) + enc_len(len(body)) + body)

print(f"# 축온 감시견 — 임계 **{LIMIT:.0f}°C** · {DURATION:.0f}초 · 대상 {HOST}", flush=True)
s.settimeout(1.0)
t0 = time.perf_counter()
end = t0 + DURATION
peak, last_print, n = {}, 0.0, 0
f = open(OUT, "w")
while time.perf_counter() < end:
    try:
        b1 = s.recv(1)
    except socket.timeout:
        continue
    if not b1: break
    if b1[0] >> 4 != 3:
        rl = read_len(s)
        if rl: s.recv(rl)
        continue
    rl = read_len(s)
    if rl is None: break
    payload = b""
    while len(payload) < rl:
        c = s.recv(rl - len(payload))
        if not c: break
        payload += c
    tlen = struct.unpack("!H", payload[:2])[0]
    raw = payload[2 + tlen:].decode(errors="replace")
    t = time.perf_counter() - t0
    try:
        env = json.loads(raw)
        lvl1 = env.get("data", env)
        inner = lvl1.get("data", lvl1) if isinstance(lvl1, dict) else lvl1
    except Exception:
        continue
    temps = {a.get("name"): a.get("temp") for a in inner.get("axis", [])}
    if not temps:
        continue
    n += 1
    f.write(json.dumps({"t": round(t, 4), "temp": temps,
                        "power": inner.get("power"), "current": inner.get("current")}) + "\n")
    f.flush()
    for k, v in temps.items():
        if v is not None:
            peak[k] = max(peak.get(k, -999), v)

    bad = [k for k, v in temps.items() if v is None or v == -1 or v >= LIMIT]
    if bad:
        f.close()
        abort(f"축온 임계 도달 — {', '.join(f'{k}={temps[k]}°C' for k in bad)}")

    if t - last_print >= 2.0:
        last_print = t
        print(f"[{t:5.1f}s] " + " ".join(f"{k}={v}" for k, v in temps.items()) +
              f"  | 최고 {max(v for v in temps.values() if v is not None)}°C", flush=True)

f.close()
print(f"\n✅ 창 종료 — 임계 미도달. 표본 {n}건 · 축별 최고: " +
      " ".join(f"{k}={v}" for k, v in peak.items()), flush=True)
