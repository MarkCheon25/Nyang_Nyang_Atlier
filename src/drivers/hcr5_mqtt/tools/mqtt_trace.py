#!/usr/bin/env python3
"""MQTT 전량 추적 — 모든 메시지에 **타임스탬프**를 찍어 JSONL 로 떨군다. 읽기 전용.

같은 폴더의 기존 도구는 집계만 준다 — `mqtt_sub.py` 는 신규 토픽만 찍고,
`mqtt_full.py` 는 스키마를 1건씩 뜬다. **시각이 있어야 재는 것**은 이걸로 뜬다:

  - `move/joint/here`  발행 시각 → 재발행 간격(≥137ms) · 선점 거동(V-14)
  - `motion/joint/position`      → 도달 오차(V-7) · 이동 궤적
  - `motion/joint/speed`         → 단위·부호 확인(T19)
  - `event/motion`               → moveHere 완료 이벤트
  - `monitor/robot`              → 축온 감시 (창 안에서 실시간)

플러그인은 **성공 발행을 로그로 남기지 않는다**(실패만 찍는다). 그래서 발행 간격은
ROS 로그가 아니라 버스에서 떠야 한다 — 이 도구가 존재하는 이유다.

사용법: mqtt_trace.py [host] [sec] [out.jsonl]   기본 192.168.0.20 · 30초 · /tmp/mqtt_trace.jsonl
출력  : 한 줄 = {"t": 시작 후 경과초, "topic": 토픽, "d": 파싱된 페이로드}
실행  : **호스트에서** 돈다 (랜선 프로필 `hcr5` 필요). 순수 stdlib — 브로커 라이브러리 불필요.
근거  : `ros2_control_hw_interface/중간결과물.md` §3-C (260811-오일리스부싱)
"""
import socket, sys, struct, time, json

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.20"
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
OUT = sys.argv[3] if len(sys.argv) > 3 else "/tmp/mqtt_trace.jsonl"
PORT = 1883


def enc_len(n):
    out = b""
    while True:
        d = n % 128
        n //= 128
        if n > 0:
            d |= 0x80
        out += bytes([d])
        if n == 0:
            break
    return out


def enc_str(s):
    b = s.encode()
    return struct.pack("!H", len(b)) + b


def read_len(sock):
    val = 0
    mult = 1
    while True:
        b = sock.recv(1)
        if not b:
            return None
        d = b[0]
        val += (d & 0x7f) * mult
        if not (d & 0x80):
            break
        mult *= 128
    return val


vh = enc_str("MQTT") + bytes([4, 0x02]) + struct.pack("!H", 60)
body = vh + enc_str("trace01")
s = socket.create_connection((HOST, PORT), timeout=5)
s.sendall(bytes([0x10]) + enc_len(len(body)) + body)
hdr = s.recv(1)
if not hdr or hdr[0] >> 4 != 2:
    print(f"CONNACK 아님: {hdr!r}", file=sys.stderr)
    sys.exit(1)
rl = read_len(s)
rest = s.recv(rl)
if (rest[1] if len(rest) > 1 else 255) != 0:
    print("브로커 거부", file=sys.stderr)
    sys.exit(1)

sub = struct.pack("!H", 1) + enc_str("#") + bytes([0])
s.sendall(bytes([0x82]) + enc_len(len(sub)) + sub)

s.settimeout(1.0)
t0 = time.time()
end = t0 + DURATION
counts = {}
n = 0
with open(OUT, "w") as f:
    while time.time() < end:
        try:
            b1 = s.recv(1)
        except socket.timeout:
            continue
        if not b1:
            break
        ptype = b1[0] >> 4
        rl = read_len(s)
        if rl is None:
            break
        payload = b""
        while len(payload) < rl:
            chunk = s.recv(rl - len(payload))
            if not chunk:
                break
            payload += chunk
        if ptype != 3:
            continue
        ts = time.time()
        tlen = struct.unpack("!H", payload[:2])[0]
        topic = payload[2:2 + tlen].decode(errors="replace")
        raw = payload[2 + tlen:].decode(errors="replace")
        counts[topic] = counts.get(topic, 0) + 1
        n += 1
        try:
            body_json = json.loads(raw)
        except Exception:
            body_json = raw
        f.write(json.dumps({"t": round(ts - t0, 6), "topic": topic, "d": body_json},
                           ensure_ascii=False) + "\n")
s.close()

print(f"=== 수집 {n}건 / {len(counts)}개 토픽 / {DURATION}초 → {OUT}")
for t, c in sorted(counts.items(), key=lambda x: -x[1]):
    mark = "  ★" if t in ("move/joint/here", "move/stop", "event/motion") else ""
    print(f"  {c:6d}회  {t}{mark}")
