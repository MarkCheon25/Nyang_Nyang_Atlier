#!/usr/bin/env python3
"""순수 stdlib MQTT 구독자 — 읽기 전용. 로봇이 발행하는 토픽을 관찰한다."""
import socket, sys, struct, time

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.20"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 1883
DURATION = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0
TOPIC = sys.argv[4] if len(sys.argv) > 4 else "#"

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

def connect_pkt(cid="obsvr01"):
    vh = enc_str("MQTT") + bytes([4, 0x02]) + struct.pack("!H", 30)  # clean session
    pl = enc_str(cid)
    body = vh + pl
    return bytes([0x10]) + enc_len(len(body)) + body

def subscribe_pkt(topic, pid=1):
    body = struct.pack("!H", pid) + enc_str(topic) + bytes([0])  # QoS0
    return bytes([0x82]) + enc_len(len(body)) + body

def read_len(sock):
    mult = 1; val = 0
    while True:
        b = sock.recv(1)
        if not b: return None
        d = b[0]; val += (d & 0x7f) * mult
        if not (d & 0x80): break
        mult *= 128
    return val

s = socket.create_connection((HOST, PORT), timeout=5)
s.sendall(connect_pkt())
hdr = s.recv(1)
if not hdr or hdr[0] >> 4 != 2:
    print(f"CONNACK 아님: {hdr!r}"); sys.exit(1)
rl = read_len(s); rest = s.recv(rl)
rc = rest[1] if len(rest) > 1 else 255
print(f"[CONNACK] return_code={rc} ({'수락' if rc==0 else '거부'})")
if rc != 0: sys.exit(1)

s.sendall(subscribe_pkt(TOPIC))
print(f"[SUBSCRIBE] topic='{TOPIC}', {DURATION}초 수신...\n")

s.settimeout(1.0)
end = time.time() + DURATION
topics = {}
count = 0
while time.time() < end:
    try:
        b1 = s.recv(1)
    except socket.timeout:
        continue
    if not b1: break
    ptype = b1[0] >> 4
    rl = read_len(s)
    if rl is None: break
    payload = b""
    while len(payload) < rl:
        chunk = s.recv(rl - len(payload))
        if not chunk: break
        payload += chunk
    if ptype == 3:  # PUBLISH
        tlen = struct.unpack("!H", payload[:2])[0]
        topic = payload[2:2+tlen].decode(errors="replace")
        msg = payload[2+tlen:]
        count += 1
        if topic not in topics:
            topics[topic] = 0
            shown = msg[:200].decode(errors="replace")
            print(f"🆕 [{topic}]  {shown}")
        topics[topic] += 1
    elif ptype == 9:  # SUBACK
        print(f"[SUBACK] {payload.hex()}")

print(f"\n=== 요약: {count}건 수신, {len(topics)}개 토픽 ===")
for t, c in sorted(topics.items(), key=lambda x:-x[1]):
    print(f"  {c:5d}회  {t}")
s.close()
