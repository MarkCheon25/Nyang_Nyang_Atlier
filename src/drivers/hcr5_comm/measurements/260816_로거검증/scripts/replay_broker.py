#!/usr/bin/env python3
"""mqtt_logger.py 검증용 재생 브로커 — 실캡처 JSONL.gz 를 MQTT PUBLISH 로 되쏜다.

`measurements/260811_T15/scripts/mqtt_stub.py` 를 확장했다 (그쪽은 응답만 하고 발행은 안 한다).
**실기 없이 `tools/mqtt_logger.py` 를 회귀 검증하는 도구**다 — 실기 시간이 비싼 자원이라 남겨 둔다.
검증 결과는 같은 디렉터리의 `README.md`.

  replay_broker.py <port> <capture.jsonl.gz> [speed] [--inject]

--inject 를 주면 재생 도중 인위적 이상을 끼워 넣는다:
  · 관절 점프  base 를 +12° 한 표본 (soft 임계 1.0° 초과, hard 180°/s 도 초과)
  · 표본 결측  0.5초 공백 (GAP 0.20s 초과)
  · 에러       error/network 280002 (08-05 드라이브 트립과 같은 형식)
"""
import gzip
import json
import socket
import struct
import sys
import threading
import time

PORT = int(sys.argv[1])
CAP = sys.argv[2]
SPEED = float(sys.argv[3]) if len(sys.argv) > 3 and not sys.argv[3].startswith("--") else 1.0
INJECT = "--inject" in sys.argv


def enc_len(n):
    o = b""
    while True:
        d = n % 128
        n //= 128
        if n > 0:
            d |= 0x80
        o += bytes([d])
        if n == 0:
            break
    return o


def pub(topic, obj):
    t = topic.encode()
    body = struct.pack("!H", len(t)) + t + json.dumps(obj, ensure_ascii=False).encode()
    return bytes([0x30]) + enc_len(len(body)) + body


def load():
    recs = []
    with gzip.open(CAP, "rt") as f:
        for l in f:
            try:
                r = json.loads(l)
            except Exception:
                continue
            if "topic" in r and "t" in r:
                recs.append((r["t"], r["topic"], r.get("d")))
    recs.sort(key=lambda x: x[0])
    return recs


RECS = load()
print(f"[replay] {len(RECS)}건 적재, {RECS[-1][0]:.1f}s 분량, speed={SPEED}x, inject={INJECT}", flush=True)


def reader(c, stop):
    """CONNECT/SUBSCRIBE/PINGREQ 응답 — 발행 스레드와 분리."""
    try:
        while not stop.is_set():
            h = c.recv(1)
            if not h:
                break
            t = h[0] >> 4
            ln, mult = 0, 1
            while True:
                b = c.recv(1)
                if not b:
                    return
                ln += (b[0] & 127) * mult
                if not (b[0] & 128):
                    break
                mult *= 128
            body = c.recv(ln) if ln else b""
            if t == 1:
                c.sendall(bytes([0x20, 0x02, 0x00, 0x00]))                      # CONNACK
            elif t == 8:
                c.sendall(bytes([0x90, 0x03, body[0], body[1], 0x00]))          # SUBACK
                stop.subscribed = True
            elif t == 12:
                c.sendall(bytes([0xD0, 0x00]))                                  # PINGRESP
                stop.pings += 1
            elif t == 14:
                return
    except Exception:
        pass
    finally:
        stop.set()


def serve(c):
    stop = threading.Event()
    stop.subscribed = False
    stop.pings = 0
    threading.Thread(target=reader, args=(c, stop), daemon=True).start()
    for _ in range(50):
        if stop.subscribed:
            break
        time.sleep(0.05)
    print("[replay] SUBSCRIBE 확인, 재생 시작", flush=True)

    t0 = time.time()
    n = 0
    injected = set()
    try:
        for i, (t, topic, d) in enumerate(RECS):
            if stop.is_set():
                break
            due = t0 + t / SPEED
            while True:
                dt = due - time.time()
                if dt <= 0:
                    break
                time.sleep(min(dt, 0.05))
            c.sendall(pub(topic, d))
            n += 1

            if INJECT and topic == "motion/joint/position":
                el = t / SPEED
                if el > 5 and "jump" not in injected:
                    injected.add("jump")
                    bad = json.loads(json.dumps(d))
                    bad["data"]["data"]["base"] = bad["data"]["data"]["base"] + 12.0
                    c.sendall(pub(topic, bad))          # 직후에 12° 튄 표본 1건
                    print(f"[replay] ★주입 관절점프 +12° @{el:.1f}s", flush=True)
                elif el > 8 and "gap" not in injected:
                    injected.add("gap")
                    print(f"[replay] ★주입 표본결측 0.5s @{el:.1f}s", flush=True)
                    time.sleep(0.5)
                elif el > 11 and "err" not in injected:
                    injected.add("err")
                    c.sendall(pub("error/network", {
                        "type": "pub", "uuid": "inject-0001",
                        "data": {"thng_id": 1, "data": {"code": 200000, "subCode": 280002,
                                 "msg": "EVENT_ERROR_DRIVE_ERROR", "params": [1, 64]}}}))
                    print(f"[replay] ★주입 error/network 280002 @{el:.1f}s", flush=True)
    except (BrokenPipeError, ConnectionResetError, OSError):
        pass
    print(f"[replay] 종료 — {n}건 발행, PINGREQ {stop.pings}회 응답", flush=True)


s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("127.0.0.1", PORT))
s.listen(1)
print(f"[replay] 127.0.0.1:{PORT} 대기", flush=True)
c, _ = s.accept()
serve(c)
try:
    c.close()
    s.close()
except Exception:
    pass
