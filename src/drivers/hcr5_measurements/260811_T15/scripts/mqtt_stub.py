#!/usr/bin/env python3
"""V-11 경로 ③ 전용 — 최소 MQTT 스텁 브로커.
CONNECT/SUBSCRIBE/PINGREQ 에만 응답하고 **어떤 상태도 발행하지 않는다.**
→ 플러그인이 '브로커는 붙었지만 첫 표본을 못 받았다' 경로로 죽는지 보기 위한 것."""
import socket, threading, sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 1884

def handle(c):
    try:
        while True:
            h = c.recv(1)
            if not h: return
            t = h[0] >> 4
            # remaining length (가변 길이 정수)
            ln, mult = 0, 1
            while True:
                b = c.recv(1)
                if not b: return
                ln += (b[0] & 127) * mult
                if not (b[0] & 128): break
                mult *= 128
            body = c.recv(ln) if ln else b""
            if t == 1:    c.sendall(bytes([0x20, 0x02, 0x00, 0x00]))          # CONNACK
            elif t == 8:  c.sendall(bytes([0x90, 0x03, body[0], body[1], 0x00]))  # SUBACK
            elif t == 12: c.sendall(bytes([0xD0, 0x00]))                      # PINGRESP
            elif t == 14: return                                              # DISCONNECT
    except Exception:
        pass
    finally:
        c.close()

s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("127.0.0.1", PORT)); s.listen(8)
print(f"stub broker on 127.0.0.1:{PORT}", flush=True)
while True:
    c, _ = s.accept()
    threading.Thread(target=handle, args=(c,), daemon=True).start()
