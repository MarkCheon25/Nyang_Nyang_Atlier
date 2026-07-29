#!/usr/bin/env python3
"""HCR-5 MQTT 명령 관찰 캡처 — 읽기 전용(구독만). 로봇에 아무것도 쓰지 않는다.
펜던트 조작 시 나타나는 '명령 토픽/메시지'를 baseline(상태 브로드캐스트)과 구분해 기록한다.
사용법: capture.py <host> <port> <outdir>
종료: SIGTERM/Ctrl-C 시 요약 출력."""
import socket, struct, time, json, sys, os, signal

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.20"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 1883
OUT  = sys.argv[3] if len(sys.argv) > 3 else "."
BASELINE_SECS = 6.0

KNOWN_BASELINE = {
    "motion/tool/position","motion/joint/position","motion/flange/position",
    "status/robot","status/operation","status/safety",
    "motion/joint/range","motion/joint/speed","monitor/robot",
    "monitor/io/configurable","monitor/io/digital","monitor/io/analog","monitor/io/tool",
    "heartbeat",
}

raw_f = open(os.path.join(OUT, "raw.jsonl"), "w")
nov_f = open(os.path.join(OUT, "novel.jsonl"), "w")

def enc_len(n):
    o=b""
    while True:
        d=n%128; n//=128
        if n>0: d|=0x80
        o+=bytes([d])
        if n==0: break
    return o
def enc_str(s):
    b=s.encode(); return struct.pack("!H",len(b))+b
def read_len(sock):
    m=1; v=0
    while True:
        b=sock.recv(1)
        if not b: return None
        d=b[0]; v+=(d&0x7f)*m
        if not(d&0x80): break
        m*=128
    return v

s = socket.create_connection((HOST,PORT), timeout=5)
# CONNECT: clean session, keepalive=0 (비활성)
vh = enc_str("MQTT")+bytes([4,0x02])+struct.pack("!H",0)
body = vh + enc_str("hcr-observer-cap")
s.sendall(bytes([0x10])+enc_len(len(body))+body)
ca = s.recv(4)
if not ca or ca[0]>>4 != 2 or (len(ca)>=4 and ca[3]!=0):
    print("[ERR] CONNACK 실패:", ca.hex()); sys.exit(1)
sub = struct.pack("!H",1)+enc_str("#")+bytes([0])
s.sendall(bytes([0x82])+enc_len(len(sub))+sub)

start = time.time()
baseline = set(KNOWN_BASELINE)
learning = True
novel_topics = {}      # topic -> count
last_op = None
total = 0
last_alive = start
last_ping = start
running = True

def summary(*_):
    global running
    running = False
signal.signal(signal.SIGTERM, summary)
signal.signal(signal.SIGINT, summary)

print(f"[START] {HOST}:{PORT} '#' 구독. baseline 학습 {BASELINE_SECS}s...", flush=True)
s.settimeout(1.0)

while running:
    now = time.time()
    if now - last_ping > 20:
        try: s.sendall(bytes([0xC0,0x00]))  # PINGREQ
        except OSError: pass
        last_ping = now
    if learning and now - start >= BASELINE_SECS:
        learning = False
        print(f"[READY] baseline={len(baseline)} topics. 이제 펜던트 조작 시 새 명령 토픽을 잡습니다.", flush=True)
        print(f"        baseline: {sorted(baseline)}", flush=True)
    if now - last_alive >= 5:
        el = int(now-start)
        print(f"[ALIVE] t={el}s total={total} novel_topics={len(novel_topics)}"
              + (f" -> {sorted(novel_topics)}" if novel_topics else ""), flush=True)
        last_alive = now
    try:
        b1 = s.recv(1)
    except socket.timeout:
        continue
    except OSError:
        break
    if not b1: break
    ptype = b1[0]>>4
    rl = read_len(s)
    if rl is None: break
    pl=b""
    while len(pl)<rl:
        c=s.recv(rl-len(pl))
        if not c: break
        pl+=c
    if ptype != 3:  # PUBLISH 만 처리 (2=CONNACK,9=SUBACK,13=PINGRESP 등 무시)
        continue
    tlen = struct.unpack("!H", pl[:2])[0]
    topic = pl[2:2+tlen].decode(errors="replace")
    raw = pl[2+tlen:]
    total += 1
    try:
        obj = json.loads(raw.decode(errors="replace"))
    except Exception:
        obj = {"_raw": raw.decode(errors="replace")}
    rec = {"t": round(now-start,3), "topic": topic, "msg": obj}
    raw_f.write(json.dumps(rec, ensure_ascii=False)+"\n"); raw_f.flush()

    typ = obj.get("type") if isinstance(obj, dict) else None

    # status/operation 상태 전이 추적 (서보/모드 변화 = 조작 신호)
    if topic == "status/operation" and isinstance(obj, dict):
        d = obj.get("data",{})
        d = d.get("data",d) if isinstance(d,dict) else d
        cur = json.dumps(d, sort_keys=True) if isinstance(d,dict) else str(d)
        if cur != last_op:
            print(f"[STATE] status/operation 변경 @{rec['t']}s: {cur}", flush=True)
            last_op = cur

    if learning:
        baseline.add(topic)
        continue

    is_novel = (topic not in baseline) or (typ not in (None, "pub"))
    if is_novel:
        nov_f.write(json.dumps(rec, ensure_ascii=False)+"\n"); nov_f.flush()
        c = novel_topics.get(topic,0)
        if c < 5:
            print(f"[NOVEL] {topic}  type={typ}  {json.dumps(obj, ensure_ascii=False)[:400]}", flush=True)
        novel_topics[topic] = c+1

# 요약
print("\n=== 캡처 요약 ===", flush=True)
print(f"총 {total}건, baseline {len(baseline)}개, novel 토픽 {len(novel_topics)}개", flush=True)
for t,c in sorted(novel_topics.items(), key=lambda x:-x[1]):
    print(f"  {c:5d}회  {t}", flush=True)
raw_f.close(); nov_f.close()
try: s.close()
except Exception: pass
