#!/usr/bin/env python3
"""핵심 토픽의 전체 페이로드 1건씩 — 읽기 전용."""
import socket, struct, time, json, sys

HOST="192.168.0.20"; PORT=1883
WANT={"motion/joint/position","motion/tool/position","status/operation",
      "status/safety","monitor/robot","status/robot","motion/joint/range"}

def enc_len(n):
    o=b""
    while True:
        d=n%128;n//=128
        if n>0:d|=0x80
        o+=bytes([d])
        if n==0:break
    return o
def enc_str(s):
    b=s.encode();return struct.pack("!H",len(b))+b
def read_len(sock):
    m=1;v=0
    while True:
        b=sock.recv(1)
        if not b:return None
        d=b[0];v+=(d&0x7f)*m
        if not(d&0x80):break
        m*=128
    return v

s=socket.create_connection((HOST,PORT),timeout=5)
vh=enc_str("MQTT")+bytes([4,0x02])+struct.pack("!H",30)
body=vh+enc_str("obsvr02")
s.sendall(bytes([0x10])+enc_len(len(body))+body)
s.recv(4)
sub=struct.pack("!H",1)+enc_str("#")+bytes([0])
s.sendall(bytes([0x82])+enc_len(len(sub))+sub)

s.settimeout(1.0)
seen={}
end=time.time()+10
while time.time()<end and len(seen)<len(WANT):
    try:b1=s.recv(1)
    except socket.timeout:continue
    if not b1:break
    if b1[0]>>4!=3:
        rl=read_len(s)
        if rl:
            got=0
            while got<rl:
                c=s.recv(rl-got); got+=len(c)
        continue
    rl=read_len(s)
    pl=b""
    while len(pl)<rl:
        c=s.recv(rl-len(pl))
        if not c:break
        pl+=c
    tlen=struct.unpack("!H",pl[:2])[0]
    topic=pl[2:2+tlen].decode(errors="replace")
    if topic in WANT and topic not in seen:
        try:
            obj=json.loads(pl[2+tlen:].decode(errors="replace"))
            seen[topic]=obj.get("data",{}).get("data",obj.get("data"))
        except Exception:
            seen[topic]=pl[2+tlen:].decode(errors="replace")
for t in WANT:
    print(f"\n### {t}")
    print(json.dumps(seen.get(t,"(수신 못함)"),ensure_ascii=False,indent=2)[:900])
s.close()
