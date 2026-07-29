#!/usr/bin/env python3
"""HCR-5 MQTT 명령 송신기 (순수 stdlib). 펜던트 관찰로 역설계한 pubWithAck RPC 패턴 구현.
⚠️ 이 스크립트는 로봇에 '쓰기'를 한다 — Mark의 명시적 GO + 비상정지 대기 상태에서만 실행.

패턴:
  명령 토픽에 {"type":"pubWithAck","uuid":U,"data":{"thng_id":1, ...}} publish
  → 토픽명 == U 로 {"code":0,"data":{},"msg":"success"} 응답이 옴.

서브커맨드:
  servo on|off                      set/operation
  mode manual|auto                  robot/mode
  limitcheck on|off                 set/limitCheck
  jog <joint 1-6> <pos|neg> [speed] [ms]   jogJoint/start → (ms 후 자동) jogJoint/stop
  stopjog                           jogJoint/stop (수동 즉시정지)
  send <topic> <json-data> [ack]    임의 명령
"""
import socket, struct, time, json, sys, uuid, threading

HOST="192.168.0.20"; PORT=1883

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

class Mqtt:
    def __init__(self, host, port):
        self.s=socket.create_connection((host,port),timeout=5)
        vh=enc_str("MQTT")+bytes([4,0x02])+struct.pack("!H",0)
        body=vh+enc_str("hcr-cmd-"+uuid.uuid4().hex[:6])
        self.s.sendall(bytes([0x10])+enc_len(len(body))+body)
        ca=self.s.recv(4)
        assert ca and ca[0]>>4==2 and ca[3]==0, f"CONNACK 실패 {ca!r}"
    def subscribe(self, topic):
        sub=struct.pack("!H",1)+enc_str(topic)+bytes([0])
        self.s.sendall(bytes([0x82])+enc_len(len(sub))+sub)
        # SUBACK 소비
        self.s.recv(5)
    def publish(self, topic, payload):
        pl=enc_str(topic)+payload.encode()   # QoS0, no packet id
        self.s.sendall(bytes([0x30])+enc_len(len(pl))+pl)
    def wait_publish(self, want_topic, timeout=5):
        self.s.settimeout(timeout)
        end=time.time()+timeout
        while time.time()<end:
            try: b1=self.s.recv(1)
            except socket.timeout: return None
            if not b1: return None
            if b1[0]>>4!=3:
                rl=read_len(self.s)
                if rl:
                    got=0
                    while got<rl:
                        c=self.s.recv(rl-got); got+=len(c)
                continue
            rl=read_len(self.s); pl=b""
            while len(pl)<rl:
                c=self.s.recv(rl-len(pl)); pl+=c
            tl=struct.unpack("!H",pl[:2])[0]
            topic=pl[2:2+tl].decode(errors="replace")
            if topic==want_topic:
                try: return json.loads(pl[2+tl:].decode())
                except: return {"_raw":pl[2+tl:].decode(errors='replace')}
        return None
    def close(self):
        try: self.s.close()
        except: pass

def cmd(m, topic, data, ack=True, ack_timeout=5):
    u=str(uuid.uuid1())
    if ack: m.subscribe(u)
    env={"type":"pubWithAck" if ack else "pub","uuid":u,"data":dict(data, thng_id=1)}
    print(f"→ publish [{topic}] {json.dumps(env['data'],ensure_ascii=False)}  (ack={ack})")
    if ack:
        m.publish(topic, json.dumps(env))
        r=m.wait_publish(u, timeout=ack_timeout)
        print(f"← 응답 [{u[:8]}..] {json.dumps(r,ensure_ascii=False) if r else '(응답 없음/타임아웃)'}")
        return r
    else:
        m.publish(topic, json.dumps(env)); return None

def main():
    a=sys.argv[1:]
    if not a: print(__doc__); return
    m=Mqtt(HOST,PORT)
    try:
        c=a[0]
        if c=="servo":
            cmd(m,"set/operation",{"operationStatus":"SERVO_ON" if a[1]=="on" else "SERVO_OFF"})
        elif c=="mode":
            cmd(m,"robot/mode",{"isManual": a[1]=="manual"}, ack=False)
        elif c=="limitcheck":
            cmd(m,"set/limitCheck",{"mode": a[1]=="on"})
        elif c=="jog":
            joint=int(a[1]); direction="positive" if a[2].startswith("pos") else "negative"
            speed=a[3] if len(a)>3 else "10.00"
            ms=int(a[4]) if len(a)>4 else 300
            try:
                cmd(m,"jogJoint/start",{"joint":joint,"direction":direction,"speed":str(speed)}, ack_timeout=2)
                print(f"… {ms}ms 이동")
                time.sleep(ms/1000.0)
            finally:
                # 어떤 경우에도 정지 보장
                cmd(m,"jogJoint/stop",{}, ack=False)
                print("■ jogJoint/stop 전송 (정지)")
        elif c=="stopjog":
            cmd(m,"jogJoint/stop",{}, ack=False); print("■ 정지")
        elif c=="send":
            topic=a[1]; data=json.loads(a[2]); ack=(len(a)>3 and a[3]=="ack")
            cmd(m,topic,data,ack=ack)
        else:
            print("알 수 없는 커맨드:",c); print(__doc__)
    finally:
        time.sleep(0.2); m.close()

if __name__=="__main__":
    main()
