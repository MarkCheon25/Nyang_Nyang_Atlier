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
  jog <joint 1-6> <positive|negative> [speed] [ms]  jogJoint/start → (ms 후 자동) jogJoint/stop
  stopjog                           jogJoint/stop (수동 즉시정지)
  clearcollision                    event/collision/clear (충돌 PAUSED 해제)
  directteach on|off                directTeaching/start|stop (핸드가이드)
  send <topic> <json-data> [ack]    임의 명령

인자 규칙 — 오타를 조용히 삼키지 않는다 (2026-08-02):
  on|off / manual|auto / positive|negative 는 **정확히 그 문자열**이어야 한다.
  다른 값이면 명령을 보내지 않고 종료코드 2로 죽는다. 축약형 pos/neg 는 허용.
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
        self.t_publish=None   # 마지막 publish 시각 — 로봇은 이 순간부터 움직인다
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
        self.t_publish=time.monotonic()
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

def die(msg):
    """인자가 틀리면 명령을 보내지 않고 죽는다. 조용한 폴백은 실기 앞에서 사고가 된다."""
    print(f"✗ {msg}", file=sys.stderr); sys.exit(2)

def arg(a, i, name, allowed):
    """a[i]를 꺼내되 allowed에 정확히 없으면 die. 오타·누락을 전부 잡는다."""
    if len(a)<=i: die(f"{name} 인자가 없다. 허용값: {' | '.join(allowed)}")
    v=a[i]
    if v not in allowed:
        die(f"{name} 인자가 '{v}'다. 허용값은 {' | '.join(allowed)} 뿐이다 "
            f"(예전에는 오타가 조용히 반대값으로 나갔다 — 그래서 막았다)")
    return v

def enum_direction(a, i):
    v=arg(a, i, "direction", ("positive","pos","negative","neg"))
    return "positive" if v.startswith("pos") else "negative"

def main():
    a=sys.argv[1:]
    if not a: print(__doc__); return
    m=Mqtt(HOST,PORT)
    try:
        c=a[0]
        if c=="servo":
            on = arg(a,1,"servo",("on","off"))=="on"
            cmd(m,"set/operation",{"operationStatus":"SERVO_ON" if on else "SERVO_OFF"})
        elif c=="mode":
            manual = arg(a,1,"mode",("manual","auto"))=="manual"
            cmd(m,"robot/mode",{"isManual": manual}, ack=False)
        elif c=="limitcheck":
            on = arg(a,1,"limitcheck",("on","off"))=="on"
            cmd(m,"set/limitCheck",{"mode": on})
        elif c=="jog":
            if len(a)<3: die("사용법: jog <joint 1-6> <positive|negative> [speed] [ms]")
            try: joint=int(a[1])
            except ValueError: die(f"joint 인자가 '{a[1]}'다. 1~6 정수여야 한다")
            if not 1<=joint<=6: die(f"joint는 1~6이어야 한다 (받은 값 {joint})")
            direction=enum_direction(a,2)
            speed=a[3] if len(a)>3 else "10.00"
            try:
                if not 0 < float(speed) <= 180: raise ValueError
            except ValueError: die(f"speed가 '{speed}'다. 0 초과 180 이하(°/s) 실수여야 한다")
            try: ms=int(a[4]) if len(a)>4 else 300
            except ValueError: die(f"ms 인자가 '{a[4]}'다. 정수(밀리초)여야 한다")
            if not 0 < ms <= 5000: die(f"ms는 1~5000이어야 한다 (받은 값 {ms})")

            # ★ ack 대기가 회전시간에 얹히지 않게 한다 (2026-08-02).
            #   로봇은 start publish를 받는 순간부터 돈다. 예전 코드는 ack를 최대 2초 기다린 뒤
            #   거기서 다시 ms를 더 잤다 → 300ms 요청이 최악의 경우 2300ms(≈23°) 회전했다.
            #   이제 ack 타임아웃을 ms 이내로 묶고, 실제 경과분을 빼고 남은 만큼만 잔다.
            budget=ms/1000.0
            ack_to=max(0.05, min(2.0, budget))
            print(f"▶ joint_{joint} {direction} {speed}°/s, 최대 {ms}ms "
                  f"(ack 대기 포함 — 총 회전시간이 {ms}ms를 넘지 않는다)")
            try:
                r=cmd(m,"jogJoint/start",{"joint":joint,"direction":direction,"speed":str(speed)},
                      ack_timeout=ack_to)
                if r is None:
                    print("⚠ ack 없음 — 명령이 수용됐는지 확인 불가. 로봇은 이미 돌고 있을 수 있다.",
                          file=sys.stderr)
                elapsed = (time.monotonic()-m.t_publish) if m.t_publish else 0.0
                remain = max(0.0, budget-elapsed)
                print(f"… ack까지 {elapsed*1000:.0f}ms 소요 → 추가 {remain*1000:.0f}ms 대기")
                time.sleep(remain)
            finally:
                # 어떤 경우에도 정지 보장
                cmd(m,"jogJoint/stop",{}, ack=False)
                print("■ jogJoint/stop 전송 (정지) — ack 없는 단방향이다. "
                      "실제 정지는 motion/joint/speed 로 눈으로 확인할 것")
        elif c=="stopjog":
            cmd(m,"jogJoint/stop",{}, ack=False); print("■ 정지")
        elif c=="clearcollision":
            cmd(m,"event/collision/clear",{})  # pubWithAck, 충돌 PAUSED 래치 해제
        elif c=="directteach":
            on = arg(a,1,"directteach",("on","off"))=="on"
            cmd(m,"directTeaching/start" if on else "directTeaching/stop",{}, ack=False)
            print("✋ 핸드가이드 " + ("ON" if on else "OFF"))
        elif c=="send":
            if len(a)<3: die("사용법: send <topic> <json-data> [ack]")
            topic=a[1]
            try: data=json.loads(a[2])
            except json.JSONDecodeError as e: die(f"json-data 파싱 실패: {e}")
            if topic.startswith("jogJoint/start"):
                die("send 로 jogJoint/start 를 직접 쏘지 마라 — 자동 정지가 없어서 "
                    "프로세스가 끝나도 로봇이 계속 돈다. 'jog' 서브커맨드를 쓸 것 "
                    "(정말 필요하면 이 가드를 지우고 stopjog 를 손에 쥐고 실행)")
            ack=(len(a)>3 and a[3]=="ack")
            cmd(m,topic,data,ack=ack)
        else:
            print("알 수 없는 커맨드:",c); print(__doc__)
    finally:
        time.sleep(0.2); m.close()

if __name__=="__main__":
    main()
