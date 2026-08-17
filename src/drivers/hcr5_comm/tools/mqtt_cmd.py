#!/usr/bin/env python3
"""HCR-5 MQTT 명령 송신기 (순수 stdlib). 펜던트 관찰로 역설계한 pubWithAck RPC 패턴 구현.
⚠️ 이 스크립트는 로봇에 '쓰기'를 한다 — Mark의 명시적 GO + 비상정지 대기 상태에서만 실행.

패턴:
  명령 토픽에 {"type":"pubWithAck","uuid":U,"data":{"thng_id":1, ...}} publish
  → 토픽명 == U 로 {"code":0,"data":{},"msg":"success"} 응답이 옴.

서브커맨드 (쓰기 = 로봇이 움직인다):
  servo on|off                      set/operation
  mode manual|auto                  robot/mode
  limitcheck on|off                 set/limitCheck
  jog <joint 1-6> <positive|negative> [speed] [ms]  jogJoint/start → (ms 후 자동) jogJoint/stop
  stopjog                           jogJoint/stop (수동 즉시정지)
  movej <j1..j6> [--max-delta D] [--timeout S]      move/joint/here — 절대 관절이동
  movestop                          move/stop (movej 중단)
  clearcollision                    event/collision/clear (충돌 PAUSED 해제)
  directteach on|off                directTeaching/start|stop (핸드가이드)
  send <topic> <json-data> [ack]    임의 명령

서브커맨드 (읽기 = 로봇이 안 움직인다):
  pos                               get/command/pos — 현재 관절각·TCP·flange
  fk <j1..j6>                       robot/convertPose — 관절각 → TCP 포즈 (순기구학)
  ik <x> <y> <z> <rx> <ry> <rz>     robot/convertJointAngle — TCP 포즈 → 관절각 (역기구학)
                                    시드는 현재 자세를 자동 사용

인자 규칙 — 오타를 조용히 삼키지 않는다 (2026-08-02):
  on|off / manual|auto / positive|negative 는 **정확히 그 문자열**이어야 한다.
  다른 값이면 명령을 보내지 않고 종료코드 2로 죽는다. 축약형 pos/neg 는 허용.

movej 안전 규칙 (2026-08-05):
  move/joint/here 는 jog 와 달리 **한 번 발행하면 목표까지 자율 주행한다** (실측: 명령 없이
  6.3초간 61° 이동 후 자력 도착). 자동 정지가 없으므로 아래 3중 가드를 둔다.
    ① 발행 전 get/command/pos 로 현재 자세를 읽어 **최대 이동량이 --max-delta(기본 5°) 이내**인지
       검증한다. 넘으면 발행하지 않고 죽는다. 현재 자세를 못 읽어도 죽는다.
    ② 목표가 공식 관절한계(±360°, J3 ±165°) 밖이면 죽는다.
    ③ 도착 신호 event/motion{"event":"moveHere"} 를 --timeout(기본 30s)까지 기다리고,
       못 받으면 finally 에서 move/stop 을 쏜다.
  ack 는 '접수'일 뿐 '도착'이 아니다 — 도착 판정은 event/motion 으로만 한다.
"""
import socket, struct, time, json, sys, uuid, threading

HOST="192.168.0.20"; PORT=1883

# 조그 실측으로 확정한 매핑 (2026-08-05) — jogJoint 의 joint 인덱스 1~6 이 이 순서다.
# 6축을 하나씩 +방향 조그해 motion/joint/position 의 어느 키가 변하는지로 확인했고,
# 축간 간섭은 없었다. URDF 의 joint_1~6 과도 같은 순서다.
JOINT_NAMES=["base","shoulder","elbow","wrist1","wrist2","wrist3"]
# 공식 가동범위 (HCR-5 User Manual v2.0 Appendix F): ±360°, J3 만 ±165°.
# URDF 값(joint_1·6 이 0~360 등)은 CAD 더미값이라 여기서 쓰지 않는다 — 업무목록 T1.
JOINT_LIMITS=[(-360.0,360.0)]*6
JOINT_LIMITS[2]=(-165.0,165.0)

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
    def wait_any(self, want_topics, timeout=5):
        """want_topics 중 하나가 도착하면 (topic, obj). 타임아웃/절단이면 (None, None).
        관심 없는 PUBLISH 와 비-PUBLISH 패킷은 소비하고 계속 기다린다 — 스트림을
        중간에 버리면 다음 패킷 경계가 어긋나므로 remaining length 만큼 반드시 읽는다."""
        end=time.time()+timeout
        while True:
            remain=end-time.time()
            if remain<=0: return (None,None)
            self.s.settimeout(remain)
            try: b1=self.s.recv(1)
            except socket.timeout: return (None,None)
            if not b1: return (None,None)
            rl=read_len(self.s)
            if rl is None: return (None,None)
            pl=b""
            while len(pl)<rl:
                c=self.s.recv(rl-len(pl))
                if not c: return (None,None)
                pl+=c
            if b1[0]>>4!=3: continue          # CONNACK/SUBACK/PINGRESP 등
            tl=struct.unpack("!H",pl[:2])[0]
            topic=pl[2:2+tl].decode(errors="replace")
            if topic in want_topics:
                try: return (topic, json.loads(pl[2+tl:].decode()))
                except Exception: return (topic, {"_raw":pl[2+tl:].decode(errors='replace')})
    def wait_publish(self, want_topic, timeout=5):
        return self.wait_any({want_topic}, timeout)[1]
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

def take_opt(a, name, default):
    """--이름 값 을 리스트에서 떼어내 (남은리스트, 값) 반환. 값이 숫자가 아니면 die."""
    if name not in a: return a, default
    i=a.index(name)
    if len(a)<=i+1: die(f"{name} 뒤에 값이 없다")
    try: v=float(a[i+1])
    except ValueError: die(f"{name} 값이 '{a[i+1]}'다 — 숫자여야 한다")
    return a[:i]+a[i+2:], v

def floats(a, n, what):
    if len(a)<n: die(f"{what} {n}개가 필요하다 (받은 것 {len(a)}개)")
    try: return [float(x) for x in a[:n]]
    except ValueError: die(f"{what}는 모두 실수여야 한다 (받은 값: {' '.join(a[:n])})")

def ack_body(r):
    """ack 는 {"type":"pub","uuid":U,"data":{"code":0,"data":{...},"msg":"success"}} 로
    한 겹 더 싸여 온다. 안쪽(code/data/msg)을 꺼낸다. 값 없는 ack 면 None."""
    if isinstance(r, dict) and isinstance(r.get("data"), dict) and "code" in r["data"]:
        return r["data"]
    return None

def query_pos(m, why):
    """get/command/pos — 관절각(도)·TCP·flange 를 한 번에. 읽기 전용 RPC."""
    b=ack_body(cmd(m,"get/command/pos",{},ack_timeout=3))
    if not b or b.get("code")!=0 or "joint" not in (b.get("data") or {}):
        die(f"현재 자세를 읽지 못했다 ({why}) — 검증 없이 움직이게 둘 수 없어 중단한다")
    return b["data"]

def show_pose(d):
    print("  관절각(도):")
    for i,(n,v) in enumerate(zip(JOINT_NAMES, d["joint"]), 1):
        print(f"    joint_{i} {n:<9} {v:10.4f}")
    for key in ("tcp","flange"):
        if key in d:
            p=d[key]["position"]; o=d[key]["orientation"]
            print(f"  {key:<6} pos(mm) x={p['x']:9.3f} y={p['y']:9.3f} z={p['z']:9.3f}"
                  f"   rot(도) {o['x']:8.3f} {o['y']:8.3f} {o['z']:8.3f}")

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
        elif c=="movej":
            a2, max_delta = take_opt(a,  "--max-delta", 5.0)
            a2, tmo       = take_opt(a2, "--timeout",  30.0)
            if len(a2)<7: die("사용법: movej <j1> <j2> <j3> <j4> <j5> <j6> "
                              "[--max-delta D] [--timeout S]")
            target=floats(a2[1:], 6, "관절각")
            if not 0 < max_delta <= 30:
                die(f"--max-delta 는 0 초과 30 이하여야 한다 (받은 값 {max_delta}). "
                    "그보다 크게 움직여야 한다면 여러 번 나눠서 갈 것")
            if not 0 < tmo <= 300: die(f"--timeout 은 0 초과 300 이하여야 한다 (받은 값 {tmo})")
            for i,(v,(lo,hi)) in enumerate(zip(target, JOINT_LIMITS), 1):
                if not lo<=v<=hi:
                    die(f"joint_{i}({JOINT_NAMES[i-1]}) 목표 {v}°가 공식 한계 {lo}~{hi}° 밖이다")

            # ① 현재 자세를 읽어 이동량을 검증한다. 못 읽으면 query_pos 안에서 죽는다.
            cur=query_pos(m, "movej 이동량 검증")["joint"]
            delta=[t-c for t,c in zip(target,cur)]
            mx=max(abs(d) for d in delta)
            print("▶ movej 이동량 검토:")
            for i,(n,c0,t0,d0) in enumerate(zip(JOINT_NAMES,cur,target,delta), 1):
                mark="  ←" if abs(d0)==mx else ""
                print(f"    joint_{i} {n:<9} {c0:10.3f} → {t0:10.3f}   Δ{d0:+8.3f}°{mark}")
            if mx>max_delta:
                die(f"최대 이동량 {mx:.3f}°가 상한 {max_delta}°를 넘는다 — 발행하지 않았다. "
                    f"의도한 값이면 --max-delta 를 올리되, 로봇 반경과 e-stop 을 먼저 확인할 것")
            print(f"  최대 이동량 {mx:.3f}° ≤ 상한 {max_delta}° — 통과. 발행한다")

            # ③ 도착 신호를 놓치지 않도록 발행 前에 구독한다.
            m.subscribe("event/motion")
            arrived=False
            try:
                # 펜던트는 2자리로 반올림해 보내지만 그건 UI 표시 정밀도일 뿐이다.
                # 입력값을 그대로 싣는다 — 컨트롤러가 어디까지 반영하는지는 §분해능 실측 참조.
                r=cmd(m,"move/joint/here",{"jointAngle":target}, ack_timeout=5)
                if r is None:
                    print("⚠ ack 없음 — 수용 여부 불명. 로봇은 이미 움직이고 있을 수 있다.",
                          file=sys.stderr)
                t0=time.monotonic()
                end=time.time()+tmo
                while time.time()<end:
                    tp,o=m.wait_any({"event/motion"}, timeout=max(0.1, end-time.time()))
                    if tp is None: break
                    ev=(o or {}).get("data",{})
                    ev=ev.get("data",ev) if isinstance(ev,dict) else ev
                    if isinstance(ev,dict) and ev.get("event")=="moveHere":
                        arrived=True
                        print(f"● 도착 event/motion moveHere — 발행 후 {time.monotonic()-t0:.2f}s")
                        break
            finally:
                if not arrived:
                    cmd(m,"move/stop",{}, ack=False)
                    print(f"■ move/stop 전송 — {tmo}s 안에 도착 이벤트를 못 받아 정지시켰다",
                          file=sys.stderr)
            fin=query_pos(m, "movej 도달 오차 확인")["joint"]
            print("◆ 도달 결과:")
            for i,(n,t0,f0) in enumerate(zip(JOINT_NAMES,target,fin), 1):
                print(f"    joint_{i} {n:<9} 목표 {t0:10.3f}  실제 {f0:10.3f}   오차 {f0-t0:+8.4f}°")
            print(f"  최대 오차 {max(abs(f0-t0) for t0,f0 in zip(target,fin)):.4f}°")
        elif c=="movestop":
            cmd(m,"move/stop",{}, ack=False); print("■ move/stop 전송 (movej 중단)")
        elif c=="pos":
            show_pose(query_pos(m, "현재 자세 조회"))
        elif c=="fk":
            joints=floats(a[1:], 6, "관절각")
            b=ack_body(cmd(m,"robot/convertPose",{"info":{"joint":joints},"poseType":"tcp"}))
            if b and b.get("code")==0:
                d=b["data"]; p=d["position"]; o=d["orientation"]
                print(f"  TCP pos(mm) x={p['x']:9.3f} y={p['y']:9.3f} z={p['z']:9.3f}")
                print(f"      rot(도)  {o['x']:9.3f} {o['y']:9.3f} {o['z']:9.3f}")
        elif c=="ik":
            p=floats(a[1:], 6, "TCP 포즈(x y z rx ry rz)")
            seed=query_pos(m, "ik 시드 자세")["joint"]
            b=ack_body(cmd(m,"robot/convertJointAngle",
                  {"info":{"position":{"x":p[0],"y":p[1],"z":p[2]},
                           "orientation":{"x":p[3],"y":p[4],"z":p[5]},
                           "joint":seed},
                   "poseType":"tcp"}))
            if b and b.get("code")==0:
                sol=b["data"]["joint"]
                print("  해:")
                for i,(n,v,s0) in enumerate(zip(JOINT_NAMES,sol,seed), 1):
                    print(f"    joint_{i} {n:<9} {v:10.4f}   (시드 대비 {v-s0:+8.3f}°)")
                print(f"  → 그대로 실행하려면: movej {' '.join(f'{v:.2f}' for v in sol)}")
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
            if topic.startswith("move/joint/here"):
                die("send 로 move/joint/here 를 직접 쏘지 마라 — 이동량 검증도 도착 대기도 "
                    "없이 목표까지 자율 주행한다. 'movej' 서브커맨드를 쓸 것 "
                    "(정말 필요하면 이 가드를 지우고 movestop 을 손에 쥐고 실행)")
            ack=(len(a)>3 and a[3]=="ack")
            cmd(m,topic,data,ack=ack)
        else:
            print("알 수 없는 커맨드:",c); print(__doc__)
    finally:
        time.sleep(0.2); m.close()

if __name__=="__main__":
    main()
