#!/usr/bin/env python3
"""검증 세션 독립 재산출 — 원자료(버스 JSONL)에서 직접 다시 센다.
구현 세션의 an.py/t19.py 는 쓰지 않는다."""
import json, statistics as st, math, sys

B="/tmp/claude-1000/-home-markch04-Markch-ws-260809-Nyang-Nyang-Atlier"
F={"C":f"{B}/d1213c6f-3417-46aa-b98f-d48d37646267/scratchpad/trace_C.jsonl",
   "T19":f"{B}/d1213c6f-3417-46aa-b98f-d48d37646267/scratchpad/trace_T19.jsonl",
   "ACT":f"{B}/d1213c6f-3417-46aa-b98f-d48d37646267/scratchpad/trace_activate.jsonl",
   "A":f"{B}/04466e83-7a3c-4f05-b227-4cad6be7f394/scratchpad/A_bus.jsonl"}
KEYS=["base","shoulder","elbow","wrist1","wrist2","wrist3"]

def load(p):
    out=[]
    for line in open(p):
        try: out.append(json.loads(line))
        except: pass
    return out

def report(tag,path):
    rs=load(path)
    pos=[(r["t"],[r["d"]["data"]["data"][k] for k in KEYS]) for r in rs if r["topic"]=="motion/joint/position"]
    spd=[(r["t"],[r["d"]["data"]["data"][k] for k in KEYS]) for r in rs if r["topic"]=="motion/joint/speed"]
    pub=[(r["t"],r["d"]["uuid"],r["d"]["data"]["jointAngle"]) for r in rs if r["topic"]=="move/joint/here"]
    stop=[r for r in rs if r["topic"]=="move/stop"]
    ev=[r["t"] for r in rs if r["topic"]=="event/motion"]
    acks={r["topic"]:(r["t"],r["d"]["data"].get("code"),r["d"]["data"].get("msg")) for r in rs
          if len(r["topic"])==36 and r["topic"].count("-")==4}
    ops=[(r["t"],r["d"]["data"].get("operationStatus")) for r in rs if r["topic"]=="set/operation"]
    temps=[max(a["temp"] for a in r["d"]["data"]["data"]["axis"]) for r in rs if r["topic"]=="monitor/robot"]
    pw=[(r["d"]["data"]["data"]["power"],r["d"]["data"]["data"]["current"]) for r in rs if r["topic"]=="monitor/robot"]
    print(f"\n{'='*78}\n### {tag}  창 {rs[-1]['t']-rs[0]['t']:.1f}s · 전체 {len(rs)}줄 · position {len(pos)} · speed {len(spd)}")
    print(f"  move/joint/here **{len(pub)}건** · move/stop **{len(stop)}건** · event/motion {len(ev)}건 · set/operation {ops}")
    if temps: print(f"  축온 최고 **{max(temps)}°C** (시작 {temps[0]} 끝 {temps[-1]}) · 부하 최대 **{max(p for p,_ in pw)}W / {max(c for _,c in pw):.3f}A**")

    # ── ack 왕복 · 실패
    rt=[]; fail=0
    for t,u,_ in pub:
        if u in acks:
            at,code,msg=acks[u]; rt.append((at-t)*1000)
            if code not in (0,): fail+=1; print(f"   ⚠ ack code={code} msg={msg}")
        else: fail+=1; print(f"   ⚠ ack 없음 uuid={u[:8]} t={t:.3f}")
    if rt: print(f"  ack 왕복(ms): 최소 **{min(rt):.1f}** · 중앙 **{st.median(rt):.1f}** · 평균 **{st.mean(rt):.1f}** · 최대 **{max(rt):.1f}** (n={len(rt)}) · 실패 **{fail}건**")

    # ── 재발행 간격
    if len(pub)>1:
        gaps=[(pub[i+1][0]-pub[i][0])*1000 for i in range(len(pub)-1)]
        big=[g for g in gaps if g>1000]
        core=[g for g in gaps if g<=1000]
        print(f"  재발행 간격(ms): 최소 **{min(gaps):.1f}** · 중앙 **{st.median(core) if core else float('nan'):.1f}** · <137ms **{sum(1 for g in gaps if g<137)}건** · >1s(창 사이) {len(big)}건")

    # ── 이동 구간 판정: base 값이 변하는 구간
    if pos:
        pp=[max(p[i] for _,p in pos)-min(p[i] for _,p in pos) for i in range(6)]
        print(f"  창 전체 관절 p-p(°): "+" ".join(f"{KEYS[i]}={pp[i]:.6f}" for i in range(6)))
        dup=sum(1 for i in range(1,len(pos)) if pos[i][1]==pos[i-1][1])
        dt=[(pos[i][0]-pos[i-1][0])*1000 for i in range(1,len(pos))]
        print(f"  버스 발행률 **{len(pos)/(pos[-1][0]-pos[0][0]):.2f}Hz** · 최대 간격 **{max(dt):.2f}ms** · 직전과 동일값 **{dup/(len(pos)-1)*100:.1f}%**")
    return dict(rs=rs,pos=pos,spd=spd,pub=pub,acks=acks,ev=ev,ops=ops)

D={k:report(k,v) for k,v in F.items()}
