#!/usr/bin/env python3
"""구간별 재산출 — 명령 연쇄 / stop-and-go / 도달오차 / velocity / T19 부호."""
import json, statistics as st, math
B="/tmp/claude-1000/-home-markch04-Markch-ws-260809-Nyang-Nyang-Atlier"
F={"C":f"{B}/d1213c6f-3417-46aa-b98f-d48d37646267/scratchpad/trace_C.jsonl",
   "T19":f"{B}/d1213c6f-3417-46aa-b98f-d48d37646267/scratchpad/trace_T19.jsonl",
   "A":f"{B}/04466e83-7a3c-4f05-b227-4cad6be7f394/scratchpad/A_bus.jsonl"}
KEYS=["base","shoulder","elbow","wrist1","wrist2","wrist3"]
def load(p):
    rs=[]
    for l in open(p):
        try: rs.append(json.loads(l))
        except: pass
    return rs

def seg_of(pubs, gap=1.0):
    """발행 시각을 1초 이상 공백으로 끊어 '목표 연쇄' 로 묶는다."""
    segs=[[pubs[0]]]
    for p in pubs[1:]:
        if p[0]-segs[-1][-1][0] > gap: segs.append([p])
        else: segs[-1].append(p)
    return segs

for tag in ("C","T19"):
    rs=load(F[tag])
    pos=[(r["t"],[r["d"]["data"]["data"][k] for k in KEYS]) for r in rs if r["topic"]=="motion/joint/position"]
    spd=[(r["t"],[r["d"]["data"]["data"][k] for k in KEYS]) for r in rs if r["topic"]=="motion/joint/speed"]
    pub=[(r["t"],r["d"]["data"]["jointAngle"]) for r in rs if r["topic"]=="move/joint/here"]
    print(f"\n{'='*80}\n### {tag} — 목표 연쇄 분해")
    for si,s in enumerate(seg_of(pub),1):
        t0,t1=s[0][0],s[-1][0]; b0,b1=s[0][1][0],s[-1][1][0]
        gaps=[(s[i+1][0]-s[i][0])*1000 for i in range(len(s)-1)]
        steps=[abs(s[i+1][1][0]-s[i][1][0]) for i in range(len(s)-1)]
        # 도달: 마지막 발행 이후 3초 뒤~창끝 의 위치 (정착값)
        settle=[p for t,p in pos if t>t1+2.0]
        arrive=settle[-1][0] if settle else float('nan')
        uniq=len({round(p[0],9) for p in settle})
        print(f"  연쇄{si}: 발행 **{len(s)}건** t={t0:.2f}~{t1:.2f}s · base 목표 {b0:.4f}° → **{b1:.6f}°** · 스텝 중앙 {st.median(steps) if steps else 0:.4f}°")
        print(f"        간격 중앙 **{st.median(gaps) if gaps else float('nan'):.1f}ms** · 최소 {min(gaps) if gaps else float('nan'):.1f} · 도달 **{arrive:.6f}°** → |목표−도달| = **{abs(b1-arrive):.6f}°** (정착 {len(settle)}표본 중 상이값 {uniq}종)")
        # 이동 구간(첫 발행~마지막 event 후 정착 전) 통계
        mv=[(t,p) for t,p in pos if t0-0.1<=t<=t1+1.0]
        if len(mv)>3:
            dup=sum(1 for i in range(1,len(mv)) if mv[i][1]==mv[i-1][1])
            rate=len(mv)/(mv[-1][0]-mv[0][0]); chg=(len(mv)-1-dup)/(mv[-1][0]-mv[0][0])
            vel=[]
            for i in range(1,len(mv)):
                dt=mv[i][0]-mv[i-1][0]
                if dt>0 and mv[i][1]!=mv[i-1][1]: vel.append((mv[i][1][0]-mv[i-1][1][0])/dt)
            print(f"        이동구간: 발행 **{rate:.2f}Hz** · 직전동일값 **{dup/(len(mv)-1)*100:.1f}%** · 값변화 **{chg:.2f}Hz** · base 차분속도 **{min(vel):.4f} ~ {max(vel):.4f} °/s**")
        # 추종오차(버스 기준): 발행 목표 vs 그 시각의 실제 위치
        err=[]
        for t,ja in s:
            cur=[p for pt,p in pos if pt<=t]
            if cur: err.append(abs(ja[0]-cur[-1][0]))
        if err: print(f"        버스기준 추종오차(목표−발행시점 실제): 최대 **{max(err):.4f}°** · 중앙 {st.median(err):.4f}°")

# ── T19 speed 부호·상관 (역방향 구간)
rs=load(F["T19"])
pos=[(r["t"],[r["d"]["data"]["data"][k] for k in KEYS]) for r in rs if r["topic"]=="motion/joint/position"]
spd=[(r["t"],r["d"]["data"]["data"]["base"]) for r in rs if r["topic"]=="motion/joint/speed"]
pub=[r["t"] for r in rs if r["topic"]=="move/joint/here"]
t0,t1=min(pub),max(pub)+1.0
diffs=[]
for i in range(1,len(pos)):
    dt=pos[i][0]-pos[i-1][0]
    if t0<=pos[i][0]<=t1 and dt>0 and pos[i][1]!=pos[i-1][1]:
        diffs.append((pos[i][0],(pos[i][1][0]-pos[i-1][1][0])/dt))
sp_win=[(t,v) for t,v in spd if t0<=t<=t1]
neg=sum(1 for _,v in sp_win if v<0)
print(f"\n{'='*80}\n### T19 — 역방향 구간 speed 부호")
print(f"  창 t={t0:.2f}~{t1:.2f}s · position 차분 표본 **{len(diffs)}** (음수 **{sum(1 for _,d in diffs if d<0)}** / 양수 {sum(1 for _,d in diffs if d>0)})")
print(f"  같은 구간 motion/joint/speed 표본 **{len(sp_win)}** · **음수 {neg}건** · 값 범위 {min(v for _,v in sp_win):.6f} ~ {max(v for _,v in sp_win):.6f}")
# |speed| vs |diff| 상관 — 각 차분에 시각이 가장 가까운 speed 표본을 붙인다
pairs=[]
for t,d in diffs:
    s=min(sp_win,key=lambda x:abs(x[0]-t))
    if abs(s[0]-t)<0.05: pairs.append((abs(s[1]),abs(d)))
if len(pairs)>2:
    xs=[p[0] for p in pairs]; ys=[p[1] for p in pairs]
    mx,my=st.mean(xs),st.mean(ys)
    num=sum((x-mx)*(y-my) for x,y in pairs)
    den=math.sqrt(sum((x-mx)**2 for x in xs)*sum((y-my)**2 for y in ys))
    print(f"  |speed| ↔ |차분| 상관 **r = {num/den:.4f}** (n={len(pairs)})")

# ── A_bus: V-8 구간 절단
rs=load(F["A"])
pos=[(r["t"],[r["d"]["data"]["data"][k] for k in KEYS]) for r in rs if r["topic"]=="motion/joint/position"]
ops=[(r["t"],r["d"]["data"].get("operationStatus")) for r in rs if r["topic"]=="set/operation"]
print(f"\n{'='*80}\n### A_bus — V-8 서보 ON 창 구간 절단 (set/operation {ops})")
def pp(lo,hi):
    w=[p for t,p in pos if lo<=t<=hi]
    return [max(p[i] for p in w)-min(p[i] for p in w) for i in range(6)], len(w)
for lo,hi,name in [(0,45,"창 전체"),(8,14,"궤적 구간(문서 지정 t8~14s)"),(3.5,19.0,"서보 ON 안정구간")]:
    v,n=pp(lo,hi); print(f"  {name:28s} n={n:4d} " + " ".join(f"{KEYS[i]}={v[i]:.6f}" for i in range(6)))
chg=[(pos[i][0],[abs(pos[i][1][j]-pos[i-1][1][j]) for j in range(6)]) for i in range(1,len(pos))]
big=[(t,d) for t,d in chg if max(d)>1e-6]
print(f"  값이 바뀐 시각: 처음 **t={big[0][0]:.2f}s** · 마지막 **t={big[-1][0]:.2f}s** · 총 {len(big)}회")
