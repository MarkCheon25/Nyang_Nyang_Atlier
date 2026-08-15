import json, statistics as st, math
B="/tmp/claude-1000/-home-markch04-Markch-ws-260809-Nyang-Nyang-Atlier"
KEYS=["base","shoulder","elbow","wrist1","wrist2","wrist3"]
def load(p): 
    return [json.loads(l) for l in open(p) if l.strip()]
rs=load(f"{B}/d1213c6f-3417-46aa-b98f-d48d37646267/scratchpad/trace_T19.jsonl")
pos=[(r["t"],r["d"]["data"]["data"]["base"]) for r in rs if r["topic"]=="motion/joint/position"]
spd=[(r["t"],r["d"]["data"]["data"]["base"]) for r in rs if r["topic"]=="motion/joint/speed"]
pub=[r["t"] for r in rs if r["topic"]=="move/joint/here"]
print(f"speed 전체 {len(spd)}건 · 음수 {sum(1 for _,v in spd if v<0)}건 · 전체 범위 {min(v for _,v in spd):.6f}~{max(v for _,v in spd):.6f}")
def corr(P):
    xs=[a for a,_ in P]; ys=[b for _,b in P]
    mx,my=st.mean(xs),st.mean(ys)
    num=sum((x-mx)*(y-my) for x,y in P); den=math.sqrt(sum((x-mx)**2 for x in xs)*sum((y-my)**2 for y in ys))
    return num/den if den else float('nan')
diffs=[]
for i in range(1,len(pos)):
    dt=pos[i][0]-pos[i-1][0]
    if dt>0 and pos[i][1]!=pos[i-1][1]: diffs.append((pos[i][0],(pos[i][1]-pos[i-1][1])/dt))
t0,t1=min(pub),max(pub)
print(f"\n[방법 A] 발행창 {t0:.1f}~{t1:.1f}s · 차분표본을 가장 가까운 speed(≤50ms) 에 붙임")
P=[]; nneg=0
for t,d in diffs:
    if not (t0<=t<=t1): continue
    if d<0: nneg+=1
    s=min(spd,key=lambda x:abs(x[0]-t))
    if abs(s[0]-t)<=0.05: P.append((abs(s[1]),abs(d)))
print(f"   차분 {sum(1 for t,_ in diffs if t0<=t<=t1)}표본(음수 {nneg}) · r={corr(P):.4f} (n={len(P)})")
print(f"[방법 B] speed 표본마다 그 직전 100ms 안 차분의 중앙값과 짝지음")
P2=[]
for ts,vs in spd:
    if not (t0<=ts<=t1): continue
    win=[abs(d) for t,d in diffs if ts-0.102<t<=ts]
    if win: P2.append((abs(vs),st.median(win)))
print(f"   r={corr(P2):.4f} (n={len(P2)})")
print(f"[방법 C] |차분|>0.5°/s 인 '확실히 이동중' 표본만 (방법 A 짝짓기)")
P3=[(x,y) for x,y in P if y>0.5]
print(f"   r={corr(P3):.4f} (n={len(P3)})")
print(f"[방법 D] 부호 포함 (speed vs 차분, 절대값 아님)")
P4=[]
for t,d in diffs:
    if not (t0<=t<=t1): continue
    s=min(spd,key=lambda x:abs(x[0]-t))
    if abs(s[0]-t)<=0.05: P4.append((s[1],d))
print(f"   r={corr(P4):.4f} (n={len(P4)})  ← 부호가 없으니 음의 상관이 나야 정상")

# C2 도달값 바로잡기
rsC=load(f"{B}/d1213c6f-3417-46aa-b98f-d48d37646267/scratchpad/trace_C.jsonl")
posC=[(r["t"],r["d"]["data"]["data"]["base"]) for r in rsC if r["topic"]=="motion/joint/position"]
pubC=[(r["t"],r["d"]["data"]["jointAngle"][0]) for r in rsC if r["topic"]=="move/joint/here"]
c2=[p for p in pubC if p[0]<60]; c4=[p for p in pubC if p[0]>60]
set2=[v for t,v in posC if 60<t<67.9]; set4=[v for t,v in posC if t>78]
print(f"\n### C 도달오차 (버스 기준 재산출)")
print(f"  C2 마지막 발행목표 **{c2[-1][1]:.6f}°** (t={c2[-1][0]:.2f}) → 정착 {len(set2)}표본, 최종 **{set2[-1]:.6f}°**, 상이값 {len(set(set2))}종 → |Δ| **{abs(c2[-1][1]-set2[-1]):.6f}°**")
print(f"  C4 마지막 발행목표 **{c4[-1][1]:.6f}°** (t={c4[-1][0]:.2f}) → 정착 {len(set4)}표본, 최종 **{set4[-1]:.6f}°**, 상이값 {len(set(set4))}종 → |Δ| **{abs(c4[-1][1]-set4[-1]):.6f}°**")
print(f"  문서 기재: C2 목표 5.003035 도달 5.002967 (Δ0.000068) · C4 목표 10.003035 도달 10.002617 (Δ0.000418)")
print(f"  → 문서의 '목표' 는 버스 발행값({c2[-1][1]:.6f}/{c4[-1][1]:.6f})이 아니다. 차 {abs(c2[-1][1]-5.003035)*1:.6f}° / {abs(c4[-1][1]-10.003035):.6f}°")
