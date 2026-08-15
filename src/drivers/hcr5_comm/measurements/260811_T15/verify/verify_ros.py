import json, math, statistics as st
R=180/math.pi
for tag,f in (("C","ros_trace_C.json"),("T19","ros_trace_T19.json")):
    d=json.load(open(f)); js=d["joint_states"]; cs=d["controller_state"]
    print(f"\n{'='*80}\n### {tag} — ROS 측 독립 재산출 (요약칸은 안 본다)")
    print(f"  레코드: joint_states {len(js)} · controller_state {len(cs)} · controller_state 항목수 {len(cs[0])}")
    t0=js[0][0]; span=js[-1][0]-t0
    print(f"  /joint_states 발행 **{len(js)/span:.3f}Hz** (span {span:.1f}s)")
    # 값 갱신 (position 벡터가 직전과 다른 표본)
    chg=[js[i][0] for i in range(1,len(js)) if js[i][1]!=js[i-1][1]]
    print(f"  값 갱신 총 {len(chg)}회 · 창 전체 평균 **{len(chg)/span:.2f}Hz**")
    if chg:
        # 이동 구간 = 값 갱신이 조밀한 구간 (간격 ≤1s 로 묶는다)
        segs=[[chg[0]]]
        for t in chg[1:]:
            if t-segs[-1][-1]>1.0: segs.append([t])
            else: segs[-1].append(t)
        for i,s in enumerate([x for x in segs if len(x)>20],1):
            print(f"    이동구간{i}: t={s[0]-t0:.1f}~{s[-1]-t0:.1f}s · 갱신 {len(s)}회 · **{len(s)/(s[-1]-s[0]):.2f}Hz**")
    # 추종오차 = reference - feedback (controller_state)
    err=[]; 
    for rec in cs:
        t,a,b=rec[0],rec[1],rec[2]
        err.append((t-t0,[ (a[j]-b[j])*R for j in range(6)]))
    mx=max(err,key=lambda e:abs(e[1][0]))
    print(f"  추종오차 joint_1: 최대 **{abs(mx[1][0]):.4f}°** (t={mx[0]:.2f}s) · 최종 **{err[-1][1][0]:.4f}°** · 6축 최종 " + " ".join(f"{e:.4f}" for e in err[-1][1]))
    # velocity (joint_1)
    v=[(rec[0]-t0, rec[2][0]*R) for rec in js]
    mv=[x for x in v if abs(x[1])>1e-9]
    if mv:
        print(f"  /joint_states.velocity joint_1: **{min(x[1] for x in v):.4f} ~ {max(x[1] for x in v):.4f} °/s** · 비영 표본 {len(mv)} · 음수 {sum(1 for x in mv if x[1]<0)} / 양수 {sum(1 for x in mv if x[1]>0)}")
        # 값이 바뀐 순간만 추려 '고유 velocity 값' 분포
        uniq=[]
        for i in range(1,len(v)):
            if js[i][2]!=js[i-1][2]: uniq.append(v[i][1])
        if uniq: print(f"    고유 velocity 갱신 {len(uniq)}회 · 음수 {sum(1 for x in uniq if x<0)} / 양수 {sum(1 for x in uniq if x>0)} / 0 {sum(1 for x in uniq if x==0)}")
    # 정지 표본의 velocity 가 정확히 0 인가
    zero=[x for x in v if x[1]==0.0]
    print(f"    velocity 정확히 0.0 인 표본 {len(zero)}/{len(v)}")
