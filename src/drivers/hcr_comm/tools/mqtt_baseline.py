#!/usr/bin/env python3
"""실기 기준선 계측 — **읽기 전용**. 어떤 토픽에도 publish 하지 않는다.

한 번 실행으로 아래를 뽑는다 (실기가 켜져 있는 시간이 비싼 자원이라 한 세션에 몰아 담았다).

  1. 안전 기준선   `monitor/robot` — 6축 temp·current·voltage (**60°C 초과 / `-1` 감시**)
  2. 상태 기준선   `status/{operation,robot,safety}`
  3. 발행률 실측   `motion/joint/position` 수신 간격 — 평균·표준편차·**최대**
  4. speed 규명    `motion/joint/speed` 발행 여부·주기·payload (관찰만, 판정하지 않는다)
  5. 봉투 구조     `data.data` 두 겹인지 · 관절 키 6개 이름이 그대로인지
  6. 정답지 초안   최신 관절 표본 → 규약 변환 → URDF rad (**펜던트 열은 사람이 채운다**)
  7. 차분 품질     정지 상태 관절값 흔들림과 position 차분 velocity 의 튀는 폭 (중간결과물 V-5)

출력은 마크다운 표라 산출물 문서에 그대로 붙는다.

사용: `python3 mqtt_baseline.py [호스트] [수집초]`   기본 192.168.0.20 · 30초
"""
import socket, struct, sys, time, json, math, statistics

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.20"
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
PORT = 1883

TEMP_LIMIT = 60.0  # °C — 08-05 에 58~61°C 에서 6축이 0.8초 만에 트립했다 (README §8 함정 3)

# 규약 변환 — 원본은 hcr_bridge/include/hcr_bridge/joint_convention.hpp:35-47.
# 여기 값을 고치지 말 것. 원본이 바뀌면 이쪽을 맞춘다.
MQTT_KEYS = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")
URDF_NAMES = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
SIGN = (1.0, 1.0, -1.0, 1.0, 1.0, 1.0)
DELTA_DEG = (90.0, 90.0, 0.0, 90.0, 0.0, 0.0)

TOPICS = ["monitor/robot", "status/operation", "status/robot", "status/safety",
          "motion/joint/position", "motion/joint/speed"]
RATE_TOPICS = ("motion/joint/position", "motion/joint/speed")


# ── MQTT 3.1.1 최소 구현 (mqtt_sub.py 와 같은 형식) ────────────────────────────
def enc_len(n):
    out = b""
    while True:
        d = n % 128; n //= 128
        if n > 0: d |= 0x80
        out += bytes([d])
        if n == 0: break
    return out


def enc_str(s):
    b = s.encode()
    return struct.pack("!H", len(b)) + b


def connect_pkt(cid="baseline01"):
    vh = enc_str("MQTT") + bytes([4, 0x02]) + struct.pack("!H", 60)  # clean session
    body = vh + enc_str(cid)
    return bytes([0x10]) + enc_len(len(body)) + body


def subscribe_pkt(topics, pid=1):
    body = struct.pack("!H", pid)
    for t in topics:
        body += enc_str(t) + bytes([0])  # QoS0
    return bytes([0x82]) + enc_len(len(body)) + body


def read_len(sock):
    mult, val = 1, 0
    while True:
        b = sock.recv(1)
        if not b: return None
        d = b[0]; val += (d & 0x7f) * mult
        if not (d & 0x80): break
        mult *= 128
    return val


def unwrap(raw):
    """봉투 {"type":"pub","uuid":…,"data":{"thng_id":"1","data":{…}}} → (봉투, 알맹이)."""
    env = json.loads(raw)
    lvl1 = env.get("data", env)
    inner = lvl1.get("data", lvl1) if isinstance(lvl1, dict) else lvl1
    return env, inner


def stats_ms(times):
    """도착 시각 리스트 → 간격 통계(ms)."""
    if len(times) < 2: return None
    gaps = [(times[i] - times[i - 1]) * 1000.0 for i in range(1, len(times))]
    return {
        "n": len(times), "gaps": len(gaps),
        "mean": statistics.fmean(gaps),
        "sd": statistics.stdev(gaps) if len(gaps) > 1 else 0.0,
        "min": min(gaps), "max": max(gaps),
        "hz": 1000.0 / statistics.fmean(gaps),
    }


# ── 수집 ──────────────────────────────────────────────────────────────────────
s = socket.create_connection((HOST, PORT), timeout=5)
s.sendall(connect_pkt())
hdr = s.recv(1)
if not hdr or hdr[0] >> 4 != 2:
    print(f"CONNACK 아님: {hdr!r}"); sys.exit(1)
rest = s.recv(read_len(s))
rc = rest[1] if len(rest) > 1 else 255
if rc != 0:
    print(f"[CONNACK] 거부 return_code={rc}"); sys.exit(1)

s.sendall(subscribe_pkt(TOPICS))
print(f"# 실기 기준선 계측 — 읽기 전용\n")
print(f"- 대상 `{HOST}:{PORT}` · 수집 **{DURATION:.0f}초** · "
      f"시각 {time.strftime('%Y-%m-%d %H:%M:%S')}\n")

s.settimeout(1.0)
arrivals = {t: [] for t in RATE_TOPICS}
series = {t: [] for t in RATE_TOPICS}      # (도착시각, 알맹이) — §7 차분 품질용
latest, raw_first, counts = {}, {}, {}
end = time.time() + DURATION
while time.time() < end:
    try:
        b1 = s.recv(1)
    except socket.timeout:
        continue
    if not b1: break
    if b1[0] >> 4 != 3:                      # PUBLISH 만
        rl = read_len(s)
        if rl: s.recv(rl)
        continue
    now = time.perf_counter()
    rl = read_len(s)
    if rl is None: break
    payload = b""
    while len(payload) < rl:
        chunk = s.recv(rl - len(payload))
        if not chunk: break
        payload += chunk
    tlen = struct.unpack("!H", payload[:2])[0]
    topic = payload[2:2 + tlen].decode(errors="replace")
    raw = payload[2 + tlen:].decode(errors="replace")
    counts[topic] = counts.get(topic, 0) + 1
    if topic not in raw_first: raw_first[topic] = raw
    if topic in arrivals: arrivals[topic].append(now)
    try:
        inner = unwrap(raw)[1]
        latest[topic] = inner
        if topic in series: series[topic].append((now, inner))
    except Exception:
        pass
s.close()


# ── 1. 안전 기준선 ────────────────────────────────────────────────────────────
print("## 1. 안전 기준선 — 축온\n")
mon = latest.get("monitor/robot")
if not mon:
    print("⚠️ `monitor/robot` **미수신** — 판단 불가. 즉시 보고.\n")
    alarm = True
else:
    print(f"`power`=**{mon.get('power')}** · `voltage`=**{mon.get('voltage')}V** · "
          f"`current`=**{mon.get('current')}**\n")
    print("| 축 | temp (°C) | current | voltage | 판정 |")
    print("|---|---|---|---|---|")
    alarm = False
    for ax in mon.get("axis", []):
        t = ax.get("temp")
        bad = (t is None) or (t == -1) or (t > TEMP_LIMIT)
        alarm = alarm or bad
        mark = "🚨 **이상**" if bad else "정상"
        print(f"| {ax.get('name')} | **{t}** | {ax.get('current')} | {ax.get('voltage')} | {mark} |")
    print()
print(("🚨 **축온 이상 — 그 자리에서 멈추고 Mark 에게 올린다.**"
       if alarm else
       f"✅ 6축 전부 **{TEMP_LIMIT:.0f}°C 이하**, `-1` 없음 — 계속 진행 가능.") + "\n")


# ── 2. 상태 기준선 ────────────────────────────────────────────────────────────
print("## 2. 상태 기준선\n")
print("| 토픽 | 키 | 값 |")
print("|---|---|---|")
op = latest.get("status/operation") or {}
for k in ("operationStatus", "robotState", "controllerStatus",
          "directTeach", "limitCheck", "collisionMitigation", "isCollision"):
    print(f"| `status/operation` | `{k}` | **{op.get(k, '(없음)')}** |")
print(f"| `status/robot` | — | **{latest.get('status/robot', '(미수신)')}** |")
sf = latest.get("status/safety") or {}
for k in ("reduced", "normalVelocity", "reducedVelocity"):
    print(f"| `status/safety` | `{k}` | **{sf.get(k, '(없음)')}** |")
print()


# ── 3. 발행률 ────────────────────────────────────────────────────────────────
print("## 3. 상태버스 발행률 실측\n")
print("| 토픽 | 표본 | 평균 Hz | 평균 간격 | 표준편차 | 최소 | **최대** |")
print("|---|---|---|---|---|---|---|")
for t in RATE_TOPICS:
    st = stats_ms(arrivals[t])
    if not st:
        print(f"| `{t}` | **{len(arrivals[t])}** | — | — | — | — | — |")
        continue
    print(f"| `{t}` | **{st['n']}** | **{st['hz']:.2f}** | {st['mean']:.2f}ms | "
          f"{st['sd']:.2f}ms | {st['min']:.2f}ms | **{st['max']:.2f}ms** |")
print()
pos = stats_ms(arrivals["motion/joint/position"])
if pos:
    print(f"- 기준 **29.1Hz**(08-05, README §5.2) 대비 — 실측 **{pos['hz']:.2f}Hz**\n"
          f"- `read()` 무수신 타임아웃 **1.0s** 대비 최대 간격 **{pos['max']:.2f}ms** "
          f"= 여유 **{1000.0 / pos['max']:.1f}배**\n")


# ── 4. motion/joint/speed 규명 ────────────────────────────────────────────────
print("## 4. `motion/joint/speed` — 관찰만 (판정은 S1·Mark)\n")
n_sp = counts.get("motion/joint/speed", 0)
if n_sp == 0:
    print(f"**발행 안 됨** — {DURATION:.0f}초 구독 동안 **0건**. (로봇 정지 상태)\n")
else:
    print(f"**발행됨** — **{n_sp}건**\n\n원문 첫 건:\n```json\n{raw_first['motion/joint/speed'][:400]}\n```\n")
    print(f"알맹이: `{json.dumps(latest.get('motion/joint/speed'), ensure_ascii=False)[:400]}`\n")


# ── 5. 봉투 구조 재확인 ───────────────────────────────────────────────────────
print("## 5. 봉투 구조\n")
ref = raw_first.get("motion/joint/position")
if ref:
    env = json.loads(ref)
    lvl1 = env.get("data", {})
    two = isinstance(lvl1, dict) and isinstance(lvl1.get("data"), dict)
    keys = list(lvl1.get("data", {}).keys()) if two else []
    print(f"```json\n{ref[:300]}\n```\n")
    print(f"| 확인 | 결과 |\n|---|---|")
    print(f"| 봉투 키 | `{sorted(env.keys())}` |")
    print(f"| `data.data` 두 겹 | **{'예' if two else '아니오 — S1 read() 파싱 재검토 필요'}** |")
    print(f"| `thng_id` | `{lvl1.get('thng_id')}` |")
    print(f"| 관절 키 6개 | `{keys}` |")
    ok = keys == list(MQTT_KEYS)
    print(f"| 키 이름·순서가 `joint_convention.hpp:27` 과 일치 | "
          f"**{'예' if ok else '아니오 — 즉시 보고'}** |\n")
else:
    print("⚠️ `motion/joint/position` 미수신 — 확인 불가\n")


# ── 6. 대조 정답지 초안 ───────────────────────────────────────────────────────
print("## 6. 대조 정답지 (펜던트 열은 사람이 채운다)\n")
jp = latest.get("motion/joint/position")
if jp:
    print("| 관절 | 펜던트 표시 (도) | 버스 값 (도) | 규약 변환 후 URDF (rad) |")
    print("|---|---|---|---|")
    for i, k in enumerate(MQTT_KEYS):
        deg = jp.get(k)
        rad = math.radians(SIGN[i] * deg + DELTA_DEG[i]) if deg is not None else None
        print(f"| {k} / {URDF_NAMES[i]} | ⬜ | `{deg!r}` | `{rad!r}` |")
    print()
    out = [d for d in (jp.get(k) for k in MQTT_KEYS) if d is not None]
    print(f"- 공식 가동범위(±360°, J3 ±165°) 안인가 — "
          f"**{'예' if all(abs(d) <= 360 for d in out) and abs(out[2]) <= 165 else '아니오 — 확인 필요'}**")
    w2 = jp.get("wrist2")
    print(f"- `wrist2` 현재값 **{w2!r}°** — 08-05 관측 −267° 와 같은 큰 값인가: "
          f"**{'예' if w2 is not None and abs(w2) > 180 else '아니오'}**\n")
else:
    print("⚠️ `motion/joint/position` 미수신\n")

# ── 7. 차분 velocity 품질 (정지 상태) ─────────────────────────────────────────
print("## 7. 차분 velocity 품질 — 정지 상태 (중간결과물 V-5)\n")
sp = series["motion/joint/position"]
if len(sp) > 2:
    ts = [x[0] for x in sp]
    print("| 관절 | 값 SD (도) | peak-to-peak (도) | 차분 \\|v\\| 최대 (rad/s) | 차분 v RMS (rad/s) |")
    print("|---|---|---|---|---|")
    for i, k in enumerate(MQTT_KEYS):
        vals = [x[1].get(k) for x in sp]
        if any(v is None for v in vals): continue
        rads = [math.radians(SIGN[i] * v + DELTA_DEG[i]) for v in vals]
        vel = [(rads[j] - rads[j - 1]) / (ts[j] - ts[j - 1]) for j in range(1, len(rads))]
        rms = math.sqrt(statistics.fmean([v * v for v in vel]))
        print(f"| {k} | {statistics.stdev(vals):.6f} | {max(vals) - min(vals):.6f} | "
              f"**{max(abs(v) for v in vel):.4f}** | {rms:.4f} |")
    print("\n- 로봇이 **정지**해 있으므로 참값은 **0 rad/s**다. 위 수치가 곧 정지 시 헛노이즈 폭이다.\n")
else:
    print("표본 부족\n")

ss = series["motion/joint/speed"]
if ss:
    print("같은 구간 `motion/joint/speed` 값 범위 — **단위 미상, 관찰만**:\n")
    print("| 관절 | 최소 | 최대 | 평균 |")
    print("|---|---|---|---|")
    for k in MQTT_KEYS:
        vals = [x[1].get(k) for x in ss if x[1].get(k) is not None]
        if vals:
            print(f"| {k} | {min(vals):.8g} | {max(vals):.8g} | {statistics.fmean(vals):.8g} |")
    print()

print("## 수신 집계\n")
print("| 토픽 | 건수 |\n|---|---|")
for t, c in sorted(counts.items(), key=lambda x: -x[1]):
    print(f"| `{t}` | {c} |")
