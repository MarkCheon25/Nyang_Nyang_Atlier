#!/usr/bin/env python3
"""HCR-5 상태 버스 장시간 로거 — **읽기 전용**. 어떤 토픽에도 publish 하지 않는다.

## 왜 이 도구가 따로 있나

컨트롤러가 `#` 로 자기 상태 전체를 브로드캐스트하므로, **펜던트로 조작하든 PC 가 MQTT 로
명령하든 결과가 같은 버스에 실린다.** 명령 출처와 무관한 상태 수집이 이 한 곳에서 성립한다.

기존 도구와의 분업 (README §9):
  - `mqtt_sub.py`      집계만 — 신규 토픽 목록
  - `mqtt_full.py`     스키마 1건씩
  - `mqtt_baseline.py` 착수 게이트 계측 (축온·상태·발행률)
  - `mqtt_trace.py`    **T15 계측기** — 짧은 창의 전량 JSONL. 거동을 바꾸지 않는다
  - `capture.py`       명령 역설계 — baseline 대비 신규 명령 토픽 분리
  - **이 파일**        **세션 통째로** 돌려 두는 장시간 기록 — 회전·겹침·재접속·이상징후 분리

## 이번 수집의 목적 — 연속동작 중 관절값 튐 (2026-08-16 Mark 보고)

로그가 답해야 할 질문 셋에 설계가 대응한다:

  1. **정말 튀었나, 표본이 빠진 것인가** — 브로드캐스트가 끊기면 값이 튄 것처럼 보인다.
     `JumpWatch` 가 관절 차분과 **표본 간격을 같이** 본다. 갭이면 `_logger/gap`,
     갭 없이 튀었으면 `_logger/joint_jump`. 섞으면 오진한다
  2. **로봇이 뭐라고 말했나** — `error/*`·`warn/*`·`event/*` 원문 → `events_*.csv`
  3. **직전에 무슨 명령이 있었나** — 펜던트인지 PC 인지 가르는 근거 → 같은 파일

`error/command` 150033 은 메시지가 *"there might be singular points"* 로 **추정형**이라
그대로 믿으면 안 된다 (README §8 함정 1 — 08-05 의 실제 원인은 속도 프로파일이었다).
그래서 이 로거는 **판정하지 않고 원문과 정황만 모은다.**

## 실측 기준선 (`measurements/260811_T15/bus/trace_C`, 이동 58회 포함)

| 토픽 | 발행률 | 간격 최대 | 비고 |
|---|---|---|---|
| `motion/joint/position` | 28.9Hz (34.6ms) | 97.5ms | 표본간 관절 최대변화 **0.273°** |
| `motion/joint/speed`    | 9.74Hz (103ms) | 155ms | **단위 미확정** (README T19). 위치 차분과 배율이 안 맞는다 |
| `monitor/robot`         | 9.74Hz (103ms) | 154ms | 축별 `temp` (°C 정수) |

임계값은 이 분포에서 뽑았다 — 정상 세션에서 `JUMP_SOFT`·`GAP_SEC` 초과 **0건**이었다.

## 한계 — 버스에 안 실리는 조작이 있다

펜던트 **설정 화면** 조작 일부는 REST(4000/8000)로 가서 MQTT 에 전혀 안 잡힌다
(README §8, 08-05 실측 — mongoLog 0건). 툴·속도는 **"적용"까지** 해야 `robot/setup/tcp` 로 뜬다.
**이 로그의 공백이 곧 "조작이 없었다"는 아니다.**

컨트롤러가 죽으면 브로커도 같이 죽어 그 순간 *이후* 가 안 남는다. 그래서 5초 재접속과
`_logger/disconnect` 가 중요하다 — **공백의 시각 자체가 증거**가 된다.

## 사용법

    python3 mqtt_logger.py [--host 192.168.0.20] [--out DIR] [--duration SEC]

기본 무한 실행. `Ctrl-C`(SIGINT)/SIGTERM 로 종료하면 요약을 쓰고 정상 종료한다.
기본 출력은 `~/hcr5_logs` — **리포 안에 쌓지 않는다**(용량이 크고 git 대상이 아니다).
호스트에서 돈다 (랜선 프로필 `hcr5` 필요). 순수 Python 표준 라이브러리.
**화면에는 시작·종료 한 줄과 치명적 오류만 찍는다** (Mark 지시: 조용히 기록만).

## 출력 3종

    <out>/hcr5_<YYYYMMDD-HHMMSS>.csv      회전 세그먼트 — 버스 **전량**
    <out>/events_<세션시각>.csv           이상징후·명령만 — 회전 안 함, 세션당 1개
    <out>/summary_<세션시각>.md           종료 시 요약 1회

CSV 열 = `t_iso, t_rel, topic, payload` (events 는 `note` 열이 하나 더)
  - `t_iso`   절대 시각(ISO8601 밀리초, 로컬). **세그먼트 병합·겹침 대조의 키**
  - `t_rel`   세션 시작 후 경과초
  - `payload` 수신 **원문 JSON 문자열 그대로**. `data.data` 두 겹 봉투를 풀지 않는다

**압축하지 않는다** (Mark 결정). 로거 자신의 사건은 `_logger/*` 토픽으로 **두 CSV 모두에**
끼워 넣는다: `start`·`disconnect`·`reconnect`·`gap`·`joint_jump`·`joint_jump_hard`·`stop`.

## 파일 회전과 겹침 (Mark 확인 2026-08-16)

**원점은 로거 시작 시각**이다. 거기서부터 SEGMENT 씩 자른다:

    세그먼트 k = [시작 + k·SEGMENT, 시작 + (k+1)·SEGMENT)
    파일 k 수록 범위 = 그 구간의 앞뒤로 OVERLAP 씩 넓힌 것
    · 세그먼트 k 의 마지막 OVERLAP → 파일 k+1 에도 함께 쓴다
    · 세그먼트 k 의 처음  OVERLAP → 파일 k−1 에도 함께 쓴다

⇒ 파일 하나가 **7시간 분량**, 인접 파일 **겹침 1시간**, 동시 개방 최대 2개.
겹침 구간은 같은 레코드가 두 파일에 중복으로 들어간다
(`t_iso` 로 중복 제거하면 원본이 복원된다).

**파일 이름은 그 파일을 처음 연 실제 시각**이다. 세그먼트 시작 시각을 이름으로 쓰면
겹침 때문에 파일이 이름보다 30분 먼저 열려 **이름이 거짓말을 한다**.
(벽시계 epoch 로 자르던 초판은 07:22 에 시작해도 파일이 `030000` 으로 나왔다 — UTC 6시간
경계가 KST 03/09/15/21 시라서. 읽기 어려워 원점을 시작 시각으로 옮겼다. Mark 지적 08-16.)
"""
import argparse
import csv
import json
import os
import signal
import socket
import struct
import sys
import time

# ─── 상수 ────────────────────────────────────────────────────────────────────
DEFAULT_HOST = "192.168.0.20"
PORT = 1883
DEFAULT_OUT = "~/hcr5_logs"

SEGMENT_SEC = 6 * 3600      # 파일 한 개의 기준 구간 — 6시간
OVERLAP_SEC = 30 * 60       # 앞뒤로 겹쳐 쓰는 폭 — 30분 (⇒ 인접 파일 겹침 1시간)

KEEPALIVE_SEC = 60          # CONNECT 에 싣는 값
PING_INTERVAL = 20          # PINGREQ 주기. ⚠️ 빼면 브로커가 1.5×keepalive 에 끊는다
                            #    (mqtt_trace.py 의 현행 결함 · c1_tempwatch.py 커밋 6ea20fd 와 동종)
CONNECT_TIMEOUT = 3.0       # 접속 시도 자체의 상한
RECONNECT_INTERVAL = 5.0    # **고정 5초 주기로 무한 재시도** — 로봇 전원이 내려가도 계속한다.
                            #    시도에 쓴 시간을 빼고 자므로 실제 주기가 5초로 유지된다
POLL_TIMEOUT = 1.0

# ─── 관절값 튐 진단 임계 ─────────────────────────────────────────────────────
# ⚠️ 초판은 **표본간 변화량(Δ)** 을 임계로 썼다가 폐기했다. 08-11 캡처의 최대 Δ 가 0.273° 라
#    임계를 1.0° 로 잡았는데, 08-16 실기 조그는 40~63°/s 로 돌아 34ms 에 1.4~2.1° 가 정상으로
#    나온다 → **오탐 51건.** Δ 임계는 결국 "몇 °/s 넘으면 튐"이라는 뜻이고, 정상 동작 속도와
#    이상 동작 속도는 그렇게 안 갈린다. 표본이 대표성이 없었던 것 (측정에서 뽑아도 틀릴 수 있다).
#
#    바꾼 기준: **튐은 빠른 게 아니라 속도가 불연속으로 꺾이는 것**이다. 2차 차분(가속도)을 본다.
JOINT_TOPIC = "motion/joint/position"
JOINT_KEYS = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")  # 도 단위
ACC_LIMIT_DPS2 = 4000.0     # 가속도 임계(°/s²). 08-16 실기 38분(조그 6회·프로그램 4회) 실측 분포는
                            #    p50=p99=0 · p99.9=752 · **최대 1970** — 정상 동작은 속도가 매끄럽다.
                            #    그 2배로 잡았다. 이 값이 이 도구의 유일한 "감으로 잡은" 여유폭이다
JOINT_MAX_DPS = 180.0       # HCR-5 공식 관절 최대속도 (User Manual v2.0 Appendix G).
                            #    이걸 넘으면 물리적으로 불가능 → hard. 08-16 실측 최대는 63°/s
GAP_SEC = 0.20              # 표본 간격 임계(초). 정상 최대 97.5ms 의 약 2배.
                            #    넘으면 튐이 아니라 **결측** 으로 본다
MIN_DT_SEC = 0.005          # 이보다 촘촘한 두 표본은 버스트(재전송·몰림)로 본다. 실기 정상 간격은 34.6ms 라
                            #    5ms 미만이 정상 발행일 수 없다. **이 구간에서 °/s 를 내면 발산한다** —
                            #    검증 중 dt=0 에서 294337°/s 가 찍혀 넣은 방어다
BURST_MAX_DEG = JOINT_MAX_DPS * MIN_DT_SEC   # 0.9° — 5ms 안에 물리적으로 움직일 수 있는 상한.
                            #    버스트 구간은 dt 를 못 믿으니 °/s 대신 이 절대 상한으로만 판정한다

# ─── 요약이 집계할 대상 (README §3·§4.2·§4.3) ────────────────────────────────
TEMP_TOPIC = "monitor/robot"    # 알맹이 = {power, voltage, current, axis:[{name,temp,current,voltage}]}
TEMP_LIMIT = 60.0               # °C — 08-05 에 58~61°C 에서 6축이 0.8초 만에 트립
STATE_TOPICS = ("status/operation", "status/robot", "status/safety", "status/program")

# events CSV 에서 **제외**할 고빈도 브로드캐스트. 여기 없는 토픽은 전부 이상징후·명령으로 본다
# (capture.py 의 baseline 개념 — 못 보던 명령 토픽이 새로 나와도 자동으로 걸린다)
BROADCAST_TOPICS = frozenset({
    "motion/joint/position", "motion/tool/position", "motion/flange/position",
    "motion/joint/range", "motion/joint/speed",
    "monitor/robot", "monitor/io/configurable", "monitor/io/digital",
    "monitor/io/analog", "monitor/io/tool",
    "heartbeat", "mongoLog",        # mongoLog 는 양이 많아 원본에만 남긴다
    *STATE_TOPICS,                  # status/* 는 **내용이 바뀔 때만** events 에 남긴다
})

CSV_HEADER = ("t_iso", "t_rel", "topic", "payload")
EVENT_HEADER = ("t_iso", "t_rel", "topic", "payload", "note")
MAX_LIST = 200                  # 요약이 들고 있을 표본 상한 (장시간 세션 메모리 방어)


class BusLost(Exception):
    """연결이 끊겼다 — 재접속 루프로 돌아간다."""


def iso(ts):
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(ts)) + f".{int((ts % 1) * 1000):03d}"


def stamp(ts):
    return time.strftime("%Y%m%d-%H%M%S", time.localtime(ts))


def unwrap(payload):
    """봉투 {"type":"pub","uuid":…,"data":{"thng_id":"1","data":{…}}} → 알맹이 dict (없으면 None).

    원본은 `mqtt_baseline.py:78`. 두 겹이 아닌 토픽(`modbus/device/error` 등)도 있어
    한 겹씩 벗기며 dict 인 동안만 내려간다.
    """
    try:
        env = json.loads(payload)
    except Exception:
        return None
    if not isinstance(env, dict):
        return None
    lvl1 = env.get("data", env)
    if not isinstance(lvl1, dict):
        return None
    inner = lvl1.get("data", lvl1)
    return inner if isinstance(inner, dict) else lvl1


# ─── MQTT 최소 클라이언트 (구독 전용) ────────────────────────────────────────
def _enc_len(n):
    out = b""
    while True:
        d = n % 128
        n //= 128
        if n > 0:
            d |= 0x80
        out += bytes([d])
        if n == 0:
            break
    return out


def _enc_str(s):
    b = s.encode()
    return struct.pack("!H", len(b)) + b


class BusReader:
    """`#` 구독 전용 MQTT 리더. **publish 메서드를 두지 않는다** — 실수로도 로봇에 못 쓴다."""

    def __init__(self, host, port=PORT, client_id="hcr5-logger"):
        self.host = host
        self.port = port
        self.client_id = client_id
        self.sock = None
        self._last_ping = 0.0

    def connect(self):
        s = socket.create_connection((self.host, self.port), timeout=CONNECT_TIMEOUT)
        s.settimeout(CONNECT_TIMEOUT)
        vh = _enc_str("MQTT") + bytes([4, 0x02]) + struct.pack("!H", KEEPALIVE_SEC)
        body = vh + _enc_str(self.client_id)
        s.sendall(bytes([0x10]) + _enc_len(len(body)) + body)

        hdr = s.recv(1)
        if not hdr or hdr[0] >> 4 != 2:
            s.close()
            raise OSError(f"CONNACK 아님: {hdr!r}")
        self.sock = s
        rl = self._read_len()
        rest = self._recv_exact(rl) if rl else b""
        rc = rest[1] if len(rest) > 1 else 255
        if rc != 0:
            s.close()
            self.sock = None
            raise OSError(f"브로커 거부 rc={rc}")

        sub = struct.pack("!H", 1) + _enc_str("#") + bytes([0])   # QoS0
        s.sendall(bytes([0x82]) + _enc_len(len(sub)) + sub)
        s.settimeout(POLL_TIMEOUT)
        self._last_ping = time.time()

    def _recv_exact(self, n):
        """정확히 n 바이트. 패킷 **중간에** 끊기거나 timeout 이면 BusLost.

        (헤더만 읽고 나머지를 못 읽으면 스트림이 어긋난다. 그 상태로 계속 읽으면
        토픽이 깨진 채 기록되므로, 애매하면 끊고 재접속하는 편이 맞다.)
        """
        buf = b""
        while len(buf) < n:
            try:
                chunk = self.sock.recv(n - len(buf))
            except socket.timeout:
                raise BusLost("패킷 중간 timeout")
            except OSError as e:
                raise BusLost(f"수신 오류: {e}")
            if not chunk:
                raise BusLost("EOF")
            buf += chunk
        return buf

    def _read_len(self):
        mult, val = 1, 0
        while True:
            d = self._recv_exact(1)[0]
            val += (d & 0x7f) * mult
            if not (d & 0x80):
                return val
            mult *= 128
            if mult > 128 ** 3:
                raise BusLost("Remaining Length 이상")

    def poll(self):
        """PUBLISH 1건을 `(topic, payload_str)` 로. 조용하면 None, 끊기면 BusLost."""
        now = time.time()
        if now - self._last_ping > PING_INTERVAL:
            try:
                self.sock.sendall(bytes([0xC0, 0x00]))      # PINGREQ
            except OSError as e:
                raise BusLost(f"PINGREQ 실패: {e}")
            self._last_ping = now
        try:
            b1 = self.sock.recv(1)
        except socket.timeout:
            return None
        except OSError as e:
            raise BusLost(f"수신 오류: {e}")
        if not b1:
            raise BusLost("EOF")

        ptype = b1[0] >> 4
        rl = self._read_len()
        payload = self._recv_exact(rl) if rl else b""
        if ptype != 3:                  # CONNACK·SUBACK·PINGRESP 는 버린다
            return None
        tlen = struct.unpack("!H", payload[:2])[0]
        topic = payload[2:2 + tlen].decode(errors="replace")
        return topic, payload[2 + tlen:].decode(errors="replace")

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass
        self.sock = None


# ─── 회전·겹침 기록기 ────────────────────────────────────────────────────────
class SegmentWriter:
    """벽시계 기준 세그먼트로 CSV 를 회전하고, 경계 앞뒤 OVERLAP 을 이웃 파일에 겹쳐 쓴다."""

    def __init__(self, outdir, segment=SEGMENT_SEC, overlap=OVERLAP_SEC, anchor_ts=0.0):
        self.outdir = outdir
        self.segment = segment
        self.overlap = overlap
        self.anchor = anchor_ts     # 세그먼트 0 의 시작 시각 = 로거 시작 시각
        self.open = {}          # k -> (file, writer)
        self._names = {}        # k -> 경로. 이름은 **그 파일을 처음 연 실제 시각**으로 한 번만 정한다
        self.paths = []         # 생성 순서대로 (요약용)
        self.rows = 0           # 수신 레코드 수 (겹침 중복 제외)
        self.written = 0        # 실제로 파일에 쓴 물리 행 수 (겹침 중복 포함)

    @staticmethod
    def segments_for(ts, segment=SEGMENT_SEC, overlap=OVERLAP_SEC, anchor=0.0):
        """시각 `ts` 가 기록돼야 할 세그먼트 인덱스 목록.

        `anchor` 는 세그먼트 0 의 시작 시각(= 로거 시작 시각). 여기를 원점으로 6시간씩 자른다.
        **이 함수가 겹침 규칙의 단일 원본이다 — 검증도 여기만 겨눈다.**
        """
        rel = ts - anchor
        k = int(rel // segment)
        off = rel - k * segment
        segs = [k]
        if off < overlap and k - 1 >= 0:
            # ⚠️ k−1 ≥ 0 조건이 필수다. 원점이 시작 시각이므로 **세그먼트 −1 은 존재할 수 없다**
            #    (세션 시작 이전). 이 조건이 없으면 첫 레코드(off=0)가 세그먼트 −1 에도 쓰이고,
            #    두 파일이 같은 초에 열려 이름까지 같아져 **한 파일에 같은 행이 두 번** 들어간다.
            segs.append(k - 1)                  # 이전 파일의 '뒤 30분'
        if off >= segment - overlap:
            segs.append(k + 1)                  # 다음 파일의 '앞 30분'
        return segs

    def _path(self, k):
        """세그먼트 k 의 파일 경로. 이름은 **그 파일을 처음 여는 실제 시각**이다.

        세그먼트 시작 시각을 이름으로 쓰면 겹침 때문에 파일이 이름보다 30분 먼저 열려
        이름이 거짓말을 한다. 파일을 처음 연 시각을 쓰면 이름과 내용이 어긋나지 않는다.
        """
        p = self._names.get(k)
        if p is None:
            base = os.path.join(self.outdir, f"hcr5_{stamp(time.time())}")
            p, n = base + ".csv", 1
            while p in self._names.values():     # 같은 초에 두 파일이 열려도 이름이 안 겹치게
                n += 1
                p = f"{base}_{n}.csv"
            self._names[k] = p
        return p

    def _handle(self, k):
        h = self.open.get(k)
        if h is None:
            p = self._path(k)
            fresh = (not os.path.exists(p)) or os.path.getsize(p) == 0
            f = open(p, "a", newline="", encoding="utf-8")
            w = csv.writer(f)
            if fresh:
                w.writerow(CSV_HEADER)
            h = self.open[k] = (f, w)
            if p not in self.paths:
                self.paths.append(p)
        return h

    def write(self, ts, t_rel, topic, payload):
        segs = self.segments_for(ts, self.segment, self.overlap, self.anchor)
        row = (iso(ts), f"{t_rel:.3f}", topic, payload)
        for k in segs:
            f, w = self._handle(k)
            w.writerow(row)
            f.flush()           # 전원이 갑자기 내려가도 직전까지 남아야 한다 (29Hz 라 부담 없음)
        self.rows += 1
        self.written += len(segs)
        for k in [k for k in self.open if k not in segs]:
            self.open.pop(k)[0].close()

    def close(self):
        for k in list(self.open):
            self.open.pop(k)[0].close()


class EventLog:
    """이상징후·명령만 모으는 세션 단일 CSV. 회전하지 않는다 (작다)."""

    def __init__(self, outdir, session_ts):
        self.path = os.path.join(outdir, f"events_{stamp(session_ts)}.csv")
        fresh = (not os.path.exists(self.path)) or os.path.getsize(self.path) == 0
        self.f = open(self.path, "a", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        if fresh:
            self.w.writerow(EVENT_HEADER)
        self._last_state = {}
        self.rows = 0

    def consider(self, ts, t_rel, topic, payload, inner):
        """남길 것인지 판단하고 남긴다. `note` 에 남긴 이유를 적는다."""
        # ⚠️ 접두만 보면 안 된다 — `modbus/device/error` 는 `error/` 로 시작하지 않는다.
        #    잡히기는 하되 `명령/응답` 으로 잘못 분류됐다 (2026-08-16 실기 수집 중 발견)
        if topic.startswith(("error/", "warn/", "event/")) or "error" in topic or "warn" in topic:
            note = "이벤트"
        elif topic.startswith("_logger/"):
            note = "로거"
        elif topic in STATE_TOPICS:
            cur = json.dumps(inner, sort_keys=True, ensure_ascii=False) if inner is not None else payload
            if self._last_state.get(topic) == cur:
                return
            self._last_state[topic] = cur
            note = "상태전이"
        elif topic in BROADCAST_TOPICS:
            return
        else:
            note = "명령/응답"          # 명령 토픽 · ack 응답(uuid 토픽) · 미분류
        self.write(ts, t_rel, topic, payload, note)

    def write(self, ts, t_rel, topic, payload, note=""):
        self.w.writerow((iso(ts), f"{t_rel:.3f}", topic, payload, note))
        self.f.flush()
        self.rows += 1

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass


# ─── 관절값 튐 감시 ──────────────────────────────────────────────────────────
class JumpWatch:
    """`motion/joint/position` 을 표본 간격 · 속도 · **가속도** 로 본다.

    결측과 튐을 구분하는 것이 존재 이유다:
      - dt > GAP_SEC                  → `gap`             (값이 튄 게 아니라 표본이 빠졌다)
      - dt 정상 & |Δ/dt| > 180°/s     → `joint_jump_hard`  (물리적으로 불가능)
      - dt 정상 & |Δv/dt| > ACC_LIMIT → `joint_jump`       (속도 불연속 = 튐 후보)
      - dt < 5ms & |Δ| > 0.9°         → `joint_jump_hard`  (버스트 — dt 를 못 믿으니 절대 상한으로)
    gap 이면 gap 만 낸다 — 간격이 벌어진 구간의 차분은 근거가 못 된다.

    ⚠️ **속도 상태는 gap·버스트 뒤에 반드시 버린다.** 못 믿을 dt 로 낸 속도를 다음 표본의
       가속도 계산에 물리면 그 자리에서 가짜 튐이 하나 만들어진다.

    화면에 경고하지 않는다. 기록만 남기고 판정은 사람이 한다.
    """

    def __init__(self, acc_limit=ACC_LIMIT_DPS2, hard_dps=JOINT_MAX_DPS, gap_sec=GAP_SEC):
        self.acc = acc_limit
        self.hard = hard_dps
        self.gap = gap_sec
        self.prev = None            # (ts, [6])
        self.prev_v = None          # [6] °/s — 직전 구간의 속도

    def feed(self, ts, inner):
        """관절 표본 1건 → 이상이면 `(kind, detail)`, 아니면 None."""
        if not isinstance(inner, dict):
            return None
        vals = [inner.get(k) for k in JOINT_KEYS]
        if any(not isinstance(v, (int, float)) for v in vals):
            return None
        prev, self.prev = self.prev, (ts, vals)
        if prev is None:
            return None
        dt = ts - prev[0]
        if dt <= 0:
            return None
        if dt > self.gap:
            self.prev_v = None
            return "gap", {"dt": round(dt, 4), "since": iso(prev[0]),
                           "note": "표본 결측 — 이 구간의 관절 차분은 근거가 못 된다"}

        def at(j, **extra):
            d = {"joint": JOINT_KEYS[j], "delta_deg": round(vals[j] - prev[1][j], 4),
                 "dt": round(dt, 4), "prev": round(prev[1][j], 4), "cur": round(vals[j], 4)}
            d.update(extra)
            return d

        if dt < MIN_DT_SEC:
            # 버스트 — °/s 가 발산하므로 내지 않고, 속도 상태도 끊는다
            self.prev_v = None
            i = max(range(6), key=lambda j: abs(vals[j] - prev[1][j]))
            if abs(vals[i] - prev[1][i]) <= BURST_MAX_DEG:
                return None
            return "joint_jump_hard", at(i, deg_per_s=None,
                                         note=f"dt<{MIN_DT_SEC*1000:.0f}ms 버스트 — "
                                              f"{BURST_MAX_DEG:.1f}° 를 넘어 물리적으로 불가능")

        v = [(vals[j] - prev[1][j]) / dt for j in range(6)]
        prev_v, self.prev_v = self.prev_v, v

        i = max(range(6), key=lambda j: abs(v[j]))
        if abs(v[i]) > self.hard:
            return "joint_jump_hard", at(i, deg_per_s=round(v[i], 2))

        if prev_v is None:
            return None                     # 가속도는 연속한 두 구간이 있어야 나온다
        acc = [abs(v[j] - prev_v[j]) / dt for j in range(6)]
        k = max(range(6), key=lambda j: acc[j])
        if acc[k] <= self.acc:
            return None
        return "joint_jump", at(k, deg_per_s=round(v[k], 2),
                                prev_deg_per_s=round(prev_v[k], 2),
                                deg_per_s2=round(acc[k], 1))


# ─── 요약 ────────────────────────────────────────────────────────────────────
class Summary:
    """수신하며 누적만 하고 판정하지 않는다. 종료 시 마크다운 1장으로 떨군다."""

    def __init__(self, session_ts, host, port=PORT):
        self.session_ts = session_ts
        self.host = host
        self.port = port
        self.first_ts = None
        self.last_ts = None
        self.topics = {}
        self.temp_peak = {}
        self.temp_sum = {}
        self.temp_n = {}
        self.temp_over = []
        self.states = []
        self._last_state = {}
        self.errors = []
        self.cmds = {}
        self.j_n = 0
        self.j_dt_sum = 0.0
        self.j_dt_max = 0.0
        self.j_dt_top = []
        self.jumps = []
        self.gaps = []
        self.links = []

    def feed(self, ts, topic, payload, inner):
        self.first_ts = ts if self.first_ts is None else self.first_ts
        self.last_ts = ts
        self.topics[topic] = self.topics.get(topic, 0) + 1

        if topic == TEMP_TOPIC and isinstance(inner, dict):
            for a in inner.get("axis", []) or []:
                n, t = a.get("name"), a.get("temp")
                if n is None or not isinstance(t, (int, float)):
                    continue
                self.temp_peak[n] = max(self.temp_peak.get(n, -999), t)
                self.temp_sum[n] = self.temp_sum.get(n, 0) + t
                self.temp_n[n] = self.temp_n.get(n, 0) + 1
                if (t > TEMP_LIMIT or t == -1) and len(self.temp_over) < MAX_LIST:
                    self.temp_over.append((ts, n, t))
        elif topic in STATE_TOPICS and inner is not None:
            cur = json.dumps(inner, sort_keys=True, ensure_ascii=False)
            if self._last_state.get(topic) != cur:
                self._last_state[topic] = cur
                if len(self.states) < MAX_LIST:
                    self.states.append((ts, topic, cur[:200]))
        elif topic.startswith(("error/", "warn/")) or topic == "modbus/device/error":
            if len(self.errors) < MAX_LIST:
                self.errors.append((ts, topic, payload[:300]))
        elif not topic.startswith("_logger/") and topic not in BROADCAST_TOPICS:
            c = self.cmds.setdefault(topic, [0, ts, ts])
            c[0] += 1
            c[2] = ts

    def feed_interval(self, dt):
        self.j_n += 1
        self.j_dt_sum += dt
        self.j_dt_max = max(self.j_dt_max, dt)
        self.j_dt_top.append(dt)
        if len(self.j_dt_top) > 2000:
            self.j_dt_top = sorted(self.j_dt_top, reverse=True)[:MAX_LIST]

    def note_anomaly(self, kind, ts, detail):
        bucket = self.gaps if kind == "gap" else self.jumps
        if len(bucket) < MAX_LIST:
            bucket.append((ts, kind, detail))

    def note_link(self, ts, kind, detail):
        if len(self.links) < MAX_LIST:
            self.links.append((ts, kind, detail))

    def render(self, seg, ev):
        total = sum(self.topics.values())
        dur = (self.last_ts - self.first_ts) if self.first_ts else 0.0
        L = []
        L.append(f"# HCR-5 버스 수집 요약 — {stamp(self.session_ts)}\n")
        L.append("> 이 문서는 **집계만** 한다. 원인 판정은 하지 않는다 "
                 "(`error/command` 150033 은 메시지가 추정형이라 그대로 믿으면 안 된다 — README §8 함정 1).\n")

        L.append("## 1. 세션 개요\n")
        L.append("| 항목 | 값 |")
        L.append("|---|---|")
        L.append(f"| 대상 | `{self.host}:{self.port}` |")
        L.append(f"| 구간 | {iso(self.session_ts)} → {iso(time.time())} |")
        L.append(f"| 수신 구간 길이 | {dur:.1f}s ({dur/3600:.2f}h) |")
        L.append(f"| 총 레코드 | {total}건 · 토픽 {len(self.topics)}종 |")
        L.append(f"| 원본 레코드 | {seg.rows}건 (겹침 제외) |")
        dupe = seg.written - seg.rows
        L.append(f"| 원본 물리 행 | {seg.written}행 — 겹침 중복 {dupe}건 "
                 f"({dupe/seg.rows*100 if seg.rows else 0:.1f}%) |")
        L.append(f"| events 행 | {ev.rows}건 → `{os.path.basename(ev.path)}` |")
        L.append("")
        L.append("생성 파일:\n")
        for p in seg.paths:
            sz = os.path.getsize(p) if os.path.exists(p) else 0
            L.append(f"- `{os.path.basename(p)}` — {sz/1e6:.1f}MB")
        L.append("")

        L.append("## 2. 연결 — 로그 공백의 근거\n")
        if not self.links:
            L.append("끊김 없음.\n")
        else:
            L.append("| 시각 | 사건 | 내용 |")
            L.append("|---|---|---|")
            for ts, kind, d in self.links:
                L.append(f"| {iso(ts)} | `{kind}` | {json.dumps(d, ensure_ascii=False)[:160]} |")
            L.append("")
            L.append("> 이 구간은 **로봇이 안 보낸 것이 아니라 로거가 못 들은 것**이다. 공백을 조작 없음으로 읽지 말 것.\n")

        L.append("## 3. 관절값 튐 후보 ★\n")
        thr = (f"임계: **가속도 {ACC_LIMIT_DPS2:.0f}°/s²** · hard **{JOINT_MAX_DPS}°/s**(물리 불가). "
               f"08-16 실기 38분 실측은 가속도 최대 1970°/s² · 속도 최대 63°/s 였다.")
        if not self.jumps:
            L.append(f"없음. {thr}\n")
        else:
            L.append(thr + "\n")
            L.append("| 시각 | 종류 | 관절 | Δ(도) | dt(ms) | °/s | °/s² | 직전 → 현재 |")
            L.append("|---|---|---|---|---|---|---|---|")
            for ts, kind, d in self.jumps:
                dps = d.get("deg_per_s")
                dps_s = f"{dps:+.1f}" if isinstance(dps, (int, float)) else "— (버스트)"
                a = d.get("deg_per_s2")
                a_s = f"{a:,.0f}" if isinstance(a, (int, float)) else "—"
                L.append(f"| {iso(ts)} | `{kind}` | {d['joint']} | {d['delta_deg']:+.4f} | "
                         f"{d['dt']*1000:.1f} | {dps_s} | {a_s} | {d['prev']:.4f} → {d['cur']:.4f} |")
            L.append("")
            L.append("> `joint_jump` 은 **속도가 꺾인** 자리다 — 값이 큰 곳이 아니라. "
                     "직전 속도(`prev_deg_per_s`)와 견줘 보려면 events CSV 원문을 볼 것.\n")

        L.append("## 4. 표본 간격 — 튐인지 결측인지 가르는 축\n")
        if self.j_n:
            L.append("| 항목 | 값 |")
            L.append("|---|---|")
            L.append(f"| `{JOINT_TOPIC}` 표본 | {self.j_n}건 |")
            L.append(f"| 평균 간격 | {self.j_dt_sum/self.j_n*1000:.1f}ms (≈{self.j_n/self.j_dt_sum:.1f}Hz) |")
            L.append(f"| 최대 간격 | {self.j_dt_max*1000:.1f}ms |")
            L.append(f"| GAP({GAP_SEC*1000:.0f}ms) 초과 | {len(self.gaps)}건 |")
            L.append("")
            top = sorted(self.j_dt_top, reverse=True)[:10]
            L.append("상위 간격: " + ", ".join(f"{d*1000:.0f}ms" for d in top) + "\n")
        else:
            L.append("표본 없음.\n")
        if self.gaps:
            L.append("| 시각 | dt(ms) | 직전 표본 |")
            L.append("|---|---|---|")
            for ts, _, d in self.gaps:
                L.append(f"| {iso(ts)} | {d['dt']*1000:.0f} | {d['since']} |")
            L.append("")

        L.append("## 5. 에러·경고 원문\n")
        if not self.errors:
            L.append("없음.\n")
        else:
            L.append("| 시각 | 토픽 | 원문(앞 300자) |")
            L.append("|---|---|---|")
            for ts, t, p in self.errors:
                L.append(f"| {iso(ts)} | `{t}` | `{p.replace('|', '\\|')}` |")
            L.append("")

        L.append("## 6. 상태 전이\n")
        if not self.states:
            L.append("변화 없음.\n")
        else:
            L.append("| 시각 | 토픽 | 알맹이 |")
            L.append("|---|---|---|")
            for ts, t, c in self.states:
                L.append(f"| {iso(ts)} | `{t}` | `{c.replace('|', '\\|')}` |")
            L.append("")

        L.append("## 7. 축온\n")
        if not self.temp_peak:
            L.append("`monitor/robot` 미수신.\n")
        else:
            L.append("| 축 | 최고(°C) | 평균(°C) | 판정 |")
            L.append("|---|---|---|---|")
            for n in sorted(self.temp_peak):
                avg = self.temp_sum[n] / self.temp_n[n]
                bad = self.temp_peak[n] > TEMP_LIMIT or self.temp_peak[n] == -1
                L.append(f"| {n} | **{self.temp_peak[n]}** | {avg:.1f} | "
                         f"{'🚨 한계 초과' if bad else '정상'} |")
            L.append("")
            if self.temp_over:
                L.append(f"⚠️ {TEMP_LIMIT:.0f}°C 초과/`-1` **{len(self.temp_over)}건** — "
                         f"최초 {iso(self.temp_over[0][0])} ({self.temp_over[0][1]} {self.temp_over[0][2]}°C)\n")

        L.append("## 8. 명령·응답 타임라인\n")
        if not self.cmds:
            L.append("없음 — 이 세션에 버스로 나간 명령이 없다 (또는 REST 로 갔다).\n")
        else:
            L.append("| 토픽 | 건수 | 최초 | 최종 |")
            L.append("|---|---|---|---|")
            for t, (n, f_, l_) in sorted(self.cmds.items(), key=lambda x: -x[1][0]):
                L.append(f"| `{t}` | {n} | {iso(f_)} | {iso(l_)} |")
            L.append("")

        L.append("## 9. 토픽별 건수\n")
        L.append("| 건수 | 토픽 |")
        L.append("|---|---|")
        for t, n in sorted(self.topics.items(), key=lambda x: -x[1]):
            L.append(f"| {n} | `{t}` |")
        L.append("")
        return "\n".join(L)


# ─── 본체 ────────────────────────────────────────────────────────────────────
def parse_args(argv):
    p = argparse.ArgumentParser(description="HCR-5 상태 버스 장시간 로거 (읽기 전용)")
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--port", type=int, default=PORT, help="실기는 1883 고정. 스텁 브로커 검증용으로만 바꾼다")
    p.add_argument("--out", default=DEFAULT_OUT)
    p.add_argument("--duration", type=float, default=None, help="초. 생략하면 무한")
    p.add_argument("--acc-limit", type=float, default=ACC_LIMIT_DPS2,
                   help="가속도 임계(°/s²). 실측 최대 1970 의 2배가 기본")
    p.add_argument("--gap-sec", type=float, default=GAP_SEC)
    p.add_argument("--segment-sec", type=float, default=SEGMENT_SEC)
    p.add_argument("--overlap-sec", type=float, default=OVERLAP_SEC)
    return p.parse_args(argv)


def main(argv=None):
    a = parse_args(argv)
    outdir = os.path.expanduser(a.out)
    os.makedirs(outdir, exist_ok=True)

    session_ts = time.time()
    seg = SegmentWriter(outdir, a.segment_sec, a.overlap_sec, anchor_ts=session_ts)
    ev = EventLog(outdir, session_ts)
    jw = JumpWatch(a.acc_limit, JOINT_MAX_DPS, a.gap_sec)
    summ = Summary(session_ts, a.host, a.port)

    running = {"go": True}

    def stop(*_):
        running["go"] = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def emit(ts, topic, payload):
        t_rel = ts - session_ts
        seg.write(ts, t_rel, topic, payload)
        inner = unwrap(payload)
        ev.consider(ts, t_rel, topic, payload, inner)
        summ.feed(ts, topic, payload, inner)
        return inner

    def emit_logger(kind, detail):
        ts = time.time()
        emit(ts, f"_logger/{kind}", json.dumps(detail, ensure_ascii=False))
        return ts

    emit_logger("start", {
        "host": a.host, "port": a.port, "pid": os.getpid(),
        "segment_sec": a.segment_sec, "overlap_sec": a.overlap_sec,
        "acc_limit_dps2": a.acc_limit, "jump_hard_dps": JOINT_MAX_DPS, "gap_sec": a.gap_sec,
        "note": "읽기 전용 — publish 하지 않는다",
    })
    print(f"[START] {a.host}:{a.port} → {outdir}  (Ctrl-C 로 종료, 요약 자동 생성)", file=sys.stderr, flush=True)

    reader = None
    lost_at = None          # 끊긴 시각 (None 이면 연결 정상)
    attempts = 0
    disc_logged = False

    try:
        while running["go"]:
            if a.duration is not None and time.time() - session_ts >= a.duration:
                break

            # ── 재접속 구간 — 5초 고정 주기 무한 재시도 ──────────────────
            if reader is None:
                t0 = time.time()
                try:
                    r = BusReader(a.host, a.port)
                    r.connect()
                    reader = r
                    if lost_at is not None:
                        ts = emit_logger("reconnect", {
                            "attempts": attempts, "gap_sec": round(time.time() - lost_at, 2),
                            "lost_at": iso(lost_at)})
                        summ.note_link(ts, "reconnect",
                                       {"attempts": attempts, "gap_sec": round(time.time() - lost_at, 2)})
                    lost_at = None
                    attempts = 0
                    disc_logged = False
                except Exception as e:
                    attempts += 1
                    if lost_at is None:
                        lost_at = t0
                    if not disc_logged:
                        # 전원이 몇 시간 내려가 있으면 5초마다 실패한다 — 첫 1건만 남기고 세지기만 한다
                        ts = emit_logger("disconnect", {"reason": str(e)[:200], "at": iso(t0)})
                        summ.note_link(ts, "disconnect", {"reason": str(e)[:120]})
                        disc_logged = True
                    # ⚠️ 목표 시각을 **고정**해서 비교한다. 누적 sleep 과 '남은 시간'을 맞비교하면
                    #    둘이 마주 달려 절반에서 빠져나온다 (검증에서 주기가 5초가 아닌 4초로 나왔다).
                    deadline = t0 + RECONNECT_INTERVAL
                    while running["go"]:
                        left = deadline - time.time()
                        if left <= 0:
                            break
                        time.sleep(min(0.25, left))
                continue

            # ── 수신 ────────────────────────────────────────────────────
            try:
                got = reader.poll()
            except BusLost as e:
                reader.close()
                reader = None
                lost_at = time.time()
                attempts = 0
                ts = emit_logger("disconnect", {"reason": str(e)[:200]})
                summ.note_link(ts, "disconnect", {"reason": str(e)[:120]})
                disc_logged = True
                continue
            if got is None:
                continue

            ts = time.time()
            topic, payload = got
            inner = emit(ts, topic, payload)

            if topic == JOINT_TOPIC:
                if jw.prev is not None:
                    summ.feed_interval(ts - jw.prev[0])
                hit = jw.feed(ts, inner)
                if hit:
                    kind, detail = hit
                    emit_logger(kind, detail)
                    summ.note_anomaly(kind, ts, detail)
    finally:
        # 끝내 복구 못 하고 끝나면 재시도 횟수가 통째로 사라진다 — stop 에 실어 남긴다
        stop_detail = {"rows": seg.rows, "written": seg.written, "events": ev.rows,
                       "connected": reader is not None}
        if lost_at is not None:
            stop_detail.update({"unrecovered": True, "reconnect_attempts": attempts,
                                "lost_since": iso(lost_at),
                                "lost_sec": round(time.time() - lost_at, 1)})
            summ.note_link(time.time(), "미복구 종료",
                           {"attempts": attempts, "lost_sec": round(time.time() - lost_at, 1)})
        emit_logger("stop", stop_detail)
        if reader is not None:
            reader.close()
        md = summ.render(seg, ev)
        seg.close()
        ev.close()
        sp = os.path.join(outdir, f"summary_{stamp(session_ts)}.md")
        with open(sp, "w", encoding="utf-8") as f:
            f.write(md)
        print(f"[STOP] 원본 {seg.rows}레코드({seg.written}행) · events {ev.rows}행 · 요약 → {sp}",
              file=sys.stderr, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
