# 07pc RealSense 점검 — D456 스트림 개통

> ## ⚠️ 여기 적힌 수치는 **07pc(LG 15UB470)에 물렸을 때의 실측**이다
>
> 노드 번호·USB 링크 상태·포트 배치·프레임레이트는 이 노트북에서 직접 측정한 값이다.
> 다른 PC나 실기 구성에 그대로 적용되지 않는다 —
> **특히 노드 번호와 USB 링크 속도는 옮길 때마다 다시 확인해야 한다.**
>
> 다만 D456은 **USB 장치라 PC를 옮겨 물릴 수 있다.** 그래서 이 문서는 리포에 둔다.
> 다른 PC에 연결하면 이 문서에 **그 PC의 절을 추가**해 비교 기준으로 쓴다 (§2·§3이 그 틀이다).
>
> **장치 자체의 성질과 설치 경로는** 이식 가능하므로 같은 디렉터리 `RealSense_D456.md`에 있다.
> 같은 사실을 두 곳에 쓰지 않는다 — 여기는 PC별 실측만 담는다.

측정일 2026-08-01 (세션 `260801-하바나브라운`) · 대상 `markch07@LG15UBU` (Ubuntu 24.04.4 / 커널 7.0.0-28)
선행 문서: `07pc_카메라_점검.md` (같은 PC의 **내장 웹캠** 점검, 세션 `260801-스팽글드`. 루트 디렉터리)

---

## 1. 결론

**RealSense D456은 07pc에서 정상 동작한다. 추가 설치 없이 세 스트림 모두 실시간으로 볼 수 있다.**

| 점검 항목 | 결과 |
|---|---|
| 하드웨어 부착 | ✅ Intel RealSense D456 (`8086:0b5c`), 스톡 `uvcvideo` 바인딩 |
| 접근 권한 | ✅ ACL로 접근 (`video` 그룹 가입·udev 규칙 **불필요**) |
| 컬러 스트림 | ✅ 프레임 캡처·육안 확인 (야간 실내, 자동노출 수렴 후 장면 식별됨) |
| 적외선 스트림 | ✅ **IR 프로젝터 도트 패턴 확인** — 에미터·IR센서 정상 |
| 깊이 스트림 | ✅ 컬러맵 렌더로 패널·배경 거리차 확인 |
| 동시 스트리밍 | ✅ Depth+Color 각 ~20fps, Depth+IR 각 ~26fps |
| GUI 표시 요건 | ✅ GNOME/X11 로컬 콘솔, OpenGL 4.6 (940MX), direct rendering |
| USB 링크 | ⚠️ **USB 2.0(480Mbps)로 열거됨** — 동작엔 문제 없으나 fps 상한 (§3) |
| librealsense | ❌ 미설치 (이번 점검 범위 밖 — 설치 경로는 `RealSense_D456.md` §5) |

실행: `./rs_view.sh` (같은 디렉터리)

---

## 2. 이 PC의 노드 배치 (실측)

```
USB : Bus 001 Device 010: ID 8086:0b5c Intel RealSense Depth Module 456
포트: usb1/1-1  (PCI 0000:00:14.0)
```

| 노드 | 역할 | 포맷 |
|---|---|---|
| `/dev/video2` | **Depth** | `gray16le` (Z16) |
| `/dev/video3` | 메타데이터 | — |
| `/dev/video4` | **Infrared** | `gray` / `uyvy422` |
| `/dev/video5` | 메타데이터 | — |
| `/dev/video6` | **Color** | `yuyv422` |
| `/dev/video7` | 메타데이터 | — |
| `/dev/video0~1` | 내장 Quanta 웹캠 | 루트 `07pc_카메라_점검.md` 참조 |

**이 번호는 보장되지 않는다.** 부팅·재연결 순서에 따라 밀린다. 판별법은 `RealSense_D456.md` §2, 자동탐지는 `./rs_view.sh list`.

안정 경로가 필요하면 `by-path`를 쓴다 (`by-id`는 **Depth 링크가 아예 없어서** 못 쓴다 — 두 UVC 인터페이스가 시리얼 문자열을 공유해 이름이 충돌한다):

```
Depth  /dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0
IR     /dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index2
Color  /dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.3-video-index0
```

`usb-0:1`은 **물리 포트 번호**다. 다른 포트로 옮기면 이 경로도 바뀐다.

---

## 3. USB 2.0 문제 — 이 PC에서의 진단

### 실측 프레임레이트 상한

90~150프레임 캡처 후 소요시간 역산 (기동 오버헤드 보정 시 정확히 양자화됨):

| 스트림 | 해상도 | 실측 | 보정값 |
|---|---|---|---|
| Color | 640×480 | 25.4 fps | **30** |
| Color | 1280×720 | 14.1 fps | **15** |
| Color | 1280×800 | 14.0 fps | **15** |
| Depth | 640×480 | 26.4 fps | **30** |
| Depth | 848×480 | 9.3 fps | **10** |
| Depth | 1280×720 | 4.8 fps | **5** |

**해상도 최대치는 USB3와 같다. 깎이는 것은 fps뿐이다.**

### 원인 — 포트가 아니라 케이블이 유력

| 후보 | 판정 |
|---|---|
| USB2 전용 포트에 꽂힘 | **기각** — RealSense가 꽂힌 `usb1-port1`은 `peer=usb2-port1`, 즉 USB3 겸용 커넥터다 |
| 호스트/xHCI 문제 | **기각** — Bus002(USB3 root hub, 5000M, 6포트) 정상 등록 |
| **케이블이 USB2 전용** | **가장 유력.** 확증은 케이블 교체 시험으로 |
| 카메라 링크 상태 | 가능. `realsense-viewer` → More → Hardware Reset으로 해결된 사례가 있으나 librealsense 설치가 선행 |

**이 PC의 외부 포트 지형 (실측)**

| 포트 | USB3 겸용 | 현재 |
|---|---|---|
| `usb1-port1` | ✅ (`peer=usb2-port1`) | RealSense — 480Mbps로 붙어 있음 |
| `usb1-port2` | ✅ (`peer=usb2-port2`) | 마우스 |
| `usb1-port5` | ✅ (`peer=usb2-port5`) | **비어 있음** |
| `usb1-port3` | ✅ | 내장 웹캠 (내부) |

→ **비어 있는 USB3 겸용 포트가 있다.** 보통 USB3 포트는 커넥터 내부 플라스틱이 파란색이다.

### 확인 절차 (5분, 시스템 변경 없음)

```bash
# 터미널 A — 감시
journalctl -k -f | grep --line-buffered -E 'new (high-speed|SuperSpeed) USB device'

# 터미널 B — 판정
./rs_view.sh info          # 링크속도를 요약해서 보여준다
cat /sys/bus/usb/devices/1-1/speed    # 480 → USB2 / 5000 → USB3
```

1. 다른 USB 3.x **데이터** 케이블로 교체 → `5000`이 되는지
2. 비어 있는 USB3 겸용 포트로 옮겨서 재확인
3. 그래도 480이면 카메라 링크 상태 의심 → librealsense 설치 후 Hardware Reset

### 지금 고쳐야 하나

**아니다.** 육안 모니터링 용도엔 VGA 30fps로 충분하다. 다만 아래에 해당하면 USB3가 필요해진다.

- Depth 848×480@30 이상
- 펌웨어 업데이트 (**USB2에서는 불가**)
- 좌우 IR 동시 스트리밍

---

## 4. `rs_view.sh`

이 문서와 같은 디렉터리(`src/drivers/cam_comm/`)에 있다 — **리포 버전관리 대상**이다.
루트의 `cam_view.sh`(07pc 내장 웹캠용)와 짝을 이루지만, 그쪽은 이 PC 전용 도구라 리포 밖에 남겨 뒀다.

```bash
./rs_view.sh                        # 컬러 640x480@30
./rs_view.sh depth                  # 뎁스 컬러맵
./rs_view.sh depth -g 20            # 가까운 거리 대비 강조
./rs_view.sh ir                     # 적외선 (도트 패턴)
./rs_view.sh color -s 1280x720 -f 15   # 720p (USB2 상한)
./rs_view.sh depth --shot           # 정지 1장 캡처
./rs_view.sh list                   # 노드 목록
./rs_view.sh info                   # 링크속도·포맷 점검
./rs_view.sh color -n               # 명령만 출력 (실행 안 함)
./rs_view.sh -h                     # 도움말
```

설계 메모:

- **노드 자동탐지** — USB `idVendor=8086` + 인터페이스 마지막 토큰(`Depth`/`RGB`) + `index`. 번호 하드코딩 없음. 알 수 없는 인터페이스명이면 포맷 프로브로 폴백
- **사전 점검** — ffplay/ffmpeg 존재 · 그래픽 세션(`DISPLAY`) · 장치 존재·읽기권한 · 메타데이터 노드 지정 시 경고
- **도움말 단일원본** — 상단 `#=== USAGE ===` 주석 블록을 `-h`가 뽑아 출력한다 (루트 `cam_view.sh`와 같은 방식)
- `--shot`은 앞 40프레임을 버린다 — 첫 프레임은 자동노출이 안 잡혀 새까맣게 나온다

---

## 5. 개발 중 밟은 함정 (전부 실제로 밟음)

| # | 함정 | 증상 | 대응 |
|---|---|---|---|
| 1 | **인터페이스명 부분일치 오분류** | 컬러 노드(`video6`)가 깊이로 잡힘 | 인터페이스 문자열이 `...Depth Module 456  RGB`라 **모델명 안의 `Depth`** 에 걸린다. 마지막 토큰으로 판별할 것 |
| 2 | **`-frames:v N` + `select` 조합이 멈춘다** | ffmpeg이 종료되지 않음 | `-frames:v`는 **출력** 프레임 수다. select로 1장만 통과시키면 N에 영원히 도달 못 한다. `-frames:v 1` + `select=gte(n\,N)`을 쓸 것 |
| 3 | **깊이 필터가 새까맣게 나온다** | 화면 전체가 검정 | `lutyuv=y='clip((val-300)*65535/3700,...)'` 형태가 동작하지 않는다. `format=gray,lutyuv=y=val*N` 형태를 쓸 것 (`RealSense_D456.md` §4) |
| 4 | **첫 프레임이 새까맣다** | 캡처 이미지가 전부 검정 | 자동노출 미수렴. 앞 40프레임을 버릴 것 |
| 5 | 메타데이터 노드 | `Inappropriate ioctl for device` | `video3/5/7`은 영상 노드가 아니다 |

내장 웹캠 쪽 함정 5종(첫 프레임 깨짐·`-autoexit`·`-nodisp`·`-framerate` 무시 등)은 루트 `07pc_카메라_점검.md` §6에 있다. **`-autoexit`/`-nodisp` 항목은 RealSense에도 그대로 적용된다.**

---

## 6. 이 PC의 소프트웨어 상태 (2026-08-01)

| 도구 | 상태 |
|---|---|
| `ffmpeg`/`ffplay` | ✅ 설치됨 (이 점검은 전부 이걸로 했다) |
| `librealsense2` / `realsense-viewer` | ❌ 미설치 |
| `v4l-utils` (`v4l2-ctl`) | ❌ 미설치 — 후보 `1.26.1-4build3` |
| `python3-opencv` / `numpy` | ❌ 미설치 |
| ROS2 | ❌ 미설치 (`업무목록.md` T12) |
| 커널 | `7.0.0-28-generic` 부팅 중. `6.17.0-40-generic`도 헤더까지 설치돼 있어 부팅 선택지가 있다 |

**커널 7.0에서는 `librealsense2-dkms`가 빌드되지 않는다** (패키지가 6.8/6.14/6.17로 제한). 다만 스트리밍에 불필요하다 — 상세는 `RealSense_D456.md` §5.

---

## 참고 — 원본 위치

| 무엇 | 원본 |
|---|---|
| 장치 사양·USB2 상한·설치 경로·ROS2 경로 | `RealSense_D456.md` (같은 디렉터리) |
| 뷰어 스크립트 | `rs_view.sh` (같은 디렉터리) |
| 같은 PC의 내장 웹캠 점검 | `07pc_카메라_점검.md` (**루트 디렉터리 · 07pc 전용**) |
| 이 PC의 스펙·제약 | `~/.claude/pc.md` · `~/Desktop/pc_spec_LG15UB.md` |
| 세션 경과 | `작업일지.md` 2026-08-01 (하바나브라운) |
| 미결 항목 | `업무목록.md` T5 |
