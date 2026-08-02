# RealSense D456 — 장치 사양·구동 경로

프로젝트 비전 센서 후보로 확보된 **Intel RealSense D456**의 장치 특성과 구동 경로를 정리한다.
BRD 1순위였던 D455 계열이며, `업무목록.md` **T5(카메라 모델 확정)** 의 근거 문서다.

> **이 문서는 이식 가능한 내용만 담는다.** 장치 자체의 성질과 프로젝트 타깃 OS(Ubuntu 24.04)에서의 설치 경로다.
> 특정 PC의 노드 번호·링크 상태·포트 배치 같은 머신 종속 실측은 이 문서에 넣지 않는다
> (PC별 실측은 같은 디렉터리 `07pc_RealSense_점검.md`).

최초 작성 2026-08-01 (세션 `260801-하바나브라운`) · 검증 환경 Ubuntu 24.04.4 / 커널 7.0.0-28

---

## 1. 요약

| 항목 | 내용 |
|---|---|
| 모델 | Intel RealSense **D456** — USB ID `8086:0b5c` |
| USB 디스크립터 iProduct | `Intel(R) RealSense(TM) Depth Module 456` |
| 스트림 | Depth(Z16) · Infrared(Y8/UYVY) · Color(YUYV) + 각 메타데이터 노드 |
| IMU | accel/gyro (`/dev/iio:device*`) — **udev 규칙 없이는 접근 불가** |
| 드라이버 | **스톡 `uvcvideo`로 충분.** 커널 모듈·DKMS 불필요 |
| 최소 구동 | **추가 설치 0** — `ffplay`만으로 세 스트림 모두 표시된다 (실증 완료) |
| SDK | `librealsense2` 2.58.x — Ubuntu 24.04(noble) 공식 apt 저장소 제공 |

**핵심 결론 — 깊이·적외선·컬러 세 스트림 모두 librealsense 없이 곧바로 열린다.**
librealsense는 "보기 위해" 필요한 게 아니라 **정합(align)·필터·프리셋·IMU·펌웨어 관리**를 위해 필요하다.

---

## 2. 스트림 노드 구조 — 노드 번호를 하드코딩하면 안 된다

D456은 `/dev/videoN`을 **6개** 만든다. 그중 영상이 나오는 건 3개뿐이고, 나머지 3개는 메타데이터 전용이라 열면 `Inappropriate ioctl for device`로 실패한다.

번호는 **열거 순서의 결과**라 부팅·재연결·다른 카메라 유무에 따라 밀린다. 아래 방법으로 찾을 것.

### 판별법 (v4l-utils 없이 sysfs만으로)

```bash
for v in /sys/class/video4linux/video*; do
  printf "%-8s idx=%s  iface=%s  vid:pid=%s:%s\n" "$(basename $v)" \
    "$(cat $v/index)" "$(cat $v/device/interface)" \
    "$(cat $v/device/../idVendor)" "$(cat $v/device/../idProduct)"
done
```

| 판별 축 | 값 |
|---|---|
| `idVendor` | `8086` — 내장 웹캠 등 다른 카메라를 배제 |
| `device/interface` **마지막 토큰** | `Depth` 계열 / `RGB` 계열 |
| `index` | Depth계열 `0`=깊이 · `2`=적외선 · 나머지=메타 / RGB계열 `0`=컬러 · 나머지=메타 |

> ⚠️ **함정** — 인터페이스 문자열이 `Intel(R) RealSense(TM) Depth Module 456  RGB`다.
> **모델명 안에 `Depth`가 들어 있어서** `*Depth*` 부분일치로 보면 컬러 노드까지 깊이로 오분류된다.
> 반드시 **마지막 토큰**(`${iface##* }`)으로 판별할 것. 실제로 밟은 버그다.

포맷으로도 교차 확인된다 — `gray16le`=깊이, `gray`+`uyvy422`=적외선, `yuyv422`=컬러.

```bash
ffmpeg -f v4l2 -list_formats all -i /dev/videoN
```

### 지원 해상도 (장치 디스크립터 기준)

| 스트림 | 포맷 | 해상도 |
|---|---|---|
| Depth | `gray16le` (Z16) | 256×144 · 480×270 · 640×360 · 640×480 · 848×480 · 1280×720 |
| Infrared | `gray` (Y8) / `uyvy422` | 480×270 · 640×360 · 640×480 · 848×480 · 1280×720 |
| Color | `yuyv422` | 424×240 · 640×480 · 1280×720 · 1280×800 |

Color에 **MJPEG이 없다** — 무압축 YUYV뿐이라 고해상도에서 대역폭을 많이 먹는다.

---

## 3. USB 2.0 연결 시 프레임레이트 상한 (실측)

D456은 USB3 장치지만 USB2 포트/케이블에 물리면 **USB 2.1 모드로 폴백**한다. 이때 **해상도 최대치는 그대로**이고 **프레임레이트만 깎인다.**

90~150프레임 캡처 후 소요시간으로 역산 (기동 오버헤드 보정):

| 스트림 | 해상도 | USB2 실측 상한 |
|---|---|---|
| Color | 640×480 | **30 fps** |
| Color | 1280×720 | **15 fps** |
| Color | 1280×800 | **15 fps** |
| Depth | 640×480 | **30 fps** |
| Depth | 848×480 | **10 fps** |
| Depth | 1280×720 | **5 fps** |

**동시 스트리밍은 USB2에서도 된다** — Depth+Color 640×480 동시 각 ~20fps, Depth+IR 640×480 동시 각 ~26fps로 정상 동작 확인.

> 장치 UVC 디스크립터를 파싱해 "Color 1280×800은 8fps"라고 추정한 자료가 있으나 **실측은 15fps**였다.
> 디스크립터 해석보다 실측이 정본이다.

**판단** — 육안 모니터링·저속 캡처 용도면 USB2로 충분하다. **Depth 848×480@30 이상이 필요해지면 USB3가 필수**다.
USB2 연결 여부 확인:

```bash
cat /sys/bus/usb/devices/*/speed    # 480 = USB2, 5000 = USB3
lsusb -t                            # 트리에서 480M / 5000M 확인
```

USB2로 떨어지는 원인은 대개 **케이블**(USB3 SuperSpeed 페어가 배선되지 않은 충전용/USB2 케이블)이다. 포트가 USB3 겸용인지는 아래로 확인한다 — `peer` 링크가 있으면 그 커넥터는 USB3 겸용이다.

```bash
readlink /sys/bus/usb/devices/usb1/1-0:1.0/usb1-port<N>/peer
```

USB2 모드의 부수 제약: **펌웨어 업데이트 불가**, 좌우 IR 동시 스트리밍 불가, 버스 전력 496mA(USB2 상한 500mA에 근접).

---

## 4. librealsense 없이 보기 (추가 설치 0)

`ffplay`(ffmpeg 패키지 포함)만으로 세 스트림이 다 뜬다. 노드 경로는 §2로 찾은 값을 넣는다.

```bash
# 컬러
ffplay -f v4l2 -input_format yuyv422  -video_size 640x480 -framerate 30 -i <color 노드>

# 적외선 — IR 프로젝터 도트 패턴이 보인다
ffplay -f v4l2 -input_format gray     -video_size 640x480 -framerate 30 -i <ir 노드>

# 깊이 — 컬러맵 적용
ffplay -f v4l2 -input_format gray16le -video_size 640x480 -framerate 30 -i <depth 노드> \
       -vf "format=gray,lutyuv=y=val*12,pseudocolor=preset=turbo"
```

### ⚠️ 깊이 필터 체인 — 잘못된 형태가 널리 퍼져 있다

```
쓸 것    : format=gray,lutyuv=y=val*N,pseudocolor=preset=turbo
쓰지 말 것: lutyuv=y='clip((val-300)*65535/3700,0,65535)',format=gray,...
           → 화면이 완전히 새까맣게 나온다 (실제 렌더로 확인)
```

16비트 상태에서 `clip((val-300)*...)`으로 mm 단위 클리핑을 하는 형태가 여러 자료에 보이지만, `lutyuv`가 8비트 값으로 동작해 `val-300`이 전부 0으로 잘린다. **`format=gray`로 8비트로 내린 뒤 게인을 곱하는 형태**를 쓸 것.

게인 `N`과 표시 최대거리는 대략 `65280/N` mm 관계다 — `12`≈5.4m, `8`≈8.2m, `20`≈3.3m.

> Z16 원본값의 단위는 mm로 알려져 있으나, **raw V4L2 경로에서는 librealsense의 `depth_units`가 적용되지 않는다.** 절대거리를 신뢰하지 말고 게인으로 조정할 것. 절대거리가 필요하면 librealsense를 거쳐야 한다.

### 래퍼 스크립트

`rs_view.sh`(같은 디렉터리)가 위 명령들을 감싸고 노드 자동탐지를 한다. 사용법은 `07pc_RealSense_점검.md` §4 참조.

---

## 5. librealsense2 설치 — Ubuntu 24.04 (noble)

프로젝트 타깃 OS인 24.04용 패키지가 **공식 apt 저장소에 정식 제공된다.** 소스 빌드할 이유가 없다.

> RealSense가 Intel에서 분사하면서 배포 도메인이 `librealsense.intel.com` → **`librealsense.realsenseai.com`** 으로 바뀌었다.
> 옛 문서·블로그의 URL과 GPG 키를 그대로 쓰면 `apt update`가 `NO_PUBKEY`로 실패한다.

**확인된 사실** (2026-08-01 직접 조회) — `dists/noble/Release` 존재, `Date: Sun, 19 Jul 2026`, `librealsense2-utils` 2.56.4 ~ **2.58.3** 제공.

```bash
sudo install -m 0755 -d /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
  | sudo gpg --dearmor -o /etc/apt/keyrings/librealsenseai.gpg
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo noble main" \
  | sudo tee /etc/apt/sources.list.d/librealsenseai.list
sudo apt update
sudo apt install librealsense2-utils librealsense2-udev-rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

다운로드 약 **19.3MB** / 설치 약 46MB. `librealsense2`(런타임)·`librealsense2-gl`은 의존성으로 따라온다.
`libglfw3`와 **`at`**(atd 데몬이 함께 뜬다)가 새로 들어온다.

### 깔지 말 것

| 패키지 | 이유 |
|---|---|
| `librealsense2-dkms` | **불필요하고 최신 커널에서 실패한다.** 패키지의 `BUILD_EXCLUSIVE_KERNEL`이 `^6\.(8\|14\|17)\.` 로 제한돼 있어 커널 7.x에서 빌드가 깨지고 apt가 half-configured로 남는다. 스톡 uvcvideo가 Z16·Y8·YUYV를 이미 다 노출하므로 스트리밍에 필요 없다 |
| `librealsense2-dbg` | 다운로드만 224MB |
| snap `librealsense` | 1.12.x 레거시 — **D400 시리즈 미지원** |

### 확인 순서

```bash
rs-enumerate-devices    # 인식·펌웨어 버전·USB 타입
rs-capture              # 125KB 경량 GLFW 뷰어 — 저사양 PC는 이쪽부터
realsense-viewer        # 풀 UI (OpenGL 3.3+ 필요)
```

`realsense-viewer`는 저사양에서 GLSL 렌더 hang·CPU 점유 사례가 보고돼 있다. 무거우면 설정의 **GLSL for Rendering / Processing** 토글을 먼저 시도하고, 그래도 안 되면 `rs-capture`로 갈 것. 포인트클라우드 탭은 켜지 말 것.

USB 타입이 노란색 `2.1`로 표시되는 것은 **경고일 뿐 차단이 아니다.**

### 되돌리기

```bash
sudo apt remove librealsense2-utils librealsense2-gl librealsense2
sudo apt remove librealsense2-udev-rules      # librealsense2 를 먼저 지워야 한다
sudo rm /etc/apt/sources.list.d/librealsenseai.list /etc/apt/keyrings/librealsenseai.gpg
```

---

## 6. ROS2 경로

프로젝트 최종 구조는 ROS2 토픽 구독이다. Jazzy(24.04)에서:

- `packages.ros.org` 저장소 추가 → `ros-jazzy-realsense2-camera`
- `desktop` 풀셋(2.86GB)은 과잉. `ros-base` + 필요한 것만 고를 것 (대략 다운로드 243MB / 설치 1.05GB 규모)
- `realsense2_camera`는 librealsense2를 요구하므로 §5가 선행된다
- 확인은 `rqt_image_view`(가벼움) → RViz2 순. **RViz2 포인트클라우드는 저사양 PC에서 부담이 크다**

§2의 노드 번호 함정은 ROS2에서도 그대로다 — 파라미터에 `video2` 같은 번호를 하드코딩하지 말 것. `realsense2_camera`는 시리얼번호로 지정할 수 있다.

---

## 7. 미확인 — 확실한 것처럼 쓰지 않는다

| 항목 | 상태 |
|---|---|
| 카메라 펌웨어 버전 | librealsense 없이는 읽을 수 없다. USB `bcdDevice 51.10`은 FW 버전이 **아니다** |
| librealsense 2.58.x의 커널 7.x 런타임 동작 | 미설치 상태라 미검증. 커널 버전 게이트가 없다는 점은 긍정 신호 |
| 실물 라벨상의 정확한 모델명 | USB 디스크립터는 `Depth Module 456`까지만 알려준다. D456과 D455의 구분은 실물 확인 필요 |
| raw 경로 깊이 유효픽셀 비율 | 자동노출·레이저파워·비주얼 프리셋이 적용되지 않은 상태의 값이라 librealsense 경유 시 달라질 수 있다 |
| IMU 동작 | udev 규칙 설치 전이라 미검증 |

---

## 참고 — 원본 위치

| 무엇 | 원본 |
|---|---|
| PC별 실측·노드번호·케이블 진단 | `07pc_RealSense_점검.md` (같은 디렉터리) |
| 뷰어 스크립트 | `rs_view.sh` (같은 디렉터리) |
| 카메라 모델 확정 진행상황 | `업무목록.md` T5 |
| 내장 웹캠 쪽 실측 | `07pc_카메라_점검.md` (루트 디렉터리, **07pc 전용**) |
