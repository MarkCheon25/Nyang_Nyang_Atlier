#!/usr/bin/env bash
#
# cam_view.sh — 노트북 내장 웹캠 실시간 뷰어 (최소기능)
#
# ffplay 한 줄을 감싼 래퍼. 추가 설치 없이 동작한다 (ffmpeg 패키지에 ffplay 포함).
#
# 아래 USAGE 블록이 도움말의 유일한 원본이다 — `-h` 는 이 블록을 그대로 뽑아 출력한다.
# 도움말을 고칠 때는 여기만 고치면 된다.
#
#=== USAGE ===
# 사용법: ./cam_view.sh [옵션]
#
#   -d, --device PATH   비디오 장치 지정 (기본: 내장 웹캠 자동 탐지)
#   -s, --size WxH      해상도 (기본: 640x480)
#                       지원: 176x144 320x240 352x288 640x360 640x480 1280x720
#   -f, --fps N         프레임레이트를 장치에 '요청'한다 (기본: 5)
#                       주의: 이 웹캠은 이 값을 무시하고 항상 ~10fps 를 준다 (실측)
#       --raw           MJPEG 대신 무압축 YUYV422 사용
#       --low-latency   버퍼링 최소화 (지연 감소, CPU 사용 증가)
#   -l, --list          이 PC의 비디오 장치 목록만 출력하고 종료
#   -h, --help          이 도움말
#
# 환경변수로도 지정 가능: CAM_DEV  CAM_SIZE  CAM_FPS  CAM_FMT  CAM_NAME_MATCH
#
# 사용 예:
#   ./cam_view.sh                       # 640x480 @5fps — 기본
#   ./cam_view.sh -s 1280x720 -f 15     # 720p @15fps
#   ./cam_view.sh --low-latency         # 버퍼링 최소화
#   ./cam_view.sh --raw                 # 무압축 YUYV422
#   ./cam_view.sh --list                # 장치 목록 확인
#   ./cam_view.sh -d /dev/video0        # 장치 직접 지정
#   ./cam_view.sh -h                    # 도움말
#   CAM_FPS=30 ./cam_view.sh            # 환경변수로 지정
#
# 종료: 창에 포커스 두고 q 또는 ESC
#=== /USAGE ===
#
# ── 이 스크립트가 실제로 실행하는 원본 명령 (스크립트 없이 직접 쓸 때) ──
#
#   ffplay -f v4l2 -input_format mjpeg -video_size 640x480 -framerate 5 /dev/video0
#
#   노드 번호가 밀릴 때를 대비한 고정 경로 버전:
#     ffplay -f v4l2 -input_format mjpeg -video_size 640x480 -framerate 5 \
#       /dev/v4l/by-id/usb-Generic_USB_HD_Webcam_200901010001-video-index0
#     (단 이 웹캠 시리얼 200901010001 은 제네릭 값이라, 스크립트는 by-id 대신
#      sysfs 의 이름 + index=0 으로 캡처 노드를 찾는다)
#
#   저지연 : 위 명령에 -fflags nobuffer -flags low_delay 추가
#   720p   : -video_size 1280x720
#   정지 1장 캡처 (ffmpeg, 워밍업 20프레임 버림):
#     ffmpeg -f v4l2 -input_format mjpeg -video_size 640x480 -i /dev/video0 \
#       -vf "select=gte(n\,20)" -frames:v 1 -update 1 -y shot.jpg
#
# ── 07pc 실측 (2026-08-01) ──────────────────────────────────────────
#   내장 웹캠  = Quanta USB HD Webcam (0408:50c0) → /dev/video0(캡처) · /dev/video1(메타데이터)
#   포맷       = MJPEG / YUYV422, 640x480 · 1280x720 등
#   실제 fps   = 스트림 헤더는 30fps 라 하지만 실측은 ~10fps 다.
#                해상도(320x240·640x480·1280x720)·포맷(mjpeg·yuyv) 무엇으로 바꿔도 10fps 로 일정하다.
#                720p 에서도 안 떨어지므로 USB 대역폭 병목이 아니라 저조도 자동노출로 추정된다.
#                요구사항(0.2초당 1프레임 = 5fps)은 충족한다.
#   권한       = markch07 은 video 그룹이 아니지만 ACL(crw-rw----+)로 접근된다
#   RealSense  = /dev/video2~7 을 점유. 그래서 노드 번호를 하드코딩하지 않는다
#
# ── 주의: 라이브 카메라에 쓰면 안 되는 것 (모두 검증 완료) ───────────
#   -autoexit, -t : EOF 가 없어서 종료 트리거가 걸리지 않는다
#   -nodisp       : 비디오를 꺼버려 오디오 없는 소스에서는 즉시 실패한다
#   /dev/video1   : 메타데이터 노드다. 열어도 영상이 나오지 않는다
#   -framerate N  : 이 웹캠은 무시한다. 표시 프레임을 실제로 줄이려면 출력필터 -vf fps=N 을 쓸 것
#   첫 프레임      : 깨져서 나올 수 있다(EOI missing). 뷰어는 무시해도 되지만
#                   정지 캡처에서는 앞 20프레임을 반드시 버려야 한다

set -euo pipefail

# ── 기본값 (환경변수로 덮어쓸 수 있다) ──────────────────────────────
CAM_DEV="${CAM_DEV:-}"                 # 비우면 자동 탐지
CAM_SIZE="${CAM_SIZE:-640x480}"        # 640x480 | 1280x720 | 320x240 ...
CAM_FPS="${CAM_FPS:-5}"                # 장치에 요청만 한다. 이 웹캠은 무시하고 ~10fps 를 준다(실측)
CAM_FMT="${CAM_FMT:-mjpeg}"            # mjpeg | yuyv422
CAM_NAME_MATCH="${CAM_NAME_MATCH:-USB HD Webcam}"   # 내장 웹캠 식별 문자열

# 도움말은 상단 USAGE 주석 블록에서 뽑아 쓴다 — 같은 내용을 두 곳에 두지 않기 위해서다.
# 파일로 실행되지 않아 추출이 실패하는 경우(예: bash < cam_view.sh)에는 한 줄 요약으로 대체한다.
usage() {
  local out=""
  # set -e + pipefail 아래에서는 sed 실패가 대입문째로 스크립트를 죽인다. || true 로 막는다.
  if [[ -r "$0" ]]; then
    out="$(sed -n '/^#=== USAGE ===$/,/^#=== \/USAGE ===$/p' "$0" 2>/dev/null \
           | sed '1d;$d' | sed 's/^#//; s/^ //')" || out=""
  fi
  if [[ -n "$out" ]]; then
    printf '%s\n' "$out"
  else
    echo "사용법: ./cam_view.sh [-d 장치] [-s 해상도] [-f fps] [--raw] [--low-latency] [-l] [-h]"
  fi
}

# ── 비디오 장치 목록 ────────────────────────────────────────────────
list_devices() {
  echo "비디오 장치 목록:"
  local d node name idx role
  for d in /sys/class/video4linux/video*; do
    [[ -e "$d" ]] || { echo "  (없음)"; return; }
    node="/dev/$(basename "$d")"
    name="$(cat "$d/name" 2>/dev/null || echo '?')"
    idx="$(cat "$d/index" 2>/dev/null || echo '?')"
    # index 0 = 영상 캡처 노드, 그 외 = 메타데이터 등 부가 노드
    [[ "$idx" == "0" ]] && role="← 캡처 노드" || role="부가 노드(index=$idx)"
    # 역할은 맨 뒤에 둔다 — 한글은 printf 폭 계산이 바이트 기준이라 중간에 두면 열이 어긋난다
    printf "  %-14s %-34s %s\n" "$node" "$name" "$role"
  done
}

# ── 내장 웹캠 캡처 노드 자동 탐지 ───────────────────────────────────
# 노드 번호는 고정이 아니다. RealSense 등이 먼저 잡히면 /dev/video0 이 웹캠이 아닐 수 있으므로
# 이름(CAM_NAME_MATCH)과 index=0(캡처 노드)으로 찾는다.
detect_device() {
  local d
  for d in /sys/class/video4linux/video*; do
    [[ -e "$d" ]] || continue
    [[ "$(cat "$d/index" 2>/dev/null)" == "0" ]] || continue
    if grep -qF "$CAM_NAME_MATCH" "$d/name" 2>/dev/null; then
      echo "/dev/$(basename "$d")"
      return 0
    fi
  done
  return 1
}

# ── 인자 파싱 ───────────────────────────────────────────────────────
LOW_LATENCY=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    -d|--device)  CAM_DEV="${2:?장치 경로가 필요합니다}"; shift 2 ;;
    -s|--size)    CAM_SIZE="${2:?해상도가 필요합니다}";   shift 2 ;;
    -f|--fps)     CAM_FPS="${2:?프레임레이트가 필요합니다}"; shift 2 ;;
    --raw)        CAM_FMT="yuyv422"; shift ;;
    --low-latency) LOW_LATENCY=1; shift ;;
    -l|--list)    list_devices; exit 0 ;;
    -h|--help)    usage; exit 0 ;;
    *) echo "알 수 없는 옵션: $1" >&2; echo >&2; usage >&2; exit 2 ;;
  esac
done

# ── 사전 점검 ───────────────────────────────────────────────────────
if ! command -v ffplay >/dev/null 2>&1; then
  echo "오류: ffplay 가 없습니다. ffmpeg 패키지를 설치하세요 (sudo apt install ffmpeg)." >&2
  exit 1
fi

if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
  echo "오류: 그래픽 세션이 없습니다 (DISPLAY/WAYLAND_DISPLAY 미설정)." >&2
  echo "      SSH 접속이라면 X11 포워딩(ssh -X)이 필요합니다." >&2
  exit 1
fi

if [[ -z "$CAM_DEV" ]]; then
  if ! CAM_DEV="$(detect_device)"; then
    echo "오류: '$CAM_NAME_MATCH' 이름의 캡처 장치를 찾지 못했습니다." >&2
    echo >&2
    list_devices >&2
    echo >&2
    echo "  -d 옵션으로 직접 지정하세요. 예: ./cam_view.sh -d /dev/video0" >&2
    exit 1
  fi
fi

if [[ ! -e "$CAM_DEV" ]]; then
  echo "오류: 장치가 없습니다 — $CAM_DEV" >&2
  exit 1
fi

if [[ ! -r "$CAM_DEV" ]]; then
  echo "오류: $CAM_DEV 읽기 권한이 없습니다." >&2
  echo "      데스크톱에 직접 로그인한 세션이면 ACL로 접근됩니다." >&2
  echo "      원격/서비스 세션이라면: sudo usermod -aG video \"\$USER\" 후 재로그인." >&2
  exit 1
fi

# ── 실행 ────────────────────────────────────────────────────────────
FFPLAY_ARGS=(
  -hide_banner -loglevel warning
  -f v4l2
  -input_format "$CAM_FMT"
  -video_size "$CAM_SIZE"
  -framerate "$CAM_FPS"
)
if [[ "$LOW_LATENCY" == "1" ]]; then
  FFPLAY_ARGS+=(-fflags nobuffer -flags low_delay)
fi
FFPLAY_ARGS+=(-window_title "cam: $CAM_DEV  ${CAM_SIZE}@${CAM_FPS}fps")

echo "장치: $CAM_DEV   해상도: $CAM_SIZE   프레임레이트: ${CAM_FPS}fps   포맷: $CAM_FMT"
echo "종료: 창에 포커스 두고 q 또는 ESC"

# 첫 프레임은 깨져서 나올 수 있다(EOI missing). 뷰어에서는 한 프레임 스쳐 지나갈 뿐이라 무시한다.
# 정지 이미지를 캡처할 때는 앞 20프레임을 버려야 한다 — 프로젝트 docs 참조.
exec ffplay "${FFPLAY_ARGS[@]}" "$CAM_DEV"
