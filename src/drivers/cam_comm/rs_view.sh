#!/usr/bin/env bash
#
# rs_view.sh — Intel RealSense 실시간 뷰어 (최소기능, 추가 설치 0)
#
# ffplay 한 줄을 감싼 래퍼. librealsense 없이 스톡 uvcvideo 만으로 동작한다.
# 루트 디렉터리의 cam_view.sh(07pc 내장 웹캠용)와 짝을 이룬다 — 그쪽은 PC 전용 도구라 리포 밖이다.
#
# 아래 USAGE 블록이 도움말의 유일한 원본이다 — `-h` 는 이 블록을 그대로 뽑아 출력한다.
# 도움말을 고칠 때는 여기만 고치면 된다.
#
#=== USAGE ===
# 사용법: ./rs_view.sh [스트림] [옵션]
#
# 스트림 (기본: color)
#   color               RGB 컬러 영상
#   ir                  적외선 — 프로젝터 도트 패턴이 보인다
#   depth               깊이 — 컬러맵을 입혀서 표시
#   list                RealSense 노드 목록만 출력하고 종료
#   info                장치·USB 링크속도·지원 포맷 요약 후 종료
#
# 옵션
#   -s, --size WxH      해상도 (기본: 640x480)
#   -f, --fps N         프레임레이트 (기본: 30)
#   -d, --device PATH   노드 직접 지정 (자동탐지 건너뜀)
#   -g, --gain N        [depth] 밝기 계수 (기본: 12)
#                       표시 최대거리 ≈ 65280/N mm — 12→약 5.4m, 8→8.2m, 20→3.3m
#   -c, --colormap NAME [depth] turbo(기본) magma inferno viridis plasma cividis none
#       --shot [FILE]   실시간 표시 대신 정지 1장 캡처 (기본: rs_<스트림>_<시각>.png)
#       --low-latency   버퍼링 최소화 (지연 감소, CPU 사용 증가)
#   -n, --dry-run       실행하지 않고 명령만 출력
#   -h, --help          이 도움말
#
# 환경변수로도 지정 가능: RS_SIZE  RS_FPS  RS_DEV  RS_GAIN  RS_COLORMAP
#
# 사용 예:
#   ./rs_view.sh                        # 컬러 640x480@30
#   ./rs_view.sh depth                  # 뎁스 컬러맵
#   ./rs_view.sh depth -g 20            # 가까운 거리 위주로 대비 강조
#   ./rs_view.sh ir                     # 적외선 (도트 패턴)
#   ./rs_view.sh color -s 1280x720 -f 15   # 720p — USB2 에서는 15fps 가 상한
#   ./rs_view.sh depth --shot           # 뎁스 정지 1장
#   ./rs_view.sh list                   # 노드 확인
#   ./rs_view.sh info                   # 링크속도·포맷 점검
#   ./rs_view.sh color -n               # 명령만 보기
#
# 종료: 창에 포커스 두고 q 또는 ESC
#=== /USAGE ===
#
# ── 07pc 실측 (2026-08-01, 세션 260801-하바나브라운) ────────────────────
#
#   장치   Intel RealSense D456 (8086:0b5c) — USB 2.0(480Mbps)로 열거됨
#   노드   video2=Depth  video4=IR  video6=Color  (video3/5/7 은 메타데이터 전용)
#          번호는 열거 순서라 고정이 아니다 → 이 스크립트는 USB ID + 인터페이스명으로 찾는다
#
#   USB2 프레임레이트 상한 (90~150프레임 실측, 기동 오버헤드 보정):
#     Color 640x480→30 · 1280x720→15 · 1280x800→15
#     Depth 640x480→30 · 848x480→10  · 1280x720→5
#     Depth+Color 동시 640x480 → 각 ~20fps (정상 동작)
#   USB3 로 옮기면 위 제한이 풀린다. 상세는 07pc_RealSense_점검.md
#
# ── 뎁스 필터 체인 주의 (실측으로 확정) ─────────────────────────────────
#
#   쓰는 것 :  format=gray,lutyuv=y=val*N,pseudocolor=preset=turbo
#   쓰면 안 되는 것 :
#              lutyuv=y='clip((val-300)*65535/3700,0,65535)',format=gray,...
#              → 화면이 완전히 새까맣게 나온다. 실제로 렌더해서 확인함.
#              lutyuv 이 8비트 값으로 동작해 (val-300) 이 전부 0 으로 잘린다.
#   Z16 원본값 단위는 mm 로 알려져 있으나, librealsense 를 거치지 않는 raw 경로에서는
#   depth_units 가 적용되지 않는다. 그래서 절대거리가 아니라 -g 게인으로 조정한다.
#
# ── 라이브 카메라에 쓰면 안 되는 것 (cam_view.sh 와 공통) ───────────────
#   -autoexit, -t : EOF 가 없어서 종료 트리거가 걸리지 않는다
#   -nodisp       : 비디오를 꺼버려 오디오 없는 소스에서는 즉시 실패한다
#   메타데이터 노드(video3/5/7) : 열면 "Inappropriate ioctl for device" 로 실패한다

set -euo pipefail

# ── 기본값 (환경변수로 덮어쓸 수 있다) ──────────────────────────────
RS_DEV="${RS_DEV:-}"                    # 비우면 자동 탐지
RS_SIZE="${RS_SIZE:-640x480}"
RS_FPS="${RS_FPS:-30}"
RS_GAIN="${RS_GAIN:-12}"                # depth 밝기 계수
RS_COLORMAP="${RS_COLORMAP:-turbo}"     # depth 컬러맵
RS_VID="${RS_VID:-8086}"                # Intel

STREAM="color"
LOW_LATENCY=0
DRY_RUN=0
SHOT=""
SHOT_REQUESTED=0

# ── 도움말 ──────────────────────────────────────────────────────────
# 상단 USAGE 주석 블록에서 뽑아 쓴다 — 같은 내용을 두 곳에 두지 않기 위해서다.
# set -e + pipefail 아래에서는 sed 실패가 대입문째로 스크립트를 죽이므로 || out="" 로 막는다.
usage() {
  local out=""
  if [[ -r "$0" ]]; then
    out="$(sed -n '/^#=== USAGE ===$/,/^#=== \/USAGE ===$/p' "$0" 2>/dev/null \
           | sed '1d;$d' | sed 's/^#//; s/^ //')" || out=""
  fi
  if [[ -n "$out" ]]; then
    printf '%s\n' "$out"
  else
    echo "사용법: ./rs_view.sh [color|ir|depth|list|info] [-s WxH] [-f fps] [-g 게인] [-n] [-h]"
  fi
}

# ── RealSense 캡처 노드 탐지 ────────────────────────────────────────
# 노드 번호는 열거 순서의 결과라 고정이 아니다. 아래 세 가지로 특정한다.
#   1) USB idVendor 가 Intel(8086) 인가            → 내장 웹캠을 배제
#   2) UVC 인터페이스명의 **마지막 토큰**이 Depth 인가 RGB 인가 → 계열 구분
#   3) index (0=주 캡처, 2=IR, 나머지=메타데이터)   → 계열 안에서 구분
#
# 주의: 인터페이스 문자열은 "Intel(R) RealSense(TM) Depth Module 456  RGB" 처럼
#       모델명에 'Depth' 가 들어 있다. `*Depth*` 로 부분일치를 보면 RGB 노드까지
#       깊이로 오분류된다(실제로 밟은 버그). 반드시 마지막 토큰으로 판별할 것.
#
# 인자: 노드 sysfs 경로. 출력: depth|ir|color|meta
node_role() {
  local v="$1" idx iface tag fmts
  idx="$(cat "$v/index" 2>/dev/null || echo '')"
  iface="$(cat "$v/device/interface" 2>/dev/null || echo '')"
  iface="${iface%"${iface##*[![:space:]]}"}"   # 끝 공백 제거
  tag="${iface##* }"                            # 마지막 토큰만
  case "$tag" in
    Depth) [[ "$idx" == "0" ]] && { echo depth; return; }
           [[ "$idx" == "2" ]] && { echo ir;    return; }
           echo meta; return ;;
    RGB)   [[ "$idx" == "0" ]] && { echo color; return; }
           echo meta; return ;;
  esac
  # 알 수 없는 인터페이스명(다른 RealSense 모델 등) — 포맷을 직접 보고 판단한다
  command -v ffmpeg >/dev/null 2>&1 || { echo meta; return; }
  fmts="$(timeout 10 ffmpeg -hide_banner -f v4l2 -list_formats all -i "/dev/$(basename "$v")" 2>&1 \
          | grep -oE 'gray16le|uyvy422|yuyv422|mjpeg|gray' | sort -u | tr '\n' ' ')" || fmts=""
  case "$fmts" in
    *gray16le*)          echo depth ;;
    *uyvy422*)           echo ir ;;
    *yuyv422*|*mjpeg*)   echo color ;;
    *gray*)              echo ir ;;
    *)                   echo meta ;;
  esac
}

# 인자: 스트림 이름. 출력: 노드 경로. 없으면 1 반환.
detect_node() {
  local want="$1" v vid
  for v in /sys/class/video4linux/video*; do
    [[ -e "$v" ]] || continue
    vid="$(cat "$v/device/../idVendor" 2>/dev/null || echo '')"
    [[ "$vid" == "$RS_VID" ]] || continue
    if [[ "$(node_role "$v")" == "$want" ]]; then
      echo "/dev/$(basename "$v")"
      return 0
    fi
  done
  return 1
}

# ── 노드 목록 ───────────────────────────────────────────────────────
list_nodes() {
  local v n idx vid pid role found=0
  echo "RealSense 노드 목록:"
  for v in /sys/class/video4linux/video*; do
    [[ -e "$v" ]] || continue
    vid="$(cat "$v/device/../idVendor" 2>/dev/null || echo '')"
    pid="$(cat "$v/device/../idProduct" 2>/dev/null || echo '')"
    [[ "$vid" == "$RS_VID" ]] || continue
    found=1
    idx="$(cat "$v/index" 2>/dev/null || echo '?')"
    n="/dev/$(basename "$v")"
    case "$(node_role "$v")" in
      depth) role="depth  ← 깊이" ;;
      ir)    role="ir     ← 적외선" ;;
      color) role="color  ← 컬러" ;;
      *)     role="(메타데이터 전용 — 열면 실패한다)" ;;
    esac
    # 한글은 printf 폭 계산이 바이트 기준이라 열이 어긋난다. 역할은 맨 뒤에 둔다.
    printf "  %-13s %s:%s  index=%-2s %s\n" "$n" "$vid" "$pid" "$idx" "$role"
  done
  if [[ "$found" == "0" ]]; then
    echo "  (없음) — RealSense(idVendor=$RS_VID)가 붙어 있지 않다."
    echo "  확인: lsusb | grep -i realsense"
    return 1
  fi
}

# ── 장치 정보 ───────────────────────────────────────────────────────
show_info() {
  local d speed ver prod
  echo "── USB 링크 ────────────────────────────────────────────"
  local any=0
  for d in /sys/bus/usb/devices/[0-9]*-[0-9]*; do
    [[ -f "$d/idVendor" ]] || continue
    [[ "$(cat "$d/idVendor")" == "$RS_VID" ]] || continue
    any=1
    prod="$(cat "$d/product" 2>/dev/null || echo '?')"
    speed="$(cat "$d/speed" 2>/dev/null || echo '?')"
    ver="$(cat "$d/version" 2>/dev/null | tr -d ' ' || echo '?')"
    echo "  장치     : $prod"
    echo "  경로     : $d"
    echo "  링크속도 : ${speed} Mbps  (USB $ver)"
    if [[ "$speed" == "480" ]]; then
      echo "  ⚠ USB 2.0 로 연결됨. 동작에는 문제 없으나 프레임레이트 상한이 걸린다."
      echo "    Color 1280x720→15fps · Depth 848x480→10fps · Depth 1280x720→5fps (실측)"
      echo "    USB3 로 올리려면: 케이블을 USB3 데이터 케이블로 교체 → 이 명령으로 5000 확인"
    elif [[ "$speed" == "5000" || "$speed" == "10000" ]]; then
      echo "  ✅ USB 3.x — 프레임레이트 제한 없음"
    fi
  done
  [[ "$any" == "1" ]] || { echo "  (RealSense 미부착)"; return 1; }
  echo
  list_nodes
  echo
  echo "── 지원 포맷 ───────────────────────────────────────────"
  if ! command -v ffmpeg >/dev/null 2>&1; then
    echo "  (ffmpeg 없음 — 건너뜀)"
    return 0
  fi
  local s node
  for s in depth ir color; do
    if node="$(detect_node "$s")"; then
      echo "  [$s] $node"
      timeout 10 ffmpeg -hide_banner -f v4l2 -list_formats all -i "$node" 2>&1 \
        | sed -n 's/.*\(Raw\|Compressed\) *: */    /p' || true
    fi
  done
}

# ── 인자 파싱 ───────────────────────────────────────────────────────
# 첫 인자가 스트림 이름이면 소비한다. 옵션은 순서 무관.
case "${1:-}" in
  color|ir|depth|list|info) STREAM="$1"; shift ;;
esac

while [[ $# -gt 0 ]]; do
  case "$1" in
    color|ir|depth|list|info) STREAM="$1"; shift ;;
    -s|--size)     RS_SIZE="${2:?해상도가 필요합니다 (예: 640x480)}"; shift 2 ;;
    -f|--fps)      RS_FPS="${2:?프레임레이트가 필요합니다}";           shift 2 ;;
    -d|--device)   RS_DEV="${2:?장치 경로가 필요합니다}";              shift 2 ;;
    -g|--gain)     RS_GAIN="${2:?게인 값이 필요합니다}";               shift 2 ;;
    -c|--colormap) RS_COLORMAP="${2:?컬러맵 이름이 필요합니다}";       shift 2 ;;
    --shot)
      SHOT_REQUESTED=1
      # 다음 인자가 옵션이 아니면 파일명으로 받는다
      if [[ $# -ge 2 && "${2:0:1}" != "-" ]] && ! [[ "${2:-}" =~ ^(color|ir|depth|list|info)$ ]]; then
        SHOT="$2"; shift 2
      else
        shift
      fi ;;
    --low-latency) LOW_LATENCY=1; shift ;;
    -n|--dry-run)  DRY_RUN=1; shift ;;
    -h|--help)     usage; exit 0 ;;
    *) echo "알 수 없는 옵션: $1" >&2; echo >&2; usage >&2; exit 2 ;;
  esac
done

# ── 조회 전용 서브커맨드 ────────────────────────────────────────────
case "$STREAM" in
  list) list_nodes; exit 0 ;;
  info) show_info;  exit 0 ;;
esac

# ── 사전 점검 ───────────────────────────────────────────────────────
NEED_BIN="ffplay"
[[ "$SHOT_REQUESTED" == "1" ]] && NEED_BIN="ffmpeg"
if ! command -v "$NEED_BIN" >/dev/null 2>&1; then
  echo "오류: $NEED_BIN 가 없습니다. sudo apt install ffmpeg" >&2
  exit 1
fi

if [[ "$SHOT_REQUESTED" == "0" && "$DRY_RUN" == "0" && -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
  echo "오류: 그래픽 세션이 없습니다 (DISPLAY/WAYLAND_DISPLAY 미설정)." >&2
  echo "      SSH 접속이라면 X11 포워딩(ssh -X)이 필요합니다." >&2
  echo "      화면 없이 확인하려면 --shot 으로 정지 캡처를 쓰세요." >&2
  exit 1
fi

if [[ -z "$RS_DEV" ]]; then
  if ! RS_DEV="$(detect_node "$STREAM")"; then
    echo "오류: '$STREAM' 스트림 노드를 찾지 못했습니다." >&2
    echo >&2
    list_nodes >&2 || true
    echo >&2
    echo "  -d 옵션으로 직접 지정할 수 있습니다. 예: ./rs_view.sh $STREAM -d /dev/video2" >&2
    exit 1
  fi
fi

[[ -e "$RS_DEV" ]] || { echo "오류: 장치가 없습니다 — $RS_DEV" >&2; exit 1; }

# -d 로 직접 지정한 노드가 메타데이터 노드면 미리 알린다 (열면 ioctl 오류로 실패한다)
RS_SYS="/sys/class/video4linux/$(basename "$RS_DEV")"
if [[ -e "$RS_SYS" && "$(node_role "$RS_SYS")" == "meta" ]]; then
  echo "경고: $RS_DEV 는 메타데이터 전용 노드로 보입니다 — 영상이 나오지 않습니다." >&2
  echo "      './rs_view.sh list' 로 캡처 노드를 확인하세요." >&2
fi
if [[ ! -r "$RS_DEV" ]]; then
  echo "오류: $RS_DEV 읽기 권한이 없습니다." >&2
  echo "      데스크톱에 직접 로그인한 세션이면 ACL로 접근됩니다." >&2
  echo "      원격/서비스 세션이라면: sudo usermod -aG video \"\$USER\" 후 재로그인." >&2
  exit 1
fi

# ── 스트림별 입력 포맷과 표시 필터 ──────────────────────────────────
VF=""
case "$STREAM" in
  color) INFMT="yuyv422" ;;
  ir)    INFMT="gray" ;;
  depth)
    INFMT="gray16le"
    # 실측으로 확정한 형태 — 파일 상단 '뎁스 필터 체인 주의' 참조
    VF="format=gray,lutyuv=y=val*${RS_GAIN}"
    [[ "$RS_COLORMAP" != "none" ]] && VF="${VF},pseudocolor=preset=${RS_COLORMAP}"
    ;;
esac

# ── 명령 조립 ───────────────────────────────────────────────────────
IN_ARGS=(-f v4l2 -input_format "$INFMT" -video_size "$RS_SIZE" -framerate "$RS_FPS" -i "$RS_DEV")

if [[ "$SHOT_REQUESTED" == "1" ]]; then
  # 자동노출이 수렴하도록 앞 프레임을 흘려보내고 그 다음 1장만 저장한다.
  # (첫 프레임은 노출이 안 잡혀 새까맣게 나온다 — 실측)
  #
  # 주의: -frames:v 는 **출력** 프레임 수다. select 로 1장만 통과시키면서
  #       -frames:v 40 을 주면 40장에 영원히 도달하지 못해 ffmpeg 이 멈춘다(실제로 밟은 버그).
  #       반드시 -frames:v 1 + select=gte(...) 조합을 쓸 것.
  WARMUP=40
  if [[ -z "$SHOT" ]]; then
    SHOT="rs_${STREAM}_$(date +%Y%m%d_%H%M%S).png"
  fi
  SHOT_VF="select=gte(n\,${WARMUP})"
  [[ -n "$VF" ]] && SHOT_VF="${SHOT_VF},${VF}"
  CMD=(ffmpeg -hide_banner -loglevel error "${IN_ARGS[@]}"
       -vf "$SHOT_VF" -frames:v 1 -update 1 -y "$SHOT")
else
  CMD=(ffplay -hide_banner -loglevel warning "${IN_ARGS[@]}")
  [[ "$LOW_LATENCY" == "1" ]] && CMD+=(-fflags nobuffer -flags low_delay)
  [[ -n "$VF" ]] && CMD+=(-vf "$VF")
  CMD+=(-window_title "RealSense ${STREAM}  ${RS_DEV}  ${RS_SIZE}@${RS_FPS}fps")
fi

# ── 실행 ────────────────────────────────────────────────────────────
if [[ "$DRY_RUN" == "1" ]]; then
  printf '%q ' "${CMD[@]}"; echo
  exit 0
fi

echo "스트림: $STREAM   장치: $RS_DEV   해상도: $RS_SIZE   요청 fps: $RS_FPS"
[[ "$STREAM" == "depth" ]] && echo "뎁스 표시: 게인 ${RS_GAIN} (표시 최대거리 약 $((65280 / RS_GAIN))mm), 컬러맵 ${RS_COLORMAP}"

if [[ "$SHOT_REQUESTED" == "1" ]]; then
  echo "정지 캡처 → $SHOT  (앞 ${WARMUP}프레임은 노출 수렴용으로 버림)"
  "${CMD[@]}"
  echo "저장 완료: $SHOT"
else
  echo "종료: 창에 포커스 두고 q 또는 ESC"
  exec "${CMD[@]}"
fi
