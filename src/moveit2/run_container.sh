#!/usr/bin/env bash
# moveit2 개발 컨테이너 헬퍼
# 사용: ./run_container.sh {build|up|shell|down|logs|config}
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE="docker compose -f ${SCRIPT_DIR}/compose.yml"
# GPU 설정은 compose.yml 에 없다 — PC마다 이 오버라이드로 제공한다 (커밋 대상 아님).
# 없으면 GPU 없이 뜨고 RViz2 3D 가 소프트웨어 렌더링이 된다. README §2 참조.
if [ -f "${SCRIPT_DIR}/compose.override.yml" ]; then
    COMPOSE="${COMPOSE} -f ${SCRIPT_DIR}/compose.override.yml"
else
    echo "ℹ️  compose.override.yml 없음 — GPU 설정 없이 실행합니다 (RViz2 3D 느림)." >&2
    echo "   GPU를 쓰려면 README.md §2 의 스니펫으로 이 파일을 만드세요." >&2
fi
CONTAINER="moveit2_dev"

# 컨테이너 사용자를 호스트와 같은 UID/GID로 만들기 위해 compose 에 넘긴다.
# (ws_moveit2/ 안에 생기는 파일의 소유자가 어긋나지 않게 하기 위함)
export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"

# GPU 렌더 노드(/dev/dri) 접근 권한은 호스트 GID 기준으로 걸린다. GID 는 PC마다
# 다르므로(video 는 대개 44 로 같지만 render 는 992·993 등 제각각) 호스트에서 읽어
# compose 의 group_add 로 넘긴다. 컨테이너 안에 같은 '이름'의 그룹을 만들어도
# GID 가 다르면 소용없다.
export VIDEO_GID="$(getent group video  | cut -d: -f3)"
export RENDER_GID="$(getent group render | cut -d: -f3)"

# XAUTHORITY 가 없는 PC 대비 — 파일이 실재하지 않으면 빈 파일을 만들어 둔다.
# compose 의 bind mount 는 source 가 없으면 그 자리에 디렉터리를 새로 만들어버려서,
# 컨테이너 안의 ~/.Xauthority 가 디렉터리가 되고 X11 인증이 깨진다.
: "${XAUTHORITY:=${HOME}/.Xauthority}"
[ -e "${XAUTHORITY}" ] || touch "${XAUTHORITY}"
export XAUTHORITY

# X11 접근 허용 (로컬 사용자에게만)
allow_x11() {
    if command -v xhost >/dev/null 2>&1; then
        xhost +local:root >/dev/null 2>&1 || true
    fi
}

case "${1:-shell}" in
    build)
        allow_x11
        ${COMPOSE} build
        ;;
    up)
        allow_x11
        ${COMPOSE} up -d
        ;;
    shell|exec)
        allow_x11
        # 컨테이너가 안 떠있으면 기동
        if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
            ${COMPOSE} up -d
        fi
        docker exec -it "${CONTAINER}" bash
        ;;
    down)
        ${COMPOSE} down
        ;;
    logs)
        ${COMPOSE} logs -f
        ;;
    config)
        # 병합 결과 확인용 (compose.override.yml 이 제대로 얹혔는지).
        # 호스트 GID·UID 가 위에서 export 되어 있어 실제 값으로 보인다.
        ${COMPOSE} config
        ;;
    *)
        echo "Usage: $0 {build|up|shell|down|logs|config}"
        exit 1
        ;;
esac
