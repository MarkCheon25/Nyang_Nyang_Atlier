#!/usr/bin/env bash
# moveit2_tutorial 컨테이너 헬퍼
# 사용: ./run_container.sh {build|up|shell|down|logs}
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_DIR="${SCRIPT_DIR}/../docker"
COMPOSE="docker compose -f ${COMPOSE_DIR}/docker-compose.yml"

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
        if ! docker ps --format '{{.Names}}' | grep -q '^moveit2_tutorial$'; then
            ${COMPOSE} up -d
        fi
        docker exec -it moveit2_tutorial bash
        ;;
    down)
        ${COMPOSE} down
        ;;
    logs)
        ${COMPOSE} logs -f
        ;;
    *)
        echo "Usage: $0 {build|up|shell|down|logs}"
        exit 1
        ;;
esac
