#!/usr/bin/env bash
# operator 개발 컨테이너 헬퍼
# 사용: ./run_container.sh {build|up|shell|down|logs|config}
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE="docker compose -f ${SCRIPT_DIR}/compose.yml"
CONTAINER="operator_dev"

# moveit2·vision 에 있는 compose.override.yml 훅이 여기엔 없다 — 그 훅은 PC별
# GPU 설정을 위한 것이고, 이 컨테이너는 창을 띄우지 않아 GPU 가 필요 없다.

# 컨테이너 사용자를 호스트와 같은 UID/GID로 만들기 위해 compose 에 넘긴다.
# (ws_operator/·data/ 안에 생기는 파일의 소유자가 어긋나지 않게 하기 위함)
export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"

case "${1:-shell}" in
    build)
        ${COMPOSE} build
        ;;
    up)
        ${COMPOSE} up -d
        ;;
    shell|exec)
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
        # 병합 결과 확인용. HOST_UID·HOST_GID 가 위에서 export 되어 있어 실제 값으로 보인다.
        ${COMPOSE} config
        ;;
    *)
        echo "Usage: $0 {build|up|shell|down|logs|config}"
        exit 1
        ;;
esac
