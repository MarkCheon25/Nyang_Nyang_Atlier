#!/usr/bin/env bash
# 컨테이너 진입 시 환경 셋업
set -e

# ROS 2 환경 로드
source /opt/ros/jazzy/setup.bash

# 워크스페이스가 이미 빌드되어 있으면 자동 로드
if [ -f "$HOME/ws_simulation/install/setup.bash" ]; then
    source "$HOME/ws_simulation/install/setup.bash"
fi

exec "$@"
