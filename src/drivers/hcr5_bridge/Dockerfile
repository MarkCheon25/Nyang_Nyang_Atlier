# Nyang_Nyang_Atlier 개발 환경 — ROS 2 Jazzy + MoveIt2
# Base: OSRF 공식 ros:jazzy-ros-base (Ubuntu 24.04 Noble)
#
# src/moveit2/Dockerfile 을 복제한 것이다 (2026-08-16). 내용은 같고 바뀐 것은 둘뿐 —
#   · 워크스페이스 이름  ws_moveit2 → ws_atlier
#   · 이미지 이름        moveit2_dev:jazzy → nyang_nyang_atlier:jazzy (compose.yml)
# moveit2 쪽 원본은 팀 공용으로 그대로 살아 있다. 둘은 동시에 떠도 안 부딪힌다.
FROM ros:jazzy-ros-base

ARG USER_NAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000
ARG DEBIAN_FRONTEND=noninteractive

# ---- 시스템 패키지 ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    wget \
    curl \
    vim \
    nano \
    less \
    tree \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS 2 desktop (RViz2 포함) + MoveIt2 ---------------------------------
# hcr5_bridge 자체는 MoveIt 을 하나도 참조하지 않지만(package.xml · bringup.launch.py),
# /move_action 경로와 RViz MotionPlanning 패널을 쓰려면 필요하다 — README §3 '다른 경로'.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-jazzy-moveit \
    ros-jazzy-moveit-resources \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-rviz-visual-tools \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# ---- 사용자 계정 (호스트 UID/GID 매핑) ------------------------------------
# ros:jazzy 이미지에는 이미 ubuntu(1000) 사용자가 있을 수 있으므로 조건부 생성.
RUN set -e; \
    if getent passwd ${USER_UID} >/dev/null; then \
        existing_user="$(getent passwd ${USER_UID} | cut -d: -f1)"; \
        if [ "${existing_user}" != "${USER_NAME}" ]; then \
            usermod -l ${USER_NAME} -d /home/${USER_NAME} -m ${existing_user}; \
            groupmod -n ${USER_NAME} "$(getent group ${USER_GID} | cut -d: -f1)" || true; \
        fi; \
    else \
        groupadd --gid ${USER_GID} ${USER_NAME}; \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USER_NAME}; \
    fi; \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}; \
    chmod 0440 /etc/sudoers.d/${USER_NAME}

# ---- GPU 렌더 노드 접근 그룹 ------------------------------------------------
# /dev/dri/card*(video) · renderD*(render) 접근용 — 보조 수단이다.
# 실제 권한은 호스트 GID 로 걸리므로 이름만 같은 그룹으로는 부족하다.
# ⚠️ 이 PC(NVIDIA)는 compose.override.yml 의 NVIDIA 런타임으로 GPU 를 쓰므로 이 경로가
#    쓰이지 않는다. AMD/Intel PC 로 옮길 때는 compose 에 group_add 가 필요하다.
RUN groupadd -f video && groupadd -f render \
 && usermod -aG video,render ${USER_NAME}

# ---- rosdep 초기화 ---------------------------------------------------------
RUN rosdep update --rosdistro=jazzy || true

# ---- HCR-5 실기 브릿지 의존성 (2026-08-05) --------------------------------
# 컨트롤러의 네이티브 MQTT 버스(Mosquitto 1.4.7, 192.168.0.20:1883)를 직접 말한다.
# compose 가 network_mode: host 라 컨테이너에서 실기 서브넷에 그대로 닿는다.
#   libmosquitto-dev    MQTT 3.1.1 클라이언트 — 프로토콜을 손으로 짜지 않는다
#   nlohmann-json3-dev  명령/상태 페이로드 JSON (헤더 온리)
#   uuid-dev            pubWithAck RPC 의 uuid v1 — 로봇이 이 값을 응답 토픽명으로 쓴다
# ⚠️ 이 셋이 빠진 이미지로 빌드하면 find_package(nlohmann_json) 에서 깨진다 (04pc 실측).
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
      libmosquitto-dev nlohmann-json3-dev uuid-dev \
 && rm -rf /var/lib/apt/lists/*

# ---- entrypoint -----------------------------------------------------------
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# ---- 사용자 셸 셋업 -------------------------------------------------------
USER ${USER_NAME}
WORKDIR /home/${USER_NAME}

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
 && echo "[ -f ~/ws_atlier/install/setup.bash ] && source ~/ws_atlier/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
