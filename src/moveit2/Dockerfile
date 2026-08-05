# ROS 2 Jazzy + MoveIt2 tutorial 환경
# Base: OSRF 공식 ros:jazzy-ros-base (Ubuntu 24.04 Noble)
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
# 실제 권한은 호스트 GID 로 걸리므로 이름만 같은 그룹으로는 부족하고,
# run_container.sh → compose.yml 의 group_add 가 호스트 GID 를 넘겨 해결한다.
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
 && echo "[ -f ~/ws_moveit2/install/setup.bash ] && source ~/ws_moveit2/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
