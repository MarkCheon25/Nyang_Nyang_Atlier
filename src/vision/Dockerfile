# vision 개발 환경 (ROS 2 Jazzy + OpenCV 4.6 contrib)
# Base: OSRF 공식 ros:jazzy-ros-base (Ubuntu 24.04 Noble)
#
# moveit2 환경과 분리한 이유 — RViz2·MoveIt2 가 필요 없어 이미지가 훨씬 가볍고,
# 공용 moveit2 이미지를 건드리지 않으므로 동료 PC 에 재빌드를 강요하지 않는다.
# 두 환경 통합은 추후 과제.
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
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ---- OpenCV 4.6 + contrib --------------------------------------------------
# contrib 가 필요한 이유: centerline 추출에 ximgproc.thinning 을 쓴다.
# Canny 는 선의 이중 윤곽을 내므로 라인아트에는 부적합 (System Architecture.md §5.3).
# ArUco(F5 캘리브레이션 지원)도 4.6 에서는 contrib 소속이다.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev \
    libopencv-contrib-dev \
    ros-jazzy-cv-bridge \
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
# vision 은 CPU 만으로 동작한다. 이 그룹은 cv::imshow 로 창을 띄울 때만 쓰인다.
# 실제 권한은 호스트 GID 로 걸리므로 이름만 같은 그룹으로는 부족하고,
# run_container.sh → compose.yml 의 group_add 가 호스트 GID 를 넘겨 해결한다.
RUN groupadd -f video && groupadd -f render \
 && usermod -aG video,render ${USER_NAME}

# ---- rosdep 초기화 ---------------------------------------------------------
RUN rosdep update --rosdistro=jazzy || true

# ---- entrypoint -----------------------------------------------------------
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# ---- 사용자 셸 셋업 -------------------------------------------------------
USER ${USER_NAME}
WORKDIR /home/${USER_NAME}

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
 && echo "[ -f ~/ws_vision/install/setup.bash ] && source ~/ws_vision/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
