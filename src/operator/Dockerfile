# operator 개발 환경 (ROS 2 Jazzy — ⑥ 작업 관리 + 웹 브릿지)
# Base: OSRF 공식 ros:jazzy-ros-base (Ubuntu 24.04 Noble)
#
# 세 환경 중 가장 가볍다 — RViz2·MoveIt2 도 OpenCV 도 필요 없다.
# ⑥ 작업 관리는 순수 계산이고, 화면은 브라우저가 그린다.
#
# ⚠️ **웹(React + Node.js)은 이 이미지에 없다.** 별도 서비스로 붙을 예정이며
# 자리는 compose.yml 하단에 주석으로 잡아 두었다. ROS 컨테이너에 Node 런타임을
# 끌어들이면 두 스택의 빌드 캐시가 서로를 무효화한다.
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
    jq \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ---- 웹 ↔ ROS 브릿지 --------------------------------------------------------
# 브라우저(roslibjs)와 Node 백엔드가 ROS 에 닿는 유일한 경로 (SA §5.2).
# 아직 job_manager 가 서비스·토픽을 열지 않아 띄우지 않지만, 경계가 정해지는 즉시
# 필요해지므로 이미지에 미리 넣어 둔다 — 설치가 가볍다.
#
# ⚠️ rosbridge 는 토픽·서비스만 지원하고 **액션은 지원하지 않는다** (SA §5.3).
# 그래서 MoveIt2 액션 호출은 웹이 아니라 job_manager 가 서버측에서 한다.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rosbridge-suite \
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

# GPU·X11 설정이 없는 것은 의도된 것이다 — 이 컨테이너는 창을 띄우지 않는다.
# moveit2·vision 에 있는 compose.override.yml 훅도 여기엔 두지 않는다.

# ---- rosdep 초기화 ---------------------------------------------------------
RUN rosdep update --rosdistro=jazzy || true

# ---- entrypoint -----------------------------------------------------------
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# ---- 사용자 셸 셋업 -------------------------------------------------------
USER ${USER_NAME}
WORKDIR /home/${USER_NAME}

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
 && echo "[ -f ~/ws_operator/install/setup.bash ] && source ~/ws_operator/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
