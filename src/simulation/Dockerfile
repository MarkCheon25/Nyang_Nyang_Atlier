# simulation 개발 환경 (ROS 2 Jazzy + MuJoCo 3.4 — 접촉·필압 검증)
# Base: OSRF 공식 ros:jazzy-ros-base (Ubuntu 24.04 Noble)
#
# **MoveIt2 는 넣지 않는다.** 이 환경이 답하는 질문은 "펜이 종이에 어떻게 닿는가"이고,
# 궤적 계획(F4.2)은 moveit2 환경의 몫이다. 궤적을 넣어 돌리는 데는
# joint_trajectory_controller 만 있으면 된다 — 그만큼 이미지가 가볍다.
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
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# ---- MuJoCo + ros2_control -------------------------------------------------
# BRD D5("MuJoCo 3.x + ROS2 Jazzy 연동 방식")의 답 — **apt 바이너리가 실재한다.**
# ros-controls 조직이 유지하는 공식 패키지이며 소스 빌드가 필요 없다.
# (2026-07-28 확인: mujoco-ros2-control 0.0.3 / mujoco-vendor = MuJoCo 3.4.0)
#
# ⚠️ 0.0.3 은 아주 초기 버전이다. 막히는 지점이 나오면 dfki-ric/mujoco_ros2_control
# (소스 빌드, FT 센서 지원이 명시적)로 갈아끼우는 것이 대안 — 하드웨어 인터페이스
# 경계라 교체 비용이 작다. README "연동 패키지" 참조.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-mujoco-ros2-control \
    ros-jazzy-mujoco-ros2-control-plugins \
    ros-jazzy-mujoco-ros2-control-msgs \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-force-torque-sensor-broadcaster \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
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
# MuJoCo 뷰어를 띄울 때만 쓰인다. 확인 경로(SVG)는 파일 출력이라 GPU 없이도 되지만,
# 씬을 눈으로 보려면 필요하다. 실제 권한은 호스트 GID 로 걸리므로
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
 && echo "[ -f ~/ws_simulation/install/setup.bash ] && source ~/ws_simulation/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
