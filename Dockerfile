# =============================================================================
# STAGE 1: base
# ROS2 install + non-root user matching host UID/GID + locale/sudo setup,
# PLUS this project's ROS package deps (formerly Dockerfile.deps).
# Build once, tag, reuse. The committed docker-compose.yml builds only this
# stage — it's the shareable, project-portable image.
# =============================================================================
ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop AS base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
        locales \
        sudo \
        curl \
        ca-certificates \
        gnupg2 \
        lsb-release \
        software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# --- Non-root user, UID/GID passed at build time so bind-mounted files
#     created in the container are owned by you on the host, not root. ---
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME
ENV USERNAME=$USERNAME

RUN rosdep update --rosdistro $ROS_DISTRO || true
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# --- project-specific ROS packages (trim/extend this list per project) ---
USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-teleop-twist-joy \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-rviz-common \
        ros-${ROS_DISTRO}-rviz-default-plugins \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
CMD ["bash"]

# =============================================================================
# STAGE 2: dev
# Editor, shell QoL, and build/debug tooling shared across every ROS2 project.
# Neovim language servers, formatters, linters and debug adapters are managed
# by Mason inside the running container, so this layer does not need to change
# when the host Neovim config is updated.
# Built on top of base+deps. Only the docker-compose override builds this stage.
# =============================================================================
FROM base AS dev

USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        tmux \
        zsh \
        ripgrep \
        fd-find \
        fzf \
        unzip \
        wget \
        tar \
        gzip \
        build-essential \
        gdb \
        default-jre-headless \
        python3 \
        python3-venv \
        python3-pip \
        xterm \
        x11-apps \
    && mkdir -p /etc/apt/keyrings \
    && curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key \
       | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_24.x nodistro main" \
       > /etc/apt/sources.list.d/nodesource.list \
    && rm -rf /var/lib/apt/lists/*

# --- Node.js 24 LTS (required by Mason for npm-based packages) ---
RUN apt-get update && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/*

# --- Neovim, latest stable, glibc build (matches the Ubuntu base image) ---
ARG NVIM_ARCH=linux-x86_64
RUN curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-${NVIM_ARCH}.tar.gz \
    && tar -C /opt -xzf nvim-${NVIM_ARCH}.tar.gz \
    && ln -s /opt/nvim-${NVIM_ARCH}/bin/nvim /usr/local/bin/nvim \
    && rm nvim-${NVIM_ARCH}.tar.gz

# --- Mason writes its packages and Neovim writes state to ~/.local/share/nvim.
#     Pre-create the directory owned by the runtime user so a named volume
#     mounted here gets the right ownership instead of falling back to root.
# ---
RUN mkdir -p /home/$USERNAME/.local/share/nvim /home/$USERNAME/.local/state/nvim \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/.local

USER $USERNAME
CMD ["bash"]
