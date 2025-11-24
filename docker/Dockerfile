# =========================
# Base: Ubuntu 22.04 (ROS 2 Humble + Gazebo Garden + ros_gz bridge)
# =========================

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Locale
RUN apt-get update && apt-get install -y locales && locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# Basics & utilities
RUN apt-get update && apt-get install -y \
    curl wget gnupg2 lsb-release ca-certificates software-properties-common \
    git build-essential cmake \
    python3-pip python3-apt \
    sudo bash-completion \
    mesa-utils mesa-utils-extra libgl1-mesa-dri x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Ensure 'universe' is enabled (needed on minimal images)
RUN add-apt-repository universe && apt-get update

# --- Install ROS dev helpers via pip (more reliable across arches) ---
RUN pip3 install --no-cache-dir -U \
    colcon-common-extensions \
    vcstool \
    rosdep

# -------------------------
# ROS 2 Humble
# -------------------------

RUN apt-get update && apt-get install -y curl gnupg2 && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Gazebo (Garden) + ros_gz
# -------------------------

# Gazebo (Garden) + ros_gz — correct repo path
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O - | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y \
    gz-garden \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image \
    ros-humble-ros-gz-interfaces \
    && rm -rf /var/lib/apt/lists/*

# Useful ROS dev tools
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Setup rosdep
RUN rosdep init || true
RUN bash -lc "source /opt/ros/humble/setup.bash && rosdep update"

# Create non-root user (optional but nice)
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID $USERNAME && useradd -m -u $UID -g $GID -s /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Workspace
RUN mkdir -p /ws/ros2_ws/src
WORKDIR /ws/ros2_ws

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
