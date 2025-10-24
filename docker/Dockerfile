# =============================================================
#  ROS Noetic + Husky + Gazebo + Navigation + SLAM + Frontier
#  Option B (clean): code-only bind mount; build artifacts in volumes
#  Ubuntu 20.04 | GUI (X11) compatible
# =============================================================
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && apt-get install -y \
    locales lsb-release gnupg2 curl wget git sudo \
    build-essential vim nano apt-utils tzdata \
 && locale-gen en_US en_US.UTF-8 && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ----------------------------
# ROS Noetic
# ----------------------------
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list' \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
 && apt-get update && apt-get install -y ros-noetic-desktop-full

# Source ROS for all shells
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# ----------------------------
# Dev tools + rosdep + catkin_tools
# ----------------------------
RUN apt-get update && apt-get install -y \
    python3-rosdep python3-rosinstall python3-wstool \
    python3-catkin-tools python3-rosinstall-generator \
 && rosdep init \
 && rosdep update

# ----------------------------
# Core runtime pkgs (APT binaries)
# ----------------------------
# Husky sim + Gazebo integration
RUN apt-get update && apt-get install -y \
    ros-noetic-husky-desktop \
    ros-noetic-husky-simulator \
    ros-noetic-husky-navigation \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control

# Navigation + costmaps + planning
RUN apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-move-base

# SLAM
RUN apt-get install -y \
    ros-noetic-slam-gmapping

# Minimal runtime utilities only
RUN apt-get install -y ros-noetic-tf2-tools

# Python deps for frontier ranking
RUN apt-get update && apt-get install -y \
    python3-pip python3-numpy python3-opencv

# ----------------------------
# GUI / X11
# ----------------------------
RUN apt-get update && apt-get install -y \
    mesa-utils x11-apps x11-xserver-utils \
    libgl1-mesa-glx libxext6 libxrender1 libglu1-mesa-dev
ENV QT_X11_NO_MITSHM=1

# ----------------------------
# Workspace entrypoint
# ----------------------------
WORKDIR /ws
COPY entrypoint.sh /ws/entrypoint.sh
RUN chmod +x /ws/entrypoint.sh

CMD ["/bin/bash"]
