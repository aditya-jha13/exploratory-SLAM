# =============================================================
#  ROS Noetic + Husky + Gazebo + Navigation + SLAM + Frontier
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
# Dev tools + rosdep
# ----------------------------
RUN apt-get update && apt-get install -y \
    python3-rosdep python3-rosinstall python3-wstool \
    python3-catkin-tools python3-rosinstall-generator
RUN rosdep init && rosdep update

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

# SLAM choices (pick at runtime)
RUN apt-get install -y \
    ros-noetic-slam-gmapping \
    ros-noetic-rtabmap-ros

# Exploration packages
RUN apt-get install -y \
    ros-noetic-frontier-exploration \
    ros-noetic-explore-lite

# depthimage_to_laserscan (apt is simpler than source)
RUN apt-get install -y ros-noetic-depthimage-to-laserscan

# Useful utils
RUN apt-get install -y \
    ros-noetic-tf2-tools ros-noetic-rqt ros-noetic-rqt-common-plugins

# ----------------------------
# Python deps for your frontier_ranker
# ----------------------------
RUN apt-get update && apt-get install -y \
    python3-pip python3-numpy python3-opencv

# ----------------------------
# Catkin workspace for your custom packages (visited layer, ranker)
# ----------------------------
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# (Optional) drop your packages here later with docker build --add or COPY:
# COPY visited_layer_ros /root/catkin_ws/src/visited_layer_ros
# COPY frontier_ranker_ros /root/catkin_ws/src/frontier_ranker_ros

WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# ----------------------------
# GUI / X11
# ----------------------------
RUN apt-get update && apt-get install -y \
    mesa-utils x11-apps x11-xserver-utils \
    libgl1-mesa-glx libxext6 libxrender1 libglu1-mesa-dev
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

WORKDIR /root
CMD ["/bin/bash"]
