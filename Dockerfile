# =============================================================
#  ROS Noetic + Husky + Navigation + depthimage_to_laserscan
#  Ubuntu 20.04 | GUI (X11) compatible
# =============================================================
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# ----------------------------
# 1. System setup
# ----------------------------
RUN apt-get update && apt-get install -y \
    locales lsb-release gnupg2 curl wget git sudo \
    build-essential vim nano \
    apt-utils tzdata \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ----------------------------
# 2. ROS Noetic installation
# ----------------------------
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list' \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
 && apt-get update && apt-get install -y ros-noetic-desktop-full

# Source ROS setup in every new shell
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# ----------------------------
# 3. Build dependencies
# ----------------------------
RUN apt-get update && apt-get install -y \
    python3-rosdep python3-rosinstall python3-wstool \
    python3-catkin-tools python3-rosinstall-generator

RUN rosdep init && rosdep update

# ----------------------------
# 4. Create Catkin workspace
# ----------------------------
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Clone repositories
RUN git clone https://github.com/husky/husky.git \
 && git clone https://github.com/ros-planning/navigation.git \
 && git clone https://github.com/ros-perception/depthimage_to_laserscan.git

# ----------------------------
# 5. Install package dependencies
# ----------------------------
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# ----------------------------
# 6. Build workspace
# ----------------------------
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source workspace automatically
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# ----------------------------
# 7. GUI dependencies (for RViz / Gazebo / rqt)
# ----------------------------
RUN apt-get update && apt-get install -y \
    mesa-utils \
    x11-apps \
    x11-xserver-utils \
    libgl1-mesa-glx \
    libxext6 libxrender1 libxcomposite-dev libglu1-mesa-dev

# Set environment for X11 forwarding
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

# ----------------------------
# 8. Final setup
# ----------------------------
WORKDIR /root
CMD ["/bin/bash"]

