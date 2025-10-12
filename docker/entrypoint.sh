#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash

# Configure and build if needed
if [ ! -f /ws/devel/setup.bash ]; then
  echo "[entrypoint] First-time catkin config & build..."
  catkin config \
    --extend /opt/ros/noetic \
    --source-space /ws/src \
    --build-space /ws/build \
    --devel-space /ws/devel \
    --install-space /ws/install
  catkin build
fi

echo "source /ws/devel/setup.bash" >> ~/.bashrc
source /ws/devel/setup.bash

# Keep shell interactive for you / VS Code
exec bash
