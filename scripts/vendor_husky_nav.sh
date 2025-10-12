#!/usr/bin/env bash
set -euo pipefail

# Clone Husky and Navigation into src/vendor/. Run on host (not in container).
# After cloning, rebuild inside the container: `docker exec -it husky_gui bash` then `catkin build`

ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
VENDOR_DIR="$ROOT_DIR/src/vendor"
mkdir -p "$VENDOR_DIR"

clone_or_update() {
  local repo_url=$1
  local dest=$2
  local branch=$3
  if [ -d "$dest/.git" ]; then
    echo "[vendor] Updating $dest"
    git -C "$dest" fetch --all --tags
    git -C "$dest" checkout "$branch"
    git -C "$dest" pull --ff-only
  else
    echo "[vendor] Cloning $repo_url -> $dest (branch: $branch)"
    git clone --depth 1 -b "$branch" "$repo_url" "$dest"
  fi
}

# Husky meta-repo (includes husky_description, husky_gazebo, husky_navigation, etc.)
clone_or_update https://github.com/husky/husky.git "$VENDOR_DIR/husky" noetic-devel

# ROS Navigation stack (only if you intend to modify nav source)
# Comment out if you only need to tweak params via your own package.
clone_or_update https://github.com/ros-planning/navigation.git "$VENDOR_DIR/navigation" noetic-devel

echo "[vendor] Done. Next steps:"
echo "  1) docker compose -f docker/docker-compose.yml up -d"
echo "  2) docker exec -it husky_gui bash"
echo "  3) rosdep install --from-paths /ws/src --ignore-src -r -y"
echo "  4) catkin build"

