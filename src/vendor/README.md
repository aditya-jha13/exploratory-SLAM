Vendor overlay for upstream packages you want to edit locally.

Recommended:
- husky (meta-repo): worlds, Gazebo, description, navigation for Husky
- navigation (only if modifying nav source; otherwise param overlays suffice)

Clone helpers
- Run: scripts/vendor_husky_nav.sh (on host)

Build
- Start container: docker compose -f docker/docker-compose.yml up -d
- Inside: rosdep install --from-paths /ws/src --ignore-src -r -y
- Inside: catkin build

Note: src/ is bind-mounted read-only into /ws/src. Edit on host; build inside container.
