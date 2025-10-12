# exploratory-SLAM

Clean ROS workspace with Docker volumes (Option B). Host mounts only `src/` read-only; build artifacts live in Docker named volumes for fast, clean rebuilds.

Quick start
- Prereqs: Docker + Docker Compose plugin installed.
- Allow X11 (Linux): `xhost +local:` (revoke later with `xhost -local:`)
- Build image: `docker compose -f docker/docker-compose.yml build`
- Run container: `docker compose -f docker/docker-compose.yml up -d`
- Shell inside: `docker exec -it husky_gui bash`
- Build runs automatically on first start via `docker/entrypoint.sh` using volumes: `/ws/build`, `/ws/devel`, `/ws/install`.

Run the exploration demo
1) Start the container (new terminal on host)
   - `xhost +local:root`
   - `docker compose -f docker/docker-compose.yml up -d --build`

2) In the container shell
   - `docker exec -it husky_gui bash`
   - Optional first-time build for this package: `catkin build frontier_ranker_ros && source /ws/devel/setup.bash`

3) Start Gazebo with offline world + spawn Husky (LiDAR enabled)
   - `roslaunch frontier_ranker_ros sim_boxes.launch`
   - Note: `sim_boxes.launch` already enables a 2D LiDAR (UST10) via env vars so `/front/scan` is available for mapping.
   - If you use other worlds (e.g., `husky_gazebo empty_world.launch` or `husky_playpen.launch`) or spawn the robot manually, enable LiDAR first in the same shell:
     - `export HUSKY_UST10_ENABLED=1; export HUSKY_UST10_TOPIC=front/scan`
     - Alternative sensor: `export HUSKY_LMS1XX_ENABLED=1; export HUSKY_LMS1XX_TOPIC=front/scan`

4) Start mapping + navigation (move_base + gmapping)
   - `roslaunch husky_navigation gmapping_demo.launch`
   - Sanity checks:
     - `rostopic hz /front/scan` (>5 Hz)
     - `rostopic hz /map` (>1 Hz)
     - `rosservice list | grep make_plan` (expect `/move_base/make_plan`)
     - If the service is namespaced (e.g., `/husky/move_base/make_plan`), point the ranker to it:
       - `rosparam set /frontier_ranker/make_plan_service /husky/move_base/make_plan`

5) Start visited layer + frontier ranker
   - `roslaunch frontier_ranker_ros exploration_ranker.launch`
   - The ranker connects to `/move_base/make_plan` and publishes goals to `/move_base_simple/goal` as frontiers appear.

6) Start RViz (Husky preset)
   - `roslaunch husky_viz view_robot.launch`
   - Tip: hide Global/Local Costmap “costmap” displays or reduce Alpha if the magenta unknown-space overlay is distracting.

Tuning the ranker (edit `src/frontier_ranker_ros/config/params.yaml`)
- `min_cluster_size`: 5–10 initially (fewer targets early)
- `frontier_connectivity`: 4 (faster clustering) or 8 (denser)
- `beta`: increase to penalize long paths
- `gamma`: 0.0–1.5 to reduce backtracking via visited penalty
- `update_rate`: 0.2–0.5 to save CPU
- `path_sample_step`: 0.2–0.3 to make visited penalty cheaper

Troubleshooting
- No `/map`: ensure LiDAR is enabled (sim_boxes.launch sets UST10 env), and move robot briefly to seed SLAM.
- Ranker “waiting for /move_base/make_plan”: start gmapping demo; if service is namespaced, set `rosparam set /frontier_ranker/make_plan_service <service>` and relaunch.
- Software GL (llvmpipe): complete NVIDIA toolkit install or run RViz on host while running Gazebo headless in container.


Layout
- `src/` — your catkin packages (read-only in container)
- `docker/` — Dockerfile, docker-compose.yml, entrypoint
- Named volumes — `catkin_build`, `catkin_devel`, `catkin_install`

Packages scaffolded
- `visited_layer_ros` — placeholder costmap_2d layer to track visited cells
- `frontier_ranker_ros` — placeholder frontier clustering + ranking node

GPU acceleration (optional but recommended)
- Host: install NVIDIA Container Toolkit (Ubuntu 22.04):
  - `distribution=$(. /etc/os-release; echo ${ID}${VERSION_ID})`
  - `sudo mkdir -p /usr/share/keyrings`
  - `curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg`
  - `curl -s -L https://nvidia.github.io/libnvidia-container/stable/$distribution/libnvidia-container.list | sed "s#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g" | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list`
  - `sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit`
  - `sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker`
- Compose already sets:
  - `gpus: all`, `NVIDIA_VISIBLE_DEVICES=all`, `NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute`
  - PRIME offload env: `__NV_PRIME_RENDER_OFFLOAD=1`, `__GLX_VENDOR_LIBRARY_NAME=nvidia`, `__VK_LAYER_NV_optimus=NVIDIA_only`
- Verify in container:
  - `apt-get update && apt-get install -y mesa-utils`
  - `glxinfo -B` should show your NVIDIA GPU (not llvmpipe)

Local Husky and Navigation (overlay)
- Clone locally into `src/vendor/` so they’re mounted into the container and overlay apt packages:
  - `scripts/vendor_husky_nav.sh`
- After cloning, rebuild in the container:
  - `docker exec -it husky_gui bash`
  - `rosdep install --from-paths /ws/src --ignore-src -r -y`
  - `catkin build`

How to proceed
1) Visited Path Memory (costmap layer)
   - Implement a `costmap_2d` layer plugin that marks visited cells from `/tf` or `/odom` and inflates cost for revisits.
   - Export via pluginlib (add `plugin.xml`, update `package.xml`, and register in `CMakeLists.txt`).
   - Wire into `move_base` global/local costmap params to load the layer.

2) Frontier Clustering + Ranking
   - Detect frontier cells from `/map` (OccupancyGrid), cluster (e.g., DBSCAN), compute a score = alpha*size - beta*distance - gamma*visited_overlap.
   - Publish best cluster centroid as a goal (`/move_base_simple/goal`) or replace frontier_exploration’s goal selection.
   - Expose tunable params via ROS params and optional RViz markers for debug.

3) Integration and testing
   - Start Husky sim with `docker compose up`; in the container, launch Husky + navigation + your nodes.
   - Validate that paths prefer unvisited corridors and goals target larger frontier clusters.
   - Iterate on weights and visualize layer + clusters in RViz.

Notes
- Code changes on host are picked up instantly (src is bind-mounted read-only). Rebuild inside the container with `catkin build`.
- To wipe builds: `docker compose down -v` (removes named volumes).
