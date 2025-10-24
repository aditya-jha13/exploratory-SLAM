# exploratory-SLAM

Exploratory-SLAM is a minimal, reproducible ROS Noetic setup to simulate a Husky robot exploring an unknown world with online SLAM and an OpenCV-based frontier ranker. It launches Gazebo + Husky, maps with gmapping, plans with move_base, and continuously publishes the next best frontier goal.

Demo Video (coming soon)
- Placeholder for the demo: add a YouTube link or embed.
  - Simple link: https://youtu.be/your_video_id
  - Markdown preview: `[![Demo](https://img.youtube.com/vi/your_video_id/hqdefault.jpg)](https://youtu.be/your_video_id)`
  - HTML embed (works on GitHub README):
    - `<a href="https://youtu.be/your_video_id"><img src="https://img.youtube.com/vi/your_video_id/hqdefault.jpg" alt="Demo"/></a>`
  - Avoid committing large `.mp4` files to the repo; use Git LFS if you must.

Quick Start (Docker)
- Prereqs: Docker + Docker Compose plugin.
- Allow X11 (Linux): `xhost +local:` (revoke with `xhost -local:`)
- Build image: `docker compose -f docker/docker-compose.yml build`
- Start container: `docker compose -f docker/docker-compose.yml up -d`
- Shell into container: `docker exec -it husky_gui bash`
- First run builds automatically via `docker/entrypoint.sh` into volumes: `/ws/build`, `/ws/devel`, `/ws/install`.

Run the Exploration Stack
1) World + Husky (Gazebo)
   - `roslaunch frontier_ranker_ros sim_boxes.launch`
   - This enables UST10 2D LiDAR on Husky and loads `worlds/exploratory.world`.

2) SLAM + Navigation (gmapping + move_base)
   - `roslaunch husky_navigation gmapping_demo.launch`
   - Sanity checks:
     - `rostopic hz /front/scan` (>5 Hz), `rostopic hz /map` (>1 Hz)
     - `rosservice list | grep make_plan` (expect `/move_base/make_plan`)
     - If namespaced (e.g., `/husky/move_base/make_plan`), set:
       - `rosparam set /frontier_ranker/make_plan_service /husky/move_base/make_plan`

3) Frontier ranker
   - `roslaunch frontier_ranker_ros exploration_ranker.launch`
   - Publishes next goal on `/move_base_simple/goal`.

4) RViz (optional)
   - `roslaunch husky_viz view_robot.launch`

Configuration you can tune
- Frontier ranker: `src/frontier_ranker_ros/config/params.yaml`
  - `alpha, beta, gamma`: scoring weights for cluster size, path length, visited penalty.
  - `min_cluster_size, frontier_connectivity, search_radius_m, dilate_kernel`
  - `plan_tolerance, path_sample_step, update_rate, max_plans_per_cycle`
- Vendor navigation tweaks (to reproduce our “safer clearance” and “normal speed” runs):
  - `src/vendor/husky/husky_navigation/config/costmap_common.yaml` — `footprint_padding: 0.08`
  - `src/vendor/husky/husky_navigation/config/costmap_local.yaml` — `inflation.inflation_radius: 1.6`, `inflation.cost_scaling_factor: 2.5`
  - `src/vendor/husky/husky_navigation/config/planner.yaml` — under `DWAPlannerROS`
    - `occdist_scale: 2.0` (favor clearance), `path_distance_bias: 4.0`, `goal_distance_bias: 8.0`
    - `max_vel_x: 1.0`, `max_vel_trans: 0.9`, `max_vel_rot: 1.2`
  - If you hit “no valid plan” in very narrow aisles, nudge toward: `inflation_radius: 1.35`, `occdist_scale: 1.4`, `footprint_padding: 0.06`.

Docker Image Notes (trimmed)
- The image installs: ROS Noetic, Gazebo integration, Husky sim + navigation, navigation stack, gmapping, and Python deps (NumPy + OpenCV) for the ranker.
- Removed nonessential packages (e.g., rtabmap, explore-lite, depthimage_to_laserscan, RQT) to keep the image lighter and focused on `exploration_ranker.launch`.

Repository Layout
- `src/frontier_ranker_ros` — frontier detection + ranking node, worlds, launches, config
- `src/visited_layer_ros` — simple visited-grid publisher used by the ranker
- `src/vendor/husky` — Husky packages vendored from upstream (navigation configs editable)
- `docker/` — `Dockerfile`, `docker-compose.yml`, `entrypoint.sh`

Tips
- Code in `src/` is bind-mounted read-only; rebuild inside the container with `catkin build`.
- To wipe builds (clean slate): `docker compose -f docker/docker-compose.yml down -v`.
