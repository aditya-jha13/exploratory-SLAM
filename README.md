# exploratory-SLAM

Exploratory-SLAM is a minimal, reproducible ROS Noetic setup to simulate a Husky robot exploring an unknown world with online SLAM and an OpenCV-based frontier ranker. It launches Gazebo + Husky, maps with gmapping, plans with move_base, and continuously publishes the next best frontier goal.



https://github.com/user-attachments/assets/2bfaccc4-1a94-4236-867b-ec70715e4156



## Setup
- Prerequisites: Docker and Docker Compose plugin.
- Allow X11 (Linux): `xhost +local:` (revoke with `xhost -local:`)
- Build image: `docker compose -f docker/docker-compose.yml build`
- Start container: `docker compose -f docker/docker-compose.yml up -d`
- Shell into container: `docker exec -it husky_gui bash`
- First run builds automatically via `docker/entrypoint.sh` into volumes: `/ws/build`, `/ws/devel`, `/ws/install`.

## Run
### 1) World + Husky (Gazebo)
   - `roslaunch frontier_ranker_ros sim_boxes.launch`
   - This enables UST10 2D LiDAR on Husky and loads `worlds/exploratory.world`.

### 2) SLAM + Navigation (gmapping + move_base)
   - `roslaunch husky_navigation gmapping_demo.launch`
   - Sanity checks:
     - `rostopic hz /front/scan` (>5 Hz), `rostopic hz /map` (>1 Hz)
     - `rosservice list | grep make_plan` (expect `/move_base/make_plan`)
     - If namespaced (e.g., `/husky/move_base/make_plan`), set:
       - `rosparam set /frontier_ranker/make_plan_service /husky/move_base/make_plan`

### 3) Frontier ranker
   - `roslaunch frontier_ranker_ros exploration_ranker.launch`
   - Publishes next goal on `/move_base_simple/goal`.

### 4) RViz (optional)
   - `roslaunch husky_viz view_robot.launch`

## Reproducibility

To ensure consistent results, this section lists all key configuration parameters that influence the exploration behavior and navigation characteristics.

### Frontier Ranker Configuration  
File: `src/frontier_ranker_ros/config/params.yaml`

| Parameter | Description |
|------------|-------------|
| `alpha` | Weight for frontier cluster size (larger = prefer bigger frontiers) |
| `beta` | Weight for path length (larger = prefer shorter paths) |
| `gamma` | Penalty for revisiting previously explored regions |
| `min_cluster_size` | Minimum frontier cluster size considered valid |
| `frontier_connectivity` | Connectivity threshold between frontier pixels |
| `search_radius_m` | Search radius in meters |
| `dilate_kernel` | Dilation kernel size used in frontier extraction |
| `plan_tolerance` | Allowed deviation in goal plan |
| `path_sample_step` | Path sampling step for evaluation |
| `update_rate` | Ranker update frequency (Hz) |
| `max_plans_per_cycle` | Maximum plans evaluated per iteration |

These parameters define how aggressively or conservatively the robot explores.  
For faster but less exhaustive runs, reduce `gamma` and increase `alpha`.  
For more cautious exploration, increase `beta` and `plan_tolerance`.

---

### Navigation Parameters (Vendor Configuration)

Located in:  
**`src/vendor/husky/husky_navigation/config/`**

#### `costmap_common.yaml`
```yaml
footprint_padding: 0.08
```
#### `costmap_local.yaml`
```yaml
inflation:
  inflation_radius: 1.6
  cost_scaling_factor: 2.5
```
#### `planner.yaml`
```yaml
DWAPlannerROS:
  occdist_scale: 2.0          # Prioritize clearance around obstacles
  path_distance_bias: 4.0     # Encourage staying close to the global path
  goal_distance_bias: 8.0     # Weight goal proximity in trajectory evaluation

  max_vel_x: 1.0
  max_vel_trans: 0.9
  max_vel_rot: 1.2
```

## Tips
- Code in `src/` is bind-mounted read-only; rebuild inside the container with **`catkin build`**.
- To wipe builds (clean slate): `docker compose -f docker/docker-compose.yml down -v`.
