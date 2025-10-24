#!/usr/bin/env python3
import math
import time
import threading
from typing import List, Tuple, Optional

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetPlanRequest
import tf2_ros
import cv2
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool

# Helper imports (robust against stale installed copies) - was facing this issue (GPT-suggested.)
fc_find_frontiers = None
fc_path_length = None
fc_visited_penalty = None
fc_visited_fraction = None

try:
    import frontier_core as _fc
except Exception:
    _fc = None

def _load_frontier_core_from_paths():
    import os, sys
    # Candidate paths: installed script dir and package scripts dir
    candidates = []
    try:
        import rospkg
        pkg_path = rospkg.RosPack().get_path('frontier_ranker_ros')
        scripts_dir = os.path.join(pkg_path, 'scripts')
        candidates.append(os.path.join(scripts_dir, 'frontier_core.py'))
    except Exception:
        pass
    try:
        here = os.path.dirname(__file__)
        candidates.append(os.path.join(here, 'frontier_core.py'))
    except Exception:
        pass
    for path in candidates:
        try:
            if path and os.path.isfile(path):
                import importlib.util, importlib.machinery
                mod_name = 'frontier_core_fallback'
                loader = importlib.machinery.SourceFileLoader(mod_name, path)
                spec = importlib.util.spec_from_loader(mod_name, loader)
                module = importlib.util.module_from_spec(spec)
                loader.exec_module(module)
                required = [
                    'find_frontier_candidates_cv',
                    'path_length',
                    'visited_penalty_along',
                    'visited_fraction_around',
                ]
                if all(hasattr(module, n) for n in required):
                    return module
        except Exception:
            continue
    return None

def _bind_fc_aliases(module):
    global fc_find_frontiers, fc_path_length, fc_visited_penalty, fc_visited_fraction
    fc_find_frontiers = module.find_frontier_candidates_cv
    fc_path_length = module.path_length
    fc_visited_penalty = module.visited_penalty_along
    fc_visited_fraction = module.visited_fraction_around

_fc_ok = False
if _fc is not None:
    try:
        required = [
            'find_frontier_candidates_cv',
            'path_length',
            'visited_penalty_along',
            'visited_fraction_around',
        ]
        if all(hasattr(_fc, n) for n in required):
            _bind_fc_aliases(_fc)
            _fc_ok = True
    except Exception:
        _fc_ok = False

if not _fc_ok:
    _fc2 = _load_frontier_core_from_paths()
    if _fc2 is not None:
        _bind_fc_aliases(_fc2)
        _fc_ok = True

if not _fc_ok:
    raise ImportError('Unable to import frontier_core with required symbols')


class FrontierRankerNode:
    def __init__(self):
        # Parameters (read first)
        self.robot_base_frame = rospy.get_param("~robot_base_frame", "base_link")
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.cluster_min_size = int(rospy.get_param("~min_cluster_size", 10))
        self.plan_tolerance = float(rospy.get_param("~plan_tolerance", 0.5))
        self.alpha = float(rospy.get_param("~alpha", 1.0))   # weight for cluster size
        self.beta = float(rospy.get_param("~beta", 1.0))    # weight for path length
        self.gamma = float(rospy.get_param("~gamma", 2.0))  # weight for visited penalty
        self.update_rate = float(rospy.get_param("~update_rate", 0.5))
        self.path_sample_step = float(rospy.get_param("~path_sample_step", 0.1))
        self.frontier_connectivity = int(rospy.get_param("~frontier_connectivity", 8))
        self.search_radius_m = float(rospy.get_param("~search_radius_m", 25.0))
        self.dilate_kernel = int(rospy.get_param("~dilate_kernel", 3))
        # Clearance preferences for goal placement (meters)
        self.goal_clearance_m = float(rospy.get_param("~goal_clearance_m", 0.35))
        self.clearance_window_m = float(rospy.get_param("~clearance_window_m", 0.8))
        self.unknown_is_obstacle_for_goal = bool(rospy.get_param("~unknown_is_obstacle_for_goal", True))
        self.dt_mask_size = int(rospy.get_param("~dt_mask_size", 3))
        self.visited_threshold = int(rospy.get_param("~visited_threshold", 50))
        # Extra penalty if the goal point is already marked visited
        self.goal_revisit_penalty = float(rospy.get_param("~goal_revisit_penalty", 0.0))
        self.goal_revisit_radius_m = float(rospy.get_param("~goal_revisit_radius_m", 0.0))
        self.goal_revisit_area_weight = float(rospy.get_param("~goal_revisit_area_weight", 0.0))
        self.make_plan_service = rospy.get_param("~make_plan_service", "/move_base/make_plan")
        # Debug/Perf params
        self.debug_per_cluster = bool(rospy.get_param("~debug_per_cluster", False))
        self.max_debug_clusters = int(rospy.get_param("~max_debug_clusters", 5))
        self.max_plans_per_cycle = int(rospy.get_param("~max_plans_per_cycle", 50))
        self.warn_if_over_s = float(rospy.get_param("~warn_if_over_s", 0.5))
        self.use_make_plan = bool(rospy.get_param("~use_make_plan", True))
        # Hard time budget for candidate evaluation
        self.max_think_time_s = float(rospy.get_param("~max_think_time_s", 5.0))
        # Only publish a goal without a verified plan if allowed
        self.allow_heuristic_fallback = bool(rospy.get_param("~allow_heuristic_fallback", False))
        # Retargeting and tabu defaults
        self.retarget_hold_time_s = float(rospy.get_param("~retarget_hold_time_s", 2.0))
        self.retarget_min_score_gain = float(rospy.get_param("~retarget_min_score_gain", 0.5))
        self.goal_tabu_duration_s = float(rospy.get_param("~goal_tabu_duration_s", 30.0))
        # Stuck retarget behavior (immediate override)
        self.stuck_retarget_after_s = float(rospy.get_param("~stuck_retarget_after_s", 3.0))
        self.stuck_no_move_dist_m = float(rospy.get_param("~stuck_no_move_dist_m", 0.05))
        self.config_pose_tolerance_m = float(rospy.get_param("~config_pose_tolerance_m", 0.10))
        # Stuck/wander fallback
        self.stuck_timeout = float(rospy.get_param("~stuck_timeout", 12.0))
        self.stuck_min_progress = float(rospy.get_param("~stuck_min_progress", 0.3))
        self.wander_distance = float(rospy.get_param("~wander_distance", 2.0))
        self.wander_angle_step_deg = int(rospy.get_param("~wander_angle_step_deg", 30))
        # Exclude new goals near any previously commanded waypoint
        self.waypoint_exclusion_radius_m = float(rospy.get_param("~waypoint_exclusion_radius_m", 5.0))
        # Goal reached tolerance/cooldown
        self.goal_reached_radius = float(rospy.get_param("~goal_reached_radius", 0.5))
        self.goal_reached_cooldown_s = float(rospy.get_param("~goal_reached_cooldown_s", 1.0))
        self.goal_offset_m = float(rospy.get_param("~goal_offset_m", 0.5))
        self.fallback_ring_radius_m = float(rospy.get_param("~fallback_ring_radius_m", 0.5))
        self.fallback_ring_samples = int(rospy.get_param("~fallback_ring_samples", 16))
        # Fallback when no candidates are found repeatedly
        self.no_candidate_retry_count = int(rospy.get_param("~no_candidate_retry_count", 5))
        self.fallback_farthest_stride = int(rospy.get_param("~fallback_farthest_stride", 6))
        self.fallback_farthest_topk = int(rospy.get_param("~fallback_farthest_topk", 30))

        # State and locks (init before subscribing)
        self.map_lock = threading.Lock()
        self.map = None
        self.map_info = None
        self.visited_grid = None
        self.visited_info = None

        # TF available before callbacks
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers/Subscribers
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self._on_map, queue_size=1)
        self.visited_grid_sub = rospy.Subscriber("/visited_grid", OccupancyGrid, self._on_visited, queue_size=1)

        # GetPlan client
        rospy.loginfo("[frontier_ranker] waiting for %s", self.make_plan_service)
        rospy.wait_for_service(self.make_plan_service)
        self.make_plan = rospy.ServiceProxy(self.make_plan_service, GetPlan)
        rospy.loginfo("[frontier_ranker] connected to %s", self.make_plan_service)
        rospy.loginfo("[frontier_ranker] params: alpha=%.2f beta=%.2f gamma=%.2f min_cluster=%d conn=%d upd=%.2f tol=%.2f sample_step=%.2f max_plans=%d",
                      self.alpha, self.beta, self.gamma, self.cluster_min_size, self.frontier_connectivity,
                      self.update_rate, self.plan_tolerance, self.path_sample_step, self.max_plans_per_cycle)
        rospy.loginfo("[frontier_ranker] roi=%.1fm dilate=%d goal_offset=%.2f ring=%.2f samples=%d",
                      self.search_radius_m, self.dilate_kernel, self.goal_offset_m, self.fallback_ring_radius_m, self.fallback_ring_samples)

        # Timer
        self._last_goal = None
        self._last_goal_score = None
        self._last_goal_publish_time = rospy.Time(0)
        self._publish_pose_at_goal = None  # (x, y) at publish time
        # Candidates considered for the configuration when last goal was published
        self._last_config_candidates = None  # list of (score, x, y) sorted desc
        self._last_config_pose = None        # (x, y) robot pose used to compute candidates
        self._last_config_map_stamp = None   # map header stamp at that time
        self._config_published_indices = set()
        # Backoff for make_plan service failures to reduce log spam
        self._make_plan_backoff_until = rospy.Time(0)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.update_rate, 1e-3)), self._on_timer)

        # Progress tracking
        self._progress_pose = None
        self._progress_time = rospy.Time.now()

        # Reached cooldown
        self._reached_cooldown_until = rospy.Time(0)

        # Waypoint history
        self._waypoint_history = []  # list of (x, y)
        # Short-term goal tabu history (timestamp, x, y)
        self._goal_history = []
        self._no_candidate_count = 0

        # Visualization publishers
        self.goal_point_pub = rospy.Publisher("~goal_point", PointStamped, queue_size=1, latch=True)
        self.goal_marker_pub = rospy.Publisher("~goal_marker", Marker, queue_size=1, latch=True)

    def _make_marker(self, header, x: float, y: float, *, mid: int = 1,
                      ns: str = "frontier_ranker", rgb: Tuple[float, float, float] = (1.0, 0.2, 0.2),
                      alpha: float = 0.9, scale: float = 0.2, z: float = 0.05, lifetime_s: float = 2.0) -> Marker:
        mk = Marker()
        mk.header = header
        mk.ns = ns
        mk.id = mid
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.position.x = x
        mk.pose.position.y = y
        mk.pose.position.z = z
        mk.pose.orientation.w = 1.0
        mk.scale.x = scale
        mk.scale.y = scale
        mk.scale.z = scale
        mk.color.r = float(rgb[0])
        mk.color.g = float(rgb[1])
        mk.color.b = float(rgb[2])
        mk.color.a = float(alpha)
        mk.lifetime = rospy.Duration(lifetime_s)
        return mk

    def _on_visited(self, msg: OccupancyGrid):
        grid = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        self.visited_grid = grid
        self.visited_info = msg

    def _on_map(self, msg: OccupancyGrid):
        grid = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        with self.map_lock:
            self.map = grid
            self.map_info = msg

    def _get_robot_pose(self) -> Optional[Pose]:
        try:
            tfm = self.tf_buffer.lookup_transform(self.global_frame, self.robot_base_frame, rospy.Time(0), rospy.Duration(0.5))
        except Exception as ex:
            rospy.logwarn_throttle(2.0, f"[frontier_ranker] TF lookup failed: {ex}")
            return None
        p = Pose()
        p.position.x = tfm.transform.translation.x
        p.position.y = tfm.transform.translation.y
        p.position.z = tfm.transform.translation.z
        p.orientation = tfm.transform.rotation
        return p


    def _world_to_map(self, x: float, y: float, info) -> Tuple[int, int]:
        res = info.resolution
        mx = int((x - info.origin.position.x) / res)
        my = int((y - info.origin.position.y) / res)
        return mx, my

    def _is_free_world(self, x: float, y: float, grid: np.ndarray, info) -> bool:
        mx, my = self._world_to_map(x, y, info)
        if 0 <= mx < info.width and 0 <= my < info.height:
            return grid[my, mx] == 0
        return False

    def _is_clear_world(self, x: float, y: float, grid: np.ndarray, info, clear_m: float) -> bool:
        """Returns True if the disc of radius `clear_m` around (x,y) is free.
        Treats unknown as obstacle if `unknown_is_obstacle_for_goal` is True."""
        if clear_m <= 1e-6:
            return self._is_free_world(x, y, grid, info)
        res = max(getattr(info, 'resolution', 0.0), 1e-6)
        r_cells = max(1, int(clear_m / res))
        mx_c, my_c = self._world_to_map(x, y, info)
        r2 = r_cells * r_cells
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                if dx*dx + dy*dy > r2:
                    continue
                mx = mx_c + dx
                my = my_c + dy
                if not (0 <= mx < info.width and 0 <= my < info.height):
                    # Outside map treated as obstacle
                    return False
                v = int(grid[my, mx])
                if v > 0:
                    return False
                if v < 0 and self.unknown_is_obstacle_for_goal:
                    return False
        return True

    def _map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        info = self.map_info.info
        x = info.origin.position.x + (mx + 0.5) * info.resolution
        y = info.origin.position.y + (my + 0.5) * info.resolution
        return x, y

        # Old grid-scan frontier helpers removed (replaced by OpenCV pipeline)

    def _plan(self, start: Pose, goal_xy: Tuple[float, float]) -> Optional[Path]:
        if not self.use_make_plan:
            return None
        req = GetPlanRequest()
        req.start = PoseStamped()
        req.start.header.stamp = rospy.Time.now()
        req.start.header.frame_id = self.global_frame
        req.start.pose = start
        req.goal = PoseStamped()
        req.goal.header.stamp = rospy.Time.now()
        req.goal.header.frame_id = self.global_frame
        req.goal.pose.position.x = goal_xy[0]
        req.goal.pose.position.y = goal_xy[1]
        req.goal.pose.orientation.w = 1.0
        req.tolerance = self.plan_tolerance
        try:
            if rospy.Time.now() < self._make_plan_backoff_until:
                return None
            resp = self.make_plan(req)
            if resp.plan and len(resp.plan.poses) > 0:
                return resp.plan
            # Empty plan response; short backoff
            self._make_plan_backoff_until = rospy.Time.now() + rospy.Duration(1.0)
            return None
        except rospy.ServiceException as ex:
            rospy.logwarn_throttle(2.0, f"[frontier_ranker] GetPlan failed: {ex}")
            # Back off further to avoid hammering move_base when it's active
            self._make_plan_backoff_until = rospy.Time.now() + rospy.Duration(2.0)
            return None


    def _find_frontier_candidates_cv(self, grid: np.ndarray, info_msg: OccupancyGrid, pose: Pose):
        return fc_find_frontiers(
            grid,
            info_msg,
            pose,
            cluster_min_size=self.cluster_min_size,
            dilate_kernel=self.dilate_kernel,
            frontier_connectivity=self.frontier_connectivity,
            clearance_window_m=self.clearance_window_m,
            dt_mask_size=self.dt_mask_size,
            goal_clearance_m=self.goal_clearance_m,
            search_radius_m=self.search_radius_m,
        )


    @staticmethod
    def _path_length(path: Path) -> float:
        return fc_path_length(path)

    def _visited_penalty_along(self, path: Path) -> Tuple[float, int]:
        return fc_visited_penalty(
            path,
            self.visited_grid,
            self.visited_info,
            path_sample_step=self.path_sample_step,
            visited_threshold=self.visited_threshold,
        )

    def _visited_fraction_around(self, x: float, y: float) -> float:
        return fc_visited_fraction(
            x,
            y,
            self.visited_grid,
            self.visited_info,
            goal_revisit_radius_m=self.goal_revisit_radius_m,
            visited_threshold=self.visited_threshold,
        )

    def _fallback_farthest_goal(self, grid: np.ndarray, info_meta, pose: Pose):
        """Placeholder fallback: no-op. Returns None to skip fallback behavior.
        Implement selection of a farthest feasible goal if desired.
        """
        return None

    def _on_timer(self, _evt):
        t0 = time.time()
        with self.map_lock:
            grid = None if self.map is None else self.map.copy()
            info = self.map_info
        if grid is None or info is None:
            rospy.loginfo_throttle(5.0, "[frontier_ranker] Waiting for map...")
            return

        pose = self._get_robot_pose()
        if pose is None:
            rospy.loginfo_throttle(5.0, "[frontier_ranker] Waiting for TF pose %s->%s", self.global_frame, self.robot_base_frame)
            return
        t_pose = time.time()
        # Do not broadcast a new goal until current goal is reached
        if self._last_goal is not None:
            dgrx = self._last_goal[0] - pose.position.x
            dgry = self._last_goal[1] - pose.position.y
            if math.hypot(dgrx, dgry) > self.goal_reached_radius:
                # If we've not moved since last publish for a while, retarget using
                # the candidate list computed for that configuration.
                now_rt = rospy.Time.now()
                try_retarget = False
                if self._last_goal_publish_time and self._last_goal_publish_time != rospy.Time(0):
                    elapsed = (now_rt - self._last_goal_publish_time).to_sec()
                    if elapsed >= self.stuck_retarget_after_s:
                        moved = math.hypot(pose.position.x - self._publish_pose_at_goal[0],
                                           pose.position.y - self._publish_pose_at_goal[1])
                        if moved <= self.stuck_no_move_dist_m:
                            try_retarget = True
                if try_retarget:
                    # Check same configuration (map stamp and pose closeness)
                    same_map = False
                    try:
                        same_map = (self._last_config_map_stamp == info.header.stamp)
                    except Exception:
                        same_map = False
                    same_pose = (self._last_config_pose is not None and
                                 math.hypot(pose.position.x - self._last_config_pose[0],
                                            pose.position.y - self._last_config_pose[1]) <= self.config_pose_tolerance_m)
                    if same_map and same_pose and self._last_config_candidates:
                        # Pick next unused candidate by score
                        for i, (s, gx, gy) in enumerate(self._last_config_candidates):
                            if i in self._config_published_indices:
                                continue
                            if self._last_goal and abs(gx - self._last_goal[0]) <= 1e-6 and abs(gy - self._last_goal[1]) <= 1e-6:
                                continue
                            # Publish alternative goal from same configuration
                            msg = PoseStamped()
                            msg.header.stamp = now_rt
                            msg.header.frame_id = self.global_frame
                            msg.pose.position.x = gx
                            msg.pose.position.y = gy
                            msg.pose.orientation.w = 1.0
                            self.goal_pub.publish(msg)
                            self._last_goal = (gx, gy)
                            self._last_goal_publish_time = now_rt
                            self._last_goal_score = s
                            self._publish_pose_at_goal = (pose.position.x, pose.position.y)
                            # History + marker
                            self._goal_history.append((now_rt, gx, gy))
                            cutoff = now_rt - rospy.Duration(self.goal_tabu_duration_s)
                            self._goal_history = [(t, x, y) for (t, x, y) in self._goal_history if t >= cutoff]
                            pt = PointStamped()
                            pt.header.stamp = now_rt
                            pt.header.frame_id = self.global_frame
                            pt.point.x, pt.point.y, pt.point.z = gx, gy, 0.0
                            self.goal_point_pub.publish(pt)
                            mk = self._make_marker(pt.header, gx, gy, mid=2, rgb=(0.2, 0.6, 1.0))
                            self.goal_marker_pub.publish(mk)
                            self._config_published_indices.add(i)
                            rospy.logwarn("[frontier_ranker] Retargeted due to no movement for %.1fs to alternative goal (%.2f, %.2f)",
                                          elapsed, gx, gy)
                            return
                return
            else:
                rospy.loginfo("[frontier_ranker] Goal reached within %.2fm", self.goal_reached_radius)
                self._last_goal = None

        cands, roi_stats = self._find_frontier_candidates_cv(grid, info, pose)
        t_frontiers = time.time()
        clusters = cands
        t_clusters = t_frontiers
        # Exclude candidates near past waypoints
        if self._waypoint_history:
            filt=[]
            for (gx,gy,gsz) in clusters:
                ok=True
                for (hx,hy) in self._waypoint_history:
                    if math.hypot(gx-hx, gy-hy) <= self.waypoint_exclusion_radius_m:
                        ok=False
                        break
                if ok:
                    filt.append((gx,gy,gsz))
            clusters=filt
        if not clusters:
            self._no_candidate_count += 1
            rospy.loginfo_throttle(2.0, "[frontier_ranker] No candidates (roi=%dx%d frontier_px=%d) miss=%d/%d",
                                   roi_stats.get("roi_w",0), roi_stats.get("roi_h",0), roi_stats.get("frontier_px",0),
                                   self._no_candidate_count, self.no_candidate_retry_count)
            if self._no_candidate_count >= self.no_candidate_retry_count:
                fb = self._fallback_farthest_goal(grid, info.info, pose)
                if fb is not None:
                    best_goal = fb
                    best_score = -1e9
                    goto_publish = True
                    # jump to publish section
                    msg = PoseStamped()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = self.global_frame
                    msg.pose.position.x = best_goal[0]
                    msg.pose.position.y = best_goal[1]
                    msg.pose.orientation.w = 1.0
                    self.goal_pub.publish(msg)
                    self._last_goal = best_goal
                    self._waypoint_history.append((best_goal[0], best_goal[1]))
                    pt = PointStamped()
                    pt.header.stamp = rospy.Time.now()
                    pt.header.frame_id = self.global_frame
                    pt.point.x, pt.point.y, pt.point.z = best_goal[0], best_goal[1], 0.0
                    self.goal_point_pub.publish(pt)
                    mk = self._make_marker(pt.header, best_goal[0], best_goal[1], mid=1, rgb=(0.8, 0.2, 1.0))
                    self.goal_marker_pub.publish(mk)
                    return
            return


        best_score = -1e18
        best_goal = None
        best_len = 0.0
        best_pen = 0.0

        plans_attempted = 0
        plans_reached = 0
        t_plan_total = 0.0
        t_pen_total = 0.0

        # Exclude candidates near past waypoints
        if self._waypoint_history:
            pruned = []
            for (gx, gy, gsz) in clusters:
                too_close = False
                for (hx, hy) in self._waypoint_history:
                    if math.hypot(gx - hx, gy - hy) <= self.waypoint_exclusion_radius_m:
                        too_close = True
                        break
                if not too_close:
                    pruned.append((gx, gy, gsz))
            clusters = pruned
            if not clusters:
                rospy.loginfo_throttle(2.0, "[frontier_ranker] All candidates excluded by waypoint history (radius=%.2fm)",
                                       self.waypoint_exclusion_radius_m)
                return

        ranked = []       # (score, x, y) for candidates with valid path
        ranked_any = []   # (approx_score, x, y, size) for all candidates (pre-plan)
        t_eval_start = time.time()
        for idx, cand in enumerate(clusters):
            if plans_attempted >= max(1, getattr(self, "max_plans_per_cycle", 50)):
                break
            cx, cy, csize = cand
            # Approx score (no path): favor large frontier and proximity
            approx_L = math.hypot(cx - pose.position.x, cy - pose.position.y)
            approx_score = self.alpha * float(csize) - self.beta * approx_L
            ranked_any.append((approx_score, cx, cy, csize))

            # Stop early if we have exceeded think budget and already have a best path
            if (time.time() - t_eval_start) >= self.max_think_time_s and best_goal is not None:
                break

            # Nudge goal towards robot to land in free and clear space
            if getattr(self, "goal_offset_m", 0.0) > 1e-6:
                dx = pose.position.x - cx
                dy = pose.position.y - cy
                n = (dx*dx + dy*dy) ** 0.5
                if n > 1e-6:
                    nx = cx + (dx / n) * self.goal_offset_m
                    ny = cy + (dy / n) * self.goal_offset_m
                    if self._is_clear_world(nx, ny, grid, info.info, self.goal_clearance_m):
                        cx, cy = nx, ny
            # Ensure goal has required clearance; try ring sampling if not
            if not self._is_clear_world(cx, cy, grid, info.info, self.goal_clearance_m):
                import math as _m
                samples = max(4, getattr(self, "fallback_ring_samples", 16))
                r = max(0.1, getattr(self, "fallback_ring_radius_m", 0.5))
                for k in range(samples):
                    ang = 2.0 * _m.pi * float(k) / float(samples)
                    tx = cx + r * _m.cos(ang)
                    ty = cy + r * _m.sin(ang)
                    if self._is_clear_world(tx, ty, grid, info.info, self.goal_clearance_m):
                        cx, cy = tx, ty
                        break
            # If still not clear enough, skip this candidate
            if not self._is_clear_world(cx, cy, grid, info.info, self.goal_clearance_m):
                if getattr(self, "debug_per_cluster", False) and idx < getattr(self, "max_debug_clusters", 5):
                    rospy.loginfo("[frontier_ranker] cand#%d rejected: insufficient clearance (>=%.2fm)",
                                  idx, getattr(self, "goal_clearance_m", 0.0))
                continue

            t_before_plan = time.time()
            path = self._plan(pose, (cx, cy))
            dt_plan = time.time() - t_before_plan
            t_plan_total += dt_plan
            plans_attempted += 1
            if path is None:
                if getattr(self, "debug_per_cluster", False) and idx < getattr(self, "max_debug_clusters", 5):
                    rospy.loginfo("[frontier_ranker] cand#%d size=%d centroid=(%.2f,%.2f) plan: NONE (%.3fs)",
                                  idx, csize, cx, cy, dt_plan)
                continue
            plans_reached += 1

            L = self._path_length(path)
            t_before_pen = time.time()
            visited_pen, samples = self._visited_penalty_along(path)
            dt_pen = time.time() - t_before_pen
            t_pen_total += dt_pen
            # Extra penalty if the goal cell or its surrounding area is already visited
            extra_goal_pen = 0.0
            if self.visited_grid is not None and self.visited_info is not None:
                vinfo = self.visited_info.info
                mxg = int((cx - vinfo.origin.position.x) / max(vinfo.resolution, 1e-6))
                myg = int((cy - vinfo.origin.position.y) / max(vinfo.resolution, 1e-6))
                if 0 <= mxg < vinfo.width and 0 <= myg < vinfo.height:
                    vcell = int(self.visited_grid[myg, mxg])
                    if vcell >= self.visited_threshold:
                        extra_goal_pen = self.goal_revisit_penalty
                # Area fraction penalty
                area_frac = self._visited_fraction_around(cx, cy)
                if area_frac > 0.0 and self.goal_revisit_area_weight > 0.0:
                    extra_goal_pen += (self.goal_revisit_area_weight * area_frac)
            score = self.alpha * float(csize) - self.beta * L - self.gamma * visited_pen - extra_goal_pen

            if getattr(self, "debug_per_cluster", False) and idx < getattr(self, "max_debug_clusters", 5):
                rospy.loginfo("[frontier_ranker] cand#%d size=%d centroid=(%.2f,%.2f) plan: OK len=%.2f pen=%.1f(samples=%d) score=%.2f times: plan=%.3f pen=%.3f",
                              idx, csize, cx, cy, L, visited_pen, samples, score, dt_plan, dt_pen)

            ranked.append((score, cx, cy))
            if score > best_score:
                best_score = score
                best_goal = (cx, cy)
                best_len = L
                best_pen = visited_pen

            # Enforce time budget after evaluating this candidate
            if (time.time() - t_eval_start) >= self.max_think_time_s:
                break


        # If no path found within time budget, optionally fall back to best frontier by heuristic
        if best_goal is None:
            if self.allow_heuristic_fallback and ranked_any:
                ranked_any.sort(key=lambda t: t[0], reverse=True)
                approx_score, ax, ay, asz = ranked_any[0]
                best_goal = (ax, ay)
                best_score = approx_score
                best_len = math.hypot(ax - pose.position.x, ay - pose.position.y)
                best_pen = float('nan')
                rospy.logwarn_throttle(2.0, "[frontier_ranker] Publishing heuristic best (no plan in %.1fs)",
                                       max(0.0, time.time() - t_eval_start))
            else:
                rospy.loginfo_throttle(2.0, "[frontier_ranker] No candidates available to publish")
                return

        # Retarget hysteresis: hold current goal unless better enough and hold time passed
        now = rospy.Time.now()
        if self._last_goal is not None and self._last_goal_score is not None:
            if (now - self._last_goal_publish_time).to_sec() < self.retarget_hold_time_s:
                return
            if (best_score - self._last_goal_score) < self.retarget_min_score_gain:
                return

        # Store configuration candidates ranked by score (desc) for potential stuck retargeting
        ranked.sort(key=lambda t: t[0], reverse=True)
        if ranked:
            self._last_config_candidates = ranked
        else:
            # If no valid paths yet, optionally fall back to heuristic-ranked candidates
            if self.allow_heuristic_fallback and ranked_any:
                ranked_any.sort(key=lambda t: t[0], reverse=True)
                self._last_config_candidates = [(s, x, y) for (s, x, y, _sz) in ranked_any]
            else:
                self._last_config_candidates = []
        self._last_config_pose = (pose.position.x, pose.position.y)
        try:
            self._last_config_map_stamp = info.header.stamp
        except Exception:
            self._last_config_map_stamp = None
        self._config_published_indices = set()
        # Determine index of best in ranked list
        best_idx = None
        for i, (s, gx, gy) in enumerate(ranked):
            if abs(gx - best_goal[0]) <= 1e-6 and abs(gy - best_goal[1]) <= 1e-6:
                best_idx = i
                break

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.global_frame
        msg.pose.position.x = best_goal[0]
        msg.pose.position.y = best_goal[1]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self._last_goal = best_goal
        self._last_goal_publish_time = now
        self._last_goal_score = best_score
        self._publish_pose_at_goal = (pose.position.x, pose.position.y)
        if best_idx is not None:
            self._config_published_indices.add(best_idx)
        # Update tabu history (prune old)
        self._goal_history.append((now, best_goal[0], best_goal[1]))
        cutoff = now - rospy.Duration(self.goal_tabu_duration_s)
        self._goal_history = [(t,x,y) for (t,x,y) in self._goal_history if t >= cutoff]
        pt = PointStamped()
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = self.global_frame
        pt.point.x, pt.point.y, pt.point.z = best_goal[0], best_goal[1], 0.0
        self.goal_point_pub.publish(pt)

        mk = self._make_marker(pt.header, best_goal[0], best_goal[1], mid=1, rgb=(1.0, 0.2, 0.2))
        self.goal_marker_pub.publish(mk)
        t_end = time.time()
        # Detailed timing/metrics for this cycle
        try:
            w = info.info.width
            h = info.info.height
            res = info.info.resolution
        except Exception:
            w = h = -1
            res = float('nan')
        total = (t_end - t0)
        msg_tmpl = ("[frontier_ranker] cycle: map=%dx%d@%.3fm roi=%dx%d frontier_px=%d cands=%d plans=%d/%d "
                    "times(s): tf=%.3f cv=%.3f plan=%.3f pen=%.3f total=%.3f "
                    "best(L=%.2f pen=%.1f score=%.2f) goal=(%.2f,%.2f)")
        args = (w, h, res,
                roi_stats.get("roi_w",0), roi_stats.get("roi_h",0), roi_stats.get("frontier_px",0), len(clusters),
                plans_reached, plans_attempted,
                (t_pose - t0), (t_frontiers - t_pose), t_plan_total, t_pen_total, total,
                best_len, best_pen, best_score, best_goal[0], best_goal[1])
        if total > getattr(self, "warn_if_over_s", 0.5):
            rospy.logwarn(msg_tmpl, *args)
        else:
            rospy.loginfo(msg_tmpl, *args)

    def _maybe_publish_done(self, grid: np.ndarray, info_meta):
        """Publish done=True if unknown ratio below threshold for hold duration."""
        thresh = float(rospy.get_param("~done_unknown_ratio", 0.01))
        hold_s = float(rospy.get_param("~done_hold_s", 10.0))
        total = float(info_meta.width * info_meta.height)
        if total <= 0:
            return
        unknown = float((grid == -1).sum())
        ratio = unknown / total
        now = rospy.Time.now()
        if ratio <= thresh:
            if not hasattr(self, "_done_since") or self._done_since is None:
                self._done_since = now
            if (now - self._done_since).to_sec() >= hold_s:
                self.done_pub.publish(Bool(data=True))
        else:
            self._done_since = None

def main():
    rospy.init_node("frontier_ranker_node")
    FrontierRankerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
