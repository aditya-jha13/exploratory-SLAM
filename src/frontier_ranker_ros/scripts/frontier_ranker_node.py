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
        self.make_plan_service = rospy.get_param("~make_plan_service", "/move_base/make_plan")
        # Debug/Perf params
        self.debug_per_cluster = bool(rospy.get_param("~debug_per_cluster", False))
        self.max_debug_clusters = int(rospy.get_param("~max_debug_clusters", 5))
        self.max_plans_per_cycle = int(rospy.get_param("~max_plans_per_cycle", 50))
        self.warn_if_over_s = float(rospy.get_param("~warn_if_over_s", 0.5))
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
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.update_rate, 1e-3)), self._on_timer)

        # Progress tracking
        self._progress_pose = None
        self._progress_time = rospy.Time.now()

        # Reached cooldown
        self._reached_cooldown_until = rospy.Time(0)

        # Waypoint history
        self._waypoint_history = []  # list of (x, y)
        self._no_candidate_count = 0

        # Visualization publishers
        self.goal_point_pub = rospy.Publisher("~goal_point", PointStamped, queue_size=1, latch=True)
        self.goal_marker_pub = rospy.Publisher("~goal_marker", Marker, queue_size=1, latch=True)

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

    def _map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        info = self.map_info.info
        x = info.origin.position.x + (mx + 0.5) * info.resolution
        y = info.origin.position.y + (my + 0.5) * info.resolution
        return x, y

        # Old grid-scan frontier helpers removed (replaced by OpenCV pipeline)

    def _plan(self, start: Pose, goal_xy: Tuple[float, float]) -> Optional[Path]:
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
            resp = self.make_plan(req)
            if resp.plan and len(resp.plan.poses) > 0:
                return resp.plan
            return None
        except rospy.ServiceException as ex:
            rospy.logwarn_throttle(2.0, f"[frontier_ranker] GetPlan failed: {ex}")
            return None


    def _find_frontier_candidates_cv(self, grid: np.ndarray, info_msg: OccupancyGrid, pose: Pose):
        info = info_msg.info
        res = info.resolution
        mx = int((pose.position.x - info.origin.position.x) / res)
        my = int((pose.position.y - info.origin.position.y) / res)
        rad_cells = max(1, int(self.search_radius_m / max(res, 1e-6)))
        w, h = info.width, info.height
        x0 = max(1, mx - rad_cells)
        y0 = max(1, my - rad_cells)
        x1 = min(w - 2, mx + rad_cells)
        y1 = min(h - 2, my + rad_cells)
        if x1 <= x0 or y1 <= y0:
            return [], {"roi_w": 0, "roi_h": 0, "frontier_px": 0, "kept": 0}
        roi = grid[y0:y1+1, x0:x1+1]
        free = (roi == 0)
        k = max(1, int(self.dilate_kernel))
        if k % 2 == 0:
            k += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        unknown = (roi == -1)
        dil_unknown = cv2.dilate((unknown.astype(np.uint8) * 255), kernel, iterations=1) > 0
        frontier = np.logical_and(free, dil_unknown).astype(np.uint8)
        # Distance transform on free mask to favor clear goals
        free_u8 = (free.astype(np.uint8) * 255)
        dt = cv2.distanceTransform(free_u8, cv2.DIST_L2, 5 if self.dt_mask_size == 5 else 3)
        dt_m = dt * res
        connectivity = 4 if self.frontier_connectivity == 4 else 8
        num, labels, stats, cents = cv2.connectedComponentsWithStats(frontier, connectivity=connectivity)
        candidates = []
        kept = 0
        for label in range(1, num):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.cluster_min_size:
                continue
            mask_label = (labels == label)
            cx_px, cy_px = cents[label]
            r_pix = max(1, int(self.clearance_window_m / max(res, 1e-6)))
            cx_i, cy_i = int(round(cx_px)), int(round(cy_px))
            xw0 = max(0, cx_i - r_pix)
            xw1 = min(mask_label.shape[1]-1, cx_i + r_pix)
            yw0 = max(0, cy_i - r_pix)
            yw1 = min(mask_label.shape[0]-1, cy_i + r_pix)
            local_mask = mask_label[yw0:yw1+1, xw0:xw1+1]
            local_dt = dt_m[yw0:yw1+1, xw0:xw1+1]
            cand_dt = np.where(local_mask, local_dt, 0.0)
            iy, ix = np.unravel_index(np.argmax(cand_dt), cand_dt.shape)
            best_clear = cand_dt[iy, ix]
            if best_clear <= 0.0:
                mx_c = int(x0 + cx_px)
                my_c = int(y0 + cy_px)
            else:
                mx_c = x0 + xw0 + ix
                my_c = y0 + yw0 + iy
            wx, wy = self._map_to_world(int(mx_c), int(my_c))
            if best_clear < self.goal_clearance_m:
                candidates.append((wx, wy, max(1, int(0.5 * area))))
            else:
                candidates.append((wx, wy, area))
            kept += 1
        return candidates, {"roi_w": (x1 - x0 + 1), "roi_h": (y1 - y0 + 1), "frontier_px": int(frontier.sum()), "kept": kept}


    @staticmethod
    def _path_length(path: Path) -> float:
        pts = path.poses
        if len(pts) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(pts)):
            dx = pts[i].pose.position.x - pts[i - 1].pose.position.x
            dy = pts[i].pose.position.y - pts[i - 1].pose.position.y
            total += math.hypot(dx, dy)
        return total

    def _visited_penalty_along(self, path: Path) -> Tuple[float, int]:
        if self.visited_grid is None or self.visited_info is None:
            return 0.0, 0
        info = self.visited_info.info
        res = info.resolution
        penalty = 0.0
        samples = 0
        for i in range(1, len(path.poses)):
            x0 = path.poses[i - 1].pose.position.x
            y0 = path.poses[i - 1].pose.position.y
            x1 = path.poses[i].pose.position.x
            y1 = path.poses[i].pose.position.y
            seg_len = math.hypot(x1 - x0, y1 - y0)
            if seg_len < 1e-6:
                continue
            steps = max(1, int(seg_len / max(self.path_sample_step, res)))
            for sidx in range(steps + 1):
                t = float(sidx) / float(max(steps, 1))
                xs = x0 + t * (x1 - x0)
                ys = y0 + t * (y1 - y0)
                mx = int((xs - info.origin.position.x) / res)
                my = int((ys - info.origin.position.y) / res)
                if 0 <= mx < info.width and 0 <= my < info.height:
                    v = self.visited_grid[my, mx]
                    if v >= self.visited_threshold:
                        penalty += 1.0
                samples += 1
        return penalty, samples

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
                    mk = Marker()
                    mk.header = pt.header
                    mk.ns = "frontier_ranker"
                    mk.id = 1
                    mk.type = Marker.SPHERE
                    mk.action = Marker.ADD
                    mk.pose.position.x = best_goal[0]
                    mk.pose.position.y = best_goal[1]
                    mk.pose.position.z = 0.05
                    mk.pose.orientation.w = 1.0
                    mk.scale.x = 0.2
                    mk.scale.y = 0.2
                    mk.scale.z = 0.2
                    mk.color.r = 0.8
                    mk.color.g = 0.2
                    mk.color.b = 1.0
                    mk.color.a = 0.9
                    mk.lifetime = rospy.Duration(2.0)
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

        for idx, cand in enumerate(clusters):
            if plans_attempted >= max(1, getattr(self, "max_plans_per_cycle", 50)):
                break
            cx, cy, csize = cand

            # Nudge goal towards robot to land in free space
            if getattr(self, "goal_offset_m", 0.0) > 1e-6:
                dx = pose.position.x - cx
                dy = pose.position.y - cy
                n = (dx*dx + dy*dy) ** 0.5
                if n > 1e-6:
                    nx = cx + (dx / n) * self.goal_offset_m
                    ny = cy + (dy / n) * self.goal_offset_m
                    if self._is_free_world(nx, ny, grid, info.info):
                        cx, cy = nx, ny
            # Fallback ring sampling if still not free
            if not self._is_free_world(cx, cy, grid, info.info):
                import math as _m
                samples = max(4, getattr(self, "fallback_ring_samples", 16))
                r = max(0.1, getattr(self, "fallback_ring_radius_m", 0.5))
                for k in range(samples):
                    ang = 2.0 * _m.pi * float(k) / float(samples)
                    tx = cx + r * _m.cos(ang)
                    ty = cy + r * _m.sin(ang)
                    if self._is_free_world(tx, ty, grid, info.info):
                        cx, cy = tx, ty
                        break

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
            score = self.alpha * float(csize) - self.beta * L - self.gamma * visited_pen

            if getattr(self, "debug_per_cluster", False) and idx < getattr(self, "max_debug_clusters", 5):
                rospy.loginfo("[frontier_ranker] cand#%d size=%d centroid=(%.2f,%.2f) plan: OK len=%.2f pen=%.1f(samples=%d) score=%.2f times: plan=%.3f pen=%.3f",
                              idx, csize, cx, cy, L, visited_pen, samples, score, dt_plan, dt_pen)

            if score > best_score:
                best_score = score
                best_goal = (cx, cy)
                best_len = L
                best_pen = visited_pen


        # If no candidate yielded a valid plan, skip publishing this cycle
        if best_goal is None:
            rospy.loginfo_throttle(2.0, "[frontier_ranker] No reachable candidates after planning (attempted=%d)", plans_attempted)
            return

        # Retarget hysteresis: hold current goal unless better enough and hold time passed
        if self._last_goal is not None and self._last_goal_score is not None:
            if (now - self._last_goal_publish_time).to_sec() < self.retarget_hold_time_s:
                return
            if (best_score - self._last_goal_score) < self.retarget_min_score_gain:
                return

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
        # Update tabu history (prune old)
        self._goal_history.append((now, best_goal[0], best_goal[1]))
        cutoff = now - rospy.Duration(self.goal_tabu_duration_s)
        self._goal_history = [(t,x,y) for (t,x,y) in self._goal_history if t >= cutoff]
        pt = PointStamped()
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = self.global_frame
        pt.point.x, pt.point.y, pt.point.z = best_goal[0], best_goal[1], 0.0
        self.goal_point_pub.publish(pt)

        mk = Marker()
        mk.header = pt.header
        mk.ns = "frontier_ranker"
        mk.id = 1
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.position.x = best_goal[0]
        mk.pose.position.y = best_goal[1]
        mk.pose.position.z = 0.05
        mk.pose.orientation.w = 1.0
        mk.scale.x = 0.2
        mk.scale.y = 0.2
        mk.scale.z = 0.2
        mk.color.r = 1.0
        mk.color.g = 0.2
        mk.color.b = 0.2
        mk.color.a = 0.9
        mk.lifetime = rospy.Duration(2.0)
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
