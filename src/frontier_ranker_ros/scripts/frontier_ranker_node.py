#!/usr/bin/env python3
import math
import time
import threading
from typing import List, Tuple, Optional

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetPlanRequest
import tf2_ros
import cv2


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
        self.make_plan_service = rospy.get_param("~make_plan_service", "/move_base/make_plan")
        # Debug/Perf params
        self.debug_per_cluster = bool(rospy.get_param("~debug_per_cluster", False))
        self.max_debug_clusters = int(rospy.get_param("~max_debug_clusters", 5))
        self.max_plans_per_cycle = int(rospy.get_param("~max_plans_per_cycle", 50))
        self.warn_if_over_s = float(rospy.get_param("~warn_if_over_s", 0.5))

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

        # Timer
        self._last_goal = None
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.update_rate, 1e-3)), self._on_timer)

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

    def _map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        info = self.map_info.info
        x = info.origin.position.x + (mx + 0.5) * info.resolution
        y = info.origin.position.y + (my + 0.5) * info.resolution
        return x, y

    def _neighbors(self, x: int, y: int):
        if self.frontier_connectivity == 4:
            deltas = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        else:
            deltas = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        for dx, dy in deltas:
            yield x + dx, y + dy

    def _detect_frontiers(self, grid: np.ndarray) -> List[Tuple[int, int]]:
        h, w = grid.shape
        frontiers = []
        for y in range(1, h - 1):
            row = grid[y]
            for x in range(1, w - 1):
                if row[x] != 0:
                    continue
                for nx, ny in self._neighbors(x, y):
                    if 0 <= nx < w and 0 <= ny < h and grid[ny, nx] == -1:
                        frontiers.append((x, y))
                        break
        return frontiers

    def _cluster_frontiers(self, frontiers: List[Tuple[int, int]]) -> List[List[Tuple[int, int]]]:
        if not frontiers:
            return []
        frontier_set = set(frontiers)
        clusters, visited = [], set()
        for cell in frontiers:
            if cell in visited:
                continue
            queue = [cell]
            visited.add(cell)
            cluster = [cell]
            while queue:
                cx, cy = queue.pop(0)
                for nx, ny in self._neighbors(cx, cy):
                    if (nx, ny) in frontier_set and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        queue.append((nx, ny))
                        cluster.append((nx, ny))
            if len(cluster) >= self.cluster_min_size:
                clusters.append(cluster)
        return clusters

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
        unknown = (roi == -1).astype(np.uint8) * 255
        free = (roi == 0)
        k = max(1, int(self.dilate_kernel))
        if k % 2 == 0:
            k += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        dil_unknown = cv2.dilate(unknown, kernel, iterations=1) > 0
        frontier = np.logical_and(free, dil_unknown).astype(np.uint8)
        connectivity = 4 if self.frontier_connectivity == 4 else 8
        num, labels, stats, cents = cv2.connectedComponentsWithStats(frontier, connectivity=connectivity)
        candidates = []
        kept = 0
        for label in range(1, num):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.cluster_min_size:
                continue
            cx_px, cy_px = cents[label]
            mx_c = int(x0 + cx_px)
            my_c = int(y0 + cy_px)
            wx, wy = self._map_to_world(mx_c, my_c)
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
                    if v >= 50:
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

        cands, roi_stats = self._find_frontier_candidates_cv(grid, info, pose)
        t_frontiers = time.time()
        clusters = cands
        t_clusters = t_frontiers
        if not clusters:
            rospy.loginfo_throttle(2.0, "[frontier_ranker] No candidates (roi=%dx%d frontier_px=%d)",
                                   roi_stats.get("roi_w",0), roi_stats.get("roi_h",0), roi_stats.get("frontier_px",0))
            return


        best_score = -1e18
        best_goal = None
        best_len = 0.0
        best_pen = 0.0

        plans_attempted = 0
        plans_reached = 0
        t_plan_total = 0.0
        t_pen_total = 0.0

        for idx, cand in enumerate(clusters):
            if plans_attempted >= max(1, getattr(self, "max_plans_per_cycle", 50)):
                break
            cx, cy, csize = cand

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


        if best_goal is None:
            rospy.loginfo_throttle(2.0, "[frontier_ranker] No reachable clusters (attempted=%d)", plans_attempted)
            return

        if self._last_goal is not None:
            dx = best_goal[0] - self._last_goal[0]
            dy = best_goal[1] - self._last_goal[1]
            if math.hypot(dx, dy) < 0.3:
                return

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.global_frame
        msg.pose.position.x = best_goal[0]
        msg.pose.position.y = best_goal[1]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self._last_goal = best_goal
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



def main():
    rospy.init_node("frontier_ranker_node")
    FrontierRankerNode()
    rospy.spin()


if __name__ == "__main__":
    main()

