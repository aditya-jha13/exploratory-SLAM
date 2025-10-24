#!/usr/bin/env python3
import math
from typing import Tuple, Optional

import numpy as np
import cv2
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path, OccupancyGrid


def world_to_map(x: float, y: float, info) -> Tuple[int, int]:
    res = max(getattr(info, 'resolution', 0.0), 1e-6)
    mx = int((x - info.origin.position.x) / res)
    my = int((y - info.origin.position.y) / res)
    return mx, my


def map_to_world(mx: int, my: int, info) -> Tuple[float, float]:
    res = max(getattr(info, 'resolution', 0.0), 1e-6)
    x = info.origin.position.x + (mx + 0.5) * res
    y = info.origin.position.y + (my + 0.5) * res
    return x, y


def find_frontier_candidates_cv(
    grid: np.ndarray,
    info_msg: OccupancyGrid,
    pose: Pose,
    *,
    cluster_min_size: int,
    dilate_kernel: int,
    frontier_connectivity: int,
    clearance_window_m: float,
    dt_mask_size: int,
    goal_clearance_m: float,
    search_radius_m: float,
):
    info = info_msg.info
    res = max(info.resolution, 1e-6)
    mx = int((pose.position.x - info.origin.position.x) / res)
    my = int((pose.position.y - info.origin.position.y) / res)
    rad_cells = max(1, int(search_radius_m / res))
    w, h = info.width, info.height
    x0 = max(1, mx - rad_cells)
    y0 = max(1, my - rad_cells)
    x1 = min(w - 2, mx + rad_cells)
    y1 = min(h - 2, my + rad_cells)
    if x1 <= x0 or y1 <= y0:
        return [], {"roi_w": 0, "roi_h": 0, "frontier_px": 0, "kept": 0}

    roi = grid[y0:y1 + 1, x0:x1 + 1]
    free = (roi == 0)
    k = max(1, int(dilate_kernel))
    if k % 2 == 0:
        k += 1
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    unknown = (roi == -1)
    dil_unknown = cv2.dilate((unknown.astype(np.uint8) * 255), kernel, iterations=1) > 0
    frontier = np.logical_and(free, dil_unknown).astype(np.uint8)

    # Distance transform on free mask to favor clear goals
    free_u8 = (free.astype(np.uint8) * 255)
    dt = cv2.distanceTransform(free_u8, cv2.DIST_L2, 5 if dt_mask_size == 5 else 3)
    dt_m = dt * res

    connectivity = 4 if frontier_connectivity == 4 else 8
    num, labels, stats, cents = cv2.connectedComponentsWithStats(frontier, connectivity=connectivity)
    candidates = []
    kept = 0
    for label in range(1, num):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < cluster_min_size:
            continue
        mask_label = (labels == label)
        cx_px, cy_px = cents[label]
        r_pix = max(1, int(clearance_window_m / res))
        cx_i, cy_i = int(round(cx_px)), int(round(cy_px))
        xw0 = max(0, cx_i - r_pix)
        xw1 = min(mask_label.shape[1] - 1, cx_i + r_pix)
        yw0 = max(0, cy_i - r_pix)
        yw1 = min(mask_label.shape[0] - 1, cy_i + r_pix)
        local_mask = mask_label[yw0:yw1 + 1, xw0:xw1 + 1]
        local_dt = dt_m[yw0:yw1 + 1, xw0:xw1 + 1]
        cand_dt = np.where(local_mask, local_dt, 0.0)
        iy, ix = np.unravel_index(np.argmax(cand_dt), cand_dt.shape)
        best_clear = float(cand_dt[iy, ix])
        if best_clear <= 0.0:
            mx_c = int(x0 + cx_px)
            my_c = int(y0 + cy_px)
        else:
            mx_c = x0 + xw0 + ix
            my_c = y0 + yw0 + iy
        wx, wy = map_to_world(int(mx_c), int(my_c), info)
        if best_clear < goal_clearance_m:
            candidates.append((wx, wy, max(1, int(0.5 * area))))
        else:
            candidates.append((wx, wy, area))
        kept += 1
    return candidates, {"roi_w": (x1 - x0 + 1), "roi_h": (y1 - y0 + 1), "frontier_px": int(frontier.sum()), "kept": kept}


def path_length(path: Path) -> float:
    pts = path.poses
    if len(pts) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(pts)):
        dx = pts[i].pose.position.x - pts[i - 1].pose.position.x
        dy = pts[i].pose.position.y - pts[i - 1].pose.position.y
        total += math.hypot(dx, dy)
    return total


def visited_penalty_along(
    path: Path,
    visited_grid: Optional[np.ndarray],
    visited_info: Optional[OccupancyGrid],
    *,
    path_sample_step: float,
    visited_threshold: int,
) -> Tuple[float, int]:
    if visited_grid is None or visited_info is None:
        return 0.0, 0
    info = visited_info.info
    res = max(info.resolution, 1e-6)
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
        steps = max(1, int(seg_len / max(path_sample_step, res)))
        for sidx in range(steps + 1):
            t = float(sidx) / float(max(steps, 1))
            xs = x0 + t * (x1 - x0)
            ys = y0 + t * (y1 - y0)
            mx = int((xs - info.origin.position.x) / res)
            my = int((ys - info.origin.position.y) / res)
            if 0 <= mx < info.width and 0 <= my < info.height:
                v = int(visited_grid[my, mx])
                if v >= visited_threshold:
                    penalty += 1.0
            samples += 1
    return penalty, samples


def visited_fraction_around(
    x: float,
    y: float,
    visited_grid: Optional[np.ndarray],
    visited_info: Optional[OccupancyGrid],
    *,
    goal_revisit_radius_m: float,
    visited_threshold: int,
) -> float:
    if visited_grid is None or visited_info is None:
        return 0.0
    if goal_revisit_radius_m <= 1e-6:
        return 0.0
    info = visited_info.info
    res = max(info.resolution, 1e-6)
    r_cells = max(1, int(goal_revisit_radius_m / res))
    mx_center = int((x - info.origin.position.x) / res)
    my_center = int((y - info.origin.position.y) / res)
    total = 0
    visited = 0
    r2 = r_cells * r_cells
    for dy in range(-r_cells, r_cells + 1):
        for dx in range(-r_cells, r_cells + 1):
            if dx * dx + dy * dy > r2:
                continue
            mx = mx_center + dx
            my = my_center + dy
            if 0 <= mx < info.width and 0 <= my < info.height:
                total += 1
                if int(visited_grid[my, mx]) >= visited_threshold:
                    visited += 1
    if total == 0:
        return 0.0
    return float(visited) / float(total)

