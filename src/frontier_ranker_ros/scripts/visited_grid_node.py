#!/usr/bin/env python3
import math
from typing import Optional

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import tf2_ros


class VisitedGridNode:
    def __init__(self):
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.robot_base_frame = rospy.get_param("~robot_base_frame", "base_link")
        self.robot_radius = float(rospy.get_param("~robot_radius", 0.3))
        self.publish_rate = float(rospy.get_param("~publish_rate", 1.0))
        self.visit_value = int(rospy.get_param("~visit_value", 100))

        self.map_info = None
        self.grid = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self._on_map, queue_size=1)
        self.pub = rospy.Publisher("/visited_grid", OccupancyGrid, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 1e-3)), self._on_timer)

    def _on_map(self, msg: OccupancyGrid):
        need_reset = (
            self.map_info is None
            or self.map_info.info.width != msg.info.width
            or self.map_info.info.height != msg.info.height
            or abs(self.map_info.info.resolution - msg.info.resolution) > 1e-9
            or self.map_info.info.origin.position.x != msg.info.origin.position.x
            or self.map_info.info.origin.position.y != msg.info.origin.position.y
        )
        self.map_info = msg
        if need_reset:
            self.grid = np.zeros((msg.info.height, msg.info.width), dtype=np.int16)

    def _get_robot_xy(self) -> Optional[tuple]:
        try:
            tfm = self.tf_buffer.lookup_transform(self.global_frame, self.robot_base_frame, rospy.Time(0), rospy.Duration(0.5))
        except Exception as ex:
            rospy.logwarn_throttle(2.0, f"[visited_grid] TF lookup failed: {ex}")
            return None
        return (tfm.transform.translation.x, tfm.transform.translation.y)

    def _mark_circle(self, cx: int, cy: int, radius_cells: int):
        if self.grid is None:
            return
        h, w = self.grid.shape
        x0, x1 = max(0, cx - radius_cells), min(w - 1, cx + radius_cells)
        y0, y1 = max(0, cy - radius_cells), min(h - 1, cy + radius_cells)
        r2 = radius_cells * radius_cells
        for y in range(y0, y1 + 1):
            dy = y - cy
            for x in range(x0, x1 + 1):
                dx = x - cx
                if dx * dx + dy * dy <= r2:
                    self.grid[y, x] = self.visit_value

    def _on_timer(self, _evt):
        if self.map_info is None or self.grid is None:
            return
        pose = self._get_robot_xy()
        if pose is None:
            return
        res = self.map_info.info.resolution
        mx = int((pose[0] - self.map_info.info.origin.position.x) / res)
        my = int((pose[1] - self.map_info.info.origin.position.y) / res)
        rad_cells = max(1, int(self.robot_radius / res))
        if 0 <= mx < self.map_info.info.width and 0 <= my < self.map_info.info.height:
            self._mark_circle(mx, my, rad_cells)

        out = OccupancyGrid()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.global_frame
        out.info = self.map_info.info
        out.data = self.grid.reshape(-1).tolist()
        self.pub.publish(out)


def main():
    rospy.init_node("visited_grid_node")
    VisitedGridNode()
    rospy.spin()


if __name__ == "__main__":
    main()

