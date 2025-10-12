# frontier_ranker_ros

Clusters frontier cells and selects exploration goals using a scoring function that accounts for cluster size, distance, and overlap with visited areas.

Initial plan:
- Subscribe to occupancy grid `/map` and detect frontier cells (known–unknown boundaries)
- Cluster frontiers (e.g., DBSCAN via OpenCV / custom)
- Rank clusters with score = alpha*size - beta*distance - gamma*visited_overlap
- Publish the chosen goal as a `geometry_msgs/PoseStamped` or `move_base_simple/goal`

This package is a placeholder; implementation to follow.
