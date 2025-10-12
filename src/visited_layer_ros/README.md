# visited_layer_ros

A costmap_2d layer plugin that marks traversed cells as visited and exposes a costmap layer to penalize revisiting.

Planned features:
- Subscribe to `/tf` or `/odom` for robot pose
- Mark cells in a binary grid as visited as the robot traverses
- Expose a costmap layer with configurable penalty for visited cells
- Optional decay over time and clearing services

This package is a placeholder; implementation to follow.
