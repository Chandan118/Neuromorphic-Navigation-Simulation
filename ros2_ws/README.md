# ROS 2 Integration

This workspace provides a minimal ROS&nbsp;2 Foxy/Humble package named `neuromorphic_navigation_ros`
that exposes the Python simulation through ROS&nbsp;2 publishers. The bridge runs the existing
simulation and publishes agent poses, accumulated paths, and status messages so that they can be
visualised in RViz or consumed by other ROS&nbsp;2 nodes.

## Workspace Layout

```
ros2_ws/
├── src/
│   └── neuromorphic_navigation_ros/    # ROS 2 Python package
│       ├── neuromorphic_navigation_ros/
│       │   ├── __init__.py
│       │   └── bridge_node.py
│       ├── resource/
│       │   └── neuromorphic_navigation_ros
│       ├── launch/
│       │   └── simulation_bridge.launch.py
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
└── README.md                           # This file
```

## Quick Start

1. Source your ROS&nbsp;2 environment (e.g. `source /opt/ros/humble/setup.bash`).
2. Copy this project into the `ros2_ws/src` directory (or keep the repo checked out in place).
3. Build the workspace:
   ```bash
   cd ros2_ws
   colcon build --packages-select neuromorphic_navigation_ros
   source install/setup.bash
   ```
4. Run the bridge node:
   ```bash
   ros2 launch neuromorphic_navigation_ros simulation_bridge.launch.py
   ```

Environment variable `NEUROMORPHIC_SIM_ROOT` can be set to point at the root of the Python simulation
if the ROS&nbsp;2 workspace lives elsewhere. When unset, the bridge defaults to looking two directories
up from the provided config file path.

The node publishes:

- `pose` (`geometry_msgs/PoseStamped`) – latest agent pose.
- `path` (`nav_msgs/Path`) – full trajectory of the current run.
- `status` (`std_msgs/String`) – textual updates about progress and energy usage.

Set the launch parameter `algorithm` to any option in `main.py` (e.g. `NeuromorphicSpiking`, `ArtificialChemotaxis`, `CPGNavigator`, `SwarmConsensus`, or `DeepRL`) to stream a different controller.

See the top-level README for more details on how to integrate this bridge via Docker.
