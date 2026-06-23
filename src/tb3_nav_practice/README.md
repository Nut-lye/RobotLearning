# tb3_nav_practice

Teaching package for module C robot navigation practice with TurtleBot3, Gazebo, RViz, and Nav2.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tb3_nav_practice
source install/setup.bash
```

Using `--symlink-install` keeps edits in `src/tb3_nav_practice/config/nav2_params.yaml`
visible after restarting the launch file.

## Terminal 1: Gazebo

```bash
ros2 launch tb3_nav_practice turtlebot3_world_simtime0.launch.py
```

This launch file loads the world from `worlds/` and sets Gazebo model paths locally to avoid runtime downloads from the online model database.

## Terminal 2: Nav2 and RViz

```bash
ros2 launch tb3_nav_practice turtlebot3_nav2_local.launch.py
```

This launch file uses the local map and `config/nav2_params.yaml`, so module C parameter screenshots can be taken directly from this workspace.
