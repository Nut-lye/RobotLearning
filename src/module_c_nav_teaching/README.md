# module_c_nav_teaching

Module C teaching package for Gazebo, RViz, and Nav2.

The package is split by teaching responsibility:

```text
config/      Nav2 and SLAM parameters
docs/        Teaching notes for what each file does
launch/      Step-by-step launch files
maps/        Known-map navigation examples
models/      Place competition-provided Gazebo models here
rviz/        Clean Nav2 RViz view without the TurtleBot3 Docking panel
scenarios/   World-map-parameter correspondence table
worlds/      Gazebo scenes
```

## Install

ROS 2 version:

```bash
source /opt/ros/humble/setup.bash
```

Install required ROS 2 packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-nav2-rviz-plugins \
  ros-humble-slam-toolbox \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-navigation2 \
  ros-humble-tf2-ros \
  ros-humble-xacro \
  ros-humble-rviz2
```

Install build tools:

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip
```

Set TurtleBot3 model:

```bash
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
source ~/.bashrc
```

Install workspace dependencies with rosdep:

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select module_c_nav_teaching module_c_bt_demo
source install/setup.bash
```

## Known-Map Practice

Terminal 1:

```bash
ros2 launch module_c_nav_teaching 01_gazebo_world.launch.py world:=02_open_obstacles.world x_pose:=0.0 y_pose:=0.0
```

Terminal 2:

```bash
ros2 launch module_c_nav_teaching 02_nav2_known_map.launch.py map:=02_open_obstacles.yaml
```

Other known-map pairs:

```bash
world:=01_tb3_world.world      map:=01_tb3_world.yaml
world:=02_open_obstacles.world map:=02_open_obstacles.yaml
world:=03_narrow_door.world    map:=03_narrow_door.yaml
```

Use the tuned Module C parameters by default:

```bash
ros2 launch module_c_nav_teaching 02_nav2_known_map.launch.py map:=03_narrow_door.yaml params_file:=nav2_params_module_c.yaml
```

Use the baseline parameters for comparison:

```bash
ros2 launch module_c_nav_teaching 02_nav2_known_map.launch.py map:=03_narrow_door.yaml params_file:=nav2_params_baseline.yaml
```

## SLAM Practice

Terminal 1:

```bash
ros2 launch module_c_nav_teaching 01_gazebo_world.launch.py world:=04_local_room_slam.world x_pose:=0.0 y_pose:=0.0
```

Terminal 2:

```bash
ros2 launch module_c_nav_teaching 03_slam_mapping.launch.py
```

## Why This Package Exists

`tb3_nav_practice` is a minimal runnable package. This package is a teaching version:
each launch file maps to one practice operation, and each world has a visible place in
`scenarios/module_c_scenarios.yaml`.
