# Gazebo local models

Put competition-provided Gazebo model folders here when a world file uses `model://...`.

This package's Gazebo launch file adds this directory to `GAZEBO_MODEL_PATH` before starting
`gzserver`, so Gazebo can resolve local models without downloading them at runtime.
