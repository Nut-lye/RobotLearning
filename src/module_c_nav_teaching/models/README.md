# Local Gazebo Models

Put competition-provided Gazebo model folders here when a world file uses `model://...`.

`01_gazebo_world.launch.py` adds this folder to `GAZEBO_MODEL_PATH` before starting Gazebo.
That keeps Gazebo from blocking on the online model database during class or competition.
