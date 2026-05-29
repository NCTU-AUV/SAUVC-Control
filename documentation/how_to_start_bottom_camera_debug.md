# How to Start Bottom Camera Debug

This document records the command used to start the hardware ROS stack with
bottom-camera debug image publishing enabled.

## Inside `make compose_shell`

Run inside the container:

```bash
cd rpi_ros2_ws
source /opt/ros/humble/setup.bash
source /root/uros_ws/install/local_setup.bash
source install/setup.bash
ros2 launch src/launch/orca_bringup.launch.py publish_lk_debug_image:=true
```

This enables the tile-line tracking debug image topic:

- `/orca_auv/camera/bottom/debug/tile_lines`

## On the Host Machine

Run on the local host to start `rqt`:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
rqt
```

Then open `rqt_image_view` and select this topic:

- `/orca_auv/camera/bottom/debug/tile_lines`
