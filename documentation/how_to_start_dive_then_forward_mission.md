# How to Start Dive Then Forward Mission

This document records the startup commands for
`xy_control/dive_then_forward_mission_node.py`.

## Simulation Workflow

### 1. Build SAUVC-RPI

Run in `SAUVC-RPI`:

```bash
make init
```

If you already built the workspace and only changed the `xy_control` package,
you can rebuild only that package inside the container:

```bash
make compose_shell
cd rpi_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select xy_control
```

### 2. Start SAUVC-Simulation

Run in `SAUVC-Simulation`:

```bash
make -f Makefile_ubuntu launch
```

### 3. Start the SAUVC-RPI simulation control stack

Run in `SAUVC-RPI`:

```bash
make sim_launch
```

This starts the ROS2 control nodes used in simulation, including:

- `supervisor_node`
- `waypoint_target_publisher`
- depth PID
- bottom camera PID
- wrench sum

### 4. Enter the SAUVC-RPI container

Run in `SAUVC-RPI`:

```bash
make compose_shell
```

### 5. Start the mission node

Run inside the container:

```bash
cd rpi_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run xy_control dive_then_forward_mission_node --ros-args -r __ns:=/orca_auv
```

The default mission behavior is:

- reset bottom-camera pose feedback and MoveToPoint setpoint to zero
- enable `depth_hold`
- enable `bottom_camera_hold`
- use the current bottom-camera XY feedback as the origin
- latch the startup bottom-camera yaw feedback and keep that heading while diving
- keep the startup bottom-camera X/Y position while diving
- dive to `0.1 m`
- move to `(startup_x + 1000 px, startup_y)` while holding `0.1 m`
- rotate in place to `startup_yaw + 180 deg`
- move back to `(startup_x, startup_y)` while holding `0.1 m`
- surface to `0.0 m`

### 6. Stop the mission node

When the mission reaches `DONE`, the node stays alive for debugging.
Stop it with:

```bash
Ctrl-C
```

## Short-Distance Test

To test a shorter forward distance, run inside the container:

```bash
cd rpi_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run xy_control dive_then_forward_mission_node --ros-args -r __ns:=/orca_auv -p target_x_px:=200.0
```

You can also adjust motion speed:

```bash
ros2 run xy_control dive_then_forward_mission_node --ros-args -r __ns:=/orca_auv -p target_x_px:=200.0 -p speed_px_s:=80.0
```

To adjust the surfacing target or the turn amount:

```bash
ros2 run xy_control dive_then_forward_mission_node --ros-args -r __ns:=/orca_auv -p surface_depth_m:=0.0 -p turn_yaw_offset_rad:=3.141592653589793
```

## One-Line Alternative

If you do not want to enter `compose_shell`, run this directly in `SAUVC-RPI`:

```bash
docker compose exec orca /bin/bash -lc "cd rpi_ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run xy_control dive_then_forward_mission_node --ros-args -r __ns:=/orca_auv"
```

## Hardware Workflow

If you are running on the real vehicle instead of simulation:

1. Run in `SAUVC-RPI`:

```bash
make launch
```

2. Enter the container:

```bash
make compose_shell
```

3. Start the mission node inside the container:

```bash
cd rpi_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run xy_control dive_then_forward_mission_node --ros-args -r __ns:=/orca_auv
```

## Wi-Fi Safe Detached Hardware Start

When starting the real vehicle over Wi-Fi, do not run the mission in an
interactive SSH shell. Start both the bringup stack and the mission detached on
the Raspberry Pi so the mission process is not tied to the SSH/Wi-Fi session.

Run on the Raspberry Pi:

```bash
cd ~/SAUVC-RPI
make launch_detached
make mission_dive_detached
```

Follow bringup logs:

```bash
make launch_logs
```

Follow mission logs:

```bash
make mission_dive_logs
```

Check whether the detached mission is still running:

```bash
make mission_dive_status
```

Stop the mission and request `SAFE_DISABLED`:

```bash
make mission_dive_stop
```

Extra ROS parameters can be passed with `MISSION_ARGS`:

```bash
make mission_dive_detached MISSION_ARGS="-p target_depth_m:=0.6 -p speed_px_s:=80.0"
```
