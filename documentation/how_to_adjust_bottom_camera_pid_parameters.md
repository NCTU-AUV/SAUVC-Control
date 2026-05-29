# How to Adjust Bottom Camera PID Parameters

This document records the runtime `ros2 param set` commands for the bottom
camera PID controllers.

## Bottom Camera Wrench Limits

The bottom camera PID outputs are combined by
`bottom_camera_pid_bridge_node` before publishing
`/orca_auv/control/wrench_sources/bottom_camera`.

Current output limits:

- `force.x`: `-10.0 N` to `+10.0 N`
- `force.y`: `-10.0 N` to `+10.0 N`
- `torque.z`: `-5.0 Nm` to `+5.0 Nm`

These limits are applied after the X/Y world-frame forces are rotated into the
body frame, so they are final body-frame wrench limits.

Bridge node name:

- `/orca_auv/bottom_camera_pid_bridge_node`

Runtime-tunable limit parameters:

- `max_force_x_N`
- `max_force_y_N`
- `max_yaw_torque_Nm`

Commands:

```bash
ros2 param set /orca_auv/bottom_camera_pid_bridge_node max_force_x_N 10.0
ros2 param set /orca_auv/bottom_camera_pid_bridge_node max_force_y_N 10.0
ros2 param set /orca_auv/bottom_camera_pid_bridge_node max_yaw_torque_Nm 5.0
```

Useful inspection commands:

```bash
ros2 param list /orca_auv/bottom_camera_pid_bridge_node
ros2 param get /orca_auv/bottom_camera_pid_bridge_node max_force_x_N
ros2 param get /orca_auv/bottom_camera_pid_bridge_node max_force_y_N
ros2 param get /orca_auv/bottom_camera_pid_bridge_node max_yaw_torque_Nm
```

## Before Running Commands

Run after the ROS stack is already started.

If you are inside `make compose_shell`, source the workspace first:

```bash
cd rpi_ros2_ws
source /opt/ros/humble/setup.bash
source /root/uros_ws/install/local_setup.bash
source install/setup.bash
```

## `x_coordinate_pid_controller_node`

Node name:

- `/orca_auv/x_coordinate_pid_controller_node`

Runtime-tunable parameters:

- `proportional_gain`
- `integral_gain`
- `derivative_gain`
- `derivative_smoothing_factor`

Commands:

```bash
ros2 param set /orca_auv/x_coordinate_pid_controller_node proportional_gain 0.05
ros2 param set /orca_auv/x_coordinate_pid_controller_node integral_gain 0.00001
ros2 param set /orca_auv/x_coordinate_pid_controller_node derivative_gain 0.03
ros2 param set /orca_auv/x_coordinate_pid_controller_node derivative_smoothing_factor 0.3
```

## `y_coordinate_pid_controller_node`

Node name:

- `/orca_auv/y_coordinate_pid_controller_node`

Runtime-tunable parameters:

- `proportional_gain`
- `integral_gain`
- `derivative_gain`
- `derivative_smoothing_factor`

Commands:

```bash
ros2 param set /orca_auv/y_coordinate_pid_controller_node proportional_gain 0.03
ros2 param set /orca_auv/y_coordinate_pid_controller_node integral_gain 0.00002
ros2 param set /orca_auv/y_coordinate_pid_controller_node derivative_gain 0.001
ros2 param set /orca_auv/y_coordinate_pid_controller_node derivative_smoothing_factor 0.2
```

## `yaw_angle_pid_controller_node`

Node name:

- `/orca_auv/yaw_angle_pid_controller_node`

Runtime-tunable parameters:

- `proportional_gain`
- `integral_gain`
- `derivative_gain`
- `derivative_smoothing_factor`

Commands:

```bash
ros2 param set /orca_auv/yaw_angle_pid_controller_node proportional_gain 0.5
ros2 param set /orca_auv/yaw_angle_pid_controller_node integral_gain 0.0
ros2 param set /orca_auv/yaw_angle_pid_controller_node derivative_gain 0.02
ros2 param set /orca_auv/yaw_angle_pid_controller_node derivative_smoothing_factor 0.3
```

## Useful Inspection Commands

```bash
ros2 param list /orca_auv/x_coordinate_pid_controller_node
ros2 param list /orca_auv/y_coordinate_pid_controller_node
ros2 param list /orca_auv/yaw_angle_pid_controller_node
```

```bash
ros2 param get /orca_auv/x_coordinate_pid_controller_node proportional_gain
ros2 param get /orca_auv/y_coordinate_pid_controller_node integral_gain
ros2 param get /orca_auv/yaw_angle_pid_controller_node derivative_gain
```

## Notes

- `derivative_smoothing_factor` must stay between `0.0` and `1.0`.
- `controller_loop_timer_period_s` is a declared parameter, but changing it at
  runtime does not recreate the timer, so it is not a practical live-tuning
  parameter.
- Even if you increase PID gains, the published bottom-camera wrench is still
  limited by the bridge to `force.x/y = +/-10.0 N` and `torque.z = +/-5.0 Nm`.
