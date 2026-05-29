# How to Adjust Depth Control Parameters

This document records runtime commands for depth control tuning.

## Before Running Commands

Run after the ROS stack is already started.

If you are inside `make compose_shell`, source the workspace first:

```bash
cd rpi_ros2_ws
source /opt/ros/humble/setup.bash
source /root/uros_ws/install/local_setup.bash
source install/setup.bash
```

## Depth PID

Node name:

- `/orca_auv/depth_pid_controller_node`

Runtime-tunable parameters:

- `proportional_gain`
- `integral_gain`
- `derivative_gain`
- `derivative_smoothing_factor`

Commands:

```bash
ros2 param set /orca_auv/depth_pid_controller_node proportional_gain 40.0
ros2 param set /orca_auv/depth_pid_controller_node integral_gain 3.0
ros2 param set /orca_auv/depth_pid_controller_node derivative_gain 12.0
ros2 param set /orca_auv/depth_pid_controller_node derivative_smoothing_factor 0.0
```

## Depth Force Bias

Node name:

- `/orca_auv/output_sink_force_to_output_wrench_node`

Runtime-tunable parameter:

- `depth_force_bias_N`

`depth_force_bias_N` is added to the depth PID output before publishing the
depth wrench:

```text
final_sink_force_N = pid_sink_force_N + depth_force_bias_N
```

Direction convention:

- Positive `depth_force_bias_N` adds constant downward sink force.
- Negative `depth_force_bias_N` adds constant upward force.
- `0.0` keeps the previous pure-PID behavior.

Commands:

```bash
ros2 param set /orca_auv/output_sink_force_to_output_wrench_node depth_force_bias_N 1.5
ros2 param get /orca_auv/output_sink_force_to_output_wrench_node depth_force_bias_N
```
