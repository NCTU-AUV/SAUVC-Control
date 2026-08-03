#!/usr/bin/env python3
import math
from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from xy_translation_control_interfaces.action import MoveToPoint


class MissionState(Enum):
    WAIT_SYSTEM = auto()
    CALL_RESET_BOTTOM_CAMERA_POSE = auto()
    CALL_RESET_MOVE_TO_POINT = auto()
    WAIT_STARTUP_FEEDBACK = auto()
    CALL_RESET_CONTROLLERS = auto()
    CALL_DEPTH_HOLD = auto()
    WAIT_TRIGGER_RESULT = auto()
    WAIT_REACH_DEPTH = auto()
    CALL_BOTTOM_CAMERA_HOLD = auto()
    WAIT_MOVE_SERVER = auto()
    SEND_MOVE_GOAL = auto()
    WAIT_GOAL_ACCEPT = auto()
    MOVING = auto()
    WAIT_FORWARD_HOLD_STABLE = auto()
    PREPARE_TURN = auto()
    WAIT_REACH_TURN_YAW = auto()
    CALL_DISABLE_DEPTH_HOLD = auto()
    DONE = auto()
    FAILED = auto()


class DiveThenForwardMissionNode(Node):
    """Run a dive-then-forward mission."""

    def __init__(self):
        super().__init__("dive_then_forward_mission_node")

        # =========================
        # Parameters
        # =========================

        # Target depth in meters.
        self.declare_parameter("target_depth_m", 0.6)

        # Allowed depth error before we consider the AUV to have reached target depth.
        self.declare_parameter("depth_tolerance_m", 0.1)

        # Depth must stay within tolerance for this long before moving forward.
        self.declare_parameter("depth_stable_time_s", 2.0)

        # Move relative to the current bottom-camera XY feedback.
        # Forward direction is +X.
        self.declare_parameter("target_x_px", 4000.0)
        self.declare_parameter("target_y_px", 0.0)

        # The yaw target is latched from the first bottom-camera yaw feedback.
        # This keeps the startup heading instead of forcing a fixed yaw angle.
        self.declare_parameter("turn_yaw_offset_rad", math.pi)

        # Maximum yaw target increment during the turn.
        self.declare_parameter("turn_step_rad", math.pi / 4.0)

        # Only used for warning logs. The controller itself still tries to correct yaw.
        self.declare_parameter("yaw_tolerance_rad", 0.10)

        # Kept for compatibility with existing commands. Staged turns advance
        # once each yaw step target is crossed.
        self.declare_parameter("yaw_stable_time_s", 1.0)

        # Speed of waypoint setpoint generation in px/s.
        self.declare_parameter("speed_px_s", 200.0)

        # Forward target must stay within XY tolerance before turning.
        self.declare_parameter("forward_hold_tolerance_px", 25.0)
        self.declare_parameter("forward_hold_stable_time_s", 1.0)
        self.declare_parameter("max_forward_hold_wait_s", 30.0)

        # Deprecated compatibility parameter. The mission now disables depth hold
        # after returning instead of commanding a surfaced depth target.
        self.declare_parameter("surface_depth_m", 0.2)

        # Mission loop period.
        self.declare_parameter("control_period_s", 0.1)

        # Timeout for waiting to reach depth. Simulation can dive slowly.
        self.declare_parameter("max_depth_wait_s", 180.0)

        # Timeout for movement action.
        self.declare_parameter("max_move_wait_s", 120.0)

        # Timeout for the 180 degree turn.
        self.declare_parameter("max_turn_wait_s", 60.0)

        self.target_depth_m = float(self.get_parameter("target_depth_m").value)
        self.depth_tolerance_m = float(self.get_parameter("depth_tolerance_m").value)
        self.depth_stable_time_s = float(self.get_parameter("depth_stable_time_s").value)

        self.target_x_px = float(self.get_parameter("target_x_px").value)
        self.target_y_px = float(self.get_parameter("target_y_px").value)

        self.turn_yaw_offset_rad = float(self.get_parameter("turn_yaw_offset_rad").value)
        self.turn_step_rad = float(self.get_parameter("turn_step_rad").value)
        if not math.isfinite(self.turn_step_rad) or self.turn_step_rad <= 0.0:
            self.get_logger().warn(
                "turn_step_rad must be finite and > 0. Using pi/4 rad."
            )
            self.turn_step_rad = math.pi / 4.0
        self.target_yaw_rad = None
        self.startup_yaw_rad = None
        self.yaw_tolerance_rad = float(self.get_parameter("yaw_tolerance_rad").value)

        self.speed_px_s = float(self.get_parameter("speed_px_s").value)
        self.forward_hold_tolerance_px = float(
            self.get_parameter("forward_hold_tolerance_px").value
        )
        self.forward_hold_stable_time_s = float(
            self.get_parameter("forward_hold_stable_time_s").value
        )
        self.max_forward_hold_wait_s = float(
            self.get_parameter("max_forward_hold_wait_s").value
        )
        self.control_period_s = float(self.get_parameter("control_period_s").value)
        self.max_depth_wait_s = float(self.get_parameter("max_depth_wait_s").value)
        self.max_move_wait_s = float(self.get_parameter("max_move_wait_s").value)
        self.max_turn_wait_s = float(self.get_parameter("max_turn_wait_s").value)

        # =========================
        # Publishers / Subscribers
        # =========================

        # Depth target for depth PID.
        self.depth_target_pub = self.create_publisher(
            Float64,
            "control/targets/depth_m",
            10,
        )

        # Yaw target for bottom-camera yaw PID.
        self.yaw_target_pub = self.create_publisher(
            Float64,
            "control/targets/bottom_camera/yaw_rad",
            10,
        )

        # Bottom-camera XY hold targets for keeping startup position during dive.
        self.x_target_pub = self.create_publisher(
            Float64,
            "control/pid/bottom_camera/x/reference_px",
            10,
        )

        self.y_target_pub = self.create_publisher(
            Float64,
            "control/pid/bottom_camera/y/reference_px",
            10,
        )

        # Current depth feedback.
        self.depth_sub = self.create_subscription(
            Float64,
            "state/depth_m",
            self._on_depth,
            10,
        )

        # Current yaw feedback from bottom-camera pose estimation.
        self.yaw_feedback_sub = self.create_subscription(
            Float64,
            "state/bottom_camera/yaw_rad",
            self._on_yaw_feedback,
            10,
        )

        # Optional XY feedback, only for logging / checking.
        self.x_feedback_sub = self.create_subscription(
            Float64,
            "control/pid/bottom_camera/x/feedback_px",
            self._on_x_feedback,
            10,
        )

        self.y_feedback_sub = self.create_subscription(
            Float64,
            "control/pid/bottom_camera/y/feedback_px",
            self._on_y_feedback,
            10,
        )

        # =========================
        # Supervisor services
        # =========================

        self.reset_controllers_client = self.create_client(
            Trigger,
            "system_manager/reset_controllers",
        )

        self.depth_hold_client = self.create_client(
            Trigger,
            "system_manager/set_mode/depth_hold",
        )

        self.disable_depth_hold_client = self.create_client(
            Trigger,
            "system_manager/disable/depth_hold",
        )

        self.bottom_camera_hold_client = self.create_client(
            Trigger,
            "system_manager/set_mode/bottom_camera_hold",
        )

        self.bottom_camera_reset_client = self.create_client(
            Trigger,
            "camera/bottom/reset_pose",
        )

        self.move_to_point_reset_client = self.create_client(
            Trigger,
            "control/targets/reset_move_to_point",
        )

        # =========================
        # MoveToPoint action client
        # =========================

        self.move_client = ActionClient(
            self,
            MoveToPoint,
            "control/targets/move_to_point",
        )

        # =========================
        # Mission runtime variables
        # =========================

        self.state = MissionState.WAIT_SYSTEM

        self.current_depth_m = None
        self.current_yaw_rad = None
        self.current_x_px = None
        self.current_y_px = None
        self.startup_x_px = None
        self.startup_y_px = None
        self.startup_xy_logged = False
        self.bottom_camera_pose_reset_done = False
        self.hold_x_px = None
        self.hold_y_px = None
        self.active_depth_target_m = self.target_depth_m

        self.depth_reached_since_s = None
        self.forward_hold_reached_since_s = None
        self.turn_step_targets_rad = []
        self.turn_step_index = 0

        self.pending_trigger_future = None
        self.pending_trigger_name = ""
        self.pending_trigger_next_state = None

        self.move_goal_name = ""
        self.move_goal_x_px = None
        self.move_goal_y_px = None
        self.move_goal_next_state = None
        self.goal_handle = None
        self.move_result_future = None

        self.state_start_time_s = self._now_s()

        self.timer = self.create_timer(self.control_period_s, self._tick)

        self.get_logger().info(
            "Mission node started. "
            f"dive depth = {self.target_depth_m:.3f} m, "
            f"forward offset = ({self.target_x_px:.1f}, {self.target_y_px:.1f}) px, "
            f"turn yaw offset = {self.turn_yaw_offset_rad:.3f} rad, "
            f"turn step = {self.turn_step_rad:.3f} rad, "
            f"forward hold tolerance = {self.forward_hold_tolerance_px:.1f} px, "
            "return action = disable depth hold, "
            f"speed = {self.speed_px_s:.1f} px/s"
        )

    # =========================
    # Subscriber callbacks
    # =========================

    def _on_depth(self, msg: Float64):
        if math.isfinite(msg.data):
            self.current_depth_m = float(msg.data)

    def _on_yaw_feedback(self, msg: Float64):
        if math.isfinite(msg.data):
            self.current_yaw_rad = float(msg.data)
            if self.bottom_camera_pose_reset_done and self.startup_yaw_rad is None:
                self.startup_yaw_rad = self.current_yaw_rad
                self.target_yaw_rad = self.startup_yaw_rad
                self.get_logger().info(
                    f"Latched startup yaw target = {self.startup_yaw_rad:.3f} rad"
                )

    def _on_x_feedback(self, msg: Float64):
        if math.isfinite(msg.data):
            self.current_x_px = float(msg.data)
            if self.bottom_camera_pose_reset_done and self.startup_x_px is None:
                self.startup_x_px = self.current_x_px
                self._log_startup_xy_if_ready()

    def _on_y_feedback(self, msg: Float64):
        if math.isfinite(msg.data):
            self.current_y_px = float(msg.data)
            if self.bottom_camera_pose_reset_done and self.startup_y_px is None:
                self.startup_y_px = self.current_y_px
                self._log_startup_xy_if_ready()

    # =========================
    # Main mission loop
    # =========================

    def _tick(self):
        # Keep depth and yaw targets alive during the whole active mission.
        # Depth target keeps the AUV at the current mission depth target.
        # Yaw target keeps the AUV at the active mission heading.
        if self.state not in (MissionState.DONE, MissionState.FAILED):
            self._publish_depth_target()
            self._publish_yaw_target()
            self._publish_hold_xy_target()

        if self.state == MissionState.WAIT_SYSTEM:
            self._tick_wait_system()

        elif self.state == MissionState.CALL_RESET_BOTTOM_CAMERA_POSE:
            self._call_trigger_service(
                client=self.bottom_camera_reset_client,
                service_name="bottom_camera_reset_pose",
                next_state=MissionState.CALL_RESET_MOVE_TO_POINT,
            )

        elif self.state == MissionState.CALL_RESET_MOVE_TO_POINT:
            self._call_trigger_service(
                client=self.move_to_point_reset_client,
                service_name="move_to_point_reset",
                next_state=MissionState.WAIT_STARTUP_FEEDBACK,
            )

        elif self.state == MissionState.WAIT_STARTUP_FEEDBACK:
            self._tick_wait_startup_feedback()

        elif self.state == MissionState.CALL_RESET_CONTROLLERS:
            self._call_trigger_service(
                client=self.reset_controllers_client,
                service_name="reset_controllers",
                next_state=MissionState.CALL_DEPTH_HOLD,
            )

        elif self.state == MissionState.CALL_DEPTH_HOLD:
            self._call_trigger_service(
                client=self.depth_hold_client,
                service_name="depth_hold",
                next_state=MissionState.CALL_BOTTOM_CAMERA_HOLD,
            )

        elif self.state == MissionState.WAIT_TRIGGER_RESULT:
            self._tick_wait_trigger_result()

        elif self.state == MissionState.WAIT_REACH_DEPTH:
            self._tick_wait_reach_depth()

        elif self.state == MissionState.CALL_BOTTOM_CAMERA_HOLD:
            self._call_trigger_service(
                client=self.bottom_camera_hold_client,
                service_name="bottom_camera_hold",
                next_state=MissionState.WAIT_REACH_DEPTH,
            )

        elif self.state == MissionState.WAIT_MOVE_SERVER:
            self._tick_wait_move_server()

        elif self.state == MissionState.SEND_MOVE_GOAL:
            self._send_move_goal()

        elif self.state == MissionState.WAIT_GOAL_ACCEPT:
            # Result is handled in _on_goal_response callback.
            pass

        elif self.state == MissionState.MOVING:
            self._tick_moving()

        elif self.state == MissionState.WAIT_FORWARD_HOLD_STABLE:
            self._tick_wait_forward_hold_stable()

        elif self.state == MissionState.PREPARE_TURN:
            self._tick_prepare_turn()

        elif self.state == MissionState.WAIT_REACH_TURN_YAW:
            self._tick_wait_reach_turn_yaw()

        elif self.state == MissionState.CALL_DISABLE_DEPTH_HOLD:
            self._call_trigger_service(
                client=self.disable_depth_hold_client,
                service_name="disable_depth_hold",
                next_state=MissionState.DONE,
            )

        elif self.state == MissionState.DONE:
            # Mission finished successfully.
            pass

        elif self.state == MissionState.FAILED:
            # Mission failed. Keep node alive for debugging.
            pass

    # =========================
    # State handlers
    # =========================

    def _tick_wait_system(self):
        if self.current_depth_m is None:
            self.get_logger().info(
                "Waiting for state/depth_m...",
                throttle_duration_sec=2.0,
            )
            return

        if self.current_x_px is None or self.current_y_px is None:
            self.get_logger().info(
                "Waiting for bottom-camera XY feedback...",
                throttle_duration_sec=2.0,
            )
            return

        if self.current_yaw_rad is None:
            self.get_logger().info(
                "Waiting for bottom-camera yaw feedback...",
                throttle_duration_sec=2.0,
            )
            return

        if not self.reset_controllers_client.service_is_ready():
            self.get_logger().info(
                "Waiting for system_manager/reset_controllers service...",
                throttle_duration_sec=2.0,
            )
            return

        if not self.depth_hold_client.service_is_ready():
            self.get_logger().info(
                "Waiting for system_manager/set_mode/depth_hold service...",
                throttle_duration_sec=2.0,
            )
            return

        if not self.disable_depth_hold_client.service_is_ready():
            self.get_logger().info(
                "Waiting for system_manager/disable/depth_hold service...",
                throttle_duration_sec=2.0,
            )
            return

        if not self.bottom_camera_hold_client.service_is_ready():
            self.get_logger().info(
                "Waiting for system_manager/set_mode/bottom_camera_hold service...",
                throttle_duration_sec=2.0,
            )
            return

        if not self.bottom_camera_reset_client.service_is_ready():
            self.get_logger().info(
                "Waiting for camera/bottom/reset_pose service...",
                throttle_duration_sec=2.0,
            )
            return

        if not self.move_to_point_reset_client.service_is_ready():
            self.get_logger().info(
                "Waiting for control/targets/reset_move_to_point service...",
                throttle_duration_sec=2.0,
            )
            return

        self._set_state(MissionState.CALL_RESET_BOTTOM_CAMERA_POSE)

    def _tick_wait_startup_feedback(self):
        if self.startup_yaw_rad is None:
            self.get_logger().info(
                "Waiting for reset bottom-camera yaw feedback...",
                throttle_duration_sec=2.0,
            )
            return

        if self.startup_x_px is None or self.startup_y_px is None:
            self.get_logger().info(
                "Waiting for reset bottom-camera XY feedback...",
                throttle_duration_sec=2.0,
            )
            return

        self.active_depth_target_m = self.target_depth_m
        self.target_yaw_rad = self.startup_yaw_rad
        self._set_hold_xy_target(self.startup_x_px, self.startup_y_px)
        self._set_state(MissionState.CALL_RESET_CONTROLLERS)

    def _tick_wait_reach_depth(self):
        if self.current_depth_m is None:
            self.get_logger().info(
                "Waiting for depth feedback...",
                throttle_duration_sec=2.0,
            )
            return

        if self._time_in_state_s() > self.max_depth_wait_s:
            self._fail(
                "Timeout while waiting to reach depth. "
                f"current_depth = {self.current_depth_m:.3f}, "
                f"target_depth = {self.target_depth_m:.3f}"
            )
            return

        error = abs(self.current_depth_m - self.target_depth_m)

        if error <= self.depth_tolerance_m:
            if self.depth_reached_since_s is None:
                self.depth_reached_since_s = self._now_s()

            stable_time_s = self._now_s() - self.depth_reached_since_s

            self.get_logger().info(
                "Depth is within tolerance. "
                f"current = {self.current_depth_m:.3f}, "
                f"target = {self.target_depth_m:.3f}, "
                f"error = {error:.3f}, "
                f"stable_time = {stable_time_s:.1f}s",
                throttle_duration_sec=1.0,
            )

            if stable_time_s >= self.depth_stable_time_s:
                self._start_move_goal(
                    name="forward",
                    x_px=self.startup_x_px + self.target_x_px,
                    y_px=self.startup_y_px + self.target_y_px,
                    next_state=MissionState.WAIT_FORWARD_HOLD_STABLE,
                )

        else:
            self.depth_reached_since_s = None

            self.get_logger().info(
                "Waiting to reach target depth. "
                f"current = {self.current_depth_m:.3f}, "
                f"target = {self.target_depth_m:.3f}, "
                f"error = {error:.3f}",
                throttle_duration_sec=1.0,
            )

    def _tick_wait_forward_hold_stable(self):
        if self.hold_x_px is None or self.hold_y_px is None:
            self._fail("No forward hold target before turning.")
            return

        if self.current_x_px is None or self.current_y_px is None:
            self.get_logger().info(
                "Waiting for bottom-camera XY feedback before turning...",
                throttle_duration_sec=2.0,
            )
            return

        if self._time_in_state_s() > self.max_forward_hold_wait_s:
            error_px = self._xy_error_px(self.hold_x_px, self.hold_y_px)
            self._fail(
                "Timeout while waiting for forward hold to stabilize. "
                f"current = ({self.current_x_px:.1f}, {self.current_y_px:.1f}) px, "
                f"target = ({self.hold_x_px:.1f}, {self.hold_y_px:.1f}) px, "
                f"error = {error_px:.1f} px"
            )
            return

        error_px = self._xy_error_px(self.hold_x_px, self.hold_y_px)

        if error_px <= self.forward_hold_tolerance_px:
            if self.forward_hold_reached_since_s is None:
                self.forward_hold_reached_since_s = self._now_s()

            stable_time_s = self._now_s() - self.forward_hold_reached_since_s
            self.get_logger().info(
                "Forward hold is within tolerance. "
                f"current = ({self.current_x_px:.1f}, {self.current_y_px:.1f}) px, "
                f"target = ({self.hold_x_px:.1f}, {self.hold_y_px:.1f}) px, "
                f"error = {error_px:.1f} px, "
                f"stable_time = {stable_time_s:.1f}s",
                throttle_duration_sec=1.0,
            )

            if stable_time_s >= self.forward_hold_stable_time_s:
                self._set_state(MissionState.PREPARE_TURN)

        else:
            self.forward_hold_reached_since_s = None
            self.get_logger().info(
                "Waiting for forward hold before turning. "
                f"current = ({self.current_x_px:.1f}, {self.current_y_px:.1f}) px, "
                f"target = ({self.hold_x_px:.1f}, {self.hold_y_px:.1f}) px, "
                f"error = {error_px:.1f} px",
                throttle_duration_sec=1.0,
            )

    def _tick_prepare_turn(self):
        if self.startup_yaw_rad is None:
            self.get_logger().info(
                "Waiting for startup yaw before turning...",
                throttle_duration_sec=2.0,
            )
            return

        if self.startup_x_px is None or self.startup_y_px is None:
            self.get_logger().info(
                "Waiting for startup XY before turning...",
                throttle_duration_sec=2.0,
            )
            return

        turn_hold_x_px = self.startup_x_px + self.target_x_px
        turn_hold_y_px = self.startup_y_px + self.target_y_px
        self._set_hold_xy_target(turn_hold_x_px, turn_hold_y_px)

        self.turn_step_targets_rad = self._build_turn_step_targets()
        self.turn_step_index = 0

        if not self.turn_step_targets_rad:
            self._fail("No turn yaw targets could be generated.")
            return

        self.target_yaw_rad = self.turn_step_targets_rad[self.turn_step_index]
        final_target_yaw_rad = self.turn_step_targets_rad[-1]
        self.get_logger().info(
            "Starting staged in-place turn. "
            f"hold_xy = ({turn_hold_x_px:.1f}, {turn_hold_y_px:.1f}) px, "
            f"step = {self._turn_step_text()}, "
            f"step_target_yaw = {self.target_yaw_rad:.3f} rad, "
            f"final_target_yaw = {final_target_yaw_rad:.3f} rad"
        )
        self._set_state(MissionState.WAIT_REACH_TURN_YAW)

    def _tick_wait_reach_turn_yaw(self):
        if self.current_yaw_rad is None:
            self.get_logger().info(
                "Waiting for bottom-camera yaw feedback while turning...",
                throttle_duration_sec=2.0,
            )
            return

        if self.target_yaw_rad is None:
            self._fail("No target yaw while turning.")
            return

        if self._time_in_state_s() > self.max_turn_wait_s:
            self._fail(
                "Timeout while waiting to complete staged turn. "
                f"step = {self._turn_step_text()}, "
                f"current_yaw = {self.current_yaw_rad:.3f}, "
                f"target_yaw = {self.target_yaw_rad:.3f}"
            )
            return

        yaw_error = self._angle_error(self.target_yaw_rad, self.current_yaw_rad)

        if self._has_reached_or_crossed_turn_step(yaw_error):
            self.get_logger().info(
                "Turn yaw step target crossed. "
                f"step = {self._turn_step_text()}, "
                f"current = {self.current_yaw_rad:.3f}, "
                f"target = {self.target_yaw_rad:.3f}, "
                f"error = {yaw_error:.3f}",
                throttle_duration_sec=1.0,
            )

            if self._advance_turn_step():
                return

            self._start_move_goal(
                name="return",
                x_px=self.startup_x_px,
                y_px=self.startup_y_px,
                next_state=MissionState.CALL_DISABLE_DEPTH_HOLD,
            )

        else:
            self.get_logger().info(
                "Waiting to cross staged turn target. "
                f"step = {self._turn_step_text()}, "
                f"current = {self.current_yaw_rad:.3f}, "
                f"target = {self.target_yaw_rad:.3f}, "
                f"error = {yaw_error:.3f}",
                throttle_duration_sec=1.0,
            )

    def _tick_wait_move_server(self):
        if self.move_goal_x_px is None or self.move_goal_y_px is None:
            self._fail("Internal error: no pending MoveToPoint goal.")
            return

        if not self.move_client.server_is_ready():
            self.get_logger().info(
                "Waiting for MoveToPoint action server...",
                throttle_duration_sec=2.0,
            )
            return

        self._set_state(MissionState.SEND_MOVE_GOAL)

    def _tick_moving(self):
        if self._time_in_state_s() > self.max_move_wait_s:
            self._fail("Timeout during MoveToPoint.")
            return

        # Depth warning. Depth PID should still be correcting it.
        if self.current_depth_m is not None:
            depth_error = abs(self.current_depth_m - self.target_depth_m)

            if depth_error > self.depth_tolerance_m * 3.0:
                self.get_logger().warn(
                    "Depth deviation during movement. "
                    f"current = {self.current_depth_m:.3f}, "
                    f"target = {self.target_depth_m:.3f}, "
                    f"error = {depth_error:.3f}",
                    throttle_duration_sec=1.0,
                )

        # Yaw warning. Yaw PID should still be correcting it.
        if self.target_yaw_rad is not None and self.current_yaw_rad is not None:
            yaw_error = self._angle_error(self.target_yaw_rad, self.current_yaw_rad)

            if abs(yaw_error) > self.yaw_tolerance_rad:
                self.get_logger().warn(
                    "Yaw deviation during movement. "
                    f"current = {self.current_yaw_rad:.3f}, "
                    f"target = {self.target_yaw_rad:.3f}, "
                    f"error = {yaw_error:.3f}",
                    throttle_duration_sec=1.0,
                )

        if self.current_x_px is not None and self.current_y_px is not None:
            yaw = self.current_yaw_rad
            yaw_text = f"{yaw:.3f}" if yaw is not None else "nan"
            self.get_logger().info(
                "Current bottom-camera feedback: "
                f"x = {self.current_x_px:.1f}, "
                f"y = {self.current_y_px:.1f}, "
                f"yaw = {yaw_text}",
                throttle_duration_sec=2.0,
            )

    # =========================
    # Service handling
    # =========================

    def _call_trigger_service(self, client, service_name: str, next_state: MissionState):
        if not client.service_is_ready():
            self.get_logger().info(
                f"Waiting for {service_name} service...",
                throttle_duration_sec=2.0,
            )
            return

        self.get_logger().info(f"Calling supervisor service: {service_name}")

        request = Trigger.Request()
        self.pending_trigger_future = client.call_async(request)
        self.pending_trigger_name = service_name
        self.pending_trigger_next_state = next_state

        self._set_state(MissionState.WAIT_TRIGGER_RESULT)

    def _tick_wait_trigger_result(self):
        if self.pending_trigger_future is None:
            self._fail("Internal error: no pending trigger service call.")
            return

        if not self.pending_trigger_future.done():
            return

        try:
            response = self.pending_trigger_future.result()
        except Exception as exc:
            self._fail(f"{self.pending_trigger_name} service call failed: {exc}")
            return

        if not response.success:
            self._fail(
                f"{self.pending_trigger_name} rejected. "
                f"message = {response.message}"
            )
            return

        self.get_logger().info(
            f"{self.pending_trigger_name} succeeded. "
            f"message = {response.message}"
        )

        trigger_name = self.pending_trigger_name
        next_state = self.pending_trigger_next_state

        self.pending_trigger_future = None
        self.pending_trigger_name = ""
        self.pending_trigger_next_state = None

        if trigger_name == "bottom_camera_reset_pose":
            self._clear_startup_feedback_latch()

        self._set_state(next_state)

    # =========================
    # Action handling
    # =========================

    def _start_move_goal(
        self,
        name: str,
        x_px: float,
        y_px: float,
        next_state: MissionState,
    ):
        self.move_goal_name = name
        self.move_goal_x_px = float(x_px)
        self.move_goal_y_px = float(y_px)
        self.move_goal_next_state = next_state
        self._set_state(MissionState.WAIT_MOVE_SERVER)

    def _send_move_goal(self):
        if self.move_goal_x_px is None or self.move_goal_y_px is None:
            self._fail("Internal error: no pending MoveToPoint goal to send.")
            return

        goal = MoveToPoint.Goal()

        goal.x_px = self.move_goal_x_px
        goal.y_px = self.move_goal_y_px
        goal.speed_px_s = self.speed_px_s

        yaw_target_text = (
            f"{self.target_yaw_rad:.3f}"
            if self.target_yaw_rad is not None
            else "nan"
        )
        self.get_logger().info(
            "Sending MoveToPoint goal: "
            f"name = {self.move_goal_name}, "
            f"x = {goal.x_px:.1f}, "
            f"y = {goal.y_px:.1f}, "
            f"speed = {goal.speed_px_s:.1f}, "
            f"depth target = {self.active_depth_target_m:.3f} m, "
            f"yaw target = {yaw_target_text} rad"
        )

        send_future = self.move_client.send_goal_async(
            goal,
            feedback_callback=self._on_move_feedback,
        )
        send_future.add_done_callback(self._on_goal_response)

        self._set_state(MissionState.WAIT_GOAL_ACCEPT)

    def _on_goal_response(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self._fail(f"Failed to send MoveToPoint goal: {exc}")
            return

        if not self.goal_handle.accepted:
            self._fail("MoveToPoint goal was rejected.")
            return

        self.get_logger().info("MoveToPoint goal accepted.")

        self.move_result_future = self.goal_handle.get_result_async()
        self.move_result_future.add_done_callback(self._on_move_result)

        self._set_state(MissionState.MOVING)

    def _on_move_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(
            "MoveToPoint feedback: "
            f"target_x = {feedback.target_x_px:.1f}, "
            f"target_y = {feedback.target_y_px:.1f}, "
            f"progress = {feedback.progress * 100.0:.1f}%, "
            f"remaining = {feedback.remaining_distance_px:.1f} px",
            throttle_duration_sec=1.0,
        )

    def _on_move_result(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            self._fail(f"Failed to get MoveToPoint result: {exc}")
            return

        if result.success:
            self.get_logger().info(
                f"MoveToPoint {self.move_goal_name} completed. "
                f"message = {result.message}"
            )
            self._set_hold_xy_target(self.move_goal_x_px, self.move_goal_y_px)

            next_state = self.move_goal_next_state
            self.move_goal_name = ""
            self.move_goal_x_px = None
            self.move_goal_y_px = None
            self.move_goal_next_state = None
            self.goal_handle = None
            self.move_result_future = None

            if next_state is None:
                self._fail("Internal error: MoveToPoint completed without next state.")
                return

            self._set_state(next_state)
        else:
            self._fail(f"MoveToPoint failed. message = {result.message}")

    # =========================
    # Target publishers
    # =========================

    def _publish_depth_target(self):
        msg = Float64()
        msg.data = self.active_depth_target_m
        self.depth_target_pub.publish(msg)

    def _publish_yaw_target(self):
        if self.target_yaw_rad is None:
            return

        msg = Float64()
        msg.data = self.target_yaw_rad
        self.yaw_target_pub.publish(msg)

    def _publish_hold_xy_target(self):
        if self.state in (MissionState.WAIT_GOAL_ACCEPT, MissionState.MOVING):
            return

        if self.hold_x_px is None or self.hold_y_px is None:
            return

        self._publish_float(self.x_target_pub, self.hold_x_px)
        self._publish_float(self.y_target_pub, self.hold_y_px)

    # =========================
    # Utility functions
    # =========================

    def _set_state(self, new_state: MissionState):
        if self.state == new_state:
            return

        self.get_logger().info(f"State: {self.state.name} -> {new_state.name}")

        self.state = new_state
        self.state_start_time_s = self._now_s()

        if new_state == MissionState.WAIT_REACH_DEPTH:
            self.depth_reached_since_s = None

        if new_state == MissionState.WAIT_FORWARD_HOLD_STABLE:
            self.forward_hold_reached_since_s = None

    def _fail(self, reason: str):
        self.get_logger().error(f"Mission failed: {reason}")
        self._set_state(MissionState.FAILED)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _time_in_state_s(self) -> float:
        return self._now_s() - self.state_start_time_s

    def _clear_startup_feedback_latch(self):
        self.bottom_camera_pose_reset_done = True
        self.startup_yaw_rad = None
        self.target_yaw_rad = None
        self.startup_x_px = None
        self.startup_y_px = None
        self.startup_xy_logged = False
        self.hold_x_px = None
        self.hold_y_px = None
        self.turn_step_targets_rad = []
        self.turn_step_index = 0

    def _log_startup_xy_if_ready(self):
        if self.startup_xy_logged:
            return

        if self.startup_x_px is None or self.startup_y_px is None:
            return

        self.startup_xy_logged = True
        self.get_logger().info(
            "Latched startup XY target = "
            f"({self.startup_x_px:.1f}, {self.startup_y_px:.1f}) px"
        )

    def _set_hold_xy_target(self, x_px: float, y_px: float):
        self.hold_x_px = float(x_px)
        self.hold_y_px = float(y_px)

    def _xy_error_px(self, target_x_px: float, target_y_px: float) -> float:
        return math.hypot(
            self.current_x_px - target_x_px,
            self.current_y_px - target_y_px,
        )

    def _build_turn_step_targets(self):
        if self.startup_yaw_rad is None:
            return []

        if not math.isfinite(self.turn_yaw_offset_rad):
            return []

        total_offset_rad = abs(self.turn_yaw_offset_rad)
        if total_offset_rad <= 1e-9:
            return [self.startup_yaw_rad]

        step_rad = min(abs(self.turn_step_rad), total_offset_rad)
        direction = 1.0 if self.turn_yaw_offset_rad >= 0.0 else -1.0

        targets = []
        next_offset_rad = step_rad
        while next_offset_rad < total_offset_rad:
            targets.append(self.startup_yaw_rad + direction * next_offset_rad)
            next_offset_rad += step_rad

        targets.append(self.startup_yaw_rad + self.turn_yaw_offset_rad)
        return targets

    def _advance_turn_step(self) -> bool:
        next_index = self.turn_step_index + 1
        if next_index >= len(self.turn_step_targets_rad):
            return False

        self.turn_step_index = next_index
        self.target_yaw_rad = self.turn_step_targets_rad[self.turn_step_index]
        self.get_logger().info(
            "Advancing staged turn target. "
            f"step = {self._turn_step_text()}, "
            f"target_yaw = {self.target_yaw_rad:.3f} rad"
        )
        return True

    def _has_reached_or_crossed_turn_step(self, yaw_error: float) -> bool:
        turn_direction = 1.0 if self.turn_yaw_offset_rad >= 0.0 else -1.0
        return turn_direction * yaw_error <= 0.0

    def _turn_step_text(self) -> str:
        total_steps = len(self.turn_step_targets_rad)
        if total_steps <= 0:
            return "unknown"
        current_step = min(self.turn_step_index + 1, total_steps)
        return f"{current_step}/{total_steps}"

    @staticmethod
    def _publish_float(pub, value: float):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    @staticmethod
    def _angle_error(target: float, current: float) -> float:
        """Return shortest angular error target - current in range [-pi, pi]."""
        error = target - current
        return math.atan2(math.sin(error), math.cos(error))


def main(args=None):
    rclpy.init(args=args)

    node = DiveThenForwardMissionNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
