#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float64MultiArray
from std_srvs.srv import Trigger
from xy_translation_control_interfaces.action import MoveToPoint


class WaypointTargetPublisher(Node):
    """
    Action server that publishes bottom-camera XY PID setpoints.

    The node does not use vehicle feedback. A goal succeeds when the generated
    setpoint reaches the requested coordinate at the requested speed.
    """

    def __init__(self):
        super().__init__("waypoint_target_publisher")

        self.declare_parameter("action_name", "control/targets/move_to_point")
        self.declare_parameter("target_topic", "control/targets/bottom_camera_point_px")
        self.declare_parameter("target_x_topic", "control/pid/bottom_camera/x/reference_px")
        self.declare_parameter("target_y_topic", "control/pid/bottom_camera/y/reference_px")
        self.declare_parameter("done_topic", "control/targets/done")
        self.declare_parameter("default_speed_px_s", 20.0)
        self.declare_parameter("publish_period", 0.1)
        self.declare_parameter("initial_x_px", 0.0)
        self.declare_parameter("initial_y_px", 0.0)
        self.declare_parameter("publish_initial_target", True)

        self.action_name = self.get_parameter("action_name").value
        self.target_topic = self.get_parameter("target_topic").value
        self.target_x_topic = self.get_parameter("target_x_topic").value
        self.target_y_topic = self.get_parameter("target_y_topic").value
        self.done_topic = self.get_parameter("done_topic").value
        self.default_speed_px_s = float(self.get_parameter("default_speed_px_s").value)
        self.publish_period = float(self.get_parameter("publish_period").value)
        if self.publish_period <= 0.0:
            self.get_logger().warn("publish_period <= 0; using 0.1 s.")
            self.publish_period = 0.1
        self._initial_point = (
            float(self.get_parameter("initial_x_px").value),
            float(self.get_parameter("initial_y_px").value),
        )
        self._current_point = self._initial_point
        self.publish_initial_target = bool(self.get_parameter("publish_initial_target").value)

        self.pub_target = self.create_publisher(Float64MultiArray, self.target_topic, 10)
        self.pub_target_x = self.create_publisher(Float64, self.target_x_topic, 10)
        self.pub_target_y = self.create_publisher(Float64, self.target_y_topic, 10)
        self.pub_done = self.create_publisher(Bool, self.done_topic, 10)
        self._reset_service = self.create_service(
            Trigger,
            "control/targets/reset_move_to_point",
            self._on_reset,
        )

        self._active_goal = None
        self._action_server = ActionServer(
            self,
            MoveToPoint,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self._publish_done(True)
        if self.publish_initial_target:
            self._publish_target_point(*self._current_point, log=True)

        self.get_logger().info(
            f"action : {self.action_name}\n"
            f"target : {self.target_topic}\n"
            f"target_x: {self.target_x_topic}\n"
            f"target_y: {self.target_y_topic}\n"
            f"done   : {self.done_topic}\n"
            f"default_speed_px_s: {self.default_speed_px_s}\n"
            f"publish_period: {self.publish_period}"
        )

    def _goal_callback(self, goal_request: MoveToPoint.Goal):
        values = (
            goal_request.x_px,
            goal_request.y_px,
            goal_request.speed_px_s,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().warn("Rejected move_to_point goal with non-finite value.")
            return GoalResponse.REJECT

        if float(goal_request.speed_px_s) < 0.0:
            self.get_logger().warn("Rejected move_to_point goal with speed < 0.")
            return GoalResponse.REJECT

        speed = self._resolve_speed(goal_request.speed_px_s)
        if speed <= 0.0:
            self.get_logger().warn("Rejected move_to_point goal with speed <= 0.")
            return GoalResponse.REJECT

        if self._active_goal is not None and self._active_goal.is_active:
            self.get_logger().warn("Rejected move_to_point goal because another goal is active.")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _on_reset(self, request, response):
        if self._active_goal is not None and self._active_goal.is_active:
            response.success = False
            response.message = "Cannot reset while a MoveToPoint goal is active."
            return response

        self._current_point = self._initial_point
        self._publish_target_point(*self._current_point, log=True)
        self._publish_done(True)
        response.success = True
        response.message = "MoveToPoint setpoint reset."
        return response

    def _execute_callback(self, goal_handle):
        self._active_goal = goal_handle
        goal = goal_handle.request
        target = (float(goal.x_px), float(goal.y_px))
        speed = self._resolve_speed(goal.speed_px_s)
        start = self._current_point
        total_distance = self._distance(start, target)

        self.get_logger().info(
            f"MoveToPoint goal accepted: target=({target[0]:.3f}, {target[1]:.3f}), "
            f"speed={speed:.3f} px/s"
        )
        self._publish_done(False)

        result = MoveToPoint.Result()
        if total_distance < 1e-9:
            self._publish_target_point(*target, log=True)
            goal_handle.succeed()
            self._publish_done(True)
            result.success = True
            result.message = "Setpoint already at target."
            self._active_goal = None
            return result

        remaining_distance = total_distance
        feedback = MoveToPoint.Feedback()

        while rclpy.ok() and goal_handle.is_active:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._publish_done(True)
                result.success = False
                result.message = "MoveToPoint goal canceled."
                self.get_logger().info(result.message)
                self._active_goal = None
                return result

            step = speed * self.publish_period
            cx, cy = self._current_point
            dx = target[0] - cx
            dy = target[1] - cy
            remaining_distance = math.hypot(dx, dy)

            if remaining_distance <= step or remaining_distance < 1e-9:
                self._current_point = target
                remaining_distance = 0.0
            else:
                scale = step / remaining_distance
                self._current_point = (cx + dx * scale, cy + dy * scale)
                remaining_distance -= step

            self._publish_target_point(*self._current_point)
            feedback.target_x_px = float(self._current_point[0])
            feedback.target_y_px = float(self._current_point[1])
            feedback.remaining_distance_px = float(remaining_distance)
            feedback.progress = float(1.0 - remaining_distance / total_distance)
            goal_handle.publish_feedback(feedback)

            if remaining_distance <= 0.0:
                goal_handle.succeed()
                self._publish_done(True)
                result.success = True
                result.message = "Generated setpoint reached target."
                self.get_logger().info(result.message)
                self._active_goal = None
                return result

            time.sleep(self.publish_period)

        if goal_handle.is_active:
            goal_handle.abort()

        self._publish_done(True)
        result.success = False
        result.message = "MoveToPoint aborted because ROS is shutting down."
        self._active_goal = None
        return result

    def _resolve_speed(self, requested_speed: float) -> float:
        requested_speed = float(requested_speed)
        if requested_speed > 0.0:
            return requested_speed
        return self.default_speed_px_s

    def _publish_done(self, done: bool):
        msg = Bool()
        msg.data = bool(done)
        self.pub_done.publish(msg)

    def _publish_target_point(self, tx: float, ty: float, log: bool = False):
        msg = Float64MultiArray()
        msg.data = [float(tx), float(ty)]
        self.pub_target.publish(msg)
        self._publish_float(self.pub_target_x, tx)
        self._publish_float(self.pub_target_y, ty)
        if log:
            self.get_logger().info(f"[target] ({tx:.3f}, {ty:.3f})")

    @staticmethod
    def _publish_float(pub, value: float):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    @staticmethod
    def _distance(a, b) -> float:
        return math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))


def main():
    rclpy.init()
    node = WaypointTargetPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
