import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench


def _clamp_symmetric(value: float, limit: float) -> float:
    if limit <= 0.0:
        return value
    return min(limit, max(-limit, value))


class BottomCameraPIDBridgeNode(Node):
    """
    Combine bottom-camera X/Y/yaw PID outputs into a body-frame wrench.

    Responsibilities:
      - Collect per-axis PID outputs and rotate world-frame forces into the
        AUV body frame using yaw.
      - Publish the combined force/torque as a geometry_msgs/Wrench.
    """

    def __init__(self):
        super().__init__("bottom_camera_pid_bridge_node")

        # Input/output topics. X/Y PID controllers consume reference/feedback directly.
        self.declare_parameter("yaw_topic", "state/bottom_camera/yaw_rad")
        self.declare_parameter("output_topic", "control/wrench_command")
        self.declare_parameter("x_manipulated_topic", "control/pid/bottom_camera/x/force_world_N")
        self.declare_parameter("y_manipulated_topic", "control/pid/bottom_camera/y/force_world_N")
        self.declare_parameter("yaw_manipulated_topic", "control/pid/bottom_camera/yaw/torque_Nm")
        self.declare_parameter("max_force_x_N", 10.0)
        self.declare_parameter("max_force_y_N", 10.0)
        self.declare_parameter("max_yaw_torque_Nm", 5.0)

        self.yaw_topic = self.get_parameter("yaw_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.x_manipulated_topic = self.get_parameter("x_manipulated_topic").value
        self.y_manipulated_topic = self.get_parameter("y_manipulated_topic").value
        self.yaw_manipulated_topic = self.get_parameter("yaw_manipulated_topic").value
        self.max_force_x_N = float(self.get_parameter("max_force_x_N").value)
        self.max_force_y_N = float(self.get_parameter("max_force_y_N").value)
        self.max_yaw_torque_Nm = float(self.get_parameter("max_yaw_torque_Nm").value)

        # State
        self._yaw = 0.0
        self._have_orientation = False

        self._x_force_world = 0.0
        self._y_force_world = 0.0
        self._yaw_torque = 0.0
        self._have_x_force = False
        self._have_y_force = False
        self._have_yaw_torque = False

        # Pub/Sub to consume PID outputs and publish the resulting wrench.
        self.pub_wrench = self.create_publisher(Wrench, self.output_topic, 10)

        self.sub_yaw = self.create_subscription(
            Float64,
            self.yaw_topic,
            self._on_yaw,
            10,
        )
        self.sub_x_manipulated = self.create_subscription(
            Float64,
            self.x_manipulated_topic,
            self._on_x_output,
            10,
        )
        self.sub_y_manipulated = self.create_subscription(
            Float64,
            self.y_manipulated_topic,
            self._on_y_output,
            10,
        )
        self.sub_yaw_manipulated = self.create_subscription(
            Float64,
            self.yaw_manipulated_topic,
            self._on_yaw_output,
            10,
        )

        self.get_logger().info(
            f"yaw    : {self.yaw_topic}\n"
            f"output : {self.output_topic}\n"
            f"x_out : {self.x_manipulated_topic} y_out: {self.y_manipulated_topic}\n"
            f"yaw_out: {self.yaw_manipulated_topic}"
        )

    def _on_yaw(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self._yaw = float(msg.data)
        self._have_orientation = True
        self._maybe_publish_wrench()

    def _on_x_output(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self._x_force_world = float(msg.data)
        self._have_x_force = True
        self._maybe_publish_wrench()

    def _on_y_output(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self._y_force_world = float(msg.data)
        self._have_y_force = True
        self._maybe_publish_wrench()

    def _on_yaw_output(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self._yaw_torque = float(msg.data)
        self._have_yaw_torque = True
        self._maybe_publish_wrench()

    def _maybe_publish_wrench(self):
        if not (self._have_x_force or self._have_y_force or self._have_yaw_torque):
            return

        fx_world = self._x_force_world if self._have_x_force else 0.0
        fy_world = self._y_force_world if self._have_y_force else 0.0
        yaw_torque = self._yaw_torque if self._have_yaw_torque else 0.0
        fx_body, fy_body = self._rotate_world_to_body(fx_world, fy_world)
        fx_body = _clamp_symmetric(fx_body, self.max_force_x_N)
        fy_body = _clamp_symmetric(fy_body, self.max_force_y_N)
        yaw_torque = _clamp_symmetric(yaw_torque, self.max_yaw_torque_Nm)

        w = Wrench()
        w.force.x = float(fx_body)
        w.force.y = float(fy_body)
        w.force.z = 0.0
        w.torque.x = 0.0
        w.torque.y = 0.0
        w.torque.z = float(yaw_torque)
        self.pub_wrench.publish(w)

    def _rotate_world_to_body(self, fx_world: float, fy_world: float):
        if self._have_orientation:
            cos_yaw = math.cos(-self._yaw)
            sin_yaw = math.sin(-self._yaw)
            fx = fx_world * cos_yaw - fy_world * sin_yaw
            fy = fx_world * sin_yaw + fy_world * cos_yaw
        else:
            fx = fx_world
            fy = fy_world
        return fx, fy


def main(args=None):
    rclpy.init(args=args)

    bridge_node = BottomCameraPIDBridgeNode()
    rclpy.spin(bridge_node)

    bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
