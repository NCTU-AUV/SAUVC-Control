import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class YawReferenceUnwrapperNode(Node):
    """Publish a yaw PID reference equivalent to the target but near current yaw."""

    def __init__(self):
        super().__init__("yaw_reference_unwrapper_node")

        self.declare_parameter("target_topic", "control/targets/bottom_camera/yaw_rad")
        self.declare_parameter("feedback_topic", "state/bottom_camera/yaw_rad")
        self.declare_parameter("output_topic", "control/pid/bottom_camera/yaw/reference_rad")

        self.target_topic = self.get_parameter("target_topic").value
        self.feedback_topic = self.get_parameter("feedback_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self._target_yaw = 0.0
        self._feedback_yaw = 0.0
        self._have_target = False
        self._have_feedback = False

        self.pub_reference = self.create_publisher(Float64, self.output_topic, 10)
        self.sub_target = self.create_subscription(
            Float64,
            self.target_topic,
            self._on_target,
            10,
        )
        self.sub_feedback = self.create_subscription(
            Float64,
            self.feedback_topic,
            self._on_feedback,
            10,
        )

        self.get_logger().info(
            f"target  : {self.target_topic}\n"
            f"feedback: {self.feedback_topic}\n"
            f"output  : {self.output_topic}"
        )

    def _on_target(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self._target_yaw = float(msg.data)
        self._have_target = True
        self._publish_reference()

    def _on_feedback(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self._feedback_yaw = float(msg.data)
        self._have_feedback = True
        self._publish_reference()

    def _publish_reference(self):
        if not self._have_target:
            return

        if self._have_feedback:
            error = self._wrap_to_pi(self._target_yaw - self._feedback_yaw)
            reference = self._feedback_yaw + error
        else:
            reference = self._target_yaw

        msg = Float64()
        msg.data = float(reference)
        self.pub_reference.publish(msg)

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = YawReferenceUnwrapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
