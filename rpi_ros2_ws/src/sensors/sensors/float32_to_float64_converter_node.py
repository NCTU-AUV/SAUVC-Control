import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import Float64


class Float32ToFloat64ConverterNode(Node):

    def __init__(self):
        super().__init__('float32_to_float64_converter_node')

        self._float32_topic_subscriber = self.create_subscription(
            Float32,
            'converters/float32_input',
            self._float32_topic_subscription_callback,
            10)

        self._float64_topic_publisher = self.create_publisher(Float64, 'converters/float64_output', 10)

    def _float32_topic_subscription_callback(self, msg):
        output_msg = Float64()
        output_msg.data = msg.data
        self._float64_topic_publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)

    float32_to_float64_converter_node = Float32ToFloat64ConverterNode()

    rclpy.spin(float32_to_float64_converter_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    float32_to_float64_converter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
