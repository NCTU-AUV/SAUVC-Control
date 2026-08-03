import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion



class IMUToOrientationNode(Node):

    def __init__(self):
        super().__init__('imu_to_orientation_node')

        self._imu_subscriber = self.create_subscription(
            Imu,
            'sensors/imu',
            self._imu_subscription_callback,
            10)

        self._orientation_publisher = self.create_publisher(Quaternion, 'state/orientation', 10)

    def _imu_subscription_callback(self, msg):
        output_msg = Quaternion()

        output_msg.x = msg.orientation.x
        output_msg.y = msg.orientation.y
        output_msg.z = msg.orientation.z
        output_msg.w = msg.orientation.w

        self._orientation_publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)

    imu_to_orientation_node = IMUToOrientationNode()

    rclpy.spin(imu_to_orientation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_to_orientation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
