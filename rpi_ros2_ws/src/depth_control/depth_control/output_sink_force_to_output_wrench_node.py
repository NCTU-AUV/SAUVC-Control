import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion as QuaternionMsg
from geometry_msgs.msg import Wrench

from .math_utility.quaternion import Quaternion


class OutputSinkForceToOutputWrenchNode(Node):

    def __init__(self):
        super().__init__('output_sink_force_to_output_wrench_node')

        self._output_sink_force_subscriber = self.create_subscription(
            Float64,
            'control/pid/depth/sink_force_N',
            self._output_sink_force_subscription_callback,
            10)

        self._set_output_wrench_at_center_publisher = self.create_publisher(Wrench, 'control/wrench_command', 10)

        self._orientation_subscriber = self.create_subscription(
            QuaternionMsg,
            'state/orientation',
            self._orientation_subscription_callback,
            10)

        self._orientation_quaternion = Quaternion(1, 0, 0, 0)
        self.declare_parameter('use_sink_force_direction', False)
        self.declare_parameter('depth_force_bias_N', 5.0)
        # 垂直軸能出的總力上限（4 顆 × max_thruster_force_N）。PID 的
        # output_limit 只箝住 PID 自己的輸出，偏壓是在這裡才加上去的，
        # 所以真正送上匯流排的 force.z 必須在這裡再箝一次 —— 否則分配層
        # 會算出超過單顆上限的推力，而 saturation_mode: scale 會把**八顆**
        # 推進器一起等比例縮小，自主端的前進與轉向被靜默削弱而毫無回報。
        self.declare_parameter('max_sink_force_N', 60.0)

    def _orientation_subscription_callback(self, msg):
        self._orientation_quaternion = Quaternion(msg.w, msg.x, msg.y, msg.z)

    def _get_sink_force_direction_unit_vector(self):
        world_frame_sink_direction = Quaternion(0, 0, 0, 1)
        vehicle_frame_sink_direction = self._orientation_quaternion.inverse * world_frame_sink_direction * self._orientation_quaternion
        return vehicle_frame_sink_direction.vector_part

    def _output_sink_force_subscription_callback(self, msg):
        pid_sink_force_N = msg.data
        depth_force_bias_N = self.get_parameter('depth_force_bias_N').get_parameter_value().double_value
        output_sink_force_N = pid_sink_force_N + depth_force_bias_N

        max_sink_force_N = self.get_parameter('max_sink_force_N').get_parameter_value().double_value
        if max_sink_force_N > 0.0:
            clamped = max(-max_sink_force_N, min(max_sink_force_N, output_sink_force_N))
            if clamped != output_sink_force_N:
                self.get_logger().warning(
                    f'深度輸出 {output_sink_force_N:.1f} N 超過垂直軸上限 '
                    f'{max_sink_force_N:.1f} N，已箝制',
                    throttle_duration_sec=2.0)
            output_sink_force_N = clamped

        use_sink_force_direction = self.get_parameter('use_sink_force_direction').get_parameter_value().bool_value

        msg = Wrench()

        if use_sink_force_direction:
            sink_force_direction_unit_vector = self._get_sink_force_direction_unit_vector()
            msg.force.x = output_sink_force_N * sink_force_direction_unit_vector.x
            msg.force.y = output_sink_force_N * sink_force_direction_unit_vector.y
            msg.force.z = output_sink_force_N * sink_force_direction_unit_vector.z
        else:
            msg.force.x = 0.0
            msg.force.y = 0.0
            msg.force.z = output_sink_force_N
        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = 0.0

        self._set_output_wrench_at_center_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    output_sink_force_to_output_wrench_node = OutputSinkForceToOutputWrenchNode()

    rclpy.spin(output_sink_force_to_output_wrench_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    output_sink_force_to_output_wrench_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
