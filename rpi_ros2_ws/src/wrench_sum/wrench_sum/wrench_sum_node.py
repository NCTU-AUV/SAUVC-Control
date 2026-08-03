import rclpy
from geometry_msgs.msg import Wrench
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
import numpy as np


class WrenchSum(LifecycleNode):
    def __init__(self):
        super().__init__('wrench_sum_node')

        # --- 1. Declare parameters ---
        # Default input topic list (modify in launch file)
        # 節點預設值只是保底，實際清單一律由 launch 指定（見 orca_bringup）。
        self.declare_parameter('input_topics', [
            'control/wrench_sources/gui',
            'control/wrench_sources/depth',
            'control/wrench_sources/decision',
        ])
        # Output topic name
        self.declare_parameter('output_topic', 'control/wrench_command')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('source_timeout_s', 0.5)

        # Get parameter values
        self.input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.source_timeout_s = float(self.get_parameter('source_timeout_s').value)
        if publish_rate <= 0.0:
            self.get_logger().warn('publish_rate <= 0; using 30.0 Hz')
            publish_rate = 30.0

        # --- 2. Initialize storage ---
        # Use Dictionary to store the latest data for each topic
        # Key: topic_name, Value: np.array([fx, fy, fz, tx, ty, tz])
        self.wrench_buffer = {topic: np.zeros(6) for topic in self.input_topics}
        self.last_update_time = {topic: None for topic in self.input_topics}
        self.stale_topics = set()
        self.active = False

        # --- 3. Create Subscribers ---
        self.subs = []
        for topic in self.input_topics:
            # Use lambda to capture the current topic name
            self.subs.append(
                self.create_subscription(
                    Wrench,
                    topic,
                    lambda msg, t=topic: self.listener_callback(msg, t),
                    10
                )
            )
            self.get_logger().info(f'Subscribed to: {topic}')

        # --- 4. Create Publisher ---
        self.publisher_ = self.create_publisher(Wrench, output_topic, 10)
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.publish_sum)

    def on_configure(self, state) -> TransitionCallbackReturn:
        self._clear_sources()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self._clear_sources()
        self.active = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.active = False
        self._clear_sources()
        self._publish_wrench(np.zeros(6))
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.active = False
        self._clear_sources()
        self._publish_wrench(np.zeros(6))
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self.active = False
        self._clear_sources()
        self._publish_wrench(np.zeros(6))
        return TransitionCallbackReturn.SUCCESS

    def listener_callback(self, msg, topic_name):
        """
        Update Buffer value when receiving Wrench from any source, then publish the summed result.
        """
        # Apply NumPy format
        wrench_arr = np.array([
            msg.force.x,
            msg.force.y,
            msg.force.z,
            msg.torque.x,
            msg.torque.y,
            msg.torque.z
        ])

        # Update the latest value for this topic
        self.wrench_buffer[topic_name] = wrench_arr
        self.last_update_time[topic_name] = self.get_clock().now()
        self.stale_topics.discard(topic_name)
        self.publish_sum()

    def publish_sum(self):
        if not self.active:
            return

        active_wrenches = []
        now = self.get_clock().now()
        for topic, wrench in self.wrench_buffer.items():
            if self.last_update_time[topic] is None:
                active_wrenches.append(np.zeros(6))
                continue
            if self._is_source_stale(topic, now):
                if topic not in self.stale_topics:
                    self.get_logger().warn(
                        f'Wrench source stale; zeroing residual output: {topic}',
                        throttle_duration_sec=5.0,
                    )
                    self.stale_topics.add(topic)
                active_wrenches.append(np.zeros(6))
                continue
            active_wrenches.append(wrench)

        # Sum only currently active source values. Sources that never published
        # or timed out contribute zero instead of retaining old force commands.
        net_wrench_arr = sum(active_wrenches, np.zeros(6))
        self._publish_wrench(net_wrench_arr)

    def _publish_wrench(self, wrench_arr):
        msg_out = Wrench()
        msg_out.force.x = float(wrench_arr[0])
        msg_out.force.y = float(wrench_arr[1])
        msg_out.force.z = float(wrench_arr[2])
        msg_out.torque.x = float(wrench_arr[3])
        msg_out.torque.y = float(wrench_arr[4])
        msg_out.torque.z = float(wrench_arr[5])

        self.publisher_.publish(msg_out)

    def _clear_sources(self):
        self.wrench_buffer = {topic: np.zeros(6) for topic in self.input_topics}
        self.last_update_time = {topic: None for topic in self.input_topics}
        self.stale_topics.clear()

    def _is_source_stale(self, topic_name, now):
        last_update = self.last_update_time[topic_name]
        if last_update is None:
            return True
        if self.source_timeout_s <= 0.0:
            return False
        age_s = (now - last_update).nanoseconds / 1e9
        return age_s > self.source_timeout_s


def main(args=None):
    rclpy.init(args=args)
    node = WrenchSum()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
