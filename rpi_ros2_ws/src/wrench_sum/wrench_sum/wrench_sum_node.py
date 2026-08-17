import rclpy
from geometry_msgs.msg import Wrench
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String
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

        # 需要模式許可才能進總和的來源。
        #
        # supervisor 的 AUTONOMOUS 一直宣稱自己「放行」決策來源，但那個放行
        # 從來沒有實作：autonomous 群組沒有 lifecycle 節點可以啟停，所以它只
        # 改得動 system_manager/mode 這個字串，改不動任何實際的資料流。而
        # _set_mode(MANUAL) 會 _activate_wrench_sum()，於是 MANUAL 底下
        # 決策來源照樣被加總 —— 操作者手動前進的同時，行為樹的偏航力矩也在
        # 推同一台載具，兩者都「正常運作」而沒有任何錯誤訊息。
        #
        # 這裡是唯一能修的位置：所有來源在這裡匯流，也只有這裡知道總和。
        self.declare_parameter('gated_topics', [
            'control/wrench_sources/decision',
        ])
        # 閘門開啟的模式。字串必須與 system_manager/control_mode.py 的
        # ControlMode 值一致。
        self.declare_parameter('gate_open_modes', [
            'AUTONOMOUS',
            'AUTONOMOUS_AND_DEPTH_HOLD',
        ])
        self.declare_parameter('mode_topic', 'system_manager/mode')
        # 模式訊息本身也會過期。supervisor 以 5 Hz 發布，斷掉代表它已經不在
        # 線上 —— 此時繼續沿用最後一次看到的 AUTONOMOUS 等於在沒有 supervisor
        # 監督的情況下放行自主指令，所以逾時要關閘門而不是維持原狀。
        self.declare_parameter('mode_timeout_s', 1.0)

        # Get parameter values
        self.input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.source_timeout_s = float(self.get_parameter('source_timeout_s').value)
        self.gated_topics = set(
            self.get_parameter('gated_topics').get_parameter_value().string_array_value)
        self.gate_open_modes = set(
            self.get_parameter('gate_open_modes').get_parameter_value().string_array_value)
        mode_topic = self.get_parameter('mode_topic').get_parameter_value().string_value
        self.mode_timeout_s = float(self.get_parameter('mode_timeout_s').value)
        if publish_rate <= 0.0:
            self.get_logger().warn('publish_rate <= 0; using 30.0 Hz')
            publish_rate = 30.0

        # 打錯字的後果是「閘門靜默地不存在」，而不是任何可見的錯誤 —— 正是
        # 這個 bug 原本的樣子，所以寧可吵一點。
        unknown_gated = self.gated_topics - set(self.input_topics)
        if unknown_gated:
            self.get_logger().error(
                f'gated_topics names sources that are not in input_topics: '
                f'{sorted(unknown_gated)}; those gates do nothing')

        # --- 2. Initialize storage ---
        # Use Dictionary to store the latest data for each topic
        # Key: topic_name, Value: np.array([fx, fy, fz, tx, ty, tz])
        self.wrench_buffer = {topic: np.zeros(6) for topic in self.input_topics}
        self.last_update_time = {topic: None for topic in self.input_topics}
        self.stale_topics = set()
        self.active = False
        # 預設關閉。閘門只在確實收到一則開啟模式的訊息之後才打開 —— 節點比
        # supervisor 早起來是常態，而「還不知道模式」不能等於「放行自主」。
        self.current_mode = None
        self.last_mode_time = None
        self.gate_open = False

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

        if self.gated_topics:
            self.create_subscription(String, mode_topic, self._mode_callback, 10)
            self.get_logger().info(
                f'Gating {sorted(self.gated_topics)} on {mode_topic} '
                f'in {sorted(self.gate_open_modes)}')

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
        gate_open = self._is_gate_open(now)
        for topic, wrench in self.wrench_buffer.items():
            if topic in self.gated_topics and not gate_open:
                active_wrenches.append(np.zeros(6))
                continue
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

    def _mode_callback(self, msg):
        self.current_mode = msg.data
        self.last_mode_time = self.get_clock().now()

    def _is_gate_open(self, now):
        """Whether gated sources may contribute to the sum right now.

        Closed unless a fresh mode message names one of gate_open_modes. Both
        "no mode seen yet" and "mode went stale" close it: the gate exists to
        keep the autonomy stack off the wrench bus while the operator is
        driving, and neither of those states is evidence that autonomy was
        requested.
        """
        if not self.gated_topics:
            return True

        open_now = self.current_mode in self.gate_open_modes
        if open_now and self.mode_timeout_s > 0.0 and self.last_mode_time is not None:
            age_s = (now - self.last_mode_time).nanoseconds / 1e9
            if age_s > self.mode_timeout_s:
                open_now = False
                self.get_logger().warn(
                    f'{self.current_mode} last seen {age_s:.1f}s ago; '
                    'closing the gate on autonomy wrench sources',
                    throttle_duration_sec=5.0,
                )

        if open_now != self.gate_open:
            self.gate_open = open_now
            state = 'open' if open_now else 'closed'
            self.get_logger().info(
                f'Autonomy wrench gate {state} (mode: {self.current_mode})')
        return open_now

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
