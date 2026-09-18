"""把 thrusters/pwm_us 轉成 MAVLink servo 指令，送給 ArduSub 飛控。

這個節點取代原本 STM32 子模組在鏈路上的位置：

    wrench → 每顆出力 → thrusters/pwm_us → 【本節點】→ MAVLink → ArduSub → ESC

上游完全不動。thruster_force_to_pwm_output_signal_node 與
thruster_initialization_node 發什麼、何時發，都與 STM32 時代一模一樣，
差別只在誰去把那串微秒值變成真的波形。這樣切的理由是 STM32 那條路徑
可以原封不動留著當 fallback —— 兩邊訂同一個 topic，要換回去只是不啟動本節點。

---- 飛控端必須先設好的參數（不設的話這個節點送什麼都沒有反應）----

1. SERVO1_FUNCTION ~ SERVO8_FUNCTION 全部設成 0（Disabled）。
   ArduPilot 只有在輸出通道是 Disabled / RCPassThru / RCIN1-16 時才吃
   MAV_CMD_DO_SET_SERVO；維持 ArduSub 預設的 Motor1..8（33~40）的話，
   飛控自己的推力分配每個迴圈都會把我們寫進去的值蓋掉，
   而且不會有任何錯誤訊息 —— 表現出來就是「推進器完全不動」。
   選 Disabled 而不是 RCPassThru，是因為後者會被遙控器輸入蓋掉。
   我們刻意不用 ArduSub 內建的 8 推進器 frame：載具幾何與分配矩陣的
   單一來源是 hardware.yaml，交給飛控等於維護兩份會漂移的幾何。

2. SERVO_RATE 設成 200 以上（預設 50 Hz）。
   STM32 時代是 333 Hz，維持 50 Hz 不會壞但推力響應會明顯變鈍。

3. MatekH743 沒有實體 safety switch，BRD_SAFETY_DEFLT 要是 0，
   否則輸出會被 safety 擋住。

---- 已知的風險與 plan B ----

每個週期最多送 8 筆 COMMAND_LONG，20 Hz 就是 160 筆/秒。頻寬不是問題
（USB CDC），但 ArduPilot 是在主迴圈裡逐筆處理指令並回 COMMAND_ACK，
這個速率已經超出一般用法。若實測發現飛控反應延遲或 ACK 大量掉，
plan B 是改用 RC_CHANNELS_OVERRIDE：一則訊息帶 8 個通道、不需要 ACK，
本來就是設計來串流的，代價是飛控端要改成 SERVOn_FUNCTION = RCIN1..8（51~58）
並處理 RC failsafe。送出的部分集中在 _send_pwm_output_signal_values()，
要換只動那一個方法。

---- 與 STM32 行為的差異 ----

STM32 的 thrusters/set_enabled 為 false 時是「完全停止輸出脈衝」。
DO_SET_SERVO 沒有對應的做法，所以這裡改成輸出中位值（1500 µs）。
對 BlueRobotics 的 ESC 來說持續中位值正是解鎖條件，ESC 初始化流程
照樣會過，但要知道這兩者不等價：停止脈衝時 ESC 會進入失訊保護，
輸出中位值則是明確地叫它「停在原地」。
"""

import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray

from pymavlink import mavutil


class ThrusterPWMToMAVLinkServoOutputNode(Node):

    # 一次 drain 最多處理幾則收到的訊息。沒有上限的話，飛控的遙測串流
    # 可能讓這個迴圈永遠跑不完，把 executor 餓死。
    MAX_MESSAGES_PER_DRAIN = 200

    def __init__(self):
        super().__init__("thruster_pwm_to_mavlink_servo_output_node")

        self._thruster_count = 8

        # 推進器編號 → 飛控 servo 輸出編號。預設 0..7 對到 SERVO1..SERVO8，
        # 拉線順序不同時改這裡，不要去改上游的分配矩陣。
        self._servo_output_channels = [
            int(channel) for channel in self.declare_parameter(
                "servo_output_channels", [1, 2, 3, 4, 5, 6, 7, 8]
            ).value
        ]

        self._neutral_pwm_output_signal_value_us = int(
            self.declare_parameter("neutral_pwm_output_signal_value_us", 1500).value
        )
        # 送進飛控前的最後一道限幅，對應查表檔 thruster_lookup_table_16V.csv
        # 的 1100~1900。正常情況上游不會超出，這裡是防止上游出錯時
        # 直接把超範圍的值餵給 ESC。
        self._min_pwm_output_signal_value_us = int(
            self.declare_parameter("min_pwm_output_signal_value_us", 1100).value
        )
        self._max_pwm_output_signal_value_us = int(
            self.declare_parameter("max_pwm_output_signal_value_us", 1900).value
        )

        self._output_rate_hz = float(self.declare_parameter("output_rate_hz", 20.0).value)

        # 收不到 thrusters/pwm_us 超過這個時間就輸出中位值。
        # STM32 那條路徑沒有這個保護：RPi 掛掉或序列埠斷線時 CCR 會維持
        # 最後一次寫入的值，推進器就這樣一直轉下去，只剩實體 kill switch 能停。
        # 這不是 kill switch 的替代品，是補上那個缺口。<= 0 代表停用。
        self._command_timeout_s = float(self.declare_parameter("command_timeout_s", 0.5).value)

        # 值沒變就不重送，省掉大部分的 COMMAND_LONG。但每隔一段時間仍要
        # 全部重送一次，否則單一指令掉包會讓某顆推進器永遠停在錯的值上
        # （MAVLink 沒有重傳，COMMAND_ACK 我們也只拿來記錄）。
        self._full_refresh_interval_s = float(
            self.declare_parameter("full_refresh_interval_s", 1.0).value
        )
        self._heartbeat_rate_hz = float(self.declare_parameter("heartbeat_rate_hz", 1.0).value)

        self._mavlink_device = str(self.declare_parameter("mavlink_device", "/dev/ttyACM0").value)
        # USB CDC 是虛擬序列埠，baud 實際上不影響傳輸速率，但 pymavlink 要有值。
        self._mavlink_baud = int(self.declare_parameter("mavlink_baud", 115200).value)
        # 我們是 GCS 的身分。255 是 MAVLink 慣例上的地面站 system id。
        self._mavlink_source_system = int(self.declare_parameter("mavlink_source_system", 255).value)
        self._mavlink_source_component = int(
            self.declare_parameter("mavlink_source_component", 190).value
        )
        # 0 代表從飛控的 HEARTBEAT 自動學。寫死值只在同一條線上有多台
        # 裝置時才需要。
        self._target_system = int(self.declare_parameter("target_system", 0).value)
        self._target_component = int(
            self.declare_parameter("target_component", mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1).value
        )

        self._latest_pwm_output_signal_value_us = [
            self._neutral_pwm_output_signal_value_us for _ in range(self._thruster_count)
        ]
        self._last_sent_pwm_output_signal_value_us = [None for _ in range(self._thruster_count)]
        self._latest_command_receive_time_s = None
        self._last_full_refresh_time_s = 0.0
        self._is_output_enabled = True
        self._is_link_established = False
        self._has_warned_about_timeout = False

        self._connection = mavutil.mavlink_connection(
            self._mavlink_device,
            baud=self._mavlink_baud,
            source_system=self._mavlink_source_system,
            source_component=self._mavlink_source_component,
            autoreconnect=True,
        )
        self.get_logger().info(
            f"MAVLink 連線已開啟：{self._mavlink_device}，等待飛控 HEARTBEAT"
        )

        self._pwm_output_signal_value_subscription = self.create_subscription(
            msg_type=Int32MultiArray,
            topic="thrusters/pwm_us",
            callback=self._pwm_output_signal_value_subscription_callback,
            qos_profile=10
        )

        self._set_enabled_subscription = self.create_subscription(
            msg_type=Bool,
            topic="thrusters/set_enabled",
            callback=self._set_enabled_subscription_callback,
            qos_profile=10
        )

        self._receive_timer = self.create_timer(0.05, self._receive_timer_callback)
        self._output_timer = self.create_timer(
            1.0 / self._output_rate_hz, self._output_timer_callback
        )
        self._heartbeat_timer = self.create_timer(
            1.0 / self._heartbeat_rate_hz, self._heartbeat_timer_callback
        )

    # --- 時間 ----------------------------------------------------------------

    def _now_s(self):
        # 用 ROS 時鐘而不是 time.monotonic()，這樣 bag 回放與模擬時間也適用。
        return self.get_clock().now().nanoseconds / 1e9

    # --- 訂閱 ----------------------------------------------------------------

    def _pwm_output_signal_value_subscription_callback(self, msg: Int32MultiArray):
        received_values = list(msg.data)
        if len(received_values) < self._thruster_count:
            self.get_logger().warn(
                f"thrusters/pwm_us 只有 {len(received_values)} 筆，"
                f"不足 {self._thruster_count} 顆，缺的補中位值"
            )
            received_values += [self._neutral_pwm_output_signal_value_us] * (
                self._thruster_count - len(received_values)
            )

        self._latest_pwm_output_signal_value_us = [
            self._clamp_pwm_output_signal_value_us(value)
            for value in received_values[:self._thruster_count]
        ]
        self._latest_command_receive_time_s = self._now_s()
        self._has_warned_about_timeout = False

    def _set_enabled_subscription_callback(self, msg: Bool):
        self._is_output_enabled = bool(msg.data)

    def _clamp_pwm_output_signal_value_us(self, value):
        return int(min(
            self._max_pwm_output_signal_value_us,
            max(self._min_pwm_output_signal_value_us, int(value))
        ))

    # --- MAVLink 收 ----------------------------------------------------------

    def _receive_timer_callback(self):
        for _ in range(self.MAX_MESSAGES_PER_DRAIN):
            try:
                msg = self._connection.recv_match(blocking=False)
            except Exception as error:  # 序列埠被拔掉時 pymavlink 會丟各種例外
                self.get_logger().error(f"MAVLink 讀取失敗：{error}")
                return
            if msg is None:
                return

            message_type = msg.get_type()
            if message_type == "BAD_DATA":
                continue

            if message_type == "HEARTBEAT":
                self._handle_heartbeat(msg)
            elif message_type == "COMMAND_ACK":
                self._handle_command_ack(msg)
            elif message_type == "STATUSTEXT":
                self.get_logger().info(f"飛控訊息：{msg.text}")

    def _handle_heartbeat(self, msg):
        source_system = msg.get_srcSystem()
        if source_system == self._mavlink_source_system:
            return  # 自己發的，忽略

        if self._target_system == 0:
            self._target_system = source_system
            self.get_logger().info(
                f"偵測到飛控 system {self._target_system} component {msg.get_srcComponent()}"
            )

        if not self._is_link_established:
            self._is_link_established = True
            self.get_logger().info("MAVLink 連線建立，開始送 servo 指令")
            # 重新連上時清掉快取，強制下一輪整批重送。
            self._last_sent_pwm_output_signal_value_us = [
                None for _ in range(self._thruster_count)
            ]

    def _handle_command_ack(self, msg):
        if msg.command != mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
            return
        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return
        # 最常見的失敗是 MAV_RESULT_FAILED，成因幾乎都是
        # SERVOn_FUNCTION 沒設成 Disabled（飛控會回 "Channel x is already in use"）。
        self.get_logger().warn(
            f"DO_SET_SERVO 被拒絕，result={msg.result}。"
            "請確認飛控的 SERVOn_FUNCTION 已設為 0 (Disabled)",
            throttle_duration_sec=5.0,
        )

    # --- MAVLink 送 ----------------------------------------------------------

    def _heartbeat_timer_callback(self):
        # 定期送 GCS heartbeat，飛控才知道地面站還在。
        # 之後要靠飛控自己的 GCS failsafe 收油時，這個是前提。
        try:
            self._connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        except Exception as error:
            self.get_logger().error(f"MAVLink heartbeat 送出失敗：{error}")

    def _output_timer_callback(self):
        if not self._is_link_established:
            return

        pwm_values = self._get_pwm_output_signal_values_to_send()

        now_s = self._now_s()
        is_full_refresh = (
            self._full_refresh_interval_s > 0.0
            and (now_s - self._last_full_refresh_time_s) >= self._full_refresh_interval_s
        )
        if is_full_refresh:
            self._last_full_refresh_time_s = now_s

        self._send_pwm_output_signal_values(pwm_values, force_all=is_full_refresh)

    def _get_pwm_output_signal_values_to_send(self):
        neutral = [self._neutral_pwm_output_signal_value_us for _ in range(self._thruster_count)]

        if not self._is_output_enabled:
            return neutral

        if self._command_timeout_s > 0.0:
            if self._latest_command_receive_time_s is None:
                return neutral  # 還沒收過任何指令
            if (self._now_s() - self._latest_command_receive_time_s) > self._command_timeout_s:
                if not self._has_warned_about_timeout:
                    self._has_warned_about_timeout = True
                    self.get_logger().error(
                        f"超過 {self._command_timeout_s} s 沒收到 thrusters/pwm_us，"
                        "輸出切到中位值"
                    )
                return neutral

        return self._latest_pwm_output_signal_value_us

    def _send_pwm_output_signal_values(self, pwm_values, force_all=False):
        """把 8 筆微秒值送到飛控。要換成 RC_CHANNELS_OVERRIDE 只需改這裡。"""
        for thruster_number, pwm_us in enumerate(pwm_values):
            if not force_all and self._last_sent_pwm_output_signal_value_us[thruster_number] == pwm_us:
                continue

            try:
                self._connection.mav.command_long_send(
                    self._target_system,
                    self._target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0,                                                  # confirmation
                    float(self._servo_output_channels[thruster_number]),  # param1: servo 編號
                    float(pwm_us),                                      # param2: PWM 微秒
                    0.0, 0.0, 0.0, 0.0, 0.0,
                )
            except Exception as error:
                self.get_logger().error(f"MAVLink DO_SET_SERVO 送出失敗：{error}")
                self._is_link_established = False
                return

            self._last_sent_pwm_output_signal_value_us[thruster_number] = pwm_us


def main(args=None):
    rclpy.init(args=args)

    thruster_pwm_to_mavlink_servo_output_node = ThrusterPWMToMAVLinkServoOutputNode()
    rclpy.spin(thruster_pwm_to_mavlink_servo_output_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
