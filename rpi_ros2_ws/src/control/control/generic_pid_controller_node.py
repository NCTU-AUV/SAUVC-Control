import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from std_srvs.srv import Trigger

from std_msgs.msg import Float64


class GenericPIDControllerNode(LifecycleNode):

    def __init__(self):
        super().__init__('generic_pid_controller_node')

        self._reference_input_subscriber = self.create_subscription(
            Float64,
            'control/pid/reference',
            self._reference_input_subscription_callback,
            10)

        self._output_feedback_subscriber = self.create_subscription(
            Float64,
            'control/pid/feedback',
            self._output_feedback_subscription_callback,
            10)

        self._manipulated_variable_publisher = self.create_publisher(Float64, 'control/pid/output', 10)

        self.declare_parameter('controller_loop_timer_period_s', 1 / 100)
        self._controller_loop_timer_period_s = self.get_parameter('controller_loop_timer_period_s').get_parameter_value().double_value
        self._controller_loop_timer = self.create_timer(self._controller_loop_timer_period_s, self._controller_loop_timer_callback)

        self._active = False

        self.declare_parameter('proportional_gain', 0.0)
        self.declare_parameter('integral_gain', 0.0)
        self.declare_parameter('derivative_gain', 0.0)

        self.declare_parameter('derivative_smoothing_factor', 0.5)

        # 積分抗飽和與輸出限幅。兩者都是「<= 0 代表停用」。
        #
        # 為什麼需要：載具在水池觸底、卡住或浮力沒配平時，誤差會恆定不為零，
        # 積分項就無上界地線性成長。模擬實測過一次：目標深度設在池底以下，
        # 45 秒內下沉力從 32 N 一路爬到 119 N 而且還在爬（見
        # docs/SIMULATION_FINDINGS.md §1.2）。此時就算把目標改淺，也要花
        # 數十秒讓積分吐完才會反應，中間會劇烈上浮超調。
        #
        # integral_limit 限制的是「積分項的輸出貢獻」而不是內部累加值，
        # 這樣單位與 output_limit 一致（力，牛頓），調參時比較直觀；
        # 觸限時同步把內部累加值倒算回邊界，避免它在背景繼續長大。
        self.declare_parameter('integral_limit', 0.0)
        self.declare_parameter('output_limit', 0.0)

        self._reference_input = 0.0
        self._output_feedback = 0.0

        self._integral_controller_integrated_value = 0.0
        self._derivative_controller_previous_stored_value = 0.0

        self._reset_service = self.create_service(
            Trigger,
            f"{self.get_name()}/reset",
            self._on_reset,
        )

        self.add_on_set_parameters_callback(self._on_params)

    def on_configure(self, state) -> TransitionCallbackReturn:
        derivative_smoothing_factor = self.get_parameter(
            'derivative_smoothing_factor'
        ).get_parameter_value().double_value
        if not self._is_valid_derivative_smoothing_factor(derivative_smoothing_factor):
            self.get_logger().error(
                'derivative_smoothing_factor must be finite and between 0.0 and 1.0'
            )
            return TransitionCallbackReturn.FAILURE

        self._reset_state()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self._reset_state()
        self._active = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._active = False
        self._reset_state()
        self._publish_output(0.0)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self._active = False
        self._reset_state()
        self._publish_output(0.0)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self._active = False
        self._reset_state()
        self._publish_output(0.0)
        return TransitionCallbackReturn.SUCCESS

    def _reference_input_subscription_callback(self, msg):
        self._reference_input = msg.data

    def _output_feedback_subscription_callback(self, msg):
        self._output_feedback = msg.data

    def _on_params(self, params):
        for param in params:
            if param.name != 'derivative_smoothing_factor':
                continue

            try:
                value = float(param.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason='derivative_smoothing_factor must be a number',
                )

            if not self._is_valid_derivative_smoothing_factor(value):
                return SetParametersResult(
                    successful=False,
                    reason='derivative_smoothing_factor must be finite and between 0.0 and 1.0',
                )

        return SetParametersResult(successful=True)

    def _on_reset(self, request, response):
        self._reset_state()
        response.success = True
        response.message = "PID controller state reset"
        return response

    def _reset_state(self):
        self._integral_controller_integrated_value = 0.0
        self._derivative_controller_previous_stored_value = 0.0
        self._derivative_controller_new_stored_value = 0.0

    def _publish_output(self, value: float):
        output_msg = Float64()
        output_msg.data = float(value)
        self._manipulated_variable_publisher.publish(output_msg)

    def _controller_loop_timer_callback(self):
        if not self._active:
            return

        error = self._reference_input - self._output_feedback

        proportional_gain = self.get_parameter('proportional_gain').get_parameter_value().double_value
        proportional_output = proportional_gain * error

        self._integral_controller_integrated_value += error * self._controller_loop_timer_period_s

        integral_gain = self.get_parameter('integral_gain').get_parameter_value().double_value
        integral_output = integral_gain * self._integral_controller_integrated_value

        integral_limit = self.get_parameter('integral_limit').get_parameter_value().double_value
        if integral_limit > 0.0 and abs(integral_output) > integral_limit:
            integral_output = math.copysign(integral_limit, integral_output)
            # 把內部累加值倒算回邊界，否則它會在背景無上界地繼續成長，
            # 誤差反向後要花很久才回得來（積分飽和）。
            if integral_gain != 0.0:
                self._integral_controller_integrated_value = integral_output / integral_gain

        derivative_gain = self.get_parameter('derivative_gain').get_parameter_value().double_value
        derivative_smoothing_factor = self.get_parameter('derivative_smoothing_factor').get_parameter_value().double_value

        self._derivative_controller_new_stored_value = (
            (1 - derivative_smoothing_factor ** self._controller_loop_timer_period_s) * error
            + derivative_smoothing_factor ** self._controller_loop_timer_period_s * self._derivative_controller_previous_stored_value
        )

        derivative_output = derivative_gain * (
            (self._derivative_controller_new_stored_value - self._derivative_controller_previous_stored_value)
            / self._controller_loop_timer_period_s
        )

        self._derivative_controller_previous_stored_value = self._derivative_controller_new_stored_value

        total_output = proportional_output + integral_output + derivative_output

        output_limit = self.get_parameter('output_limit').get_parameter_value().double_value
        if output_limit > 0.0 and abs(total_output) > output_limit:
            total_output = math.copysign(output_limit, total_output)

        self._publish_output(total_output)

    @staticmethod
    def _is_valid_derivative_smoothing_factor(value: float) -> bool:
        return math.isfinite(value) and 0.0 <= value <= 1.0


def main(args=None):
    rclpy.init(args=args)

    generic_pid_controller_node = GenericPIDControllerNode()

    rclpy.spin(generic_pid_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    generic_pid_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
