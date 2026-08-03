"""把 6-DOF 的合力／合力矩解算成 8 顆推進器各自該出多少力。

推進器幾何來自參數（orca_bringup/config/hardware.yaml），不再硬編碼 ——
改推進器配置或換載具時只要動 YAML。幾何參考 https://hackmd.io/@NCTU-auv/HkBgyB4a3

飽和處理也在這裡，而不是只在 thruster_force_to_pwm_output_signal_node 裡：
模擬路徑刻意跳過 PWM 轉換節點（力直接進 ros_gz_bridge），限幅若只寫在那裡，
模擬就完全沒有飽和行為，調出來的增益搬到實機會對不上（見
docs/SIMULATION_FINDINGS.md §1.3）。放在分配層之後、兩條路徑的共同節點上，
模擬與實機才會有同一組飽和行為。PWM 節點自己的 clamp 保留為最後一道防線。
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from std_msgs.msg import Float64

THRUSTER_COUNT = 8

# 預設幾何：位置 (x, y, z) 公尺、推力方向單位向量，依推進器編號 0..7 排列。
_SQRT_HALF = float(np.cos(np.pi / 4))
DEFAULT_THRUSTER_POSITIONS_M = [
    0.12711, -0.25144, 0.0,   # 0  垂直
    0.12711, 0.25144, 0.0,    # 1  垂直
    -0.12711, -0.25144, 0.0,  # 2  垂直
    -0.12711, 0.25144, 0.0,   # 3  垂直
    0.32049, -0.2383, 0.0,    # 4  水平
    0.32049, 0.2383, 0.0,     # 5  水平
    -0.32049, -0.2383, 0.0,   # 6  水平
    -0.32049, 0.2383, 0.0,    # 7  水平
]
DEFAULT_THRUSTER_DIRECTIONS = [
    0.0, 0.0, 1.0,
    0.0, 0.0, 1.0,
    0.0, 0.0, 1.0,
    0.0, 0.0, 1.0,
    _SQRT_HALF, _SQRT_HALF, 0.0,
    _SQRT_HALF, -_SQRT_HALF, 0.0,
    _SQRT_HALF, -_SQRT_HALF, 0.0,
    _SQRT_HALF, _SQRT_HALF, 0.0,
]


class WrenchToIndividualThrusterOutputForcesNode(Node):

    def __init__(self):
        super().__init__('wrench_to_individual_thrusters_output_forces_node')

        positions_m = self._declare_geometry_parameter(
            'thruster_positions_m', DEFAULT_THRUSTER_POSITIONS_M)
        directions = self._declare_geometry_parameter(
            'thruster_directions', DEFAULT_THRUSTER_DIRECTIONS)

        # 單顆推進器的出力上限（牛頓）。<= 0 代表停用限幅。
        self._max_thruster_force_N = float(
            self.declare_parameter('max_thruster_force_N', 0.0).value)
        # 'scale'：任一顆超限時，全部等比例縮放，保留指令的方向，只是變慢。
        # 'clip' ：各自獨立截斷，會扭曲合力方向（載具往非預期方向偏）。
        self._saturation_mode = str(
            self.declare_parameter('saturation_mode', 'scale').value).lower()
        if self._saturation_mode not in ('scale', 'clip'):
            self.get_logger().warn(
                f"未知的 saturation_mode '{self._saturation_mode}'，改用 'scale'")
            self._saturation_mode = 'scale'

        self._output_force_allocation_matrix = self._create_allocation_matrix(
            positions_m, directions)

        self._saturated = False

        self._output_force_publishers = [
            self.create_publisher(Float64, f'thrusters/thruster_{n}/force_N', 10)
            for n in range(THRUSTER_COUNT)
        ]
        self._wrench_subscriber = self.create_subscription(
            Wrench, 'control/wrench_command', self._wrench_callback, 10)

    def _declare_geometry_parameter(self, name, default):
        values = list(self.declare_parameter(name, default).value)
        if len(values) != THRUSTER_COUNT * 3:
            self.get_logger().error(
                f'{name} 需要 {THRUSTER_COUNT * 3} 個值（每顆推進器 3 個），'
                f'實際收到 {len(values)} 個；改用預設幾何。'
            )
            values = list(default)
        return np.array(values, dtype=float).reshape(THRUSTER_COUNT, 3)

    @staticmethod
    def _create_allocation_matrix(positions_m, directions):
        # 每一欄是一顆推進器對 [Fx Fy Fz Tx Ty Tz] 的貢獻，取偽逆解回各顆出力。
        force_rows = directions.T
        torque_rows = np.column_stack([
            np.cross(positions_m[n], directions[n]) for n in range(THRUSTER_COUNT)
        ])
        return np.linalg.pinv(np.vstack((force_rows, torque_rows)))

    def _apply_saturation(self, output_forces_N):
        if self._max_thruster_force_N <= 0.0:
            return output_forces_N

        peak = float(np.max(np.abs(output_forces_N)))
        if peak <= self._max_thruster_force_N:
            if self._saturated:
                self._saturated = False
                self.get_logger().info('推力已回到限幅範圍內')
            return output_forces_N

        if not self._saturated:
            self._saturated = True
        self.get_logger().warn(
            f'推力飽和：最大 {peak:.1f} N > 上限 {self._max_thruster_force_N:.1f} N',
            throttle_duration_sec=2.0,
        )

        if self._saturation_mode == 'scale':
            return output_forces_N * (self._max_thruster_force_N / peak)
        return np.clip(
            output_forces_N, -self._max_thruster_force_N, self._max_thruster_force_N)

    def _wrench_callback(self, msg):
        wrench_N_Nm = np.array([
            msg.force.x, msg.force.y, msg.force.z,
            msg.torque.x, msg.torque.y, msg.torque.z,
        ])

        output_forces_N = self._apply_saturation(
            self._output_force_allocation_matrix @ wrench_N_Nm)

        for n in range(THRUSTER_COUNT):
            out = Float64()
            out.data = float(output_forces_N[n])
            self._output_force_publishers[n].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WrenchToIndividualThrusterOutputForcesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
