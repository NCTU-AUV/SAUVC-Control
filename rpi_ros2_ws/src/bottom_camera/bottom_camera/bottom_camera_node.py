import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import glob
import os


class BottomCameraNode(Node):

    def __init__(self):
        super().__init__('bottom_camera_node')

        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('frame_rate', 15.0)
        self.declare_parameter('image_width', 160)
        self.declare_parameter('image_height', 120)
        self.declare_parameter('frame_id', 'bottom_camera_optical_frame')

        self._bridge = CvBridge()
        self._frame_id = self.get_parameter('frame_id').value

        camera_device = self.get_parameter('camera_device').value
        frame_rate = self.get_parameter('frame_rate').value
        image_width = int(self.get_parameter('image_width').value)
        image_height = int(self.get_parameter('image_height').value)

        self._capture = None
        self._active_camera_device = None
        self._open_camera_with_fallback(camera_device, image_width, image_height)

        period_seconds = 1.0 / frame_rate if frame_rate > 0 else 1.0 / 15.0
        self._publisher = self.create_publisher(Image, 'camera/bottom/image_raw', 10)
        self._timer = self.create_timer(period_seconds, self._publish_frame)

    def _open_camera_with_fallback(self, requested_device, image_width, image_height):
        candidate_devices = [requested_device]
        detected_devices = sorted(glob.glob('/dev/video*'))

        for device in detected_devices:
            if device not in candidate_devices:
                candidate_devices.append(device)

        for device in candidate_devices:
            if not os.path.exists(device):
                continue

            capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
            if capture.isOpened():
                self._capture = capture
                self._active_camera_device = device
                if device != requested_device:
                    self.get_logger().warn(
                        f'Configured camera device {requested_device} is unavailable; using {device} instead'
                    )
                else:
                    self.get_logger().info(f'Opened camera device: {device}')
                return

            capture.release()

        self.get_logger().error(
            f'Failed to open camera device {requested_device}. Detected devices: {detected_devices}'
        )

    def _publish_frame(self):
        if self._capture is None or not self._capture.isOpened():
            self.get_logger().warn(
                'Camera device is not opened; skipping frame capture',
                throttle_duration_sec=5.0
            )
            return

        success, frame = self._capture.read()
        if not success:
            self.get_logger().warn(
                'Failed to read frame from bottom camera',
                throttle_duration_sec=5.0
            )
            return

        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._publisher.publish(msg)

    def destroy_node(self):
        if self._capture is not None and self._capture.isOpened():
            self._capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BottomCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
