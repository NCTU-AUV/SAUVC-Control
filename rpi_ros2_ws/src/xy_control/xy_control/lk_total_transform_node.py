#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import Trigger


def _T(tx: float, ty: float) -> np.ndarray:
    """3x3 translation matrix."""
    return np.array([
        [1.0, 0.0, tx],
        [0.0, 1.0, ty],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


@dataclass
class TileLine:
    pos: float
    angle_offset: float
    weight: float


@dataclass
class TileLineDetection:
    horizontal: List[TileLine]
    vertical: List[TileLine]
    segments: np.ndarray
    horizontal_normal: np.ndarray
    vertical_normal: np.ndarray
    horizontal_dir: float


@dataclass
class LineMatch:
    track: 'LineTrack'
    observation: TileLine
    observation_idx: int
    pos_delta: float
    angle_delta: float


@dataclass
class LineTrack:
    pos: float
    angle_offset: float
    confidence: float
    missed_count: int = 0
    age: int = 1
    seen_count: int = 1
    confirmed: bool = False


@dataclass
class VisualMotion:
    tx: float
    ty: float
    rot: float
    scale: float
    inliers: int


class LkTotalTransformNode(Node):
    """Track bottom-camera tile-line motion and publish accumulated pose."""

    def __init__(self):
        super().__init__('lk_total_transform_node')

        # ---- Params (topics) ----
        self.declare_parameter('image_topic', 'camera/bottom/image_raw')
        self.declare_parameter('output_topic', 'camera/bottom/pose_px')
        self.declare_parameter('output_x_topic', 'control/pid/bottom_camera/x/feedback_px')
        self.declare_parameter('output_y_topic', 'control/pid/bottom_camera/y/feedback_px')
        self.declare_parameter('output_yaw_topic', 'state/bottom_camera/yaw_rad')
        self.declare_parameter('output_scale_topic', 'state/bottom_camera/scale')

        # raw output (debug / diagnostics)
        self.declare_parameter('publish_raw_output', True)
        self.declare_parameter('output_topic_raw', 'camera/bottom/pose_raw_px')

        # debug image overlay
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_topic', 'camera/bottom/debug/tile_lines')

        # ---- Params (rotation center) ----
        # 'image_center' or 'principal_point'
        self.declare_parameter('rotation_center_mode', 'image_center')
        self.declare_parameter('camera_info_topic', 'camera/bottom/camera_info')

        # ---- Params (image -> body mapping) ----
        self.declare_parameter('image_to_body_yaw_deg', 90.0)
        self.declare_parameter('flip_image_x', False)
        self.declare_parameter('flip_image_y', False)

        # ---- Params (image scale) ----
        self.declare_parameter('resize_width_px', 320)

        # ---- Params (tile line detection) ----
        self.declare_parameter('blur_kernel_size', 5)
        self.declare_parameter('canny_threshold1', 60.0)
        self.declare_parameter('canny_threshold2', 160.0)
        self.declare_parameter('hough_threshold', 45)
        self.declare_parameter('min_line_length_px', 45.0)
        self.declare_parameter('auto_scale_min_line_length', True)
        self.declare_parameter('min_line_length_reference_width_px', 320.0)
        self.declare_parameter('max_line_gap_px', 12.0)
        self.declare_parameter('axis_angle_tolerance_deg', 25.0)
        self.declare_parameter('use_dynamic_grid_angles', True)
        self.declare_parameter('line_merge_distance_px', 8.0)
        self.declare_parameter('min_lines_per_axis', 1)
        self.declare_parameter('track_confirm_frames', 3)
        self.declare_parameter('track_max_missed_frames', 5)
        self.declare_parameter('track_match_distance_px', 25.0)
        self.declare_parameter('track_angle_match_tolerance_deg', 12.0)
        self.declare_parameter('motion_outlier_threshold_px', 12.0)
        self.declare_parameter('min_confirmed_line_matches', 2)
        self.declare_parameter('reinit_if_fail', True)
        self.declare_parameter('tile_detection_stride', 2)
        self.declare_parameter('recovery_detection_frames', 5)
        self.declare_parameter('min_confirmed_tracks_for_stride', 2)
        self.declare_parameter('min_tracking_confidence_for_stride', 0.5)
        self.declare_parameter('max_visual_only_frames', 3)
        self.declare_parameter('yaw_visual_inliers_full_confidence', 80)
        self.declare_parameter('yaw_line_matches_full_confidence', 6)
        self.declare_parameter('yaw_line_blend_min_weight', 0.25)
        self.declare_parameter('yaw_line_blend_max_weight', 0.85)
        self.declare_parameter('grid_yaw_anchor_min_confidence', 0.75)
        self.declare_parameter('grid_yaw_anchor_min_matches', 4)
        self.declare_parameter('grid_yaw_correction_gain', 0.10)
        self.declare_parameter('grid_yaw_correction_max_rad', 0.015)
        self.declare_parameter('grid_yaw_correction_gate_rad', 0.25)
        self.declare_parameter('grid_yaw_deadband_rad', 0.005)

        # ---- Params (visual feature tracking for yaw continuity) ----
        self.declare_parameter('use_visual_feature_tracking', True)
        self.declare_parameter('max_corners', 500)
        self.declare_parameter('quality_level', 0.01)
        self.declare_parameter('min_distance', 7.0)
        self.declare_parameter('block_size', 7)
        self.declare_parameter('win_size', 21)
        self.declare_parameter('max_level', 2)
        self.declare_parameter('criteria_count', 20)
        self.declare_parameter('criteria_eps', 0.03)
        self.declare_parameter('max_track_error', 50.0)
        self.declare_parameter('ransac_reproj_threshold_px', 3.0)
        self.declare_parameter('min_visual_inliers', 10)
        self.declare_parameter('min_visual_tracked_points', 30)

        # ---- Load params ----
        self._bridge = CvBridge()
        self._image_topic = self.get_parameter('image_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._output_x_topic = self.get_parameter('output_x_topic').value
        self._output_y_topic = self.get_parameter('output_y_topic').value
        self._output_yaw_topic = self.get_parameter('output_yaw_topic').value
        self._output_scale_topic = self.get_parameter('output_scale_topic').value

        self._publish_raw_output = bool(self.get_parameter('publish_raw_output').value)
        self._output_topic_raw = self.get_parameter('output_topic_raw').value

        self._publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self._debug_image_topic = self.get_parameter('debug_image_topic').value

        self._rotation_center_mode = str(self.get_parameter('rotation_center_mode').value)
        self._camera_info_topic = str(self.get_parameter('camera_info_topic').value)

        yaw_deg = float(self.get_parameter('image_to_body_yaw_deg').value)
        flip_x = bool(self.get_parameter('flip_image_x').value)
        flip_y = bool(self.get_parameter('flip_image_y').value)
        self._image_to_body = self._build_mapping_matrix(yaw_deg, flip_x, flip_y)
        self._body_to_image = np.linalg.inv(self._image_to_body)
        self._yaw_basis_sign = 1.0 if np.linalg.det(self._image_to_body) >= 0.0 else -1.0

        self._resize_width_px = int(self.get_parameter('resize_width_px').value)
        self._blur_kernel_size = int(self.get_parameter('blur_kernel_size').value)
        self._canny_threshold1 = float(self.get_parameter('canny_threshold1').value)
        self._canny_threshold2 = float(self.get_parameter('canny_threshold2').value)
        self._hough_threshold = int(self.get_parameter('hough_threshold').value)
        self._min_line_length_px = float(self.get_parameter('min_line_length_px').value)
        self._auto_scale_min_line_length = bool(
            self.get_parameter('auto_scale_min_line_length').value
        )
        self._min_line_length_reference_width_px = max(
            1.0,
            float(self.get_parameter('min_line_length_reference_width_px').value),
        )
        self._max_line_gap_px = float(self.get_parameter('max_line_gap_px').value)
        self._axis_angle_tolerance = math.radians(
            float(self.get_parameter('axis_angle_tolerance_deg').value)
        )
        self._use_dynamic_grid_angles = bool(self.get_parameter('use_dynamic_grid_angles').value)
        self._line_merge_distance_px = float(self.get_parameter('line_merge_distance_px').value)
        self._min_lines_per_axis = int(self.get_parameter('min_lines_per_axis').value)
        self._track_confirm_frames = int(self.get_parameter('track_confirm_frames').value)
        self._track_max_missed_frames = int(self.get_parameter('track_max_missed_frames').value)
        self._track_match_distance_px = float(self.get_parameter('track_match_distance_px').value)
        self._track_angle_match_tolerance = math.radians(
            float(self.get_parameter('track_angle_match_tolerance_deg').value)
        )
        self._motion_outlier_threshold_px = float(
            self.get_parameter('motion_outlier_threshold_px').value
        )
        self._min_confirmed_line_matches = int(
            self.get_parameter('min_confirmed_line_matches').value
        )
        self._reinit_if_fail = bool(self.get_parameter('reinit_if_fail').value)
        self._tile_detection_stride = max(
            1,
            int(self.get_parameter('tile_detection_stride').value),
        )
        self._recovery_detection_frames = max(
            0,
            int(self.get_parameter('recovery_detection_frames').value),
        )
        self._min_confirmed_tracks_for_stride = max(
            1,
            int(self.get_parameter('min_confirmed_tracks_for_stride').value),
        )
        self._min_tracking_confidence_for_stride = float(
            self.get_parameter('min_tracking_confidence_for_stride').value
        )
        self._max_visual_only_frames = max(
            1,
            int(self.get_parameter('max_visual_only_frames').value),
        )
        self._yaw_visual_inliers_full_confidence = max(
            1.0,
            float(self.get_parameter('yaw_visual_inliers_full_confidence').value),
        )
        self._yaw_line_matches_full_confidence = max(
            1.0,
            float(self.get_parameter('yaw_line_matches_full_confidence').value),
        )
        self._yaw_line_blend_min_weight = self._clamp(
            float(self.get_parameter('yaw_line_blend_min_weight').value),
            0.0,
            1.0,
        )
        self._yaw_line_blend_max_weight = self._clamp(
            float(self.get_parameter('yaw_line_blend_max_weight').value),
            self._yaw_line_blend_min_weight,
            1.0,
        )
        self._grid_yaw_anchor_min_confidence = self._clamp(
            float(self.get_parameter('grid_yaw_anchor_min_confidence').value),
            0.0,
            1.0,
        )
        self._grid_yaw_anchor_min_matches = max(
            1,
            int(self.get_parameter('grid_yaw_anchor_min_matches').value),
        )
        self._grid_yaw_correction_gain = max(
            0.0,
            float(self.get_parameter('grid_yaw_correction_gain').value),
        )
        self._grid_yaw_correction_max_rad = max(
            0.0,
            float(self.get_parameter('grid_yaw_correction_max_rad').value),
        )
        self._grid_yaw_correction_gate_rad = max(
            0.0,
            float(self.get_parameter('grid_yaw_correction_gate_rad').value),
        )
        self._grid_yaw_deadband_rad = max(
            0.0,
            float(self.get_parameter('grid_yaw_deadband_rad').value),
        )

        self._use_visual_feature_tracking = bool(
            self.get_parameter('use_visual_feature_tracking').value
        )
        self._max_corners = int(self.get_parameter('max_corners').value)
        self._quality_level = float(self.get_parameter('quality_level').value)
        self._min_distance = float(self.get_parameter('min_distance').value)
        self._block_size = int(self.get_parameter('block_size').value)
        self._win_size = int(self.get_parameter('win_size').value)
        self._max_level = int(self.get_parameter('max_level').value)
        self._criteria_count = int(self.get_parameter('criteria_count').value)
        self._criteria_eps = float(self.get_parameter('criteria_eps').value)
        self._max_track_error = float(self.get_parameter('max_track_error').value)
        self._ransac_thresh = float(self.get_parameter('ransac_reproj_threshold_px').value)
        self._min_visual_inliers = int(self.get_parameter('min_visual_inliers').value)
        self._min_visual_tracked_points = int(
            self.get_parameter('min_visual_tracked_points').value
        )
        self._lk_criteria = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            self._criteria_count,
            self._criteria_eps,
        )

        # ---- CameraInfo principal point (optional) ----
        self._have_pp = False
        self._pp_cx = 0.0
        self._pp_cy = 0.0

        # ---- State ----
        self._horizontal_tracks: List[LineTrack] = []
        self._vertical_tracks: List[LineTrack] = []
        self._last_scene_tx_small = 0.0
        self._last_scene_ty_small = 0.0
        self._last_scene_translation_small = np.zeros(2, dtype=np.float64)
        self._last_scene_yaw = 0.0
        self._have_grid_orientation = False
        self._horizontal_family_dir = 0.0
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_pts: Optional[np.ndarray] = None
        self._image_frame_count = 0
        self._visual_only_frame_count = 0
        self._consecutive_detection_failures = 0
        self._recovery_detection_remaining = self._recovery_detection_frames
        self._grid_yaw_anchor_ready = False
        self._grid_yaw_anchor_angle = 0.0
        self._grid_yaw_anchor_pose_yaw = 0.0

        # running totals (vehicle pose in world frame)
        self._total_raw = np.eye(3, dtype=np.float64)
        self._total_comp = np.eye(3, dtype=np.float64)

        # ---- Pub/Sub ----
        self._pub_comp = self.create_publisher(Float64MultiArray, self._output_topic, 10)
        self._pub_x = self.create_publisher(Float64, self._output_x_topic, 10)
        self._pub_y = self.create_publisher(Float64, self._output_y_topic, 10)
        self._pub_yaw = self.create_publisher(Float64, self._output_yaw_topic, 10)
        self._pub_scale = self.create_publisher(Float64, self._output_scale_topic, 10)
        self._pub_raw = None
        if self._publish_raw_output:
            self._pub_raw = self.create_publisher(Float64MultiArray, self._output_topic_raw, 10)

        self._debug_pub = None
        if self._publish_debug_image:
            self._debug_pub = self.create_publisher(Image, self._debug_image_topic, 10)

        self._sub_img = self.create_subscription(Image, self._image_topic, self._on_image, 10)
        self._reset_service = self.create_service(
            Trigger,
            'camera/bottom/reset_pose',
            self._on_reset_pose,
        )
        self._sub_info = None
        if self._rotation_center_mode == 'principal_point':
            self._sub_info = self.create_subscription(
                CameraInfo,
                self._camera_info_topic,
                self._on_camerainfo,
                10,
            )

        self.get_logger().info(
            f'Tile-line total transform node started. image_topic={self._image_topic}, '
            f'output_topic={self._output_topic}, '
            f'rotation_center_mode={self._rotation_center_mode}, '
            f'publish_raw={self._publish_raw_output}\n'
            f'scalar outputs: x={self._output_x_topic}, y={self._output_y_topic}, '
            f'yaw={self._output_yaw_topic}, scale={self._output_scale_topic}, '
            f'tile_detection_stride={self._tile_detection_stride}'
        )

    def _on_camerainfo(self, msg: CameraInfo):
        # CameraInfo.K = [fx,0,cx, 0,fy,cy, 0,0,1]
        if len(msg.k) >= 6:
            cx = float(msg.k[2])
            cy = float(msg.k[5])
            if math.isfinite(cx) and math.isfinite(cy):
                self._pp_cx = cx
                self._pp_cy = cy
                self._have_pp = True

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @classmethod
    def _angle_blend(cls, base: float, target: float, target_weight: float) -> float:
        weight = cls._clamp(target_weight, 0.0, 1.0)
        return base + weight * cls._wrap_pi(target - base)

    @staticmethod
    def _yaw_matrix(yaw: float) -> np.ndarray:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return np.array([
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

    def _to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        try:
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}', throttle_duration_sec=5.0)
            return None

    def _downscale_gray(self, gray: np.ndarray) -> Tuple[np.ndarray, float]:
        h, w = gray.shape[:2]
        if self._resize_width_px <= 0 or w <= self._resize_width_px:
            return gray, 1.0
        scale = float(self._resize_width_px) / float(w)
        new_w = self._resize_width_px
        new_h = max(1, int(round(h * scale)))
        small = cv2.resize(gray, (new_w, new_h), interpolation=cv2.INTER_AREA)
        return small, scale

    def _effective_min_line_length_px(self, image_width_px: int) -> float:
        if not self._auto_scale_min_line_length:
            return self._min_line_length_px

        scale = max(1.0, float(image_width_px))
        scale /= self._min_line_length_reference_width_px
        return max(1.0, self._min_line_length_px * scale)

    def _detect_features(self, gray_small: np.ndarray) -> Optional[np.ndarray]:
        pts = cv2.goodFeaturesToTrack(
            gray_small,
            mask=None,
            maxCorners=self._max_corners,
            qualityLevel=self._quality_level,
            minDistance=self._min_distance,
            blockSize=self._block_size,
        )
        if pts is None:
            return None
        return pts.astype(np.float32)

    def _estimate_visual_motion(self, gray_small: np.ndarray) -> Optional[VisualMotion]:
        if not self._use_visual_feature_tracking:
            return None

        if (
            self._prev_gray is None
            or self._prev_pts is None
            or len(self._prev_pts) < self._min_visual_tracked_points
        ):
            self._prev_gray = gray_small
            self._prev_pts = self._detect_features(gray_small)
            return None

        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self._prev_gray,
            gray_small,
            self._prev_pts,
            None,
            winSize=(self._win_size, self._win_size),
            maxLevel=self._max_level,
            criteria=self._lk_criteria,
        )

        if p1 is None or st is None:
            self._prev_gray = gray_small
            self._prev_pts = self._detect_features(gray_small)
            return None

        st = st.reshape(-1)
        good_new = p1[st == 1].reshape(-1, 2)
        good_old = self._prev_pts[st == 1].reshape(-1, 2)

        if err is not None:
            err = err.reshape(-1)
            good_err = err[st == 1]
            if self._max_track_error > 0.0:
                mask = good_err <= self._max_track_error
                good_new = good_new[mask]
                good_old = good_old[mask]

        if good_new.shape[0] < self._min_visual_tracked_points:
            self._prev_gray = gray_small
            self._prev_pts = self._detect_features(gray_small)
            return None

        M2, inliers = cv2.estimateAffinePartial2D(
            good_old,
            good_new,
            method=cv2.RANSAC,
            ransacReprojThreshold=self._ransac_thresh,
            maxIters=2000,
            confidence=0.99,
            refineIters=10,
        )

        self._prev_gray = gray_small
        self._prev_pts = good_new.reshape(-1, 1, 2).astype(np.float32)

        if M2 is None or inliers is None:
            self._prev_pts = self._detect_features(gray_small)
            return None

        inlier_count = int(inliers.sum())
        if inlier_count < self._min_visual_inliers:
            self._prev_pts = self._detect_features(gray_small)
            return None

        m00, _, tx = M2[0]
        m10, _, ty = M2[1]
        scale = math.sqrt(float(m00) * float(m00) + float(m10) * float(m10))
        rot = math.atan2(float(m10), float(m00))
        return VisualMotion(
            tx=float(tx),
            ty=float(ty),
            rot=float(rot),
            scale=float(scale),
            inliers=inlier_count,
        )

    @staticmethod
    def _extract_similarity(H: np.ndarray) -> Tuple[float, float, float, float]:
        tx = float(H[0, 2])
        ty = float(H[1, 2])
        a = float(H[0, 0])
        c = float(H[1, 0])
        scale = math.sqrt(a * a + c * c)
        rot = math.atan2(c, a)
        return tx, ty, rot, scale

    def _publish(self):
        # compensated output (main topic)
        tx, ty, rot, scale = self._extract_similarity(self._total_comp)
        msg = Float64MultiArray()
        msg.data = [float(tx), float(ty), float(rot), float(scale)]
        self._pub_comp.publish(msg)
        self._publish_float(self._pub_x, tx)
        self._publish_float(self._pub_y, ty)
        self._publish_float(self._pub_yaw, rot)
        self._publish_float(self._pub_scale, scale)

        # raw output (optional diagnostics)
        if self._pub_raw is not None:
            tx2, ty2, rot2, scale2 = self._extract_similarity(self._total_raw)
            msg2 = Float64MultiArray()
            msg2.data = [float(tx2), float(ty2), float(rot2), float(scale2)]
            self._pub_raw.publish(msg2)

    def _on_reset_pose(self, request, response):
        self._prev_gray = None
        self._prev_pts = None
        self._image_frame_count = 0
        self._visual_only_frame_count = 0
        self._consecutive_detection_failures = 0
        self._recovery_detection_remaining = self._recovery_detection_frames
        self._grid_yaw_anchor_ready = False
        self._grid_yaw_anchor_angle = 0.0
        self._grid_yaw_anchor_pose_yaw = 0.0
        self._total_raw = np.eye(3, dtype=np.float64)
        self._total_comp = np.eye(3, dtype=np.float64)
        self._reset_line_tracks()
        self._publish()

        response.success = True
        response.message = 'Bottom-camera pose reset'
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def _publish_float(pub, value: float):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def _detect_tile_lines(self, gray_small: np.ndarray) -> Optional[TileLineDetection]:
        gray_eq = cv2.equalizeHist(gray_small)
        blur_size = self._blur_kernel_size
        if blur_size > 1:
            if blur_size % 2 == 0:
                blur_size += 1
            gray_eq = cv2.GaussianBlur(gray_eq, (blur_size, blur_size), 0)

        h, w = gray_small.shape[:2]
        min_line_length_px = self._effective_min_line_length_px(w)
        edges = cv2.Canny(gray_eq, self._canny_threshold1, self._canny_threshold2)
        lines = cv2.HoughLinesP(
            edges,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=self._hough_threshold,
            minLineLength=min_line_length_px,
            maxLineGap=self._max_line_gap_px,
        )

        if lines is None:
            return None

        center = np.array([0.5 * float(w), 0.5 * float(h)], dtype=np.float64)
        horizontal_obs = []
        vertical_obs = []
        kept_segments = []
        raw_segments = []

        for line in lines.reshape(-1, 4):
            x1, y1, x2, y2 = [float(v) for v in line]
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length <= 0.0:
                continue
            raw_segments.append((line.astype(np.int32), x1, y1, x2, y2, dx, dy, length))

        if not raw_segments:
            return None

        if self._use_dynamic_grid_angles:
            base_angle = self._estimate_grid_base_angle(raw_segments)
        else:
            base_angle = 0.0
        base_angle = self._align_grid_base_angle(base_angle)

        horizontal_dir = base_angle
        vertical_dir = (base_angle + 0.5 * math.pi) % math.pi
        horizontal_normal = self._line_normal(horizontal_dir)
        vertical_normal = self._line_normal(vertical_dir)
        self._horizontal_family_dir = horizontal_dir
        self._have_grid_orientation = True

        for segment, x1, y1, x2, y2, dx, dy, length in raw_segments:
            angle = math.atan2(dy, dx) % math.pi
            horizontal_error = abs(self._line_angle_diff(angle, horizontal_dir))
            vertical_error = abs(self._line_angle_diff(angle, vertical_dir))

            if horizontal_error <= self._axis_angle_tolerance:
                midpoint = np.array([0.5 * (x1 + x2), 0.5 * (y1 + y2)], dtype=np.float64)
                pos = float(horizontal_normal @ (midpoint - center))
                horizontal_obs.append((pos, angle, length))
                kept_segments.append(segment)
            elif vertical_error <= self._axis_angle_tolerance:
                midpoint = np.array([0.5 * (x1 + x2), 0.5 * (y1 + y2)], dtype=np.float64)
                pos = float(vertical_normal @ (midpoint - center))
                vertical_obs.append((pos, angle, length))
                kept_segments.append(segment)

        horizontal = self._cluster_lines(horizontal_obs)
        vertical = self._cluster_lines(vertical_obs)
        if len(horizontal) < self._min_lines_per_axis and len(vertical) < self._min_lines_per_axis:
            return None

        segments = (
            np.array(kept_segments, dtype=np.int32)
            if kept_segments
            else np.empty((0, 4), dtype=np.int32)
        )
        return TileLineDetection(
            horizontal=horizontal,
            vertical=vertical,
            segments=segments,
            horizontal_normal=horizontal_normal,
            vertical_normal=vertical_normal,
            horizontal_dir=horizontal_dir,
        )

    def _cluster_lines(self, observations) -> List[TileLine]:
        if not observations:
            return []

        observations = sorted(observations, key=lambda item: item[0])
        clusters = []
        current = [observations[0]]
        for obs in observations[1:]:
            if abs(obs[0] - current[-1][0]) <= self._line_merge_distance_px:
                current.append(obs)
            else:
                clusters.append(self._merge_cluster(current))
                current = [obs]
        clusters.append(self._merge_cluster(current))
        return clusters

    @staticmethod
    def _estimate_grid_base_angle(raw_segments) -> float:
        weighted_cos = 0.0
        weighted_sin = 0.0
        for _, _, _, _, _, dx, dy, length in raw_segments:
            angle = math.atan2(dy, dx)
            weighted_cos += length * math.cos(4.0 * angle)
            weighted_sin += length * math.sin(4.0 * angle)

        if abs(weighted_cos) < 1e-9 and abs(weighted_sin) < 1e-9:
            return 0.0

        base = 0.25 * math.atan2(weighted_sin, weighted_cos)
        return base % (0.5 * math.pi)

    def _align_grid_base_angle(self, base_angle: float) -> float:
        if not self._have_grid_orientation:
            return base_angle % math.pi

        candidates = [
            base_angle % math.pi,
            (base_angle + 0.5 * math.pi) % math.pi,
        ]
        return min(
            candidates,
            key=lambda angle: abs(self._line_angle_diff(angle, self._horizontal_family_dir)),
        )

    @staticmethod
    def _line_normal(direction_angle: float) -> np.ndarray:
        return np.array([
            -math.sin(direction_angle),
            math.cos(direction_angle),
        ], dtype=np.float64)

    @staticmethod
    def _merge_cluster(cluster) -> TileLine:
        weights = np.array([item[2] for item in cluster], dtype=np.float64)
        positions = np.array([item[0] for item in cluster], dtype=np.float64)
        angles = np.array([item[1] for item in cluster], dtype=np.float64)
        total_weight = float(weights.sum())
        if total_weight <= 0.0:
            total_weight = 1.0
            weights = np.ones_like(weights)
        pos = float(np.average(positions, weights=weights))
        doubled_angles = 2.0 * angles
        angle = 0.5 * math.atan2(
            float(np.average(np.sin(doubled_angles), weights=weights)),
            float(np.average(np.cos(doubled_angles), weights=weights)),
        )
        angle = angle % math.pi
        return TileLine(pos=pos, angle_offset=angle, weight=total_weight)

    def _estimate_scene_motion_from_tracks(
        self,
        detection: TileLineDetection,
    ) -> Optional[Tuple[float, float, float, int]]:
        horizontal_prediction = float(
            detection.horizontal_normal @ self._last_scene_translation_small
        )
        vertical_prediction = float(
            detection.vertical_normal @ self._last_scene_translation_small
        )
        vertical_matches, vertical_unmatched = self._match_track_family(
            self._vertical_tracks,
            detection.vertical,
            vertical_prediction,
        )
        horizontal_matches, horizontal_unmatched = self._match_track_family(
            self._horizontal_tracks,
            detection.horizontal,
            horizontal_prediction,
        )

        vertical_inliers = self._confirmed_inlier_matches(vertical_matches)
        horizontal_inliers = self._confirmed_inlier_matches(horizontal_matches)
        vertical_accepted, vertical_unmatched = self._accepted_matches(
            vertical_matches,
            vertical_inliers,
            vertical_unmatched,
        )
        horizontal_accepted, horizontal_unmatched = self._accepted_matches(
            horizontal_matches,
            horizontal_inliers,
            horizontal_unmatched,
        )
        inlier_count = len(vertical_inliers) + len(horizontal_inliers)

        if inlier_count < self._min_confirmed_line_matches:
            self._update_track_family(
                self._vertical_tracks,
                detection.vertical,
                vertical_accepted,
                vertical_unmatched,
                vertical_prediction,
            )
            self._update_track_family(
                self._horizontal_tracks,
                detection.horizontal,
                horizontal_accepted,
                horizontal_unmatched,
                horizontal_prediction,
            )
            return None

        vertical_delta = self._motion_from_matches(vertical_inliers, vertical_prediction)
        horizontal_delta = self._motion_from_matches(horizontal_inliers, horizontal_prediction)
        translation = self._solve_translation_from_line_deltas(
            detection.horizontal_normal,
            horizontal_delta,
            detection.vertical_normal,
            vertical_delta,
        )
        tx = float(translation[0])
        ty = float(translation[1])
        angle_deltas = [m.angle_delta for m in vertical_inliers + horizontal_inliers]
        rot = float(np.median(angle_deltas)) if angle_deltas else 0.0

        self._update_track_family(
            self._vertical_tracks,
            detection.vertical,
            vertical_accepted,
            vertical_unmatched,
            vertical_delta,
        )
        self._update_track_family(
            self._horizontal_tracks,
            detection.horizontal,
            horizontal_accepted,
            horizontal_unmatched,
            horizontal_delta,
        )
        self._last_scene_tx_small = tx
        self._last_scene_ty_small = ty
        self._last_scene_translation_small = translation
        self._last_scene_yaw = rot
        return tx, ty, rot, inlier_count

    def _confirmed_track_counts(self) -> Tuple[int, int]:
        horizontal_count = sum(
            1 for track in self._horizontal_tracks
            if track.confirmed and track.confidence > 0.0
        )
        vertical_count = sum(
            1 for track in self._vertical_tracks
            if track.confirmed and track.confidence > 0.0
        )
        return horizontal_count, vertical_count

    def _tracking_confidence(self) -> float:
        confirmed_tracks = [
            track
            for track in self._horizontal_tracks + self._vertical_tracks
            if track.confirmed and track.confidence > 0.0
        ]
        if not confirmed_tracks:
            return 0.0

        horizontal_count, vertical_count = self._confirmed_track_counts()
        family_score = 1.0 if horizontal_count > 0 and vertical_count > 0 else 0.5
        count_score = min(
            1.0,
            len(confirmed_tracks) / float(self._min_confirmed_tracks_for_stride),
        )
        confidence = sum(track.confidence for track in confirmed_tracks)
        confidence /= float(len(confirmed_tracks))
        return max(0.0, min(1.0, confidence * count_score * family_score))

    def _tracking_ready_for_stride(self) -> bool:
        horizontal_count, vertical_count = self._confirmed_track_counts()
        if horizontal_count == 0 or vertical_count == 0:
            return False
        if horizontal_count + vertical_count < self._min_confirmed_tracks_for_stride:
            return False
        return self._tracking_confidence() >= self._min_tracking_confidence_for_stride

    def _should_detect_tile_lines(
        self,
        visual_motion: Optional[VisualMotion],
    ) -> bool:
        if self._tile_detection_stride <= 1:
            return True
        if self._recovery_detection_remaining > 0:
            return True
        if visual_motion is None:
            return True
        if self._visual_only_frame_count >= self._max_visual_only_frames:
            return True
        if not self._tracking_ready_for_stride():
            return True
        return self._image_frame_count % self._tile_detection_stride == 0

    def _note_detection_success(self):
        self._consecutive_detection_failures = 0
        self._visual_only_frame_count = 0
        if self._recovery_detection_remaining > 0:
            self._recovery_detection_remaining -= 1

    def _note_detection_failure(self):
        self._consecutive_detection_failures += 1
        self._visual_only_frame_count = 0
        self._recovery_detection_remaining = max(
            self._recovery_detection_remaining,
            self._recovery_detection_frames,
        )

    def _line_yaw_confidence(self, match_count: int) -> float:
        match_score = self._clamp(
            float(match_count) / self._yaw_line_matches_full_confidence,
            0.0,
            1.0,
        )
        return match_score * self._tracking_confidence()

    def _fuse_yaw_rotation(
        self,
        line_rot: float,
        match_count: int,
        visual_motion: Optional[VisualMotion],
    ) -> float:
        if visual_motion is None:
            return line_rot

        line_confidence = self._line_yaw_confidence(match_count)
        if line_confidence <= 0.0:
            return visual_motion.rot

        visual_confidence = self._clamp(
            float(visual_motion.inliers) / self._yaw_visual_inliers_full_confidence,
            0.0,
            1.0,
        )
        line_weight = line_confidence / (line_confidence + visual_confidence + 1e-6)
        line_weight = self._clamp(
            line_weight,
            self._yaw_line_blend_min_weight,
            self._yaw_line_blend_max_weight,
        )
        return self._angle_blend(visual_motion.rot, line_rot, line_weight)

    def _grid_yaw_measurement_ready(self, match_count: int) -> bool:
        if match_count < self._grid_yaw_anchor_min_matches:
            return False
        horizontal_count, vertical_count = self._confirmed_track_counts()
        if horizontal_count == 0 or vertical_count == 0:
            return False
        return self._tracking_confidence() >= self._grid_yaw_anchor_min_confidence

    def _maybe_apply_grid_yaw_correction(
        self,
        detection: TileLineDetection,
        match_count: int,
    ) -> float:
        if not self._grid_yaw_measurement_ready(match_count):
            return 0.0

        _, _, pose_yaw, _ = self._extract_similarity(self._total_comp)
        if not self._grid_yaw_anchor_ready:
            self._grid_yaw_anchor_angle = detection.horizontal_dir
            self._grid_yaw_anchor_pose_yaw = pose_yaw
            self._grid_yaw_anchor_ready = True
            return 0.0

        scene_grid_delta = self._line_angle_diff(
            detection.horizontal_dir,
            self._grid_yaw_anchor_angle,
        )
        target_yaw = (
            self._grid_yaw_anchor_pose_yaw
            - self._yaw_basis_sign * scene_grid_delta
        )
        yaw_error = self._wrap_pi(target_yaw - pose_yaw)
        if abs(yaw_error) > self._grid_yaw_correction_gate_rad:
            return 0.0
        if abs(yaw_error) < self._grid_yaw_deadband_rad:
            return 0.0

        correction = self._clamp(
            self._grid_yaw_correction_gain * yaw_error,
            -self._grid_yaw_correction_max_rad,
            self._grid_yaw_correction_max_rad,
        )
        correction_matrix = self._yaw_matrix(correction)
        self._total_raw = self._total_raw @ correction_matrix
        self._total_comp = self._total_comp @ correction_matrix
        return correction

    def _match_track_family(
        self,
        tracks: List[LineTrack],
        observations: List[TileLine],
        predicted_delta: float,
    ) -> Tuple[List[LineMatch], List[int]]:
        if not tracks:
            return [], list(range(len(observations)))
        if not observations:
            return [], []

        candidates = []
        for track_idx, track in enumerate(tracks):
            predicted_pos = track.pos + predicted_delta
            for obs_idx, observation in enumerate(observations):
                pos_dist = abs(observation.pos - predicted_pos)
                angle_dist = abs(
                    self._line_angle_diff(
                        observation.angle_offset,
                        track.angle_offset,
                    )
                )
                if pos_dist > self._track_match_distance_px:
                    continue
                if angle_dist > self._track_angle_match_tolerance:
                    continue
                candidates.append((pos_dist + 10.0 * angle_dist, track_idx, obs_idx))

        candidates.sort(key=lambda item: item[0])
        used_tracks = set()
        used_observations = set()
        matches = []
        for _, track_idx, obs_idx in candidates:
            if track_idx in used_tracks or obs_idx in used_observations:
                continue
            track = tracks[track_idx]
            observation = observations[obs_idx]
            matches.append(LineMatch(
                track=track,
                observation=observation,
                observation_idx=obs_idx,
                pos_delta=observation.pos - track.pos,
                angle_delta=self._line_angle_diff(observation.angle_offset, track.angle_offset),
            ))
            used_tracks.add(track_idx)
            used_observations.add(obs_idx)

        unmatched = [idx for idx in range(len(observations)) if idx not in used_observations]
        return matches, unmatched

    def _confirmed_inlier_matches(self, matches: List[LineMatch]) -> List[LineMatch]:
        confirmed = [match for match in matches if match.track.confirmed]
        if len(confirmed) < 2:
            return confirmed

        median_delta = float(np.median([match.pos_delta for match in confirmed]))
        return [
            match for match in confirmed
            if abs(match.pos_delta - median_delta) <= self._motion_outlier_threshold_px
        ]

    @staticmethod
    def _accepted_matches(
        matches: List[LineMatch],
        confirmed_inliers: List[LineMatch],
        unmatched_observation_idxs: List[int],
    ) -> Tuple[List[LineMatch], List[int]]:
        inlier_ids = {id(match) for match in confirmed_inliers}
        accepted = [
            match for match in matches
            if not match.track.confirmed or id(match) in inlier_ids
        ]
        accepted_ids = {id(match) for match in accepted}
        rejected_observations = [
            match.observation_idx for match in matches
            if id(match) not in accepted_ids
        ]
        return accepted, unmatched_observation_idxs + rejected_observations

    @staticmethod
    def _motion_from_matches(matches: List[LineMatch], fallback: float) -> float:
        if not matches:
            return fallback
        return float(np.median([match.pos_delta for match in matches]))

    def _solve_translation_from_line_deltas(
        self,
        horizontal_normal: np.ndarray,
        horizontal_delta: float,
        vertical_normal: np.ndarray,
        vertical_delta: float,
    ) -> np.ndarray:
        A = np.vstack([horizontal_normal, vertical_normal])
        b = np.array([horizontal_delta, vertical_delta], dtype=np.float64)
        try:
            return np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            self.get_logger().warn(
                'Tile-line family normals are degenerate; using last scene translation.',
                throttle_duration_sec=5.0,
            )
            return self._last_scene_translation_small.copy()

    @staticmethod
    def _center_compensated_translation(
        tx: float,
        ty: float,
        rot: float,
        cx: float,
        cy: float,
    ) -> np.ndarray:
        cos_rot = math.cos(rot)
        sin_rot = math.sin(rot)
        rot_mat = np.array([
            [cos_rot, -sin_rot],
            [sin_rot, cos_rot],
        ], dtype=np.float64)
        center = np.array([cx, cy], dtype=np.float64)
        raw_t = np.array([tx, ty], dtype=np.float64)
        return raw_t + (rot_mat - np.eye(2, dtype=np.float64)) @ center

    def _update_track_family(
        self,
        tracks: List[LineTrack],
        observations: List[TileLine],
        matches: List[LineMatch],
        unmatched_observation_idxs: List[int],
        scene_delta: float,
    ):
        matched_tracks = {id(match.track) for match in matches}
        for match in matches:
            track = match.track
            observation = match.observation
            track.pos = observation.pos
            track.angle_offset = observation.angle_offset
            track.confidence = min(1.0, track.confidence + 0.25)
            track.missed_count = 0
            track.age += 1
            track.seen_count += 1
            if track.seen_count >= self._track_confirm_frames:
                track.confirmed = True

        for track in tracks:
            if id(track) in matched_tracks:
                continue
            track.pos += scene_delta
            track.confidence = max(0.0, track.confidence - 0.25)
            track.missed_count += 1
            track.age += 1

        for obs_idx in unmatched_observation_idxs:
            observation = observations[obs_idx]
            tracks.append(LineTrack(
                pos=observation.pos,
                angle_offset=observation.angle_offset,
                confidence=0.25,
                confirmed=self._track_confirm_frames <= 1,
            ))

        tracks[:] = [
            track for track in tracks
            if track.missed_count <= self._track_max_missed_frames and track.confidence > 0.0
        ]

    def _mark_tracks_missed(self):
        self._mark_track_family_missed(self._vertical_tracks, 0.0)
        self._mark_track_family_missed(self._horizontal_tracks, 0.0)

    def _predict_tracks_from_visual_motion(
        self,
        translation_small: Optional[np.ndarray],
        rotation_rad: float,
    ):
        if translation_small is None or not self._have_grid_orientation:
            return

        horizontal_normal = self._line_normal(self._horizontal_family_dir)
        vertical_dir = (self._horizontal_family_dir + 0.5 * math.pi) % math.pi
        vertical_normal = self._line_normal(vertical_dir)
        horizontal_delta = float(horizontal_normal @ translation_small)
        vertical_delta = float(vertical_normal @ translation_small)
        self._predict_track_family(
            self._horizontal_tracks,
            horizontal_delta,
            rotation_rad,
        )
        self._predict_track_family(
            self._vertical_tracks,
            vertical_delta,
            rotation_rad,
        )

    @staticmethod
    def _predict_track_family(
        tracks: List[LineTrack],
        scene_delta: float,
        angle_delta: float,
    ):
        for track in tracks:
            track.pos += scene_delta
            track.angle_offset = (track.angle_offset + angle_delta) % math.pi
            track.age += 1

    def _mark_track_family_missed(self, tracks: List[LineTrack], scene_delta: float):
        for track in tracks:
            track.pos += scene_delta
            track.confidence = max(0.0, track.confidence - 0.25)
            track.missed_count += 1
            track.age += 1
        tracks[:] = [
            track for track in tracks
            if track.missed_count <= self._track_max_missed_frames and track.confidence > 0.0
        ]

    def _reset_line_tracks(self):
        self._horizontal_tracks = []
        self._vertical_tracks = []
        self._last_scene_tx_small = 0.0
        self._last_scene_ty_small = 0.0
        self._last_scene_translation_small = np.zeros(2, dtype=np.float64)
        self._last_scene_yaw = 0.0
        self._have_grid_orientation = False
        self._horizontal_family_dir = 0.0
        if hasattr(self, '_recovery_detection_frames'):
            self._visual_only_frame_count = 0
            self._recovery_detection_remaining = self._recovery_detection_frames

    @staticmethod
    def _line_angle_diff(angle: float, reference: float) -> float:
        diff = (angle - reference + 0.5 * math.pi) % math.pi - 0.5 * math.pi
        return diff

    def _publish_debug(
        self,
        frame_bgr: np.ndarray,
        detection: TileLineDetection,
        scale_factor: float,
    ):
        if self._debug_pub is None or frame_bgr is None or detection is None:
            return

        overlay = frame_bgr.copy()
        if detection.segments.size > 0:
            segments = detection.segments.astype(np.float64)
            if scale_factor > 0.0 and scale_factor != 1.0:
                segments /= scale_factor
            for x1, y1, x2, y2 in segments.astype(np.int32):
                cv2.line(overlay, (x1, y1), (x2, y2), (0, 255, 255), 2)

        try:
            out = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to convert debug image: {exc}',
                throttle_duration_sec=5.0,
            )
            return
        self._debug_pub.publish(out)

    def _get_rotation_center(self, w: int, h: int) -> Tuple[float, float]:
        # Use principal point if requested & available; else image center
        if self._rotation_center_mode == 'principal_point' and self._have_pp:
            return self._pp_cx, self._pp_cy
        return 0.5 * float(w), 0.5 * float(h)

    def _on_image(self, msg: Image):
        self._image_frame_count += 1
        frame_bgr = self._to_bgr(msg)
        if frame_bgr is None:
            return

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        h_img, w_img = gray.shape[:2]
        gray_small, scale_factor = self._downscale_gray(gray)
        h_small, w_small = gray_small.shape[:2]
        visual_motion = self._estimate_visual_motion(gray_small)
        visual_translation_small = None
        if visual_motion is not None:
            visual_translation_small = self._center_compensated_translation(
                visual_motion.tx,
                visual_motion.ty,
                visual_motion.rot,
                0.5 * float(w_small),
                0.5 * float(h_small),
            )
        should_detect_lines = self._should_detect_tile_lines(visual_motion)
        curr_lines = None
        line_motion = None

        if should_detect_lines:
            curr_lines = self._detect_tile_lines(gray_small)
            if curr_lines is None:
                self.get_logger().warn(
                    'No tile lines detected in bottom camera image.',
                    throttle_duration_sec=5.0,
                )
                self._mark_tracks_missed()
                self._note_detection_failure()
            else:
                line_motion = self._estimate_scene_motion_from_tracks(curr_lines)

            if line_motion is None and curr_lines is not None:
                self.get_logger().warn(
                    'Insufficient confirmed tile-line tracks for transform.',
                    throttle_duration_sec=5.0,
                )
                self._note_detection_failure()
            elif line_motion is not None:
                self._note_detection_success()

        if line_motion is not None:
            tx, ty, line_rot, match_count = line_motion
            rot = self._fuse_yaw_rotation(line_rot, match_count, visual_motion)
            source = 'hybrid' if visual_motion is not None else 'tile_lines'
        elif visual_motion is not None:
            tx = visual_motion.tx
            ty = visual_motion.ty
            rot = visual_motion.rot
            match_count = visual_motion.inliers
            source = 'visual'
            self._last_scene_tx_small = float(visual_translation_small[0])
            self._last_scene_ty_small = float(visual_translation_small[1])
            self._last_scene_translation_small = visual_translation_small
            self._last_scene_yaw = rot
            if not should_detect_lines:
                self._predict_tracks_from_visual_motion(
                    visual_translation_small,
                    rot,
                )
                self._visual_only_frame_count += 1
        else:
            if self._publish_debug_image and curr_lines is not None:
                self._publish_debug(frame_bgr, curr_lines, scale_factor)
            return

        if scale_factor > 0.0:
            tx /= scale_factor
            ty /= scale_factor

        cos_rot = math.cos(rot)
        sin_rot = math.sin(rot)
        cx, cy = self._get_rotation_center(w_img, h_img)
        if line_motion is not None:
            # Tile-line translation is already measured about the image center.
            H_comp_img = np.array([
                [cos_rot, -sin_rot, tx],
                [sin_rot, cos_rot, ty],
                [0.0, 0.0, 1.0],
            ], dtype=np.float64)
            H_raw_img = _T(cx, cy) @ H_comp_img @ _T(-cx, -cy)
        else:
            H_raw_img = np.array([
                [cos_rot, -sin_rot, tx],
                [sin_rot, cos_rot, ty],
                [0.0, 0.0, 1.0],
            ], dtype=np.float64)
            # Center-compensated increment: H_c = T(-c) * H_raw * T(c)
            H_comp_img = _T(-cx, -cy) @ H_raw_img @ _T(cx, cy)

        # Map to body frame (scene transform) then invert to recover vehicle motion
        H_scene_raw_body = self._change_of_basis(H_raw_img)
        H_scene_comp_body = self._change_of_basis(H_comp_img)

        try:
            H_motion_raw_body = np.linalg.inv(H_scene_raw_body)
            H_motion_comp_body = np.linalg.inv(H_scene_comp_body)
        except np.linalg.LinAlgError:
            self.get_logger().warn(
                'Failed to invert tile-line transform; reinitializing.',
                throttle_duration_sec=5.0,
            )
            if self._reinit_if_fail:
                self._reset_line_tracks()
            return

        # Accumulate vehicle pose in world frame (world origin is the first frame)
        self._total_raw = self._total_raw @ H_motion_raw_body
        self._total_comp = self._total_comp @ H_motion_comp_body
        yaw_correction = 0.0
        if line_motion is not None and curr_lines is not None:
            yaw_correction = self._maybe_apply_grid_yaw_correction(
                curr_lines,
                match_count,
            )

        self.get_logger().debug(
            f'{source} transform: matches={match_count}, scene_tx={tx:.2f}, '
            f'scene_ty={ty:.2f}, scene_yaw={rot:.4f}, '
            f'yaw_correction={yaw_correction:.4f}'
        )

        # Publish totals
        self._publish()

        if self._publish_debug_image:
            self._publish_debug(frame_bgr, curr_lines, scale_factor)

    @staticmethod
    def _build_mapping_matrix(yaw_deg: float, flip_x: bool, flip_y: bool) -> np.ndarray:
        yaw_rad = math.radians(yaw_deg)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        rotation = np.array([
            [cos_yaw, -sin_yaw],
            [sin_yaw, cos_yaw],
        ], dtype=np.float64)
        flips = np.array([
            [-1.0 if flip_x else 1.0, 0.0],
            [0.0, -1.0 if flip_y else 1.0],
        ], dtype=np.float64)
        return rotation @ flips

    def _change_of_basis(self, transform_image: np.ndarray) -> np.ndarray:
        # Express image-frame transform in the configured body frame:
        # T_body = M * T_image * M^{-1}, where M maps image axes to body axes.
        composed = np.eye(3, dtype=np.float64)
        composed[:2, :2] = self._image_to_body @ transform_image[:2, :2] @ self._body_to_image
        composed[:2, 2] = (self._image_to_body @ transform_image[:2, 2].reshape(2, 1)).flatten()
        return composed


def main(args=None):
    rclpy.init(args=args)
    node = LkTotalTransformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
