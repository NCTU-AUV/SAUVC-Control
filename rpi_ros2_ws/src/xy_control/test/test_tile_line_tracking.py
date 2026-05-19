import math

import numpy as np
import pytest

from xy_control.lk_total_transform_node import LineMatch
from xy_control.lk_total_transform_node import LineTrack
from xy_control.lk_total_transform_node import LkTotalTransformNode
from xy_control.lk_total_transform_node import TileLine


class _Response:
    success = False
    message = ''


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


def _raw_segment(angle_rad: float, length: float = 100.0):
    dx = length * math.cos(angle_rad)
    dy = length * math.sin(angle_rad)
    return (
        np.array([0, 0, int(round(dx)), int(round(dy))], dtype=np.int32),
        0.0,
        0.0,
        dx,
        dy,
        dx,
        dy,
        length,
    )


def _angle_close_mod_pi(actual: float, expected: float, tol: float = 1e-6):
    diff = (actual - expected + 0.5 * math.pi) % math.pi - 0.5 * math.pi
    assert abs(diff) < tol


def test_grid_base_angle_folds_orthogonal_tile_lines():
    angle = math.radians(12.0)
    raw_segments = [
        _raw_segment(angle),
        _raw_segment(angle + 0.5 * math.pi),
        _raw_segment(angle + math.pi),
    ]

    base_angle = LkTotalTransformNode._estimate_grid_base_angle(raw_segments)

    _angle_close_mod_pi(base_angle, angle)


def test_dynamic_grid_angle_aligns_to_existing_family():
    node = LkTotalTransformNode.__new__(LkTotalTransformNode)
    node._have_grid_orientation = True
    node._horizontal_family_dir = math.radians(100.0)

    aligned = LkTotalTransformNode._align_grid_base_angle(
        node,
        math.radians(10.0),
    )

    _angle_close_mod_pi(aligned, math.radians(100.0))


def test_center_compensation_removes_pure_yaw_translation_drift():
    cx = 160.0
    cy = 120.0
    rot = math.radians(30.0)
    cos_rot = math.cos(rot)
    sin_rot = math.sin(rot)
    rot_mat = np.array([
        [cos_rot, -sin_rot],
        [sin_rot, cos_rot],
    ], dtype=np.float64)
    center = np.array([cx, cy], dtype=np.float64)
    raw_translation = (np.eye(2, dtype=np.float64) - rot_mat) @ center

    compensated = LkTotalTransformNode._center_compensated_translation(
        raw_translation[0],
        raw_translation[1],
        rot,
        cx,
        cy,
    )

    assert np.allclose(compensated, np.zeros(2), atol=1e-9)


def test_line_deltas_solve_xy_translation():
    node = LkTotalTransformNode.__new__(LkTotalTransformNode)
    node._last_scene_translation_small = np.zeros(2, dtype=np.float64)

    translation = LkTotalTransformNode._solve_translation_from_line_deltas(
        node,
        horizontal_normal=np.array([0.0, 1.0], dtype=np.float64),
        horizontal_delta=4.0,
        vertical_normal=np.array([1.0, 0.0], dtype=np.float64),
        vertical_delta=-3.0,
    )

    assert translation[0] == pytest.approx(-3.0)
    assert translation[1] == pytest.approx(4.0)


def test_update_track_family_confirms_matched_tracks():
    node = LkTotalTransformNode.__new__(LkTotalTransformNode)
    node._track_confirm_frames = 2
    node._track_max_missed_frames = 5
    tracks = [LineTrack(pos=5.0, angle_offset=0.0, confidence=0.5)]
    observation = TileLine(pos=8.0, angle_offset=0.01, weight=10.0)
    match = LineMatch(
        track=tracks[0],
        observation=observation,
        observation_idx=0,
        pos_delta=3.0,
        angle_delta=0.01,
    )

    LkTotalTransformNode._update_track_family(
        node,
        tracks,
        [observation],
        [match],
        [],
        scene_delta=3.0,
    )

    assert tracks[0].pos == pytest.approx(8.0)
    assert tracks[0].angle_offset == pytest.approx(0.01)
    assert tracks[0].confirmed is True
    assert tracks[0].missed_count == 0


def test_reset_pose_clears_tile_line_state_and_publishes_zero():
    node = LkTotalTransformNode.__new__(LkTotalTransformNode)
    node._prev_gray = np.ones((2, 2), dtype=np.uint8)
    node._prev_pts = np.ones((4, 1, 2), dtype=np.float32)
    node._horizontal_tracks = [LineTrack(pos=1.0, angle_offset=0.0, confidence=1.0)]
    node._vertical_tracks = [LineTrack(pos=2.0, angle_offset=0.0, confidence=1.0)]
    node._total_raw = np.array([
        [1.0, 0.0, 5.0],
        [0.0, 1.0, 6.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    node._total_comp = np.array([
        [1.0, 0.0, 7.0],
        [0.0, 1.0, 8.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    node._last_scene_tx_small = 1.0
    node._last_scene_ty_small = 2.0
    node._last_scene_translation_small = np.array([1.0, 2.0], dtype=np.float64)
    node._last_scene_yaw = 0.5
    node._have_grid_orientation = True
    node._horizontal_family_dir = 0.2
    publish_calls = []
    node._publish = lambda: publish_calls.append(True)
    node.get_logger = lambda: _Logger()

    response = LkTotalTransformNode._on_reset_pose(node, None, _Response())

    assert response.success is True
    assert response.message == 'Bottom-camera pose reset'
    assert node._prev_gray is None
    assert node._prev_pts is None
    assert node._horizontal_tracks == []
    assert node._vertical_tracks == []
    assert np.allclose(node._total_raw, np.eye(3))
    assert np.allclose(node._total_comp, np.eye(3))
    assert node._have_grid_orientation is False
    assert publish_calls == [True]
