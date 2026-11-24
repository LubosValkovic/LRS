"""Numerical checks for marker projection and timestamped ENU transforms."""

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from rclpy.parameter import Parameter

from assignment2_rover.localization import MarkerLocalizerNode
from assignment2_rover.projection import (
    BodyPose,
    CameraModel,
    PoseHistory,
    marker_translation,
    to_local_enu,
)


@pytest.fixture
def camera():
    return CameraModel(640, 480, 'down_camera_optical',
                       np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]]),
                       np.zeros(5))


def test_solvepnp_recovers_frontal_and_oblique_marker(camera):
    size = 0.6378986866791745
    half = size / 2
    square = np.array([[-half, half, 0.], [half, half, 0.],
                       [half, -half, 0.], [-half, -half, 0.]])
    for rotation in (np.array([np.pi, 0., 0.]), np.array([np.pi - .12, .08, -.05])):
        expected = np.array([.24, -.16, 2.8])
        corners, _ = cv2.projectPoints(square, rotation, expected, camera.matrix, camera.distortion)
        actual, error = marker_translation(corners.reshape(4, 2), camera, size)
        np.testing.assert_allclose(actual, expected, atol=1e-5)
        assert error < 1e-4


def test_projection_rejects_invalid_or_out_of_image_corners(camera):
    corners = np.array([[100., 100.], [200., 100.], [200., 200.], [100., 200.]])
    with pytest.raises(ValueError):
        marker_translation(corners, camera, 0.)
    with pytest.raises(ValueError):
        marker_translation(corners + 1000, camera, .64)


def test_pose_history_interpolates_position_and_full_attitude_without_extrapolation():
    history = PoseHistory()
    history.add(1_000_000_000, 'map', [0., 0., 0.], [0., 0., 0., 1.], 10.)
    end_q = Rotation.from_euler('z', 90, degrees=True).as_quat()
    history.add(1_100_000_000, 'map', [2., 4., 6.], end_q, 10.1)

    pose = history.at(1_050_000_000, max_gap=.1, now=10.2, max_age=.5)
    assert pose is not None and pose.bracket_span == pytest.approx(.1)
    np.testing.assert_allclose(pose.position, [1., 2., 3.])
    np.testing.assert_allclose(Rotation.from_quat(pose.quaternion).as_euler('zxy', degrees=True)[0],
                               45., atol=1e-6)
    assert history.at(1_200_000_000, .2, 10.2, .5) is None
    assert history.at(1_050_000_000, .1, 11., .5) is None


def test_optical_to_enu_uses_camera_mount_and_complete_body_quaternion():
    body_q = Rotation.from_euler('zyx', [35., -12., 8.], degrees=True).as_quat()
    body = BodyPose(1, 'map', np.array([10., -4., 3.]), body_q, 0.)
    mount_q = Rotation.from_euler('x', 180, degrees=True).as_quat()
    optical = np.array([.2, -.3, 2.])
    offset = np.array([.1, .05, -.03])
    expected = body.position + Rotation.from_quat(body_q).apply(
        offset + Rotation.from_quat(mount_q).apply(optical))
    actual = to_local_enu(optical, body, offset, mount_q)
    np.testing.assert_allclose(actual, expected, atol=1e-12)


def test_localizer_declares_usable_geometry_defaults(ros):
    node = ros.make(MarkerLocalizerNode,
                    parameter_overrides=[Parameter('sysid', value=1)])
    assert node.localizer.marker_size == pytest.approx(0.6378986866791745)
    np.testing.assert_allclose(node.localizer.camera_translation, [0., 0., -.03])
    np.testing.assert_allclose(node.localizer.camera_rotation,
                               [0.7071067811865476, -0.7071067811865476, 0., 0.])
