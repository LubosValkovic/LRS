# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Camera geometry and timestamped body poses, independent of ROS nodes."""

from collections import deque
from dataclasses import dataclass
import math

import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    frame: str
    matrix: np.ndarray
    distortion: np.ndarray

    def __post_init__(self):
        k = np.array(self.matrix, dtype=np.float64, copy=True)
        d = np.array(self.distortion, dtype=np.float64, copy=True).reshape(-1)
        if (self.width <= 0 or self.height <= 0 or not self.frame or k.shape != (3, 3)
                or not np.isfinite(k).all() or not np.isfinite(d).all()
                or k[0, 0] <= 0 or k[1, 1] <= 0 or k[0, 1] != 0
                or not np.allclose(k[2], [0, 0, 1]) or d.size not in (0, 4, 5, 8)):
            raise ValueError('invalid camera calibration')
        k.flags.writeable = d.flags.writeable = False
        object.__setattr__(self, 'matrix', k)
        object.__setattr__(self, 'distortion', d)


@dataclass(frozen=True)
class BodyPose:
    stamp: int
    frame: str
    position: np.ndarray
    quaternion: np.ndarray
    received_at: float
    bracket_span: float = 0.0


class PoseHistory:
    def __init__(self, size=100):
        self.samples = deque(maxlen=size)

    def add(self, stamp, frame, position, quaternion, received_at):
        p, q = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
        if (stamp <= 0 or not frame or p.shape != (3,) or q.shape != (4,)
                or not np.isfinite(p).all() or not np.isfinite(q).all()
                or np.linalg.norm(q) < 1e-6):
            self.samples.clear()
            raise ValueError('invalid body pose')
        if self.samples:
            last = self.samples[-1]
            if frame != last.frame or stamp < last.stamp:
                self.samples.clear()  # Different frame or a reset simulation clock.
            elif stamp == last.stamp:
                return
        self.samples.append(BodyPose(stamp, frame, p, q / np.linalg.norm(q), received_at))

    def at(self, stamp, max_gap, now, max_age):
        """Interpolate translation and full attitude. Never extrapolate a pose."""
        if not self.samples or now - self.samples[-1].received_at > max_age:
            return None
        for sample in self.samples:
            if sample.stamp == stamp:
                return sample
        for first, second in zip(self.samples, list(self.samples)[1:]):
            if first.stamp < stamp < second.stamp:
                if max(stamp - first.stamp, second.stamp - stamp) * 1e-9 > max_gap:
                    return None
                alpha = (stamp - first.stamp) / (second.stamp - first.stamp)
                rotation = Slerp([0., 1.], Rotation.from_quat(
                    [first.quaternion, second.quaternion]))(alpha)
                return BodyPose(stamp, first.frame,
                                first.position * (1 - alpha) + second.position * alpha,
                                rotation.as_quat(), min(first.received_at, second.received_at),
                                (second.stamp - first.stamp) * 1e-9)
        return None


def marker_translation(corners, camera, marker_size, max_reprojection=2.5):
    """Estimate the square's centre in optical coordinates using planar PnP."""
    pixels = np.asarray(corners, dtype=np.float64)
    if (pixels.shape != (4, 2) or not np.isfinite(pixels).all()
            or not math.isfinite(marker_size) or marker_size <= 0
            or not math.isfinite(max_reprojection) or max_reprojection <= 0):
        raise ValueError('invalid marker corners or projection parameters')
    if (not cv2.isContourConvex(pixels.astype(np.float32))
            or abs(cv2.contourArea(pixels.astype(np.float32))) < 16
            or np.any(pixels < 0) or np.any(pixels[:, 0] >= camera.width)
            or np.any(pixels[:, 1] >= camera.height)):
        raise ValueError('degenerate or out-of-image marker')
    half = marker_size / 2
    # Same decoded clockwise corner order as ArUco (marker-local Y points up).
    square = np.array([[-half, half, 0], [half, half, 0],
                       [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    # ITERATIVE uses planar homography initialisation for these four corners.
    # OpenCV 4.6 IPPE_SQUARE can return a wrong rotation for an exactly frontal square.
    success, rotation, translation = cv2.solvePnP(
        square, pixels, camera.matrix, camera.distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    if not success or not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ValueError('PnP failed')
    depths = (cv2.Rodrigues(rotation)[0] @ square.T + translation)[2]
    projected, _ = cv2.projectPoints(square, rotation, translation,
                                     camera.matrix, camera.distortion)
    error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - pixels) ** 2, axis=1))))
    if not np.all(depths > 0) or error > max_reprojection:
        raise ValueError('no positive-depth PnP solution within reprojection limit')
    return translation.reshape(3), error


def to_local_enu(optical_position, body, camera_translation, camera_quaternion):
    """p_ENU = p_body + R_ENU_body (t_body_camera + R_body_optical p_optical)."""
    optical = np.asarray(optical_position, dtype=float)
    offset = np.asarray(camera_translation, dtype=float)
    mount = np.asarray(camera_quaternion, dtype=float)
    if (optical.shape != (3,) or offset.shape != (3,) or mount.shape != (4,)
            or not np.isfinite(optical).all() or not np.isfinite(offset).all()
            or not np.isfinite(mount).all() or np.linalg.norm(mount) < 1e-6):
        raise ValueError('invalid camera mount or optical position')
    relative = offset + Rotation.from_quat(mount).apply(optical)
    return body.position + Rotation.from_quat(body.quaternion).apply(relative)
