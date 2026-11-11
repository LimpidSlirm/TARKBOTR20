"""Geometry in vehicle coordinates: X forward, Y left, distances in metres."""
from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np
from numpy.polynomial import Polynomial


def read_json(path):
    def reject(value):
        raise ValueError(f'Non-finite JSON number: {value}')
    return json.loads(Path(path).read_text(encoding='utf-8-sig'), parse_constant=reject)


def array(value, shape=None):
    result = np.asarray(value, dtype=np.float64)
    if not np.isfinite(result).all() or (shape is not None and result.shape != shape):
        raise ValueError('Invalid array shape or non-finite value')
    return result


def project(points, homography):
    points = array(points)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError('Expected Nx2 points')
    q = np.column_stack((points, np.ones(len(points)))) @ homography.T
    if np.any(np.abs(q[:, 2]) < 1e-9):
        raise ValueError('Point projects to the horizon')
    result = q[:, :2] / q[:, 2:3]
    if not np.isfinite(result).all():
        raise ValueError('Projection is not finite')
    return result


def robust_fit(x, y, degree, tolerance, min_points):
    """Scaled polynomial fit with iterative residual rejection; returns support."""
    x, y = array(x), array(y)
    if len(x) < min_points or np.ptp(x) < 1e-8:
        raise ValueError('Insufficient fit support')
    keep = np.ones(len(x), dtype=bool)
    for _ in range(4):
        if np.count_nonzero(keep) < max(degree + 2, min_points):
            raise ValueError('Too few inliers')
        poly = Polynomial.fit(x[keep], y[keep], degree).convert()
        residual = np.abs(poly(x) - y)
        median = np.median(residual[keep])
        mad = np.median(np.abs(residual[keep] - median))
        updated = keep & (residual <= max(tolerance, median + 3.0 * 1.4826 * mad))
        if np.array_equal(updated, keep):
            break
        keep = updated
    if np.count_nonzero(keep) < min_points:
        raise ValueError('Too few inliers')
    poly = Polynomial.fit(x[keep], y[keep], degree).convert()
    rmse = float(np.sqrt(np.mean((poly(x[keep]) - y[keep]) ** 2)))
    if rmse > tolerance:
        raise ValueError('Fit residual exceeds threshold')
    return poly, keep, rmse


def road_curvature(poly, x):
    return poly.deriv(2)(x) / (1.0 + poly.deriv(1)(x) ** 2) ** 1.5


def tracking_curvature(x, y):
    if not np.isfinite([x, y]).all() or x <= 0 or x * x + y * y < 1e-12:
        raise ValueError('Lookahead must be a finite point ahead of the vehicle')
    return 2.0 * y / (x * x + y * y)


@dataclass
class GroundCalibration:
    calibration_id: str
    image_size: tuple
    homography: np.ndarray
    hull: np.ndarray
    max_validation_error_m: float
    camera_matrix: np.ndarray | None
    distortion: np.ndarray | None
    synthetic: bool

    @classmethod
    def from_dict(cls, data):
        size = data['image_size']
        if len(size) != 2 or any(type(v) is not int or v < 16 for v in size):
            raise ValueError('image_size must be [width,height] integers >=16')
        h = array(data['homography'], (3, 3))
        if np.linalg.matrix_rank(h) != 3 or abs(np.linalg.det(h)) < 1e-14:
            raise ValueError('Singular homography')
        support = array(data['fit_image_points_px'])
        if support.ndim != 2 or support.shape[1] != 2 or len(support) < 4:
            raise ValueError('Need at least four fitting points')
        hull = cv2.convexHull(support.astype(np.float32))
        if cv2.contourArea(hull) < 100:
            raise ValueError('Degenerate image control points')
        validation = array(data['validation_image_points_px'])
        truth = array(data['validation_ground_points_m'])
        if validation.ndim != 2 or validation.shape[1] != 2 or len(validation) < 2 or truth.shape != validation.shape:
            raise ValueError('Need at least two independent validation points')
        for p in validation:
            if np.any(np.linalg.norm(support - p, axis=1) < 1e-6):
                raise ValueError('Validation points must not be fitting points')
        for p in np.vstack((support, validation)):
            if not (0 <= p[0] < size[0] and 0 <= p[1] < size[1]):
                raise ValueError('Control point outside image')
        error = np.linalg.norm(project(validation, h) - truth, axis=1)
        limit = float(data.get('max_validation_error_m', 0.02))
        if not np.isfinite(limit) or limit <= 0 or float(error.max()) > limit:
            raise ValueError('Independent ground validation failed')
        matrix = distortion = None
        if data.get('camera_matrix') is not None:
            matrix = array(data['camera_matrix'], (3, 3))
            distortion = array(data['distortion_coefficients']).reshape(-1)
            if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or len(distortion) not in (4, 5, 8, 12, 14):
                raise ValueError('Invalid camera intrinsics/distortion')
        elif data.get('input_is_undistorted') is not True:
            raise ValueError('Provide intrinsics or explicitly confirm already-undistorted input')
        name = data['calibration_id']
        if not isinstance(name, str) or not name.strip():
            raise ValueError('Missing calibration ID')
        return cls(name, tuple(size), h, hull, float(error.max()), matrix,
                   distortion, data.get('synthetic') is True)

    def contains(self, points):
        return np.array([cv2.pointPolygonTest(self.hull, (float(x), float(y)), False) >= 0
                         for x, y in points], dtype=bool)

    def undistort(self, frame):
        if self.camera_matrix is None:
            return frame.copy()
        return cv2.undistort(frame, self.camera_matrix, self.distortion)


def build_calibration(data):
    """Control pixels must be measured in the undistorted image coordinate frame."""
    src, dst = array(data['fit_image_points_px']), array(data['fit_ground_points_m'])
    if src.ndim != 2 or src.shape[1] != 2 or len(src) < 4 or dst.shape != src.shape:
        raise ValueError('Need >=4 paired image/ground control points')
    if np.linalg.matrix_rank(np.column_stack((src, np.ones(len(src))))) < 3 or np.linalg.matrix_rank(np.column_stack((dst, np.ones(len(dst))))) < 3:
        raise ValueError('Collinear control points')
    h, _ = cv2.findHomography(src, dst, method=0)
    if h is None:
        raise ValueError('Cannot solve homography')
    result = dict(data, homography=h.tolist())
    GroundCalibration.from_dict(result)  # Independent points must pass before export.
    return result
