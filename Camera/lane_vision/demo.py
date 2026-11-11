"""Synthetic painted lanes and known geometry; never a real-camera calibration."""
import cv2
import numpy as np

from .geometry import GroundCalibration, build_calibration


def control_points():
    pixels = np.array([[80, 70], [560, 70], [560, 470], [80, 470]], dtype=float)
    validation = np.array([[220, 180], [420, 360]], dtype=float)
    def ground(points):
        return np.column_stack(((480 - points[:, 1]) / 300 + 0.1, (320 - points[:, 0]) / 300)).tolist()
    return dict(calibration_id='SYNTHETIC_DEMO_NOT_FOR_REAL_CAMERA', image_size=[640, 480],
                input_is_undistorted=True, camera_matrix=None, synthetic=True,
                fit_image_points_px=pixels.tolist(), fit_ground_points_m=ground(pixels),
                validation_image_points_px=validation.tolist(), validation_ground_points_m=ground(validation),
                max_validation_error_m=0.005)


def calibration():
    return GroundCalibration.from_dict(build_calibration(control_points()))


def frame(kind='left', dashed=False, extra_lane=False, gap=None, noise=False):
    image = np.full((480, 640, 3), 30, dtype=np.uint8)
    y = np.arange(70, 471)
    x_ground = (480 - y) / 300 + 0.1
    if kind == 'left':
        center = 0.10 * x_ground ** 2
    elif kind == 'right':
        center = -0.10 * x_ground ** 2
    elif kind == 's':
        center = 0.25 * (x_ground - 0.75) ** 3 - 0.07 * (x_ground - 0.75)
    elif kind == 'offset':
        center = np.full_like(x_ground, 0.08)
    else:
        center = np.zeros_like(x_ground)
    if extra_lane:
        columns = np.full_like(y, 575)
        cv2.polylines(image, [np.column_stack((columns, y)).astype(np.int32)], False, (255, 255, 255), 6)
    for side, color in ((0.3, (255, 255, 255)), (-0.3, (0, 255, 255))):
        columns = 320 - 300 * (center + side)
        for start in range(0, len(y) - 1):
            row = y[start]
            if dashed and (row // 30) % 2 == 0:
                continue
            if gap and gap[0] <= row <= gap[1]:
                continue
            cv2.line(image, (round(columns[start]), int(row)),
                     (round(columns[start + 1]), int(y[start + 1])), color, 6)
    if noise:
        rng = np.random.default_rng(123)
        for x, row in rng.integers([0, 0], [640, 480], size=(250, 2)):
            image[row, x] = 255
    return image
