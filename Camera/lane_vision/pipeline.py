"""Stateless initial lane detector: HSV bands -> paired scanlines -> geometry.

No last-frame extrapolation, no traffic sign decisions and no movement output.
"""
from dataclasses import dataclass
import time

import cv2
import numpy as np
from numpy.polynomial import Polynomial

from .geometry import GroundCalibration, project, road_curvature, robust_fit, tracking_curvature


@dataclass
class VisionConfig:
    roi_top: float = 0.25
    roi_bottom: float = 0.96
    reference_x: float = 0.5
    scan_step: int = 6
    min_run_px: int = 3
    max_run_px: int = 30
    min_lane_width_fraction: float = 0.15
    max_lane_width_fraction: float = 0.75
    max_step_shift_px: float = 22.0
    max_width_change_px: float = 24.0
    ambiguity_margin_px: float = 5.0
    max_gap_rows: int = 3
    min_pairs: int = 16
    min_coverage: float = 0.55
    degree: int = 3
    pixel_rmse_limit: float = 4.0
    metric_rmse_limit: float = 0.02
    min_metric_span_m: float = 0.15
    min_lane_width_m: float = 0.15
    max_lane_width_m: float = 1.0
    max_heading_slope: float = 1.5
    lookahead_x_m: float = 0.6
    colors: list | None = None

    def __post_init__(self):
        if self.colors is None:
            self.colors = [dict(name='white', lower=[0, 0, 170], upper=[179, 85, 255]),
                           dict(name='yellow', lower=[15, 75, 100], upper=[40, 255, 255])]
        numeric = [value for value in vars(self).values() if isinstance(value, (int, float))]
        if not np.isfinite(numeric).all():
            raise ValueError('Configuration must be finite')
        if not 0 <= self.roi_top < self.roi_bottom <= 1 or not 0 < self.reference_x < 1:
            raise ValueError('Invalid ROI/reference')
        if not 0 < self.min_lane_width_fraction < self.max_lane_width_fraction < 1:
            raise ValueError('Invalid pixel lane-width bounds')
        if self.degree not in (1, 2, 3) or self.min_pairs < self.degree + 3:
            raise ValueError('Invalid polynomial degree/support')
        for key in ('scan_step', 'min_run_px', 'max_run_px', 'max_gap_rows', 'min_pairs', 'degree'):
            if type(getattr(self, key)) is not int:
                raise ValueError(key + ' must be an integer')
        if self.scan_step < 1 or self.min_run_px < 1 or self.max_run_px < self.min_run_px or self.max_gap_rows < 0:
            raise ValueError('Invalid scan/gap settings')
        for key in ('max_step_shift_px', 'max_width_change_px', 'ambiguity_margin_px', 'pixel_rmse_limit',
                    'metric_rmse_limit', 'min_metric_span_m', 'min_lane_width_m', 'max_heading_slope', 'lookahead_x_m'):
            if getattr(self, key) <= 0:
                raise ValueError(key + ' must be positive')
        if not 0 < self.min_coverage <= 1 or self.max_lane_width_m <= self.min_lane_width_m:
            raise ValueError('Invalid coverage/metric width')
        if not self.colors:
            raise ValueError('At least one HSV band is required')
        names = set()
        for band in self.colors:
            if not isinstance(band.get('name'), str) or not band['name'].strip() or band['name'] in names:
                raise ValueError('HSV band names must be nonempty and unique')
            names.add(band['name'])
            low, high = np.asarray(band['lower']), np.asarray(band['upper'])
            if low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all():
                raise ValueError('Invalid HSV band')
            if np.any(low < 0) or np.any(high > [179, 255, 255]) or np.any(low > high):
                raise ValueError('HSV bounds out of range; split wrapping hue bands')


class LaneDetector:
    def __init__(self, config=None, calibration: GroundCalibration | None = None):
        self.config = config or VisionConfig()
        self.calibration = calibration

    def segment(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = {band['name']: cv2.inRange(hsv, np.uint8(band['lower']), np.uint8(band['upper']))
                 for band in self.config.colors}
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        for value in masks.values():
            mask |= value
        # Small opening removes speckles; no large closing across dashed gaps.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        h = len(mask)
        mask[:int(h * self.config.roi_top)] = 0
        mask[int(h * self.config.roi_bottom):] = 0
        # Show only candidates actually used for pairing, including ROI and cleanup.
        masks = {name: cv2.bitwise_and(value, mask) for name, value in masks.items()}
        return mask, masks

    def _runs(self, row):
        edges = np.diff(np.r_[0, row > 0, 0].astype(np.int8))
        starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
        return [float((a + b - 1) / 2) for a, b in zip(starts, ends)
                if self.config.min_run_px <= b - a <= self.config.max_run_px]

    def _fragments(self, masks):
        """Observed colour components for diagnostics, without assigning lane sides."""
        fragments = []
        for name, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 6:
                    continue
                points = cv2.approxPolyDP(contour, 1.0, True).reshape(-1, 2)
                fragments.append(dict(band=name, area_px=float(area),
                                      outline_px=points.astype(int).tolist(),
                                      bbox_xywh=list(map(int, cv2.boundingRect(contour)))))
        fragments.sort(key=lambda item: item['area_px'], reverse=True)
        return fragments[:128], len(fragments)  # Bound log/overlay size; never used as a route.

    def _pairs(self, mask):
        c = self.config
        h, w = mask.shape
        rows = list(range(int(h * c.roi_bottom) - 2, int(h * c.roi_top), -c.scan_step))
        paired, gaps, gap = [], [], 0
        ambiguous = False
        diagnostics = dict(stop_reason='roi_end', stop_row_px=None, missing_scanlines=0)
        seed_limit = max(3, len(rows) // 4)
        for index, y in enumerate(rows):
            runs = self._runs(mask[y])
            candidates = [(a, b) for a, b in zip(runs, runs[1:])
                          if w * c.min_lane_width_fraction <= b - a <= w * c.max_lane_width_fraction]
            scored = []
            for a, b in candidates:
                if not paired:
                    if a < w * c.reference_x < b:
                        scored.append((abs((a + b) / 2 - w * c.reference_x), a, b))
                else:
                    _, old_a, old_b = paired[-1]
                    allowance = c.max_step_shift_px * (gap + 1)
                    if abs(a - old_a) <= allowance and abs(b - old_b) <= allowance and abs((b - a) - (old_b - old_a)) <= c.max_width_change_px:
                        scored.append((abs(a - old_a) + abs(b - old_b), a, b))
            scored.sort()
            if scored:
                _, seed_a, seed_b = scored[0]
                if any(sum(abs(run - boundary) <= c.max_step_shift_px for run in runs) > 1
                       for boundary in (seed_a, seed_b)):
                    ambiguous = True
                    diagnostics.update(stop_reason='competing_boundaries', stop_row_px=y)
                    break  # Competing nearby boundaries: do not silently choose a branch.
            if len(scored) > 1 and scored[1][0] - scored[0][0] <= c.ambiguity_margin_px:
                ambiguous = True
                diagnostics.update(stop_reason='competing_pairs', stop_row_px=y)
                break
            if scored:
                _, a, b = scored[0]
                if gap and paired:
                    gaps.append(gap)
                paired.append((float(y), a, b))
                gap = 0
            elif paired:
                gap += 1
                if gap > c.max_gap_rows:
                    diagnostics.update(stop_reason='long_gap', stop_row_px=y, missing_scanlines=gap)
                    break  # Do not reconnect a far branch across a long missing region.
            elif index >= seed_limit:
                diagnostics.update(stop_reason='near_seed_missing', stop_row_px=y)
                break  # No near-lane seed: cannot guess a distant lane.
        return np.asarray(paired, dtype=float).reshape(-1, 3), gaps, ambiguous, diagnostics

    def _preview_zones(self, result, paired, height):
        """Fixed thirds of the configured image ROI; these are NOT metric distances."""
        c = self.config
        edges = np.linspace(int(height * c.roi_top), int(height * c.roi_bottom), 4)
        scan_rows = np.asarray(list(range(int(height * c.roi_bottom) - 2,
                                         int(height * c.roi_top), -c.scan_step)))
        zones = []
        for index, name in ((2, 'NEAR'), (1, 'MID'), (0, 'FAR')):
            low, high = edges[index:index + 2]
            rows = paired[(paired[:, 0] >= low) & (paired[:, 0] < high), 0]
            samples = [point for point in result['pixel_samples']
                       if low <= point['row_px'] < high and point['observed']]
            expected = int(np.count_nonzero((scan_rows >= low) & (scan_rows < high)))
            coverage = len(rows) / max(1, expected)
            available = (result['centerline_available'] and len(rows) >= 3 and len(samples) >= 3
                         and coverage >= c.min_coverage)
            offset = float(np.median([point['normalized_offset'] for point in samples])) if available else None
            zones.append(dict(name=name, row_range_px=[float(low), float(high)],
                              observed_pair_rows=int(len(rows)), centerline_available=bool(available),
                              observed_row_coverage=float(coverage), normalized_offset=offset,
                              distance_kind='relative_image_roi'))
        return zones

    def process(self, frame, sequence=0, capture_time_ns=None):
        started = time.perf_counter()
        if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError('Expected uint8 BGR frame')
        h, w = frame.shape[:2]
        if min(h, w) < 32:
            raise ValueError('Image too small')
        cal = self.calibration
        size_ok = cal is not None and cal.image_size == (w, h)
        view = cal.undistort(frame) if size_ok else frame.copy()
        mask, masks = self.segment(view)
        result = dict(frame_seq=int(sequence), capture_time_ns=capture_time_ns,
                      metric_valid=False, road_state='LOST', reason='insufficient_near_lane_support',
                      calibration_id=cal.calibration_id if cal else None,
                      synthetic_calibration=cal.synthetic if cal else False,
                      center_points=[], pixel_samples=[], tracking_preview=None,
                      dashed_candidate=False, command_sent=False)
        result['boundary_bands'] = list(masks)
        fragments, fragment_count = self._fragments(masks)
        result.update(boundary_fragments=fragments, boundary_fragment_count=fragment_count,
                      fragments_truncated=fragment_count > len(fragments))
        if cal and not size_ok:
            result['calibration_warning'] = 'image_size_mismatch'
        paired, gaps, ambiguous, diagnostics = self._pairs(mask)
        result['pairing_diagnostics'] = diagnostics
        result['paired_observations'] = [dict(row_px=float(y), left_px=float(a), right_px=float(b))
                                         for y, a, b in paired]
        preview_paired = paired.copy()
        c = self.config
        result['observed_scanlines'] = len(paired)
        result['roi_rows_px'] = [int(h * c.roi_top), int(h * c.roi_bottom)]
        result['reference_x_px'] = float(w * c.reference_x)
        result['dashed_candidate'] = len(gaps) >= 2
        if len(paired) >= c.min_pairs:
            try:
                left, li, lr = robust_fit(paired[:, 0], paired[:, 1], c.degree, c.pixel_rmse_limit, c.min_pairs)
                right, ri, rr = robust_fit(paired[:, 0], paired[:, 2], c.degree, c.pixel_rmse_limit, c.min_pairs)
                paired = paired[li & ri]
                if len(paired) < c.min_pairs:
                    raise ValueError('Insufficient joint inliers')
                left, _, lr = robust_fit(paired[:, 0], paired[:, 1], c.degree, c.pixel_rmse_limit, c.min_pairs)
                right, _, rr = robust_fit(paired[:, 0], paired[:, 2], c.degree, c.pixel_rmse_limit, c.min_pairs)
                ys = np.linspace(paired[:, 0].max(), paired[:, 0].min(), 45)
                widths = right(ys) - left(ys)
                coverage = len(paired) / (1 + (np.ptp(paired[:, 0]) / c.scan_step))
                if coverage < c.min_coverage or np.any(widths < w * c.min_lane_width_fraction) or np.any(widths > w * c.max_lane_width_fraction):
                    raise ValueError('Lane width/coverage is inconsistent')
                center = (left + right) / 2
                result.update(road_state='AMBIGUOUS' if ambiguous else 'NORMAL',
                              reason='ambiguous_pairing' if ambiguous else 'pixel_only_calibration_required',
                              fit_quality=dict(pixel_rmse=max(lr, rr), coverage=float(coverage)))
                result['pixel_samples'] = [dict(row_px=float(y), left_px=float(left(y)), right_px=float(right(y)),
                                               center_px=float(center(y)),
                                               normalized_offset=float((w * c.reference_x - center(y)) / ((right(y) - left(y)) / 2)),
                                               observed=bool(np.min(np.abs(paired[:, 0] - y)) <= c.scan_step * 0.75)) for y in ys]
                if size_ok and not ambiguous:
                    self._metric(result, paired, cal)
            except ValueError as exc:
                result.update(road_state='AMBIGUOUS', reason=str(exc), metric_valid=False)
        elif ambiguous:
            result.update(road_state='AMBIGUOUS', reason='ambiguous_pairing')
        result['centerline_available'] = result['road_state'] == 'NORMAL' and bool(result['pixel_samples'])
        result['relative_image_zones'] = self._preview_zones(result, preview_paired, h)
        result['process_ms'] = (time.perf_counter() - started) * 1000
        return result, view, mask, masks

    def _metric(self, result, paired, cal):
        c = self.config
        lp = paired[:, [1, 0]]
        rp = paired[:, [2, 0]]
        keep = cal.contains(lp) & cal.contains(rp)
        lp, rp, rows = lp[keep], rp[keep], paired[keep, 0]
        if len(lp) < c.min_pairs:
            raise ValueError('Outside calibrated control-point region')
        lg, rg = project(lp, cal.homography), project(rp, cal.homography)
        left, li, lr = robust_fit(lg[:, 0], lg[:, 1], c.degree, c.metric_rmse_limit, c.min_pairs)
        right, ri, rr = robust_fit(rg[:, 0], rg[:, 1], c.degree, c.metric_rmse_limit, c.min_pairs)
        joint = li & ri
        if np.count_nonzero(joint) < c.min_pairs:
            raise ValueError('Insufficient metric inliers')
        lg, rg, rows = lg[joint], rg[joint], rows[joint]
        left, _, lr = robust_fit(lg[:, 0], lg[:, 1], c.degree, c.metric_rmse_limit, c.min_pairs)
        right, _, rr = robust_fit(rg[:, 0], rg[:, 1], c.degree, c.metric_rmse_limit, c.min_pairs)
        low, high = max(lg[:, 0].min(), rg[:, 0].min(), 0.001), min(lg[:, 0].max(), rg[:, 0].max())
        if high - low < c.min_metric_span_m:
            raise ValueError('No common forward metric support')
        xs = np.linspace(low, high, 45)
        widths = left(xs) - right(xs)
        if np.any(widths < c.min_lane_width_m) or np.any(widths > c.max_lane_width_m):
            raise ValueError('Metric width/coordinate orientation invalid')
        center = Polynomial.fit(xs, (left(xs) + right(xs)) / 2, c.degree).convert()
        if np.any(np.abs(center.deriv()(xs)) > c.max_heading_slope):
            raise ValueError('Heading exceeds same-X midpoint model')
        ys = center(xs)
        pixels = project(np.column_stack((xs, ys)), np.linalg.inv(cal.homography))
        curvature = road_curvature(center, xs)
        samples = []
        for x, y, k, pixel in zip(xs, ys, curvature, pixels):
            observed = bool(np.min(np.abs(rows - pixel[1])) <= c.scan_step * 0.75)
            valid = observed and bool(cal.contains([pixel])[0])
            samples.append(dict(x_m=float(x), y_m=float(y), image_px=pixel.tolist(),
                                kappa_road_inv_m=float(k) if valid else None,
                                radius_m=float(1 / abs(k)) if valid and abs(k) > 1e-5 else None,
                                radius_is_infinite=bool(valid and abs(k) <= 1e-5),
                                heading_rad=float(np.arctan(center.deriv()(x))), observed=observed, valid=valid))
        if sum(p['valid'] for p in samples) < 3:
            raise ValueError('Insufficient observed metric samples')
        result.update(metric_valid=True, reason='ok', center_points=samples,
                      support_x_m=[float(low), float(high)])
        result['fit_quality']['metric_rmse_m'] = max(lr, rr)
        # Only a geometric preview, without speed, PWM, serial or motor command.
        target = min(samples, key=lambda p: abs(p['x_m'] - c.lookahead_x_m))
        if low <= c.lookahead_x_m <= high and target['valid']:
            result['tracking_preview'] = dict(x_m=target['x_m'], y_m=target['y_m'],
                kappa_track_inv_m=float(tracking_curvature(target['x_m'], target['y_m'])), preview_only=True)
