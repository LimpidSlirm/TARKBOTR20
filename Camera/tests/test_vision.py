import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import cv2
import numpy as np
from numpy.polynomial import Polynomial

from lane_vision import demo
from lane_vision.geometry import GroundCalibration, build_calibration, project, road_curvature, tracking_curvature
from lane_vision.pipeline import LaneDetector, VisionConfig

ROOT = Path(__file__).resolve().parents[1]


class CalibrationTests(unittest.TestCase):
    def test_known_ground_points_and_round_trip(self):
        cal = demo.calibration()
        pixels = np.array([[320., 300.], [230., 450.], [410., 140.]])
        ground = project(pixels, cal.homography)
        np.testing.assert_allclose(ground[:, 0], (480 - pixels[:, 1]) / 300 + 0.1, atol=1e-8)
        np.testing.assert_allclose(ground[:, 1], (320 - pixels[:, 0]) / 300, atol=1e-8)
        np.testing.assert_allclose(project(ground, np.linalg.inv(cal.homography)), pixels, atol=1e-7)

    def test_independent_validation_and_intrinsics_required(self):
        for mode in ('bad_truth', 'reused_point', 'unknown_distortion', 'singular'):
            with self.subTest(mode=mode):
                data = build_calibration(demo.control_points())
                if mode == 'bad_truth':
                    data['validation_ground_points_m'][0][0] += 0.1
                elif mode == 'reused_point':
                    data['validation_image_points_px'][0] = data['fit_image_points_px'][0]
                elif mode == 'unknown_distortion':
                    data['input_is_undistorted'] = False
                else:
                    data['homography'] = [[0, 0, 0]] * 3
                with self.assertRaises(ValueError):
                    GroundCalibration.from_dict(data)

    def test_degenerate_calibration_rejected(self):
        data = demo.control_points()
        data['fit_image_points_px'] = [[0, 0], [10, 10], [20, 20], [30, 30]]
        with self.assertRaises(ValueError):
            build_calibration(data)

    def test_zero_distortion_intrinsics_path(self):
        data = build_calibration(demo.control_points())
        data['input_is_undistorted'] = False
        data['camera_matrix'] = [[400, 0, 320], [0, 400, 240], [0, 0, 1]]
        data['distortion_coefficients'] = [0, 0, 0, 0, 0]
        cal = GroundCalibration.from_dict(data)
        result = LaneDetector(calibration=cal).process(demo.frame('straight'))[0]
        self.assertTrue(result['metric_valid'], result['reason'])

    def test_curvature_formulas_and_invalid_lookahead(self):
        self.assertEqual(float(road_curvature(Polynomial([0.1, 0.4]), 0.5)), 0.0)
        self.assertNotEqual(tracking_curvature(1.0, 0.1), 0.0)
        self.assertAlmostEqual(tracking_curvature(0.6, 0.2), 1.0)
        for point in [(0, 0), (-0.1, 0), (float('nan'), 0)]:
            with self.subTest(point=point), self.assertRaises(ValueError):
                tracking_curvature(*point)


class DetectorTests(unittest.TestCase):
    def setUp(self):
        self.detector = LaneDetector(calibration=demo.calibration())

    def process(self, **kwargs):
        return self.detector.process(demo.frame(**kwargs))[0]

    def valid(self, result):
        self.assertTrue(result['metric_valid'], result['reason'])
        self.assertEqual(result['road_state'], 'NORMAL')
        self.assertFalse(result['command_sent'])
        return [p for p in result['center_points'] if p['valid']]

    def test_straight_two_colors_and_no_false_turn(self):
        points = self.valid(self.process(kind='straight'))
        self.assertLess(max(abs(p['kappa_road_inv_m']) for p in points), 1e-5)
        self.assertLess(max(abs(p['y_m']) for p in points), 0.003)
        self.assertTrue(all(p['radius_is_infinite'] and p['radius_m'] is None for p in points))

    def test_parallel_offset_requires_tracking_but_not_road_curvature(self):
        result = self.process(kind='offset')
        points = self.valid(result)
        self.assertLess(max(abs(p['kappa_road_inv_m']) for p in points), 1e-5)
        self.assertAlmostEqual(points[len(points) // 2]['y_m'], 0.08, delta=0.005)
        self.assertGreater(result['tracking_preview']['kappa_track_inv_m'], 0)

    def test_left_right_curvature_and_radius(self):
        for kind, sign in [('left', 1), ('right', -1)]:
            with self.subTest(kind=kind):
                points = self.valid(self.process(kind=kind))
                middle = points[len(points) // 2]
                # Painted Y=sign*0.1*X^2, expected curvature from analytic geometry.
                x = middle['x_m']
                expected = sign * 0.2 / (1 + (0.2 * x) ** 2) ** 1.5
                self.assertAlmostEqual(middle['kappa_road_inv_m'], expected, delta=0.015)
                self.assertAlmostEqual(middle['radius_m'], 1 / abs(expected), delta=0.5)

    def test_s_bend_preserves_both_curvature_signs(self):
        points = self.valid(self.process(kind='s'))
        self.assertLess(min(p['kappa_road_inv_m'] for p in points), -0.1)
        self.assertGreater(max(p['kappa_road_inv_m'] for p in points), 0.1)

    def test_no_calibration_has_no_physical_outputs(self):
        result = LaneDetector().process(demo.frame('left'))[0]
        self.assertEqual(result['road_state'], 'NORMAL')
        self.assertTrue(result['pixel_samples'])
        self.assertFalse(result['metric_valid'])
        self.assertEqual(result['center_points'], [])
        self.assertIsNone(result['tracking_preview'])
        self.assertTrue(all(p['normalized_offset'] >= 0 for p in result['pixel_samples']))

    def test_resolution_mismatch_revokes_metric_validity(self):
        image = cv2.resize(demo.frame(), (320, 240))
        result = self.detector.process(image)[0]
        self.assertFalse(result['metric_valid'])
        self.assertEqual(result['calibration_warning'], 'image_size_mismatch')

    def test_missing_boundary_and_blank_do_not_reuse_old_results(self):
        self.valid(self.process())
        image = demo.frame('straight')
        image[:, 320:] = 30
        for bad in (image, np.full_like(image, 30)):
            with self.subTest(blank=bool(np.all(bad == 30))):
                result = self.detector.process(bad)[0]
                self.assertFalse(result['metric_valid'])
                self.assertEqual(result['road_state'], 'LOST')
                self.assertEqual(result['center_points'], [])

    def test_adjacent_lane_and_noise_do_not_select_outermost_pair(self):
        result = self.process(kind='straight', extra_lane=True, noise=True)
        points = self.valid(result)
        self.assertLess(max(abs(p['y_m']) for p in points), 0.005)
        self.assertLess(max(p['right_px'] for p in result['pixel_samples']), 420)

    def test_long_gap_truncates_support_instead_of_extrapolating(self):
        result = self.process(kind='straight', gap=(190, 330))
        points = self.valid(result)
        self.assertLess(result['support_x_m'][1], 0.61)
        self.assertIsNone(result['tracking_preview'])
        self.assertTrue(all(p['image_px'][1] > 330 for p in points))

    def test_competing_branch_boundaries_are_ambiguous(self):
        image = demo.frame('straight')
        cv2.line(image, (210, 160), (210, 300), (255, 255, 255), 6)
        result = self.detector.process(image)[0]
        self.assertEqual(result['road_state'], 'AMBIGUOUS')
        self.assertFalse(result['metric_valid'])
        self.assertIsNone(result['tracking_preview'])
        self.assertEqual(result['center_points'], [])

    def test_short_dashed_gaps_are_marked_as_prediction(self):
        image = demo.frame('straight')
        for row in range(140, 440, 48):
            image[row:row + 10] = 30
        result = self.detector.process(image)[0]
        self.valid(result)
        self.assertTrue(result['dashed_candidate'])
        self.assertTrue(any(not p['observed'] for p in result['pixel_samples']))
        self.assertTrue(any(not p['valid'] for p in result['center_points']))

    def test_invalid_config_and_wrong_coordinate_orientation(self):
        for config in ({'scan_step': 0}, {'max_gap_rows': -1}, {'roi_top': 0.99},
                       {'lookahead_x_m': float('nan')}, {'degree': 9}):
            with self.subTest(config=config), self.assertRaises(ValueError):
                VisionConfig(**config)
        data = build_calibration(demo.control_points())
        data['homography'][1] = [-value for value in data['homography'][1]]
        data['validation_ground_points_m'] = [[x, -y] for x, y in data['validation_ground_points_m']]
        cal = GroundCalibration.from_dict(data)
        result = LaneDetector(calibration=cal).process(demo.frame())[0]
        self.assertFalse(result['metric_valid'])
        self.assertIn('orientation', result['reason'])


class CliTests(unittest.TestCase):
    def setUp(self):
        folder = ROOT / '.build' / 'tests'
        folder.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=folder)
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.assertTrue(self.folder.resolve().is_relative_to(ROOT.resolve()))

    def run_cli(self, *args, expected=0):
        result = subprocess.run([sys.executable, str(ROOT / 'main.py'), *map(str, args)],
                                text=True, capture_output=True, timeout=45)
        self.assertEqual(result.returncode, expected, result.stderr)
        return result

    def test_headless_demo_outputs_valid_json_and_png(self):
        out = self.folder / 'demo'
        self.run_cli('--demo', '--no-display', '--max-frames', 2, '--output', out)
        rows = [json.loads(line) for line in (out / 'frames.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertEqual(len(rows), 2)
        self.assertTrue(all(r['metric_valid'] and r['source_kind'] == 'synthetic' for r in rows))
        self.assertTrue(all(not r['command_sent'] for r in rows))
        image = cv2.imdecode(np.fromfile(str(out / 'last_frame.png'), dtype=np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(image.shape[1:], (1280, 3))
        self.assertGreater(image.shape[0], 480)  # Header does not hide the far road region.
        self.run_cli('--demo', '--no-display', '--max-frames', 1, '--output', out, expected=2)

    def test_chinese_image_path_pixel_only_and_missing_file(self):
        source = self.folder / '道路照片.png'
        ok, encoded = cv2.imencode('.png', demo.frame('left'))
        self.assertTrue(ok)
        encoded.tofile(str(source))
        out = self.folder / '照片输出'
        self.run_cli('--image', source, '--no-display', '--output', out)
        row = json.loads((out / 'frames.jsonl').read_text(encoding='utf-8'))
        self.assertFalse(row['metric_valid'])
        self.run_cli('--image', self.folder / 'missing.png', '--no-display', '--output', self.folder / 'bad', expected=2)

    def test_recorded_video_input(self):
        source = self.folder / 'lanes.avi'
        writer = cv2.VideoWriter(str(source), cv2.VideoWriter_fourcc(*'MJPG'), 10, (640, 480))
        self.assertTrue(writer.isOpened())
        for kind in ('straight', 'left', 's'):
            writer.write(demo.frame(kind))
        writer.release()
        out = self.folder / 'video'
        self.run_cli('--video', source, '--no-display', '--output', out)
        rows = (out / 'frames.jsonl').read_text(encoding='utf-8').splitlines()
        self.assertEqual(len(rows), 3)

    def test_calibration_export_command(self):
        points = self.folder / 'points.json'
        points.write_text(json.dumps(demo.control_points()), encoding='utf-8')
        output = self.folder / 'calibration.json'
        result = subprocess.run([sys.executable, str(ROOT / 'calibrate_ground.py'), '--points', str(points),
                                 '--output', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        GroundCalibration.from_dict(json.loads(output.read_text(encoding='utf-8')))
        source = self.folder / 'input.png'
        ok, encoded = cv2.imencode('.png', demo.frame('left'))
        self.assertTrue(ok)
        encoded.tofile(str(source))
        out = self.folder / 'calibrated-image'
        self.run_cli('--image', source, '--calibration', output, '--no-display', '--output', out)
        row = json.loads((out / 'frames.jsonl').read_text(encoding='utf-8'))
        self.assertTrue(row['metric_valid'])
