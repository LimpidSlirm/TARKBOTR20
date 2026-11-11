"""Glare regression: real recording crop plus controlled colour/exposure cases."""
import json
import hashlib
from pathlib import Path
import unittest

import cv2
import numpy as np

from lane_vision.pipeline import LaneDetector, VisionConfig


ROOT = Path(__file__).resolve().parents[1]


def track_detector(**overrides):
    settings = json.loads((ROOT / 'config.track.json').read_text(encoding='utf-8'))
    settings.update(overrides)
    return LaneDetector(VisionConfig(**settings))


class LightingTests(unittest.TestCase):
    def test_recorded_pale_yellow_retained_and_bright_blue_rejected(self):
        image = cv2.imdecode(np.fromfile(str(ROOT / 'tests/fixtures/lighting-road.png'), np.uint8), 1)
        self.assertIsNotNone(image)
        metadata = json.loads((ROOT / 'tests/fixtures/lighting-road.json').read_text(encoding='utf-8'))
        self.assertEqual(image.shape, (340, 480, 3))
        self.assertEqual(hashlib.sha256((ROOT / 'tests/fixtures/lighting-road.png').read_bytes()).hexdigest(), metadata['sha256'])
        # Crop starts at raw camera row 140; these boxes are manually checked floor/line interiors.
        old = LaneDetector(VisionConfig(roi_top=0, roi_bottom=1)).segment(image)[0]
        mask, bands = track_detector(roi_top=0, roi_bottom=1).segment(image)
        line = np.s_[268:279, 238:249]
        reflection = np.s_[180:210, 390:420]
        self.assertGreater(np.mean(old[reflection] > 0), 0.95)
        self.assertLess(np.mean(mask[reflection] > 0), 0.01)
        self.assertGreater(np.mean(bands['yellow'][line] > 0), 0.90)
        self.assertEqual(set(bands), {'yellow', 'red_low', 'red_high'})

    def test_pale_yellow_under_brightness_changes_and_red_wrap(self):
        detector = track_detector(roi_top=0, roi_bottom=1)
        for hue, saturation, value, band in [(37, 50, 100, 'yellow'), (37, 50, 235, 'yellow'),
                                              (3, 160, 180, 'red_low'), (176, 160, 180, 'red_high')]:
            with self.subTest(hsv=(hue, saturation, value)):
                image = cv2.cvtColor(np.full((80, 100, 3), [hue, saturation, value], np.uint8), cv2.COLOR_HSV2BGR)
                mask, bands = detector.segment(image)
                self.assertTrue(np.all(mask[5:-5, 5:-5] > 0))
                self.assertTrue(np.all(bands[band][5:-5, 5:-5] > 0))

    def test_white_glare_and_completely_washed_out_lines_are_not_invented(self):
        detector = track_detector()
        for level in (190, 235, 255):
            image = np.full((480, 640, 3), level, np.uint8)
            result, _, mask, _ = detector.process(image)
            self.assertFalse(mask.any())
            self.assertEqual(result['road_state'], 'LOST')
            self.assertFalse(result['metric_valid'])
            self.assertFalse(result['command_sent'])

    def test_display_candidates_match_cleaned_roi_mask(self):
        image = np.zeros((100, 100, 3), np.uint8)
        image[:, 20:25] = (0, 255, 255)
        image[70, 60] = (0, 255, 255)  # Isolated noise must disappear from the preview too.
        mask, bands = track_detector().segment(image)
        merged = np.bitwise_or.reduce(list(bands.values()))
        np.testing.assert_array_equal(mask, merged)
        self.assertFalse(merged[:40].any())
        self.assertFalse(merged[96:].any())
        self.assertEqual(merged[70, 60], 0)


if __name__ == '__main__':
    unittest.main()
