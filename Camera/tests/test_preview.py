"""Keep visible fragments distinct from supported lane/center geometry."""
import unittest

import cv2
import numpy as np

from lane_vision import demo
from lane_vision.display import render
from lane_vision.pipeline import LaneDetector


class PreviewTests(unittest.TestCase):
    def test_single_boundary_remains_visible_without_a_center(self):
        image = demo.frame('straight')
        image[:, 320:] = 30
        result, view, mask, bands = LaneDetector().process(image)
        self.assertEqual(result['road_state'], 'LOST')
        self.assertGreater(len(result['boundary_fragments']), 0)
        self.assertFalse(result['centerline_available'])
        self.assertEqual(result['pairing_diagnostics']['stop_reason'], 'near_seed_missing')
        self.assertTrue(all(not zone['centerline_available'] for zone in result['relative_image_zones']))
        overlay = render(view, mask, bands, result)
        # Orange outlines exist even though no paired centerline exists.
        self.assertGreater(np.count_nonzero(np.all(overlay == (0, 165, 255), axis=2)), 0)

    def test_short_support_does_not_relabel_near_curve_as_far(self):
        detector = LaneDetector(calibration=demo.calibration())
        full = detector.process(demo.frame('straight'))[0]
        image = demo.frame('straight')
        image[:340] = 30
        short = detector.process(image)[0]
        self.assertTrue(short['centerline_available'])
        self.assertEqual([z['row_range_px'] for z in full['relative_image_zones']],
                         [z['row_range_px'] for z in short['relative_image_zones']])
        near, middle, far = short['relative_image_zones']
        self.assertTrue(near['centerline_available'])
        self.assertFalse(middle['centerline_available'])
        self.assertFalse(far['centerline_available'])
        self.assertIsNone(far['normalized_offset'])
        self.assertEqual(far['distance_kind'], 'relative_image_roi')

    def test_partial_pairs_are_logged_but_do_not_create_a_curve(self):
        image = demo.frame('straight')
        image[:400] = 30
        result = LaneDetector().process(image)[0]
        self.assertGreater(len(result['paired_observations']), 0)
        self.assertLess(len(result['paired_observations']), 16)
        self.assertEqual(result['pixel_samples'], [])
        self.assertFalse(result['centerline_available'])
        self.assertEqual(result['pairing_diagnostics']['stop_reason'], 'long_gap')

    def test_ambiguous_fit_is_not_drawn_as_a_valid_green_path(self):
        image = demo.frame('straight')
        cv2.line(image, (210, 160), (210, 250), (255, 255, 255), 6)
        result, view, mask, bands = LaneDetector().process(image)
        self.assertEqual(result['road_state'], 'AMBIGUOUS')
        self.assertTrue(result['pixel_samples'])  # A local fit exists before the competing branch.
        self.assertFalse(result['centerline_available'])
        overlay = render(view, mask, bands, result)
        self.assertEqual(np.count_nonzero(np.all(overlay == (0, 255, 0), axis=2)), 0)

    def test_disconnected_marks_remain_separate_fragments(self):
        image = np.full((480, 640, 3), 30, np.uint8)
        cv2.line(image, (200, 180), (200, 270), (0, 255, 255), 8)
        cv2.line(image, (200, 360), (200, 450), (0, 255, 255), 8)
        result = LaneDetector().process(image)[0]
        self.assertEqual(len(result['boundary_fragments']), 2)
        for fragment in result['boundary_fragments']:
            _, y, _, height = fragment['bbox_xywh']
            self.assertTrue(y + height < 300 or y > 330)
        self.assertFalse(result['centerline_available'])

    def test_blank_frame_clears_fragments_pairs_and_zones(self):
        detector = LaneDetector()
        self.assertTrue(detector.process(demo.frame())[0]['centerline_available'])
        result = detector.process(np.full((480, 640, 3), 30, np.uint8))[0]
        self.assertEqual(result['boundary_fragments'], [])
        self.assertEqual(result['paired_observations'], [])
        self.assertEqual(result['boundary_fragment_count'], 0)
        self.assertFalse(result['centerline_available'])
        self.assertTrue(all(zone['observed_pair_rows'] == 0 for zone in result['relative_image_zones']))
        self.assertFalse(result['command_sent'])


if __name__ == '__main__':
    unittest.main()
