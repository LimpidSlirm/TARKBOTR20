"""Preview lane boundaries from images, videos or an explicitly selected camera."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
if (HERE / '.deps').is_dir():
    sys.path.insert(0, str(HERE / '.deps'))

import cv2
import numpy as np

from lane_vision import demo
from lane_vision.display import render
from lane_vision.geometry import GroundCalibration, read_json
from lane_vision.pipeline import LaneDetector, VisionConfig


def read_image(path):
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f'Cannot decode image: {path}')
    return image


def write_image(path, image):
    ok, encoded = cv2.imencode('.png', image)
    if not ok:
        raise ValueError('Cannot encode output image')
    encoded.tofile(str(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--image', type=Path)
    inputs.add_argument('--video', type=Path)
    inputs.add_argument('--camera', type=int, help='Only this option opens a camera')
    inputs.add_argument('--demo', action='store_true', help='Generated lanes with SYNTHETIC calibration')
    parser.add_argument('--config', type=Path, default=HERE / 'config.example.json')
    parser.add_argument('--calibration', type=Path)
    parser.add_argument('--pixel-only', action='store_true', help='Disable metric geometry, including demo')
    parser.add_argument('--no-display', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--max-frames', type=int, default=0, help='0=until source ends or Q/Escape')
    parser.add_argument('--save-every', type=int, default=30)
    args = parser.parse_args()
    if args.max_frames < 0 or args.save_every < 1:
        parser.error('max-frames must be >=0 and save-every >=1')
    if args.demo and args.calibration:
        parser.error('A real calibration cannot be applied to synthetic demo images')
    config = VisionConfig(**read_json(args.config))
    calibration = None
    if not args.pixel_only:
        if args.demo:
            calibration = demo.calibration()
        elif args.calibration:
            calibration = GroundCalibration.from_dict(read_json(args.calibration))
    out = args.output or HERE / 'output' / datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    if out.exists() and any(out.iterdir()):
        raise ValueError('Output directory must be empty (preserve previous records)')
    out.mkdir(parents=True, exist_ok=True)
    capture = None
    if args.video or args.camera is not None:
        capture = cv2.VideoCapture(str(args.video) if args.video else args.camera)
        if not capture.isOpened():
            capture.release()
            raise ValueError('Cannot open video/camera source')
    detector = LaneDetector(config, calibration)
    kind = 'synthetic' if args.demo else 'image' if args.image else 'video' if args.video else 'camera'
    frames = 0
    last_result = None
    last_overlay = None
    last_read_ns = None
    try:
        with (out / 'frames.jsonl').open('w', encoding='utf-8') as log:
            while True:
                if args.image:
                    if frames:
                        break
                    image = read_image(args.image)
                elif args.demo:
                    if frames >= 60:
                        break
                    image = demo.frame(['straight', 'left', 'right', 's'][frames // 15], noise=True)
                else:
                    ok, image = capture.read()
                    if not ok:
                        if frames == 0 or kind == 'camera':
                            raise ValueError('Empty frame/camera stream interrupted')
                        break
                # This is read completion, not a hardware exposure timestamp.
                read_ns = time.monotonic_ns()
                result, view, mask, bands = detector.process(image, frames, read_ns)
                result['source_kind'] = kind
                result['capture_time_source'] = 'read_completion_estimate'
                result['observed_fps'] = 1e9 / (read_ns - last_read_ns) if last_read_ns is not None and read_ns > last_read_ns else None
                last_read_ns = read_ns
                position = float(capture.get(cv2.CAP_PROP_POS_MSEC)) if args.video else None
                result['source_position_ms'] = position if position is not None and np.isfinite(position) else None
                log.write(json.dumps(result, ensure_ascii=False, allow_nan=False) + '\n')
                overlay = render(view, mask, bands, result)
                if frames % args.save_every == 0:
                    write_image(out / f'frame_{frames:06d}.png', overlay)
                last_overlay, last_result = overlay, result
                frames += 1
                if not args.no_display:
                    cv2.imshow('Lane preview | Q/Escape to quit | NO MOTOR OUTPUT', overlay)
                    key = cv2.waitKey(0 if args.image else 30) & 0xff
                    if key in (27, ord('q')):
                        break
                if args.max_frames and frames >= args.max_frames:
                    break
        if last_overlay is not None:
            write_image(out / 'last_frame.png', last_overlay)
        (out / 'run.json').write_text(json.dumps(dict(
            source_kind=kind, source=str(args.image if args.image else args.video if args.video else args.camera if args.camera is not None else 'generated_demo'),
            opencv_version=cv2.__version__, numpy_version=np.__version__,
            config=vars(config), calibration_id=calibration.calibration_id if calibration else None,
            frames=frames, command_sent=False), ensure_ascii=False, allow_nan=False, indent=2), encoding='utf-8')
        print(json.dumps(dict(frames=frames, output=str(out.resolve()),
                              metric_valid=last_result['metric_valid'] if last_result else False,
                              command_sent=False), ensure_ascii=False))
    finally:
        if capture is not None:
            capture.release()
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, OSError, cv2.error) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        raise SystemExit(2)
