"""Fit a ground homography from measured, already-undistorted image points."""
import argparse
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
if (HERE / '.deps').is_dir():
    sys.path.insert(0, str(HERE / '.deps'))

from lane_vision.geometry import build_calibration, read_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--points', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('Output already exists; use a new calibration version')
    result = build_calibration(read_json(args.points))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print('PASS: homography fitted and independent ground points validated')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, OSError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        raise SystemExit(2)
