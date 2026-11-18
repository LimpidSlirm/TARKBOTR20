"""OpenCV overlay with English labels (no system CJK-font dependency)."""
import cv2
import numpy as np


def render(frame, mask, bands, result):
    left = frame.copy()
    h, w = mask.shape
    right = np.zeros_like(left)
    palette = [(220, 220, 220), (0, 220, 220), (220, 80, 220), (220, 180, 40)]
    for index, (name, value) in enumerate(bands.items()):
        color = (0, 220, 220) if 'yellow' in name else (0, 60, 255) if 'red' in name else palette[index % len(palette)]
        right[value > 0] = color
    # Preserve every retained fragment even when no left/right lane can be assigned.
    # Orange outlines represent colour observations, never a selected road/path.
    for fragment in result['boundary_fragments']:
        points = np.asarray(fragment['outline_px'], dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(left, [points], True, (0, 165, 255), 2, cv2.LINE_AA)
    for point in result['paired_observations']:
        for field, color in [('left_px', (255, 180, 0)), ('right_px', (0, 255, 255))]:
            pixel = (round(point[field]), round(point['row_px']))
            cv2.circle(left, pixel, 3, color, -1)
            cv2.circle(right, pixel, 3, color, -1)
    # A rejected fit must not appear as a valid green path.
    samples = result['pixel_samples'] if result['centerline_available'] else []
    for a, b in zip(samples, samples[1:]):
        observed = a['observed'] and b['observed']
        for field, color in [('left_px', (255, 180, 0)), ('right_px', (0, 255, 255)), ('center_px', (0, 255, 0))]:
            p = (round(a[field]), round(a['row_px']))
            q = (round(b[field]), round(b['row_px']))
            if observed:
                cv2.line(left, p, q, color, 2)
                cv2.line(right, p, q, color, 2)
            else:
                cv2.circle(left, p, 2, (200, 80, 240), -1)
                cv2.circle(right, p, 2, (200, 80, 240), -1)
    for zone in result['relative_image_zones']:
        low, high = zone['row_range_px']
        label = f"{zone['name']} pairs={zone['observed_pair_rows']} "
        label += f"e={zone['normalized_offset']:+.2f}" if zone['centerline_available'] else 'CENTER N/A'
        for panel in (left, right):
            y = round(low)
            for x in range(0, w, 14):
                cv2.line(panel, (x, y), (min(x + 6, w - 1), y), (110, 110, 110), 1)
            text_y = min(h - 5, round((low + high) / 2))
            cv2.putText(panel, label, (8, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(panel, label, (8, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (240, 240, 240), 1, cv2.LINE_AA)
    top, bottom = result['roi_rows_px']
    cv2.rectangle(left, (0, top), (w - 1, bottom - 1), (120, 120, 120), 1)
    ref = round(result['reference_x_px'])
    for y in range(top, bottom, 14):
        cv2.line(left, (ref, y), (ref, min(y + 6, bottom - 1)), (150, 150, 150), 1)
    valid = [p for p in result['center_points'] if p['valid']]
    rows = [f"STATE: {result['road_state']}  metric={result['metric_valid']}",
            result['reason'][:70], f"processing {result['process_ms']:.1f} ms | NO MOTOR OUTPUT"]
    rows.append('Boundary bands: ' + ', '.join(result['boundary_bands']))
    rows.append('CENTER: supported pixel curve' if result['centerline_available'] else 'CENTER: UNAVAILABLE - fragments are not a path')
    rows.append(f"Fragments {len(result['boundary_fragments'])}/{result['boundary_fragment_count']} | paired rows {result['observed_scanlines']}")
    rows.append('Pairing stopped: ' + result['pairing_diagnostics']['stop_reason'])
    rows.append('Zones: fixed IMAGE ROI thirds, NOT meters')
    if result['synthetic_calibration']:
        rows.insert(0, 'SYNTHETIC DEMO - NOT REAL VEHICLE DATA')
    if result.get('observed_fps') is not None:
        rows[2] += f" | preview FPS {result['observed_fps']:.1f}"
    if valid:
        mid = valid[len(valid) // 2]
        radius = 'infinite' if mid['radius_is_infinite'] else f"{mid['radius_m']:.2f}"
        rows.append(f"X={mid['x_m']:.2f}m Y={mid['y_m']:+.3f}m k={mid['kappa_road_inv_m']:+.3f}/m R={radius}m")
    else:
        rows.append('Physical curvature/radius: UNAVAILABLE')
    rows.append(f"dashed candidate={result['dashed_candidate']} | purple=interpolation")
    rows.append('Orange=colour fragments; green=supported center')
    header = np.full((22 * len(rows) + 10, w * 2, 3), 15, dtype=np.uint8)
    for offset in (0, w):
        for index, text in enumerate(rows):
            cv2.putText(header, text, (offset + 8, 20 + 22 * index), cv2.FONT_HERSHEY_SIMPLEX,
                        min(0.46, w / 1400), (230, 230, 230), 1, cv2.LINE_AA)
    if valid:
        # Signed curvature profile along measured forward distance.
        x0, y0, width, height = 25, h - 100, w - 50, 80
        cv2.rectangle(right, (x0, y0), (x0 + width, y0 + height), (30, 30, 30), -1)
        cv2.line(right, (x0, y0 + height // 2), (x0 + width, y0 + height // 2), (120, 120, 120), 1)
        scale = max(0.2, max(abs(p['kappa_road_inv_m']) for p in valid))
        low, high = result['support_x_m']
        points = [(int(x0 + (p['x_m'] - low) / (high - low) * width),
                   int(y0 + height / 2 - p['kappa_road_inv_m'] / scale * height * 0.4)) for p in valid]
        cv2.polylines(right, [np.asarray(points, dtype=np.int32)], False, (0, 240, 0), 1)
        cv2.putText(right, 'road curvature vs forward X (m)', (x0, y0 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (230, 230, 230), 1)
    return np.vstack((header, np.hstack((left, right))))
