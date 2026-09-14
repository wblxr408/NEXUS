"""Height-independent check of recorded ranges against configured anchor pairs.

Run from repository root. This is an offline diagnostic, not a calibration.
"""
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
import yaml


def pair_excess(first, second, baseline):
    return np.maximum(baseline - first - second, np.abs(first - second) - baseline)


def main():
    # Analytic checks: feasible triangle, impossible sum, impossible difference.
    assert pair_excess(3., 4., 5.) <= 0
    assert pair_excess(1., 2., 5.) == 2
    assert pair_excess(8., 2., 5.) == 1
    root = Path(__file__).resolve().parents[3]
    run = Path(__file__).resolve().parent
    source = root / 'data/raw/2026-09-08_E042_platform_acceptance/stationary_v02.jsonl'
    config_path = run / 'platform_experimental.yaml'
    config = yaml.safe_load(config_path.read_text())
    anchors = np.array([config['uwb']['anchors_map_m'][str(i)] for i in range(1, 5)])
    with source.open() as stream:
        rows = [row for line in stream if
                (row := json.loads(line))['message']['mavpackettype'] == 'BATTERY_STATUS']
    raw = np.array([row['message']['voltages'][2:6] for row in rows])
    ranges = raw / 100.
    valid = (raw > 0) & (raw < 65535)
    any_violation = np.zeros(len(rows), dtype=bool)
    pairs = []
    for i, j in itertools.combinations(range(4), 2):
        baseline = float(np.linalg.norm(anchors[i] - anchors[j]))
        excess = pair_excess(ranges[:, i], ranges[:, j], baseline)
        mask = valid[:, i] & valid[:, j]
        failures = mask & (excess > .1)
        any_violation |= failures
        worst = int(np.argmax(np.where(mask, excess, -np.inf)))
        pairs.append({'slots': [i + 1, j + 1], 'baseline_m': baseline,
                      'valid_pairs': int(mask.sum()),
                      'violations_over_0_10_m': int(failures.sum()),
                      'maximum_excess_m': float(excess[worst]),
                      'worst_elapsed_s': rows[worst]['elapsed_s'],
                      'worst_ranges_m': ranges[worst].tolist()})
    result = {
        'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'config_sha256': hashlib.sha256(config_path.read_bytes()).hexdigest(),
        'samples': len(rows), 'pairs': pairs,
        'frames_with_any_violation_over_0_10_m': int(any_violation.sum()),
        'method': '|r_i-r_j| <= anchor_baseline <= r_i+r_j',
        'conditions': [
            'Uses existing slot-to-anchor mapping and configured anchor coordinates.',
            'Operator confirmed stationary; per-slot source timestamps are unavailable.',
            '0.10 m is a reporting margin, not a calibrated noise threshold.',
            'No tag height, pose, lever arm or IMU estimate enters this test.',
            'Violations do not distinguish wrong mapping, stale ranges or ranging errors.'
        ]}
    output = run / 'range_geometry_photo_followup.json'
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
