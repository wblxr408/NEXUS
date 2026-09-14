"""Per-anchor static range statistics; no change to raw observations."""

from collections import deque
import math
from statistics import fmean, median, pstdev


class RobustRangeWindow:
    def __init__(self, size=200, stale_s=5.0):
        if size < 1 or stale_s <= 0:
            raise ValueError('window size and stale timeout must be positive')
        self.rows = deque(maxlen=size)
        self.stale_s = stale_s
        self.last_arrival = None
        self.identity = None

    def push(self, packet, arrival):
        if packet.get('unit') != 'm':
            raise ValueError('range unit must be m')
        ids = packet['anchor_ids']
        values = packet['ranges_m']
        if not ids or len(set(ids)) != len(ids) or len(values) != len(ids):
            raise ValueError('anchor/range mapping is invalid')
        stamp = int(packet['sample_timestamp_ns'])
        if stamp <= 0:
            raise ValueError('sample timestamp must be positive')
        identity = (packet['tag_id'], tuple(ids), packet.get('frame_id'),
                    packet.get('timestamp_domain'))
        if (identity != self.identity or
                (self.last_arrival is not None and arrival - self.last_arrival > self.stale_s)):
            self.rows.clear()
        if self.rows and stamp <= self.rows[-1][0]:
            return None
        clean = [float(v) if isinstance(v, (float, int)) and
                 not isinstance(v, bool) and math.isfinite(v) and 0 < v < 655.35
                 else None for v in values]
        self.identity = identity
        self.last_arrival = arrival
        self.rows.append((stamp, clean))
        means, stats = [], []
        for index in range(len(ids)):
            samples = [(s, r[index]) for s, r in self.rows if r[index] is not None]
            vals = [v for _, v in samples]
            if vals:
                centre = median(vals)
                mad = median(abs(v - centre) for v in vals)
                threshold = max(3 * 1.4826 * mad, .01)
                kept = [(s, v) for s, v in samples if abs(v - centre) <= threshold]
                good = [v for _, v in kept]
                means.append(fmean(good))
                stats.append({'kept': len(good), 'rejected': len(vals) - len(good),
                              'invalid': len(self.rows) - len(vals),
                              'std_m': pstdev(good), 'threshold_m': threshold,
                              'effective_timestamp_ns': sum(s for s, _ in kept) // len(kept)})
            else:
                means.append(None)
                stats.append({'kept': 0, 'rejected': 0, 'invalid': len(self.rows),
                              'std_m': None, 'threshold_m': None,
                              'effective_timestamp_ns': None})
        return {'schema_version': 1, 'unit': 'm', 'tag_id': packet['tag_id'],
                'anchor_ids': list(ids), 'frame_id': packet.get('frame_id'),
                'timestamp_domain': packet.get('timestamp_domain'),
                'sample_timestamp_ns': stamp, 'raw_ranges_m': clean,
                'ranges_m': means, 'statistics': stats,
                'window_samples': len(self.rows), 'window_capacity': self.rows.maxlen,
                'window_start_ns': self.rows[0][0], 'window_end_ns': stamp,
                'window_duration_s': (stamp - self.rows[0][0]) / 1e9,
                'ready': len(self.rows) == self.rows.maxlen, 'stale': False,
                'method': 'per_anchor_median_mad_3sigma_mean',
                'usage': 'static_window_statistics_not_instantaneous_ranges'}
