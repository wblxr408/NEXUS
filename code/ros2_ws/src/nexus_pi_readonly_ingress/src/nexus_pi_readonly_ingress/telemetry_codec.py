"""ROS-independent validation and projection of adapter JSON."""

import math

from .time_alignment import BootTimeAligner


def finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


class TelemetryCodec:
    def __init__(self, expected_tag_id=2, anchor_ids=("1", "2", "3", "4"), range_variance_m2=None,
                 expected_imu_period_us=5000):
        self.expected_tag_id = int(expected_tag_id)
        self.anchor_ids = [str(item) for item in anchor_ids]
        self.range_variance_m2 = list(range_variance_m2 or [0.0025] * len(self.anchor_ids))
        if len(self.anchor_ids) != len(self.range_variance_m2):
            raise ValueError("anchor IDs and variances must have equal length")
        self.aligner = BootTimeAligner()
        self.uwb_clock_aligner = BootTimeAligner(reset_threshold_ns=1_000_000_000)
        self.last_imu_source_us = None
        self.last_uwb_stamp_ns = None
        self.expected_imu_period_us = int(expected_imu_period_us)
        self.estimated_missing_imu_samples = 0
        self.nonmonotonic_imu_samples = 0

    def imu_packet(self, state, arrival_unix_ns):
        imu = state.get("imu") or {}
        sample_us = finite(imu.get("sample_timestamp_us"))
        if sample_us is None or sample_us <= 0 or int(sample_us) == self.last_imu_source_us:
            return None
        accel, gyro = imu.get("acceleration_mps2") or {}, imu.get("angular_velocity_rps") or {}
        values = [finite(accel.get(axis)) for axis in "xyz"] + [finite(gyro.get(axis)) for axis in "xyz"]
        if any(value is None for value in values):
            return None
        domain = imu.get("sample_timestamp_domain")
        if domain == "flight_boot_unverified":
            stamp_ns = self.aligner.align_us(int(sample_us), arrival_unix_ns)
            output_domain = "ros_unix_ns_aligned_from_flight_boot"
        elif domain == "ros_unix_ns":
            stamp_ns = int(sample_us * 1000)
            output_domain = domain
        else:
            return None
        source_us = int(sample_us)
        if self.last_imu_source_us is not None:
            delta = source_us - self.last_imu_source_us
            if delta <= 0:
                self.nonmonotonic_imu_samples += 1
            elif delta > self.expected_imu_period_us * 1.5:
                self.estimated_missing_imu_samples += max(0, round(delta / self.expected_imu_period_us) - 1)
        self.last_imu_source_us = source_us
        return {"stamp_ns": stamp_ns, "acceleration": values[:3], "angular_velocity": values[3:],
                "timestamp_domain": output_domain,
                "latency_ms": max(0.0, (int(arrival_unix_ns) - stamp_ns) / 1e6)}

    def uwb_packet(self, state, arrival_unix_ns):
        uwb = state.get("uwb") or {}
        if int(uwb.get("tag_id", -1)) != self.expected_tag_id:
            return None
        ranges = [finite(value) for value in (uwb.get("anchor_ranges_m") or [])]
        if len(ranges) != len(self.anchor_ids) or sum(value is not None and value > 0 for value in ranges) < 3:
            return None
        stamp = finite(uwb.get("sample_timestamp_ns"))
        domain = uwb.get("sample_timestamp_domain")
        if stamp is None or domain != "ros_unix_ns_receive":
            return None
        source_stamp_ns = int(stamp)
        if source_stamp_ns == self.last_uwb_stamp_ns:
            return None
        stamp_ns = self.uwb_clock_aligner.align_ns(source_stamp_ns, arrival_unix_ns)
        self.last_uwb_stamp_ns = source_stamp_ns
        return {"schema_version": 1, "sample_timestamp_ns": stamp_ns, "frame_id": "map", "unit": "m",
                "tag_id": self.expected_tag_id, "anchor_ids": self.anchor_ids,
                "ranges_m": ranges, "variances_m2": self.range_variance_m2,
                "timestamp_domain": "ros_unix_ns_aligned_from_pi_receive"}
