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
        self.position_clock_aligner = BootTimeAligner(reset_threshold_ns=1_000_000_000)
        self.last_imu_source_us = None
        self.last_uwb_stamp_ns = None
        self.last_position_source = None
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

    def vendor_2d_position_packet(self, state, arrival_unix_ns):
        """Preserve the FC vendor's planar observation without making map odometry.

        ``GLOBAL_VISION_POSITION_ESTIMATE`` is documented by the adapter as a
        proprietary x/y value in ``uwb_raw``.  It has no trustworthy height or
        map-frame transform, so this JSON contract deliberately cannot be
        mistaken for the `/nexus/fcu/odom` input used by the localizer.
        """
        if state.get("online") is not True:
            return None
        position = state.get("position") or {}
        x_m, y_m = finite(position.get("x_m")), finite(position.get("y_m"))
        source_stamp = finite(position.get("sample_timestamp_ns"))
        source_domain = str(position.get("sample_timestamp_domain") or "")
        source_frame = str(position.get("frame_id") or "").strip()
        if x_m is None or y_m is None or source_stamp is None or source_stamp <= 0 or not source_frame:
            return None
        source_key = (source_domain, int(source_stamp))
        if source_key == self.last_position_source:
            return None

        packet = {
            "schema_version": 1,
            "frame_id": "vendor_2d/" + source_frame,
            "source_frame_id": source_frame,
            "unit": "m",
            "dimensions": 2,
            "position_m": [x_m, y_m],
            "coordinate_frame_status": "vendor_proprietary_2d_not_map",
            "source_sample_timestamp_ns": int(source_stamp),
            "source_timestamp_domain": source_domain,
            "edge_receive_timestamp_ns": int(arrival_unix_ns),
        }
        if source_domain == "flight_boot_unverified":
            packet["sample_timestamp_ns"] = self.position_clock_aligner.align_ns(
                int(source_stamp), arrival_unix_ns)
            packet["timestamp_domain"] = "ros_unix_ns_aligned_from_flight_boot"
        else:
            # The record is still useful for live vendor-position display, but
            # an unspecified vendor timebase must not become a ROS measurement
            # timestamp for fusion or time synchronization.
            packet["sample_timestamp_ns"] = None
            packet["timestamp_domain"] = "unavailable_unverified_vendor_timebase"
        self.last_position_source = source_key
        return packet
