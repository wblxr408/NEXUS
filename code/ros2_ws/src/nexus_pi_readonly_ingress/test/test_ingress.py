import json

from nexus_pi_readonly_ingress.telemetry_codec import TelemetryCodec
from nexus_pi_readonly_ingress.time_alignment import BootTimeAligner


def test_boot_time_alignment_preserves_device_delta_and_handles_reset():
    aligner = BootTimeAligner()
    first = aligner.align_us(1_000_000, 10_000_000_000)
    second = aligner.align_us(1_010_000, 10_012_000_000)
    assert second - first == 10_000_000
    reset = aligner.align_us(1_000, 11_000_000_000)
    assert reset == 11_000_000_000
    assert aligner.reset_count == 1


def state():
    return {"imu": {"sample_timestamp_us": 1_000_000,
                    "sample_timestamp_domain": "flight_boot_unverified",
                    "acceleration_mps2": {"x": 1, "y": 2, "z": -9.8},
                    "angular_velocity_rps": {"x": 0.1, "y": 0.2, "z": 0.3}},
            "uwb": {"tag_id": 2, "anchor_ids": ["1", "2", "3", "4"],
                    "anchor_ranges_m": [1.0, 2.0, 3.0, 4.0],
                    "sample_timestamp_ns": 9_900_000_000,
                    "sample_timestamp_domain": "ros_unix_ns_receive"},
            "position": {"frame_id": "uwb_raw", "x_m": 1.25, "y_m": -0.75,
                         "z_m": None, "sample_timestamp_ns": 1_000_000_000,
                         "sample_timestamp_domain": "flight_boot_unverified"},
            "online": True}


def test_codec_outputs_imu_and_platform_range_contracts_once():
    codec = TelemetryCodec()
    value = state()
    imu = codec.imu_packet(value, 10_000_000_000)
    ranges = codec.uwb_packet(value, 10_000_000_000)
    assert imu["stamp_ns"] == 10_000_000_000
    assert imu["acceleration"] == [1.0, 2.0, -9.8]
    assert ranges["anchor_ids"] == ["1", "2", "3", "4"]
    assert ranges["frame_id"] == "map" and ranges["unit"] == "m"
    assert json.dumps(ranges, allow_nan=False)
    assert codec.imu_packet(value, 10_000_000_001) is None
    assert codec.uwb_packet(value, 10_000_000_001) is None


def test_codec_rejects_wrong_tag_and_insufficient_ranges():
    codec = TelemetryCodec(expected_tag_id=2)
    value = state()
    value["uwb"]["tag_id"] = 1
    assert codec.uwb_packet(value, 10_000_000_000) is None
    value["uwb"]["tag_id"] = 2
    value["uwb"]["anchor_ranges_m"] = [1.0, None, None, 4.0]
    assert codec.uwb_packet(value, 10_000_000_000) is None


def test_pi_receive_clock_is_aligned_to_edge_clock():
    codec = TelemetryCodec()
    value = state()
    value["uwb"]["sample_timestamp_ns"] = 100_000_000_000
    first = codec.uwb_packet(value, 10_000_000_000)
    assert first["sample_timestamp_ns"] == 10_000_000_000
    assert first["timestamp_domain"] == "ros_unix_ns_aligned_from_pi_receive"
    value["uwb"]["sample_timestamp_ns"] += 20_000_000
    second = codec.uwb_packet(value, 10_021_000_000)
    assert second["sample_timestamp_ns"] - first["sample_timestamp_ns"] == 20_000_000


def test_vendor_2d_position_is_transported_without_becoming_map_odometry():
    codec = TelemetryCodec()
    packet = codec.vendor_2d_position_packet(state(), 10_000_000_000)
    assert packet["frame_id"] == "vendor_2d/uwb_raw"
    assert packet["source_frame_id"] == "uwb_raw"
    assert packet["position_m"] == [1.25, -0.75]
    assert packet["dimensions"] == 2
    assert packet["coordinate_frame_status"] == "vendor_proprietary_2d_not_map"
    assert packet["sample_timestamp_ns"] == 10_000_000_000
    assert packet["timestamp_domain"] == "ros_unix_ns_aligned_from_flight_boot"
    assert json.dumps(packet, allow_nan=False)
    assert codec.vendor_2d_position_packet(state(), 10_000_000_001) is None


def test_vendor_2d_position_keeps_unknown_vendor_timebase_unusable_for_fusion():
    codec = TelemetryCodec()
    value = state()
    value["position"]["sample_timestamp_domain"] = "vendor_time_usec_unverified"
    packet = codec.vendor_2d_position_packet(value, 10_000_000_000)
    assert packet["sample_timestamp_ns"] is None
    assert packet["timestamp_domain"] == "unavailable_unverified_vendor_timebase"
    assert packet["source_sample_timestamp_ns"] == 1_000_000_000
