import pytest

from nexus_channel_contract import EnvelopeError, build_envelope, decode_envelope, encode_envelope, map_ros1_topic


def test_envelope_round_trip_preserves_sample_time():
    envelope = build_envelope(
        "nav_msgs/Odometry", 3, 123456789, 123456999,
        "map", "PLATFORM", "VALID", "", {"x_m": 1.0})
    assert decode_envelope(encode_envelope(envelope)) == envelope


def test_invalid_frame_and_timestamp_are_rejected():
    with pytest.raises(EnvelopeError):
        build_envelope("x", 0, 1, 2, "", 1, "VALID", "", {})
    with pytest.raises(EnvelopeError):
        build_envelope("x", 0, 0, 1, "map", 1, "VALID", "", {})


def test_receive_time_and_validity_are_explicit():
    with pytest.raises(EnvelopeError):
        build_envelope("x", 0, 10, 9, "map", 1, "VALID", "", {})
    with pytest.raises(EnvelopeError):
        build_envelope("x", 0, 10, 10, "map", 1, "INVALID", "", {})
    result = build_envelope(
        "x", 0, 10, 10, "map", 1, "INVALID", "stale", {})
    assert result["invalid_reason"] == "stale"


def test_topic_mapping_is_explicit():
    assert map_ros1_topic("/odom_global_001") == "/nexus/fcu/odom"
    with pytest.raises(EnvelopeError):
        map_ros1_topic("/unknown")


def test_target_payload_requires_identity_units_and_confidence():
    with pytest.raises(EnvelopeError):
        build_envelope(
            "TargetObservation", 0, 1, 2, "map", 2, "VALID", "", {})
    payload = {"target_id": "target_1", "unit": "m", "covariance": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "confidence": 0.8}
    assert build_envelope(
        "TargetObservation", 0, 1, 2, "map", 2, "VALID", "", payload,
    )["payload"]["target_id"] == "target_1"


def test_illegal_source_mode_and_sequence_gap_are_rejected():
    with pytest.raises(EnvelopeError):
        build_envelope("x", 0, 1, 2, "map", 99, "VALID", "", {})
    from nexus_channel_contract import validate_sequence
    with pytest.raises(EnvelopeError):
        validate_sequence(3, previous_sequence=1)


def test_freshness_rule_rejects_stale_and_future_samples():
    from nexus_channel_contract import validate_freshness
    envelope = build_envelope(
        "x", 0, 100, 101, "map", 1, "VALID", "", {})
    assert validate_freshness(envelope, 110, 10) is envelope
    with pytest.raises(EnvelopeError, match="stale"):
        validate_freshness(envelope, 111, 10)
    with pytest.raises(EnvelopeError, match="future"):
        validate_freshness(envelope, 99, 10)
