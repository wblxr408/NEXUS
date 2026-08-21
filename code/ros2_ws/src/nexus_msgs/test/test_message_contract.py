from nexus_msgs.msg import TargetObservation


def test_source_modes_are_stable():
    assert TargetObservation.SOURCE_DIRECT_UWB == 1
    assert TargetObservation.SOURCE_DIRECT_VISION == 2
    assert TargetObservation.SOURCE_PLATFORM_RELATIVE == 3
    assert TargetObservation.SOURCE_FUSED == 4


def test_default_message_has_explicit_shape():
    message = TargetObservation()
    assert message.target_id == ""
    assert message.receive_timestamp_ns == 0
    assert message.validity == TargetObservation.VALIDITY_UNKNOWN
    assert message.invalid_reason == ""
    assert message.last_valid_sample_timestamp_ns == 0
    assert message.unit == ""
    assert len(message.covariance) == 36
    assert message.confidence == 0.0


def test_validity_constants_are_stable():
    assert TargetObservation.VALIDITY_UNKNOWN == 0
    assert TargetObservation.VALIDITY_VALID == 1
    assert TargetObservation.VALIDITY_INVALID == 2
