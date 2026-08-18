from nexus_msgs.msg import TargetObservation


def test_source_modes_are_stable():
    assert TargetObservation.SOURCE_DIRECT_UWB == 1
    assert TargetObservation.SOURCE_DIRECT_VISION == 2
    assert TargetObservation.SOURCE_PLATFORM_RELATIVE == 3
    assert TargetObservation.SOURCE_FUSED == 4


def test_default_message_has_explicit_shape():
    message = TargetObservation()
    assert message.target_id == ""
    assert len(message.covariance) == 36
    assert message.confidence == 0.0
