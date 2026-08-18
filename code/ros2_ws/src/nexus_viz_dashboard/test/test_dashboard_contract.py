from nexus_msgs.msg import TargetObservation


def test_dashboard_contract_uses_target_message():
    message = TargetObservation()
    message.target_id = "target_1"
    message.header.frame_id = "map"
    assert message.target_id == "target_1"
    assert message.header.frame_id == "map"
