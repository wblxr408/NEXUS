from pathlib import Path

from nexus_msgs.msg import TargetObservation


def test_dashboard_contract_uses_target_message():
    message = TargetObservation()
    message.target_id = "target_1"
    message.header.frame_id = "map"
    message.validity = TargetObservation.VALIDITY_VALID
    message.unit = "m"
    assert message.target_id == "target_1"
    assert message.header.frame_id == "map"
    assert message.validity == TargetObservation.VALIDITY_VALID


def test_dashboard_renders_gazebo_imx219_raw_frames_in_camera_canvas():
    package_root = Path(__file__).parents[1]
    bridge = (package_root / "web" / "src" / "services" / "rosbridge_client.js").read_text()
    app = (package_root / "web" / "src" / "app" / "main.js").read_text()
    assert 'subscribe("/nexus/camera/imx219/image_raw", "sensor_msgs/msg/Image", 1000)' in bridge
    assert "previewFromRawImage" in bridge
    assert "store.updateCamera" in bridge
    assert 'byId("cam-canvas")' in app
