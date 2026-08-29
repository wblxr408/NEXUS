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
    assert 'subscribe(IMX219_COMPRESSED_TOPIC, "sensor_msgs/msg/CompressedImage", 1000)' in bridge
    assert "previewFromCompressedImage" in bridge
    assert "store.updateCamera" in bridge
    assert 'byId("cam-canvas")' in app


def test_dashboard_replaces_left_sandbox_with_carla_stream_panel():
    package_root = Path(__file__).parents[1]
    layout = (package_root / "web" / "src" / "components" / "dashboard_layout.js").read_text()
    assert "CARLA 实时数据流" in layout
    assert 'id="ui-carla-map"' in layout
    assert 'id="ui-algorithm-list"' in layout
    assert 'id="ui-log-list"' in layout


def test_dashboard_connects_to_carla_status_gateway():
    package_root = Path(__file__).parents[1]
    gateway = (package_root / "web" / "src" / "services" / "carla_status_client.js").read_text()
    app = (package_root / "web" / "src" / "app" / "main.js").read_text()
    assert "carla_ws" in gateway
    assert "installCarlaStatusGateway(store)" in app
