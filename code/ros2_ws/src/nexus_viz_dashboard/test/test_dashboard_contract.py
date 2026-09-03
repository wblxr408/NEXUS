from pathlib import Path

import pytest

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
    # IA §3.3: 640x480 @5-10 FPS, so the rosbridge throttle and the decode gate share 150 ms.
    assert "const CAMERA_INTERVAL_MS = 150;" in bridge
    assert 'subscribe(IMX219_COMPRESSED_TOPIC, "sensor_msgs/msg/CompressedImage", CAMERA_INTERVAL_MS)' in bridge
    assert "now - lastCameraDecodeMs < CAMERA_INTERVAL_MS" in bridge
    assert "previewFromCompressedImage" in bridge
    assert "store.updateCamera" in bridge
    assert 'byId("cam-canvas")' in app


def test_position_panel_exposes_sigma_and_solver_provenance():
    package_root = Path(__file__).parents[1]
    layout = (package_root / "web" / "src" / "components" / "dashboard_layout.js").read_text()
    store = (package_root / "web" / "src" / "features" / "telemetry" / "dashboard_store.js").read_text()
    bridge = (package_root / "web" / "src" / "services" / "rosbridge_client.js").read_text()
    for identifier in ("ui-sigma-x", "ui-sigma-y", "ui-sigma-z", "ui-n-views", "ui-baseline",
                       "ui-depth-source", "ui-chain"):
        assert f'id="{identifier}"' in layout
    assert "updateSolverProvenance" in store
    # sigma comes from the real covariance diagonal, never from a fabricated constant.
    assert "[0, 7, 14]" in bridge


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


def test_covariance_ellipsoid_uses_one_sigma_axes_and_skips_unusable_covariance():
    import rclpy

    from nexus_viz_dashboard.dashboard_node import DashboardNode

    rclpy.init()
    try:
        node = DashboardNode()
        published = []
        node._target_marker_pub.publish = published.append
        message = TargetObservation()
        message.header.frame_id = "map"
        message.target_id = "target_1"
        message.validity = TargetObservation.VALIDITY_VALID
        covariance = [0.0] * 36
        covariance[0], covariance[7], covariance[14] = 4.0e-4, 9.0e-4, 1.6e-3
        message.covariance = covariance
        node._publish_covariance_ellipsoid(message)
        assert len(published) == 1
        marker = published[0]
        assert marker.ns == "nexus_target_covariance"
        scales = sorted([marker.scale.x, marker.scale.y, marker.scale.z])
        # Marker scale is a full axis length, so 1-sigma semi-axes are 2 * sigma.
        assert scales == pytest.approx([0.04, 0.06, 0.08], abs=1e-9)

        message.covariance = [float("nan")] * 36
        node._publish_covariance_ellipsoid(message)
        message.covariance = [0.0] * 36
        node._publish_covariance_ellipsoid(message)
        assert len(published) == 1
        node.destroy_node()
    finally:
        rclpy.shutdown()
