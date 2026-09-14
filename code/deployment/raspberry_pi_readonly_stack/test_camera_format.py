"""Check the libcamera pixel-format contract without opening any hardware."""

from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "raspberry_pi_e016_self_localization/app"))
from imx219_frame_source import Imx219FrameSource


def test_byte_ordered_rgb_and_request_lifetime(monkeypatch):
    pixels = np.array([[[255, 0, 0], [0, 255, 0], [0, 0, 255]]], np.uint8)
    configurations = []

    class Request:
        def get_metadata(self):
            return {"SensorTimestamp": 100000, "ExposureTime": 100}

        def make_array(self, name):
            return pixels

        def release(self):
            pixels[:] = 0

    class Camera:
        def create_video_configuration(self, **configuration):
            configurations.append(configuration)
            return configuration

        def configure(self, configuration):
            pass

        def start(self):
            pass

        def stop(self):
            pass

        def close(self):
            pass

        def capture_request(self):
            return Request()

    monkeypatch.setitem(sys.modules, "libcamera", SimpleNamespace(Transform=lambda **kw: kw))
    monkeypatch.setitem(sys.modules, "picamera2", SimpleNamespace(Picamera2=Camera))
    with Imx219FrameSource(width=3, height=1) as camera:
        frame = camera.capture()
    assert configurations[0]["main"]["format"] == "BGR888"
    np.testing.assert_array_equal(frame.rgb, [[[255, 0, 0], [0, 255, 0], [0, 0, 255]]])
    assert frame.sensor_timestamp_ns == 100000
