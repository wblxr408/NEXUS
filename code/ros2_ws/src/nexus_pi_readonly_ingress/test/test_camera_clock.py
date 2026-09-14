import pytest

from nexus_pi_readonly_ingress.camera_clock import CameraClockClient, estimate_exchange


def test_clock_offset_and_network_asymmetry_bound():
    # Edge clock leads Pi by 10 s, uplink 3 ms, downlink 7 ms, service 1 ms.
    offset, rtt = estimate_exchange(11_000_000_000, 1_003_000_000,
                                    1_004_000_000, 11_011_000_000)
    assert rtt == 10_000_000
    assert offset == 10_002_000_000
    assert abs(offset - 10_000_000_000) <= rtt / 2


def test_clock_rejects_reversed_exchange():
    with pytest.raises(ValueError):
        estimate_exchange(100, 30, 20, 110)


def test_alignment_requires_fresh_probe_and_matching_boot(monkeypatch):
    monkeypatch.setattr("time.monotonic_ns", lambda: 1000)
    monkeypatch.setattr("time.time_ns", lambda: 11000)
    client = CameraClockClient("127.0.0.1")
    client.boot_id = "test_boot"
    client.samples = [dict(offset_ns=10000, rtt_ns=200, edge_unix_ns=10900,
                           monotonic_ns=900, boot_id="test_boot")]
    assert client.align(500, "test_boot")[0] == 10500
    with pytest.raises(ValueError):
        client.align(500, "other_boot")
    monkeypatch.setattr("time.monotonic_ns", lambda: 6_000_000_000)
    with pytest.raises(ValueError):
        client.align(500, "test_boot")
