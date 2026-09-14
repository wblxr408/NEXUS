# Raspberry Pi read-only stack v2

This is an add-only deployment alongside the existing `fcu_core` and Domain
launches. It never starts `fcu_mission`, publishes control commands, arms, or
sets velocity/height. The MAVLink adapter inherits the existing passive GCS
heartbeat and reads the vendor TCP stream, avoiding `/dev/ttyAMA0` contention.
`uav_readonly_adapter.py` is a fixed copy of the repository's tested read-only
base inside this new directory, so an older same-named Pi deployment cannot be
imported accidentally.

Configured facts are in `config/hardware_user_v01.yaml`: tag 2, four
user-provided anchor coordinates, 9 degree map yaw, and a bottom-facing IMX219.
The coordinates are marked user-provided/not-surveyed and are not precision
evidence.

`config/hardware_user_v02.yaml` adds the complete 2026-09-04 vendor UI
transcription. The user-confirmed tag ID is 2 and tag total is 1. The anchor
coordinates are user-confirmed field measurements; no separate uncertainty was
provided. Flight-control values are retained only as provenance and are never written to the controller.
Use `record_readonly_params_v02.sh` to copy this configuration into a new run;
the original v01 entry points remain available unchanged.

```bash
# camera only (works without the flight controller)
./run_camera_only_v2.sh --frames 30

# camera stream for the edge ROS2 ingress
./run_camera_stream_v2.sh

# telemetry only; the edge device connects to the Pi TCP port
./run_telemetry_tag2_domain0.sh --tcp-listen-host 0.0.0.0 --tcp-listen-port 14551

# bounded camera + telemetry capture
./record_readonly_v2.sh 30

# transport test only: simulated tag-2 telemetry, never connected to the FCU
./synthetic_telemetry_test_server.py --duration 20
```

`capture_live_camera_v2.py` retains the sensor monotonic timestamp and adds an
explicit Unix receive timestamp for ROS transport. `uav_readonly_adapter_v2.py`
keeps the corrected vendor conversion for all accelerometer axes (`/1000`) and
adds a receive timestamp to UWB packets, whose MAVLink carrier has no device
sample-time field. TCP is served by the Pi because Windows/WSL UDP ingress was
observed blocked; the edge computer initiates the connection and no firewall
rule is changed.
The v2 adapter forwards every received `SCALED_IMU` sample instead of reducing
the approximately 200 Hz device stream to the slower status snapshot rate.
