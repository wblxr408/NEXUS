# nexus_pi_readonly_ingress

Add-only ROS2 ingress for the Raspberry Pi read-only adapter v2. It listens to
JSON UDP and publishes `/nexus/fcu/imu`, `/nexus/uwb/ranges`, and telemetry
health. A separate file relay publishes the real IMX219 frame as ROS `Image`,
`CompressedImage`, and uncalibrated `CameraInfo` while camera calibration is
deferred.

Flight-controller boot timestamps are mapped onto the ROS Unix clock while
preserving device-time deltas and detecting boot-clock resets. UWB has no
device sample timestamp in its vendor MAVLink carrier, so v2 uses and labels
the Pi receive timestamp; ingress also aligns the Pi host clock to the edge
host clock instead of assuming that both wall clocks are synchronized.

The camera node defaults to a TCP client for the Pi's port 14552 and publishes
raw/compressed image plus uncalibrated CameraInfo and camera health. In TCP
mode the ROS stamp is the edge receive time; the Pi sensor and receive stamps
are preserved on `/nexus/camera/imx219/metadata` until measured clock
synchronization is added.

This package does not open a serial device and contains no control publisher.
The default transport is a TCP client to `192.168.1.143:14551`, so the local
computer initiates the connection and Windows/WSL needs no inbound UDP rule.

`readonly_ingress_v02.launch.py` selects `hardware_user_v02.yaml`, which records
the 2026-09-04 vendor UI values with the user-confirmed UWB tag ID 2 and
field-measured anchor coordinates. The flight controller values in that file
are provenance only and are not sent to the aircraft.
