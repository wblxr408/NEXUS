# awesome-uwb-localization UWB-only adapter

Source: <https://github.com/qxiaofan/awesome-uwb-localization>, inspected at
`b3cd36e5d78da4daea2f350cfebeb6b8b8378810`.

The upstream repository is a ROS1 range-localization package, not an awesome
list. Its core uses fixed SE3 anchor vertices, moving tag vertices,
`EdgeSE3Range`, a Cauchy robust kernel, a sliding trajectory, and g2o
Levenberg-Marquardt optimization.

`graph_range.py` ports that UWB-only range graph to the NEXUS 3D observation
contract using NumPy. It is registered as `uwb.awesome_uwb`. The adapter does
not port the upstream optical-flow, Visual SLAM, IMU, lidar, relative-ranging,
or ROS Kinetic transport paths.

The Python adapter is used because the upstream build pins historical ROS1
and g2o interfaces that are not ABI-compatible with the current Ubuntu 22.04
and ROS2 environment. Ground Truth is never accepted by the runner.
