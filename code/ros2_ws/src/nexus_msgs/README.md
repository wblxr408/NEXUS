# nexus_msgs

ROS2 message contracts for external target localization. `TargetObservation` always represents the external target, never the drone platform pose. `header.stamp` is sample time; `receive_timestamp_ns` is the time the current layer received or formed the message. Every message carries explicit validity, reason, last-valid sample time and metre unit. Invalid messages use NaN pose/covariance placeholders and are never fused. Real target IDs, frames, extrinsics and source data remain TBD until hardware acceptance.
