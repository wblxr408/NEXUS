"""Transport-neutral ROS1/ROS2 envelope contract."""

import json
import math


SCHEMA_VERSION = 1
REQUIRED_FIELDS = (
    "schema_version", "message_type", "sequence", "sample_timestamp_ns",
    "frame_id", "source_mode", "payload",
)
ROS1_TO_ROS2_TOPIC_MAP = {
    "/odom_global_001": "/nexus/fcu/odom",
    "/imu_global_001": "/nexus/fcu/imu",
}
VALID_SOURCE_MODES = frozenset({1, 2, 3, 4, "SOURCE_DIRECT_UWB",
                                "SOURCE_DIRECT_VISION", "SOURCE_PLATFORM_RELATIVE",
                                "SOURCE_FUSED", "PLATFORM", "VISION", "UWB", "FUSED"})
TARGET_MESSAGE_TYPES = frozenset({
    "nexus_msgs/TargetObservation", "TargetObservation", "target_observation",
})
VALID_UNITS = frozenset({"m", "meter", "meters"})


class EnvelopeError(ValueError):
    """Raised when an envelope violates the channel contract."""


def build_envelope(message_type, sequence, sample_timestamp_ns, frame_id, source_mode, payload):
    envelope = {
        "schema_version": SCHEMA_VERSION,
        "message_type": message_type,
        "sequence": sequence,
        "sample_timestamp_ns": sample_timestamp_ns,
        "frame_id": frame_id,
        "source_mode": source_mode,
        "payload": payload,
    }
    validate_envelope(envelope)
    return envelope


def validate_envelope(envelope):
    if not isinstance(envelope, dict):
        raise EnvelopeError("envelope must be an object")
    missing = [field for field in REQUIRED_FIELDS if field not in envelope]
    if missing:
        raise EnvelopeError(f"missing envelope fields: {', '.join(missing)}")
    if envelope["schema_version"] != SCHEMA_VERSION:
        raise EnvelopeError(f"unsupported schema_version: {envelope['schema_version']}")
    if not isinstance(envelope["message_type"], str) or not envelope["message_type"]:
        raise EnvelopeError("message_type must be a non-empty string")
    if not isinstance(envelope["sequence"], int) or envelope["sequence"] < 0:
        raise EnvelopeError("sequence must be a non-negative integer")
    if not isinstance(envelope["sample_timestamp_ns"], int) or envelope["sample_timestamp_ns"] <= 0:
        raise EnvelopeError("sample_timestamp_ns must be a positive integer")
    if not isinstance(envelope["frame_id"], str) or not envelope["frame_id"]:
        raise EnvelopeError("frame_id must be a non-empty string")
    source_mode = envelope["source_mode"]
    if not isinstance(source_mode, (int, str)) or isinstance(source_mode, bool):
        raise EnvelopeError("source_mode must be an integer or named mode")
    if source_mode not in VALID_SOURCE_MODES:
        raise EnvelopeError(f"unsupported source_mode: {source_mode}")
    if not isinstance(envelope["payload"], dict):
        raise EnvelopeError("payload must be an object")
    payload = envelope["payload"]
    if envelope["message_type"] in TARGET_MESSAGE_TYPES:
        target_id = payload.get("target_id")
        if not isinstance(target_id, str) or not target_id:
            raise EnvelopeError("target observation payload requires target_id")
        unit = payload.get("unit")
        if unit not in VALID_UNITS:
            raise EnvelopeError("target observation payload unit must be metres (m)")
        confidence = payload.get("confidence")
        if not isinstance(confidence, (int, float)) or isinstance(confidence, bool) or not 0.0 <= float(confidence) <= 1.0:
            raise EnvelopeError("target observation confidence must be between 0 and 1")
        covariance = payload.get("covariance")
        if covariance is None:
            raise EnvelopeError("target observation payload requires covariance")
        if isinstance(covariance, list) and len(covariance) == 9:
            values = covariance
        elif (isinstance(covariance, list) and len(covariance) == 3
              and all(isinstance(row, list) and len(row) == 3 for row in covariance)):
            values = [value for row in covariance for value in row]
        else:
            raise EnvelopeError("target observation covariance must be a 3x3 or flat 9-value matrix")
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(float(value)) for value in values):
            raise EnvelopeError("target observation covariance must contain finite numbers")
    return envelope


def validate_sequence(sequence, previous_sequence=None):
    """Validate a non-negative sequence, optionally against its predecessor."""
    if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0:
        raise EnvelopeError("sequence must be a non-negative integer")
    if previous_sequence is not None:
        if not isinstance(previous_sequence, int) or isinstance(previous_sequence, bool):
            raise EnvelopeError("previous_sequence must be an integer")
        if sequence != previous_sequence + 1:
            raise EnvelopeError("sequence must increase by one")
    return sequence


def encode_envelope(envelope):
    validate_envelope(envelope)
    return json.dumps(envelope, ensure_ascii=False, separators=(",", ":"), sort_keys=True).encode("utf-8")


def decode_envelope(data):
    try:
        envelope = json.loads(data.decode("utf-8") if isinstance(data, bytes) else data)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise EnvelopeError(f"invalid JSON envelope: {error}") from error
    return validate_envelope(envelope)


def map_ros1_topic(topic):
    try:
        return ROS1_TO_ROS2_TOPIC_MAP[topic]
    except KeyError as error:
        raise EnvelopeError(f"unmapped ROS1 topic: {topic}") from error
