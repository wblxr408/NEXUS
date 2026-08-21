"""Transport-neutral ROS1/ROS2 envelope contract."""

import json
import math


SCHEMA_VERSION = 1
REQUIRED_FIELDS = (
    "schema_version", "message_type", "sequence", "sample_timestamp_ns",
    "receive_timestamp_ns", "frame_id", "source_mode", "validity",
    "invalid_reason", "payload",
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
VALID_VALIDITY = frozenset({1, 2, "VALID", "INVALID"})


class EnvelopeError(ValueError):
    """Raised when an envelope violates the channel contract."""


def build_envelope(message_type, sequence, sample_timestamp_ns,
                   receive_timestamp_ns, frame_id, source_mode, validity,
                   invalid_reason, payload):
    envelope = {
        "schema_version": SCHEMA_VERSION,
        "message_type": message_type,
        "sequence": sequence,
        "sample_timestamp_ns": sample_timestamp_ns,
        "receive_timestamp_ns": receive_timestamp_ns,
        "frame_id": frame_id,
        "source_mode": source_mode,
        "validity": validity,
        "invalid_reason": invalid_reason,
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
    receive_timestamp_ns = envelope["receive_timestamp_ns"]
    if not isinstance(receive_timestamp_ns, int) or isinstance(receive_timestamp_ns, bool):
        raise EnvelopeError("receive_timestamp_ns must be a positive integer")
    if receive_timestamp_ns < envelope["sample_timestamp_ns"]:
        raise EnvelopeError("receive_timestamp_ns cannot precede sample_timestamp_ns")
    if not isinstance(envelope["frame_id"], str) or not envelope["frame_id"]:
        raise EnvelopeError("frame_id must be a non-empty string")
    source_mode = envelope["source_mode"]
    if not isinstance(source_mode, (int, str)) or isinstance(source_mode, bool):
        raise EnvelopeError("source_mode must be an integer or named mode")
    if source_mode not in VALID_SOURCE_MODES:
        raise EnvelopeError(f"unsupported source_mode: {source_mode}")
    validity = envelope["validity"]
    if isinstance(validity, bool) or validity not in VALID_VALIDITY:
        raise EnvelopeError(f"unsupported validity: {validity}")
    invalid_reason = envelope["invalid_reason"]
    if not isinstance(invalid_reason, str):
        raise EnvelopeError("invalid_reason must be a string")
    is_valid = validity in {1, "VALID"}
    if is_valid and invalid_reason:
        raise EnvelopeError("valid envelope must have an empty invalid_reason")
    if not is_valid and not invalid_reason:
        raise EnvelopeError("invalid envelope requires invalid_reason")
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
        if is_valid and any(isinstance(value, bool) or not isinstance(value, (int, float))
                            or not math.isfinite(float(value)) for value in values):
            raise EnvelopeError("target observation covariance must contain finite numbers")
        if not is_valid and any(isinstance(value, bool) or not isinstance(value, (int, float))
                                for value in values):
            raise EnvelopeError("invalid target covariance must still contain numeric placeholders")
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


def validate_freshness(envelope, now_timestamp_ns, max_age_ns):
    """Reject missing, future, or stale samples using a caller-frozen threshold."""
    validate_envelope(envelope)
    if (not isinstance(now_timestamp_ns, int) or isinstance(now_timestamp_ns, bool)
            or now_timestamp_ns <= 0):
        raise EnvelopeError("now_timestamp_ns must be a positive integer")
    if (not isinstance(max_age_ns, int) or isinstance(max_age_ns, bool)
            or max_age_ns < 0):
        raise EnvelopeError("max_age_ns must be a non-negative integer")
    sample_timestamp_ns = envelope["sample_timestamp_ns"]
    if sample_timestamp_ns > now_timestamp_ns:
        raise EnvelopeError("sample timestamp is in the future")
    if now_timestamp_ns - sample_timestamp_ns > max_age_ns:
        raise EnvelopeError("stale envelope")
    return envelope


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
