"""Measured UWB CSV import and common offline evaluation helpers."""

from __future__ import annotations

import csv
import hashlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:
    from generators.range_simulation import RangeSample, UwbObservationFrame
except ModuleNotFoundError:  # allow analysis-only installs without simulation on PYTHONPATH
    @dataclass(frozen=True)
    class RangeSample:
        anchor_id: str
        anchor_position_m: tuple[float, float, float]
        range_m: float
        stddev_m: float

    @dataclass(frozen=True)
    class UwbObservationFrame:
        timestamp_ns: int
        tag_id: str
        ranges: list[RangeSample]


REQUIRED_COLUMNS = {
    "frame_seq", "sample_timestamp_ns", "receive_timestamp_ns", "tag_id",
    "anchor_id", "anchor_x_m", "anchor_y_m", "anchor_z_m", "range_m",
    "stddev_m",
}
UNIT_SCALE = {"m": 1.0, "cm": 0.01, "mm": 0.001}
NORMALIZED_COLUMNS = [
    "frame_seq", "sample_timestamp_ns", "receive_timestamp_ns", "tag_id",
    "anchor_id", "anchor_x_m", "anchor_y_m", "anchor_z_m", "range_m", "stddev_m",
    "point_id", "repeat_id",
]


@dataclass(frozen=True)
class RealObservationFrame(UwbObservationFrame):
    """Algorithm-facing frame with provenance needed by the evaluator."""

    frame_seq: int = 0
    receive_timestamp_ns: int = 0
    point_id: str = ""
    repeat_id: str = ""


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_observation_frames(path: str | Path, *, range_unit: str = "m", min_anchors: int = 3):
    """Read one-row-per-anchor measured observations and group them by frame.

    Frames with fewer than ``min_anchors`` are retained so the selected
    algorithms can report an explicit invalid result rather than receiving
    fabricated ranges.
    """
    if range_unit not in UNIT_SCALE:
        raise ValueError(f"unsupported range unit '{range_unit}'; use m, cm, or mm")
    if min_anchors < 3:
        raise ValueError("min_anchors must be at least three for 2D positioning")
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        columns = set(reader.fieldnames or ())
        missing = REQUIRED_COLUMNS - columns
        if missing:
            raise ValueError(f"observations CSV missing columns: {', '.join(sorted(missing))}")
        grouped = {}
        order = []
        for line_number, row in enumerate(reader, start=2):
            try:
                key = (int(row["frame_seq"]), int(row["sample_timestamp_ns"]), row["tag_id"])
                receive_ns = int(row["receive_timestamp_ns"])
                anchor_id = str(row["anchor_id"])
                position = tuple(float(row[f"anchor_{axis}_m"]) for axis in ("x", "y", "z"))
                raw_range = float(row["range_m"])
                stddev = float(row["stddev_m"])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"invalid observation at CSV line {line_number}: {exc}") from exc
            if not np.all(np.isfinite(position)) or not np.isfinite(raw_range) or not np.isfinite(stddev):
                raise ValueError(f"non-finite observation at CSV line {line_number}")
            range_m = raw_range * UNIT_SCALE[range_unit]
            if range_m <= 0.0:
                raise ValueError(f"range must be finite and positive at CSV line {line_number}")
            if stddev < 0.0:
                raise ValueError(f"stddev must be non-negative at CSV line {line_number}")
            if receive_ns < key[1]:
                raise ValueError(f"receive timestamp precedes sample timestamp at CSV line {line_number}")
            if key not in grouped:
                grouped[key] = {"rows": [], "receive": receive_ns,
                                "point_id": row.get("point_id", ""),
                                "repeat_id": row.get("repeat_id", "")}
                order.append(key)
            elif anchor_id in {item.anchor_id for item in grouped[key]["rows"]}:
                raise ValueError(f"duplicate anchor '{anchor_id}' in frame {key[0]}")
            grouped[key]["rows"].append(RangeSample(anchor_id, position, range_m, stddev))
            grouped[key]["receive"] = max(grouped[key]["receive"], receive_ns)

    frames = []
    previous_timestamp = None
    for frame_seq, timestamp_ns, tag_id in order:
        if previous_timestamp is not None and timestamp_ns <= previous_timestamp:
            raise ValueError("sample timestamps must be strictly increasing between frames")
        previous_timestamp = timestamp_ns
        entry = grouped[(frame_seq, timestamp_ns, tag_id)]
        frames.append(RealObservationFrame(
            timestamp_ns=timestamp_ns,
            tag_id=tag_id,
            ranges=entry["rows"],
            frame_seq=frame_seq,
            receive_timestamp_ns=entry["receive"],
            point_id=entry["point_id"] or "unknown",
            repeat_id=entry["repeat_id"] or "1",
        ))
    if not frames:
        raise ValueError("observations CSV must contain at least one row")
    return frames


def normalize_observations(input_path: str | Path, output_path: str | Path, *,
                           range_unit: str, anchors_path: str | Path | None = None):
    """Convert a raw one-row-per-anchor CSV into the canonical metre schema.

    Raw files may call the range column ``range``, ``distance`` or
    ``range_value``. Timestamps must be nanoseconds unless the input already
    uses the canonical ``*_timestamp_ns`` names. Anchor coordinates are read
    from the CSV when present, otherwise from a YAML file with an ``anchors``
    list containing ``id`` and ``position_m``.
    """
    if range_unit not in UNIT_SCALE:
        raise ValueError(f"unsupported range unit '{range_unit}'; use m, cm, or mm")
    anchor_map = {}
    if anchors_path is not None:
        import yaml
        with Path(anchors_path).open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        for entry in document.get("anchors", []):
            anchor_map[str(entry["id"])] = tuple(float(value) for value in entry["position_m"])
    with Path(input_path).open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        range_column = next((name for name in ("range_m", "range", "distance", "range_value") if name in fields), None)
        if range_column is None:
            raise ValueError("raw CSV must contain range_m, range, distance, or range_value")
        sample_column = next((name for name in ("sample_timestamp_ns", "timestamp_ns", "sample_timestamp") if name in fields), None)
        if sample_column is None:
            raise ValueError("raw CSV must contain sample_timestamp_ns or timestamp_ns")
        receive_column = next((name for name in ("receive_timestamp_ns", "receive_timestamp") if name in fields), None)
        if receive_column is None:
            raise ValueError("raw CSV must contain receive_timestamp_ns")
        frame_column = next((name for name in ("frame_seq", "frame", "sequence") if name in fields), None)
        output_rows = []
        last_frame = None
        generated_frame = 0
        for line_number, row in enumerate(reader, start=2):
            try:
                sample_ns = int(row[sample_column]); receive_ns = int(row[receive_column])
                anchor_id = str(row["anchor_id"])
                range_value = float(row[range_column]) * UNIT_SCALE[range_unit]
                stddev = float(row.get("stddev_m", row.get("stddev", "0")) or 0.0)
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"invalid raw observation at CSV line {line_number}: {exc}") from exc
            if not np.isfinite(range_value) or range_value <= 0:
                raise ValueError(f"range must be finite and positive at CSV line {line_number}")
            if receive_ns < sample_ns:
                raise ValueError(f"receive timestamp precedes sample timestamp at CSV line {line_number}")
            if anchor_id in anchor_map:
                position = anchor_map[anchor_id]
            else:
                if anchor_map:
                    raise ValueError(f"anchor '{anchor_id}' is absent from anchors YAML at CSV line {line_number}")
                try:
                    position = tuple(float(row[f"anchor_{axis}_m"]) for axis in ("x", "y", "z"))
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError(f"missing coordinates for anchor '{anchor_id}' at CSV line {line_number}") from exc
            if frame_column is not None:
                frame_seq = int(row[frame_column])
            else:
                if last_frame != sample_ns:
                    generated_frame += 1
                frame_seq = generated_frame
            last_frame = sample_ns
            output_rows.append({
                "frame_seq": frame_seq, "sample_timestamp_ns": sample_ns,
                "receive_timestamp_ns": receive_ns, "tag_id": row.get("tag_id", "target"),
                "anchor_id": anchor_id, "anchor_x_m": position[0], "anchor_y_m": position[1],
                "anchor_z_m": position[2], "range_m": range_value, "stddev_m": stddev,
                "point_id": row.get("point_id", "unknown"), "repeat_id": row.get("repeat_id", "1"),
            })
    if not output_rows:
        raise ValueError("raw CSV must contain at least one row")
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=NORMALIZED_COLUMNS)
        writer.writeheader(); writer.writerows(output_rows)
    return output


def load_truth_points(path: str | Path):
    required = {"point_id", "x_m", "y_m", "z_m", "zone", "truth_method", "truth_uncertainty_m"}
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = required - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"truth CSV missing columns: {', '.join(sorted(missing))}")
        points = {}
        for line_number, row in enumerate(reader, start=2):
            point_id = row["point_id"]
            if point_id in points:
                raise ValueError(f"duplicate truth point '{point_id}'")
            values = {axis: float(row[f"{axis}_m"]) for axis in ("x", "y", "z")}
            uncertainty = float(row["truth_uncertainty_m"])
            if not np.all(np.isfinite(list(values.values()))) or not np.isfinite(uncertainty) or uncertainty < 0:
                raise ValueError(f"invalid truth point at CSV line {line_number}")
            points[point_id] = {"x_m": values["x"], "y_m": values["y"], "z_m": values["z"],
                                "zone": row["zone"], "truth_method": row["truth_method"],
                                 "truth_uncertainty_m": uncertainty}
    if not points:
        raise ValueError("truth CSV must contain at least one point")
    return points


def reset_runner_state(runner):
    """Reset module-level EKF/UKF state before a point/repeat sequence."""
    for name in ("_state",):
        if hasattr(runner, name):
            delattr(runner, name)
