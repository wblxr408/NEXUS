#!/usr/bin/env python3
import argparse
from pathlib import Path

from measurement_export import (
    normalize_rows, read_csv_rows, read_ros1_bag_rows, read_ros2_bag_rows,
    write_normalized_csv,
)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Convert ROS1 bag, ROS2 bag, or flight-controller SD CSV to NEXUS CSV")
    parser.add_argument("--format", required=True, choices=("ros1_bag", "ros2_bag", "sd_csv"))
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--topic", help="required for ros1_bag and ros2_bag")
    parser.add_argument("--target-id", default="target_0")
    parser.add_argument("--frame-id", default="TBD")
    parser.add_argument("--source-mode", default="UNKNOWN")
    parser.add_argument("--data-type", choices=("measured", "test_sample"), default="measured")
    args = parser.parse_args(argv)
    if args.format in {"ros1_bag", "ros2_bag"} and not args.topic:
        parser.error("--topic is required for bag conversion")
    if args.format == "sd_csv":
        rows = read_csv_rows(args.input)
    elif args.format == "ros1_bag":
        rows = read_ros1_bag_rows(args.input, args.topic)
    else:
        rows = read_ros2_bag_rows(args.input, args.topic)
    normalized = normalize_rows(
        rows, target_id=args.target_id, frame_id=args.frame_id,
        source_mode=args.source_mode, data_type=args.data_type)
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    write_normalized_csv(args.output, normalized)


if __name__ == "__main__":
    main()
