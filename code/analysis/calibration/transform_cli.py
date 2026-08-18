import argparse
import json

from geometry_utils import load_json_points, umeyama_alignment


def main(argv=None):
    parser = argparse.ArgumentParser(description="Estimate a rigid target-from-source transform")
    parser.add_argument("--input", required=True, help="JSON file with source and target Nx3 arrays")
    parser.add_argument("--output", required=True, help="JSON output path")
    args = parser.parse_args(argv)
    source, target = load_json_points(args.input)
    with open(args.input, "r", encoding="utf-8") as stream:
        input_document = json.load(stream)
    rotation, translation = umeyama_alignment(source, target)
    result = {
        "rotation": rotation.tolist(),
        "translation_m": translation.tolist(),
        "coordinate_frame": "TBD",
        "source_count": int(source.shape[0]),
        "samples": int(source.shape[0]),
        "unit": "m",
        "statistic_definition": "Umeyama least-squares rigid alignment; no error rejection applied",
        "outlier_rule": "none; calibration input is not silently filtered",
        "outlier_count": 0,
        "dropped_frames": 0,
        "is_test_sample": bool(input_document.get("is_test_sample", False)),
    }
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")


if __name__ == "__main__":
    main()
