import argparse
import json

from pnp import reprojection_error, solve_pnp


def main(argv=None):
    parser = argparse.ArgumentParser(description="Compute target PnP pose and reprojection error")
    parser.add_argument("--input", required=True, help="JSON file with object/image points and camera parameters")
    parser.add_argument("--output", required=True, help="JSON output path")
    args = parser.parse_args(argv)
    with open(args.input, "r", encoding="utf-8") as stream:
        document = json.load(stream)
    rotation, translation = solve_pnp(document["object_points"], document["image_points"], document["camera_matrix"], document["distortion"])
    result = {
        "rotation_vector": rotation.tolist(),
        "translation_m": translation.tolist(),
        "reprojection_error_px": reprojection_error(
            document["object_points"], document["image_points"], document["camera_matrix"], document["distortion"], rotation, translation),
        "coordinate_frame": document.get("frame", "TBD"),
        "samples": len(document["object_points"]),
        "unit": "m (translation), px (reprojection error)",
        "statistic_definition": "PnP pose with RMS pixel reprojection error",
        "outlier_rule": document.get("outlier_rule", "none; image points are not silently filtered"),
        "outlier_count": int(document.get("outlier_count", 0)),
        "dropped_frames": int(document.get("dropped_frames", 0)),
        "is_test_sample": bool(document.get("is_test_sample", False)),
    }
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")


if __name__ == "__main__":
    main()
