#!/usr/bin/env python3
"""Generate one synchronized RGB sequence for ORB-SLAM2, UWB and GDR-Net.

The RGB files used by ORB are also BOP ``test`` images used by the target-pose
network.  Map-frame truth is kept in ``ground_truth`` and never appears in the
BOP camera metadata or algorithm input files.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from generate_gdr_net_dataset import COLOR_RGB, _box_mesh, _cylinder_mesh, _draw_mesh, _meters_scene, _write_ply


def look_at(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    forward = target - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    return np.vstack((right, down, forward))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    parser.add_argument("--scene", default="simulation/sandbox_scene.yaml")
    parser.add_argument("--target-config", default="simulation/gdr_net_dataset_v03.yaml")
    parser.add_argument("--frames", type=int, default=180)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--seed", type=int, default=20260829)
    parser.add_argument("--post-init-speed", type=float, default=0.08)
    parser.add_argument("--scene-split", choices=("train_pbr", "test"), default="test")
    parser.add_argument("--time-offset-s", type=float, default=0.0, help="camera-trajectory offset; use a nonzero value for train/test separation")
    args = parser.parse_args()

    output = Path(args.out)
    scene_dir = output / args.scene_split / "000001"
    truth_dir, models_dir = output / "ground_truth", output / "models"
    for directory in (scene_dir / "rgb", scene_dir / "mask", scene_dir / "mask_visib", truth_dir, models_dir):
        directory.mkdir(parents=True, exist_ok=True)
    scene = yaml.safe_load(Path(args.scene).read_text(encoding="utf-8"))
    config = yaml.safe_load(Path(args.target_config).read_text(encoding="utf-8"))
    all_objects, _deck = _meters_scene(scene)
    object_by_id = {item["id"]: item for item in all_objects}
    selected = []
    for target in config["targets"]:
        source = object_by_id[target["source_object_id"]]
        vertices, faces = _cylinder_mesh(source["size"]) if source["shape"] == "cylinder" else _box_mesh(source["size"])
        selected.append({**target, "source": source, "vertices": vertices, "faces": faces})
    model_info = {}
    for item in selected:
        vertices_mm = item["vertices"] * 1000.0
        object_id = int(item["object_id"])
        _write_ply(models_dir / f"obj_{object_id:06d}.ply", vertices_mm, item["faces"])
        low, high = vertices_mm.min(0), vertices_mm.max(0)
        model_info[str(object_id)] = {
            "diameter": float(np.max(np.linalg.norm(vertices_mm[:, None] - vertices_mm[None, :], axis=2))),
            "min_x": float(low[0]), "min_y": float(low[1]), "min_z": float(low[2]),
            "size_x": float(high[0] - low[0]), "size_y": float(high[1] - low[1]), "size_z": float(high[2] - low[2]),
            "symmetry": item["symmetry"], "source_object_id": item["source_object_id"], "unit": "mm",
        }
    (models_dir / "models_info.json").write_text(json.dumps(model_info, indent=2) + "\n", encoding="utf-8")

    width, height, fx, fy, cx, cy = 640, 480, 535.0, 535.0, 319.5, 239.5
    camera = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    rng = np.random.default_rng(args.seed)
    landmarks = np.column_stack((rng.uniform(0.15, 3.85, 700), rng.uniform(0.15, 4.55, 700), rng.uniform(0.02, 1.70, 700)))
    grid = np.array([(x, y, z) for z in (0.04, 0.35, 0.8, 1.3) for x in np.linspace(0.2, 3.8, 15) for y in np.linspace(0.2, 4.5, 18)])
    landmarks = np.vstack((landmarks, grid))
    colors = rng.integers(30, 240, size=(len(landmarks), 3), dtype=np.uint8)
    patterns = rng.integers(0, 2, size=(len(landmarks), 9, 9), dtype=np.uint8)
    timestamp_base_ns = 1_779_000_000_000_000_000
    timestamp_step_ns = int(round(1e9 / args.fps))
    camera_gt, platform_gt, target_gt, attitudes, uwb_rows = [], [], [], [], []
    bop_gt, bop_info, bop_camera, rgb_rows = {}, {}, {}, []

    for sequence in range(args.frames):
        time_s = sequence / args.fps + args.time_offset_s
        motion_time = min(time_s, 3.0)
        position = np.array([
            2.0 + 0.72 * math.sin(0.9 * motion_time),
            2.35 + 0.85 * math.sin(0.55 * motion_time + 0.4),
            2.75 + 0.12 * math.sin(0.17 * motion_time),
        ])
        if time_s > 3.0:
            position += np.array([args.post_init_speed * (time_s - 3.0), 0.0, 0.0])
        rotation_camera_map = look_at(position, np.array([2.0 + 0.15 * math.sin(0.09 * motion_time), 2.35, 0.25]))
        points_camera = (rotation_camera_map @ (landmarks - position).T).T
        valid = points_camera[:, 2] > 0.15
        pixels_h = points_camera @ camera.T
        pixels = pixels_h[:, :2] / pixels_h[:, 2:3]
        image = np.full((height, width, 3), (122, 148, 164), dtype=np.uint8)
        visible = np.where(valid & (pixels[:, 0] >= 6) & (pixels[:, 0] < width - 6) & (pixels[:, 1] >= 6) & (pixels[:, 1] < height - 6))[0]
        for index in visible[np.argsort(points_camera[visible, 2])[::-1]]:
            x, y = np.rint(pixels[index]).astype(int)
            cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), tuple(int(value) for value in colors[index]), -1)
            for py in range(9):
                for px in range(9):
                    if patterns[index, py, px]:
                        image[y - 4 + py, x - 4 + px] = (20, 20, 20)
        instance_mask = np.zeros((height, width), dtype=np.uint16)
        frame_gt, frame_info = [], []
        # Overlay the actual ten target models into exactly the RGB frames
        # given to ORB. Other textured landmarks remain available for SLAM.
        for item in selected:
            object_id = int(item["object_id"])
            vertices_world = item["vertices"] + item["source"]["position"]
            _draw_mesh(image, instance_mask, vertices_world, item["faces"], COLOR_RGB[item["source"].get("color", "silver")], position, rotation_camera_map, camera, object_id)
            translation = rotation_camera_map @ (item["source"]["position"] - position)
            frame_gt.append({"obj_id": object_id, "cam_R_m2c": rotation_camera_map.reshape(-1).round(12).tolist(), "cam_t_m2c": (translation * 1000.0).round(8).tolist()})
            mask = np.where(instance_mask == object_id, 255, 0).astype(np.uint8)
            cv2.imwrite(str(scene_dir / "mask" / f"{sequence:06d}_{object_id - 1:06d}.png"), mask)
            cv2.imwrite(str(scene_dir / "mask_visib" / f"{sequence:06d}_{object_id - 1:06d}.png"), mask)
            ys, xs = np.where(mask > 0)
            bbox = [int(xs.min()), int(ys.min()), int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1)] if len(xs) else [0, 0, 0, 0]
            frame_info.append({"bbox_obj": bbox, "bbox_visib": bbox, "px_count_all": int(len(xs)), "px_count_visib": int(len(xs)), "visib_fract": 1.0 if len(xs) else 0.0})
        image_path = scene_dir / "rgb" / f"{sequence:06d}.png"
        cv2.imwrite(str(image_path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        timestamp = timestamp_base_ns + sequence * timestamp_step_ns
        rgb_rows.append((timestamp / 1e9, image_path.relative_to(output).as_posix()))
        camera_gt.append((sequence, timestamp, *position, *rotation_camera_map.reshape(-1)))
        platform_gt.append((sequence, timestamp, *position, "base_link"))
        attitudes.append((sequence, timestamp, *rotation_camera_map.reshape(-1), "base_link"))
        for item in selected:
            target_gt.append((sequence, timestamp, int(item["object_id"]), *item["source"]["position"], *np.eye(3).reshape(-1), "map"))
        for anchor in config["uwb"]["anchors_m"]:
            anchor_position = np.asarray(anchor["position_m"], dtype=float)
            measured_range = np.linalg.norm(position - anchor_position) + rng.normal(0.0, float(config["uwb"]["range_noise_std_m"]))
            uwb_rows.append((sequence, timestamp, anchor["id"], measured_range, 1, "base_link"))
        bop_gt[str(sequence)], bop_info[str(sequence)] = frame_gt, frame_info
        bop_camera[str(sequence)] = {"cam_K": camera.reshape(-1).tolist(), "depth_scale": 1.0}

    with (output / "rgb.txt").open("w", encoding="utf-8") as stream:
        stream.write("# ORB/GDRN synchronized sequence\n# timestamp filename\n#\n")
        stream.writelines(f"{timestamp:.9f} {name}\n" for timestamp, name in rgb_rows)
    (output / "camera.yaml").write_text(f"""%YAML:1.0
Camera.fx: {fx}\nCamera.fy: {fy}\nCamera.cx: {cx}\nCamera.cy: {cy}\nCamera.k1: 0.0\nCamera.k2: 0.0\nCamera.p1: 0.0\nCamera.p2: 0.0\nCamera.k3: 0.0\nCamera.fps: {args.fps}\nCamera.RGB: 0\nORBextractor.nFeatures: 1800\nORBextractor.scaleFactor: 1.2\nORBextractor.nLevels: 8\nORBextractor.iniThFAST: 12\nORBextractor.minThFAST: 5\n""", encoding="utf-8")
    (scene_dir / "scene_gt.json").write_text(json.dumps(bop_gt, indent=2) + "\n", encoding="utf-8")
    (scene_dir / "scene_gt_info.json").write_text(json.dumps(bop_info, indent=2) + "\n", encoding="utf-8")
    (scene_dir / "scene_camera.json").write_text(json.dumps(bop_camera, indent=2) + "\n", encoding="utf-8")
    headers = [f"R_map_camera_{r}{c}" for r in range(3) for c in range(3)]
    with (truth_dir / "camera_pose_map.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream); writer.writerow(["sequence", "sample_timestamp_ns", "camera_x_m", "camera_y_m", "camera_z_m", *headers]); writer.writerows(camera_gt)
    with (truth_dir / "platform_pose.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream); writer.writerow(["sequence", "sample_timestamp_ns", "base_x_m", "base_y_m", "base_z_m", "frame_id"]); writer.writerows(platform_gt)
    with (output / "platform_attitude.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream); writer.writerow(["sequence", "sample_timestamp_ns", *[f"base_R_map_{r}{c}" for r in range(3) for c in range(3)], "frame_id"]); writer.writerows(attitudes)
    with (truth_dir / "target_pose_map.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream); writer.writerow(["sequence", "sample_timestamp_ns", "object_id", "target_x_m", "target_y_m", "target_z_m", *[f"map_R_target_{r}{c}" for r in range(3) for c in range(3)], "frame_id"]); writer.writerows(target_gt)
    with (output / "trajectory.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream); writer.writerow(["sequence", "sample_timestamp_ns", "uwb_anchor_id", "uwb_range_m", "range_valid", "frame_id"]); writer.writerows(uwb_rows)
    manifest = {"dataset_id": "nexus_sandbox_orb_gdrn_v01", "frames": args.frames, "targets": len(selected), "fps": args.fps, "seed": args.seed, "scene_split": args.scene_split, "time_offset_s": args.time_offset_s, "coordinate_frame": "map", "unit": "m", "truth_not_visible_to_algorithm": True, "rgb_contract": f"ORB rgb.txt and BOP {args.scene_split}/000001/rgb refer to identical files"}
    manifest["files"] = {str(path.relative_to(output)): _sha256(path) for path in sorted([*scene_dir.rglob("*.json"), *(output / "ground_truth").glob("*.csv"), output / "trajectory.csv", output / "platform_attitude.csv"])}
    (output / "dataset_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"generated {args.frames} synchronized ORB/GDRN frames x {len(selected)} targets at {output}")


if __name__ == "__main__":
    main()
