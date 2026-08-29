#!/usr/bin/env python3
"""Generate a compact, reproducible BOP-style synthetic 6D dataset.

The renderer intentionally uses only NumPy/OpenCV and parameterised sandbox
geometry, so it can run before Blender or a neural-network environment exists.
It produces useful image/pose/mask/trajectory contracts, but is a geometric
baseline rather than photo-realistic training data.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from contextlib import ExitStack
from pathlib import Path

import cv2
import numpy as np
import yaml


COLOR_RGB = {
    "silver": (165, 171, 178), "orange": (242, 89, 13),
    "purple": (115, 46, 140), "blue_red": (20, 89, 191),
    "green": (26, 140, 56), "yellow": (230, 184, 20),
    "white_yellow": (242, 214, 128), "concrete": (117, 128, 125),
    "road": (20, 23, 26), "grass": (31, 115, 31),
}


def _meters_scene(scene):
    """Read objects authored in millimetres and return metric copies."""
    scale = 0.001 if scene.get("unit") == "mm" else 1.0
    result = []
    deck = float(scene["dimensions"]["deck_height"]) * scale
    for raw in scene.get("objects", []):
        item = dict(raw)
        item["position"] = np.asarray(item["position"], dtype=float) * scale
        item["size"] = np.asarray(item["size"], dtype=float) * scale
        if scene.get("object_z_mode", "center") == "base":
            item["position"][2] += deck + item["size"][2] / 2.0
        result.append(item)
    return result, deck


def _box_mesh(size):
    sx, sy, sz = np.asarray(size, dtype=float) / 2.0
    vertices = np.array([
        [-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
        [-sx, -sy, sz], [sx, -sy, sz], [sx, sy, sz], [-sx, sy, sz],
    ], dtype=float)
    faces = np.array([
        [0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6], [0, 4, 5], [0, 5, 1],
        [1, 5, 6], [1, 6, 2], [2, 6, 7], [2, 7, 3], [4, 0, 3], [4, 3, 7],
    ], dtype=np.int32)
    return vertices, faces


def _cylinder_mesh(size, segments=32):
    radius = min(float(size[0]), float(size[1])) / 2.0
    half_height = float(size[2]) / 2.0
    angle = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    bottom = np.column_stack((radius * np.cos(angle), radius * np.sin(angle), np.full(segments, -half_height)))
    top = bottom.copy(); top[:, 2] = half_height
    vertices = np.vstack((bottom, top, [[0.0, 0.0, -half_height], [0.0, 0.0, half_height]]))
    bottom_center, top_center = 2 * segments, 2 * segments + 1
    faces = []
    for index in range(segments):
        nxt = (index + 1) % segments
        faces.extend(((index, nxt, segments + nxt), (index, segments + nxt, segments + index),
                      (bottom_center, nxt, index), (top_center, segments + index, segments + nxt)))
    return vertices, np.asarray(faces, dtype=np.int32)


def _write_ply(path, vertices, faces):
    with path.open("w", encoding="ascii") as stream:
        stream.write("ply\nformat ascii 1.0\n")
        stream.write(f"element vertex {len(vertices)}\nproperty float x\nproperty float y\nproperty float z\n")
        stream.write(f"element face {len(faces)}\nproperty list uchar int vertex_indices\nend_header\n")
        for point in vertices:
            stream.write(f"{point[0]:.8f} {point[1]:.8f} {point[2]:.8f}\n")
        for face in faces:
            stream.write(f"3 {face[0]} {face[1]} {face[2]}\n")


def _camera_matrix(image):
    width, height = int(image["width_px"]), int(image["height_px"])
    focal = (width / 2.0) / math.tan(math.radians(float(image["horizontal_fov_deg"])) / 2.0)
    return np.array([[focal, 0.0, (width - 1) / 2.0], [0.0, focal, (height - 1) / 2.0], [0.0, 0.0, 1.0]])


def _look_at(camera_position, look_at):
    """Return world-to-optical R where optical axes are x-right/y-down/z-forward."""
    forward = np.asarray(look_at, dtype=float) - np.asarray(camera_position, dtype=float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    return np.vstack((right, down, forward))


def _project(points_world, camera_position, rotation_camera_map, camera):
    points_camera = (rotation_camera_map @ (points_world - camera_position).T).T
    projected = points_camera @ camera.T
    pixels = projected[:, :2] / projected[:, 2:3]
    return pixels, points_camera[:, 2]


def _draw_mesh(image, mask, vertices_world, faces, color, camera_position, rotation_camera_map, camera, object_id=0):
    pixels, depth = _project(vertices_world, camera_position, rotation_camera_map, camera)
    draws = []
    for face in faces:
        z = depth[face]
        if np.any(z <= 0.03):
            continue
        polygon = np.rint(pixels[face]).astype(np.int32)
        if cv2.contourArea(polygon) == 0:
            continue
        draws.append((float(np.mean(z)), polygon))
    # Far faces are drawn first. This is sufficient for the opaque convex
    # primitives in this initial model asset set.
    for z, polygon in sorted(draws, key=lambda item: item[0], reverse=True):
        shade = max(0.55, min(1.0, 1.15 - 0.10 * z))
        shaded = tuple(int(max(0, min(255, value * shade))) for value in color)
        cv2.fillConvexPoly(image, polygon, shaded)
        if object_id:
            cv2.fillConvexPoly(mask, polygon, int(object_id))


def _bbox_from_mask(mask, object_id):
    ys, xs = np.where(mask == object_id)
    if xs.size == 0:
        return None
    return [int(xs.min()), int(ys.min()), int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1)]


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default="simulation/sandbox_scene.yaml")
    parser.add_argument("--config", default="simulation/gdr_net_dataset.yaml")
    parser.add_argument("--out", default="data/processed/nexus_sandbox_gdr_net_v01")
    args = parser.parse_args()
    scene = yaml.safe_load(Path(args.scene).read_text(encoding="utf-8"))
    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    output = Path(args.out)
    models_dir = output / "models"
    # v01 used train_pbr as a convenient container for its five geometry
    # frames.  A BOP evaluation split belongs under test; keep the default for
    # backwards-compatible reruns and let a versioned config select test.
    scene_split = str(config.get("scene_split", "train_pbr"))
    scene_dir = output / scene_split / "000001"
    isolate_map_truth = bool(config.get("isolate_map_truth", False))
    truth_dir = output / "ground_truth"
    for directory in (models_dir, scene_dir / "rgb", scene_dir / "mask", scene_dir / "mask_visib", truth_dir):
        directory.mkdir(parents=True, exist_ok=True)

    all_objects, deck = _meters_scene(scene)
    by_id = {item["id"]: item for item in all_objects}
    selected = []
    for item in config["targets"]:
        source = by_id.get(item["source_object_id"])
        if source is None:
            raise ValueError(f"target source_object_id not found: {item['source_object_id']}")
        if source.get("shape") not in {"box", "cylinder"}:
            raise ValueError(f"unsupported target shape: {source.get('shape')}")
        mesh = _cylinder_mesh(source["size"]) if source["shape"] == "cylinder" else _box_mesh(source["size"])
        selected.append({**item, "source": source, "vertices": mesh[0], "faces": mesh[1]})

    models_info = {}
    for item in selected:
        object_id = int(item["object_id"])
        vertices_mm = item["vertices"] * 1000.0
        _write_ply(models_dir / f"obj_{object_id:06d}.ply", vertices_mm, item["faces"])
        minimum, maximum = vertices_mm.min(axis=0), vertices_mm.max(axis=0)
        models_info[str(object_id)] = {
            "diameter": float(np.max(np.linalg.norm(vertices_mm[:, None, :] - vertices_mm[None, :, :], axis=2))),
            "min_x": float(minimum[0]), "min_y": float(minimum[1]), "min_z": float(minimum[2]),
            "size_x": float(maximum[0] - minimum[0]), "size_y": float(maximum[1] - minimum[1]), "size_z": float(maximum[2] - minimum[2]),
            "symmetry": item["symmetry"], "source_object_id": item["source_object_id"], "unit": "mm",
        }
    (models_dir / "models_info.json").write_text(json.dumps(models_info, indent=2) + "\n", encoding="utf-8")

    rng = np.random.default_rng(int(config["random_seed"]))
    bounds = np.asarray(config["platform"]["hover_xy_bounds_m"], dtype=float)
    count = int(config["platform"]["hover_count"])
    hover_positions = np.column_stack((
        rng.uniform(bounds[0, 0], bounds[0, 1], count),
        rng.uniform(bounds[1, 0], bounds[1, 1], count),
        rng.uniform(*config["platform"]["hover_height_m"], count),
    ))
    camera = _camera_matrix(config["image"])
    width, height = int(config["image"]["width_px"]), int(config["image"]["height_px"])
    gt, gt_info, camera_info = {}, {}, {}
    base_timestamp_ns = 1_778_000_000_000_000_000
    step_ns = int(1e9 / float(config["platform"]["sample_rate_hz"]))
    trajectory_path = output / "trajectory.csv"
    platform_gt_path = truth_dir / "platform_pose.csv"
    target_gt_path = truth_dir / "target_pose_map.csv"
    attitude_config = config.get("attitude_observation")
    if attitude_config and float(attitude_config.get("rotation_noise_std_deg", 0.0)) != 0.0:
        raise ValueError("platform attitude noise is not implemented; use rotation_noise_std_deg: 0.0")
    attitude_path = output / "platform_attitude.csv" if attitude_config else None
    with ExitStack() as stack:
        trajectory_file = stack.enter_context(trajectory_path.open("w", encoding="utf-8", newline=""))
        platform_file = stack.enter_context(platform_gt_path.open("w", encoding="utf-8", newline=""))
        target_file = stack.enter_context(target_gt_path.open("w", encoding="utf-8", newline=""))
        attitude_file = stack.enter_context(attitude_path.open("w", encoding="utf-8", newline="")) if attitude_path else None
        fields = ["sequence", "sample_timestamp_ns", "uwb_anchor_id", "uwb_range_m", "range_valid", "frame_id"]
        writer = csv.DictWriter(trajectory_file, fieldnames=fields); writer.writeheader()
        platform_fields = ["sequence", "sample_timestamp_ns", "base_x_m", "base_y_m", "base_z_m"]
        target_fields = ["sequence", "sample_timestamp_ns", "object_id", "target_x_m", "target_y_m", "target_z_m"]
        if isolate_map_truth:
            platform_fields += [f"base_R_map_{row}{column}" for row in range(3) for column in range(3)]
            target_fields += [f"map_R_target_{row}{column}" for row in range(3) for column in range(3)]
        platform_fields.append("frame_id")
        target_fields.append("frame_id")
        platform_writer = csv.DictWriter(platform_file, fieldnames=platform_fields); platform_writer.writeheader()
        target_writer = csv.DictWriter(target_file, fieldnames=target_fields); target_writer.writeheader()
        attitude_writer = None
        if attitude_file:
            attitude_fields = [
                "sequence", "sample_timestamp_ns",
                *[f"base_R_map_{row}{column}" for row in range(3) for column in range(3)],
                "frame_id",
            ]
            attitude_writer = csv.DictWriter(attitude_file, fieldnames=attitude_fields)
            attitude_writer.writeheader()
        for frame_index, position in enumerate(hover_positions):
            timestamp = base_timestamp_ns + frame_index * step_ns
            rotation_camera_map = _look_at(position, config["platform"]["look_at_map_m"])
            platform_row = {
                "sequence": frame_index, "sample_timestamp_ns": timestamp,
                "base_x_m": position[0], "base_y_m": position[1], "base_z_m": position[2],
                "frame_id": config["platform"]["frame_id"],
            }
            if isolate_map_truth:
                platform_row.update({
                    f"base_R_map_{row}{column}": rotation_camera_map[row, column]
                    for row in range(3) for column in range(3)
                })
            platform_writer.writerow(platform_row)
            if attitude_writer:
                attitude_row = {
                    "sequence": frame_index, "sample_timestamp_ns": timestamp,
                    "frame_id": attitude_config.get("frame_id", config["platform"]["frame_id"]),
                }
                attitude_row.update({
                    f"base_R_map_{row}{column}": rotation_camera_map[row, column]
                    for row in range(3) for column in range(3)
                })
                attitude_writer.writerow(attitude_row)
            image = np.full((height, width, 3), config["image"]["background_rgb"], dtype=np.uint8)
            image[:, :, :] = config["image"]["ground_rgb"]
            instance_mask = np.zeros((height, width), dtype=np.uint16)
            # A simple metric ground slab preserves visual scale and horizon-free
            # top-down context. Targets are then overlaid in depth order.
            ground_vertices = np.array([[0, 0, deck], [4, 0, deck], [4, 4.7, deck], [0, 4.7, deck]], dtype=float)
            _draw_mesh(image, instance_mask, ground_vertices, np.array([[0, 1, 2], [0, 2, 3]]), COLOR_RGB["grass"], position, rotation_camera_map, camera)
            # Render the rest of the parameterised sandbox first as an opaque
            # context.  It has no instance labels, so it cannot leak truth to
            # the estimator; selected targets are then rendered on top for the
            # initial no-occlusion baseline.  A later renderer can depth-sort
            # all triangles and add measured textures/occluders.
            selected_sources = {item["source"]["id"] for item in selected}
            for source in all_objects:
                if source["id"] in selected_sources or source.get("category") == "road_marking":
                    continue
                shape_mesh = _cylinder_mesh(source["size"]) if source.get("shape") == "cylinder" else _box_mesh(source["size"])
                context_vertices, context_faces = shape_mesh
                context_vertices = context_vertices + source["position"]
                _draw_mesh(image, instance_mask, context_vertices, context_faces, COLOR_RGB.get(source.get("color"), (150, 150, 150)), position, rotation_camera_map, camera)
            for item in selected:
                world_vertices = item["vertices"] + item["source"]["position"]
                _draw_mesh(image, instance_mask, world_vertices, item["faces"], COLOR_RGB.get(item["source"].get("color"), (180, 180, 180)), position, rotation_camera_map, camera, int(item["object_id"]))
            frame_name = f"{frame_index:06d}.png"
            cv2.imwrite(str(scene_dir / "rgb" / frame_name), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            frame_gt, frame_info = [], []
            for item in selected:
                object_id = int(item["object_id"])
                translation = rotation_camera_map @ (item["source"]["position"] - position)
                target_row = {
                    "sequence": frame_index, "sample_timestamp_ns": timestamp, "object_id": object_id,
                    "target_x_m": item["source"]["position"][0], "target_y_m": item["source"]["position"][1],
                    "target_z_m": item["source"]["position"][2], "frame_id": config["coordinate_frame"],
                }
                if isolate_map_truth:
                    # The parameterised assets are authored with their local
                    # axes aligned to map. map_R_target maps target-local
                    # coordinates into map and is evaluator-only truth.
                    target_row.update({f"map_R_target_{row}{column}": float(row == column) for row in range(3) for column in range(3)})
                target_writer.writerow(target_row)
                frame_gt.append({"obj_id": object_id, "cam_R_m2c": rotation_camera_map.reshape(-1).round(12).tolist(), "cam_t_m2c": (translation * 1000.0).round(8).tolist()})
                object_mask = np.where(instance_mask == object_id, 255, 0).astype(np.uint8)
                cv2.imwrite(str(scene_dir / "mask" / f"{frame_index:06d}_{object_id - 1:06d}.png"), object_mask)
                cv2.imwrite(str(scene_dir / "mask_visib" / f"{frame_index:06d}_{object_id - 1:06d}.png"), object_mask)
                bbox = _bbox_from_mask(instance_mask, object_id)
                frame_info.append({"bbox_obj": bbox or [0, 0, 0, 0], "bbox_visib": bbox or [0, 0, 0, 0], "px_count_all": int(np.count_nonzero(object_mask)), "px_count_visib": int(np.count_nonzero(object_mask)), "visib_fract": 1.0 if bbox else 0.0})
            gt[str(frame_index)], gt_info[str(frame_index)] = frame_gt, frame_info
            camera_info[str(frame_index)] = {"cam_K": camera.reshape(-1).round(12).tolist(), "depth_scale": 1.0}
            if not isolate_map_truth:
                # v01 compatibility: superseded by v02, which keeps this
                # estimator-visible BOP file free of map-pose truth.
                camera_info[str(frame_index)].update({"timestamp_ns": timestamp, "camera_position_map_m": position.round(8).tolist(), "camera_R_map": rotation_camera_map.reshape(-1).round(12).tolist()})
            for anchor in config["uwb"]["anchors_m"]:
                anchor_position = np.asarray(anchor["position_m"], dtype=float)
                range_m = float(np.linalg.norm(position - anchor_position) + config["uwb"]["range_bias_m"] + rng.normal(0.0, config["uwb"]["range_noise_std_m"]))
                writer.writerow({"sequence": frame_index, "sample_timestamp_ns": timestamp, "uwb_anchor_id": anchor["id"], "uwb_range_m": range_m, "range_valid": True, "frame_id": config["platform"]["frame_id"]})
    (scene_dir / "scene_gt.json").write_text(json.dumps(gt, indent=2) + "\n", encoding="utf-8")
    (scene_dir / "scene_gt_info.json").write_text(json.dumps(gt_info, indent=2) + "\n", encoding="utf-8")
    (scene_dir / "scene_camera.json").write_text(json.dumps(camera_info, indent=2) + "\n", encoding="utf-8")
    manifest = {
        "dataset_id": config["dataset_id"], "scene_mode": config["scene_mode"], "coordinate_frame": config["coordinate_frame"], "unit": config["unit"], "random_seed": config["random_seed"],
        "renderer": "opencv_parameterized_geometry_v01", "photorealistic": False,
        "frame_count": count, "target_count": len(selected), "bop_translation_unit": "mm", "trajectory": "trajectory.csv",
        # Exclude the manifest itself so rerunning into an existing directory
        # cannot make its checksum depend on the previous run's contents.
        "files": {
            str(path.relative_to(output)): _sha256(path)
            for path in [*(sorted(output.rglob("*.json"))), *(sorted(output.rglob("*.csv")))]
            if path.name != "dataset_manifest.json"
        },
        "limitations": ["Target geometry comes from parameterised YAML, not surveyed CAD.", "RGB uses flat geometric rendering without textures, lighting, occlusion or sensor noise.", "The dataset supports contract and geometry testing; it is not sufficient evidence of real-world GDR-Net performance."],
    }
    if isolate_map_truth:
        manifest["bop_scene_path"] = str(scene_dir.relative_to(output))
    if attitude_path:
        manifest["platform_attitude_observation"] = str(attitude_path.relative_to(output))
    (output / "dataset_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"generated {count} frames x {len(selected)} targets at {output}")


if __name__ == "__main__":
    main()
