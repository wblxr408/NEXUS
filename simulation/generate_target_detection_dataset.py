#!/usr/bin/env python3
"""Create trajectory-disjoint detector and 6D-pose data for ten sandbox targets.

This generator is deliberately upstream-model agnostic.  It writes detector
labels (COCO + YOLO), BOP camera->target pose labels, masks, calibrated camera
metadata, and evaluator-only map truth.  It does not modify GDR-Net or use a
ground-truth ROI as an inference input.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml

from generate_gdr_net_dataset import COLOR_RGB, _box_mesh, _cylinder_mesh, _draw_mesh, _meters_scene, _write_ply


def _box(size: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    x, y, z = np.asarray(size, dtype=float) / 2.0
    vertices = np.array([[-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z],
                         [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]], dtype=float)
    faces = np.array([[0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6], [0, 4, 5], [0, 5, 1],
                      [1, 5, 6], [1, 6, 2], [2, 6, 7], [4, 0, 3], [4, 3, 7]], dtype=np.int32)
    return vertices, faces


def _cylinder(diameter: float, height: float, segments: int = 20) -> tuple[np.ndarray, np.ndarray]:
    radius = diameter / 2.0
    angle = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    bottom = np.column_stack((radius * np.cos(angle), radius * np.sin(angle), np.full(segments, -height / 2.0)))
    top = bottom.copy(); top[:, 2] = height / 2.0
    vertices = np.vstack((bottom, top, [[0.0, 0.0, -height / 2.0], [0.0, 0.0, height / 2.0]]))
    faces: list[list[int]] = []
    for index in range(segments):
        nxt = (index + 1) % segments
        faces.extend([[index, nxt, segments + nxt], [index, segments + nxt, segments + index],
                      [2 * segments, nxt, index], [2 * segments + 1, segments + index, segments + nxt]])
    return vertices, np.asarray(faces, dtype=np.int32)


def _frustum(bottom_diameter: float, top_diameter: float, height: float, segments: int = 20) -> tuple[np.ndarray, np.ndarray]:
    angle = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    bottom = np.column_stack((bottom_diameter / 2.0 * np.cos(angle), bottom_diameter / 2.0 * np.sin(angle), np.full(segments, -height / 2.0)))
    top = np.column_stack((top_diameter / 2.0 * np.cos(angle), top_diameter / 2.0 * np.sin(angle), np.full(segments, height / 2.0)))
    vertices = np.vstack((bottom, top, [[0.0, 0.0, -height / 2.0], [0.0, 0.0, height / 2.0]]))
    faces: list[list[int]] = []
    for index in range(segments):
        nxt = (index + 1) % segments
        faces.extend([[index, nxt, segments + nxt], [index, segments + nxt, segments + index],
                      [2 * segments, nxt, index], [2 * segments + 1, segments + index, segments + nxt]])
    return vertices, np.asarray(faces, dtype=np.int32)


def _merge(parts: list[tuple[np.ndarray, np.ndarray]]) -> tuple[np.ndarray, np.ndarray]:
    vertices, faces, offset = [], [], 0
    for part_vertices, part_faces in parts:
        vertices.append(part_vertices)
        faces.append(part_faces + offset)
        offset += len(part_vertices)
    return np.vstack(vertices), np.vstack(faces)


def _translated(mesh: tuple[np.ndarray, np.ndarray], offset: list[float] | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return mesh[0] + np.asarray(offset, dtype=float), mesh[1]


def _triangular_prism(size: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    x, y, z = size / 2.0
    vertices = np.array([[-x, -y, -z], [x, -y, -z], [0.0, y, -z],
                         [-x, -y, z], [x, -y, z], [0.0, y, z]], dtype=float)
    faces = np.array([[0, 1, 2], [3, 5, 4], [0, 3, 4], [0, 4, 1], [1, 4, 5], [1, 5, 2], [2, 5, 3], [2, 3, 0]], dtype=np.int32)
    return vertices, faces


def _target_mesh(shape: str, size: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    x, y, z = size
    if shape == "cylinder":
        return _cylinder(min(x, y), z)
    if shape == "horizontal_cylinder":
        vertices, faces = _cylinder(min(y, z), x)
        rotation = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
        return (rotation @ vertices.T).T, faces
    if shape == "frustum":
        return _frustum(min(x, y), 0.52 * min(x, y), z)
    if shape == "silo_roof":
        body_height = z * 0.72
        body = _translated(_cylinder(min(x, y), body_height), [0.0, 0.0, -z * 0.14])
        roof = _translated(_frustum(min(x, y), 0.18 * min(x, y), z * 0.28), [0.0, 0.0, z * 0.36])
        return _merge([body, roof])
    if shape == "triangular_prism":
        return _triangular_prism(size)
    if shape == "hex_prism":
        return _cylinder(min(x, y), z, segments=6)
    if shape == "stacked_box":
        lower = _translated(_box(np.array([x, y, z * 0.62])), [0.0, 0.0, -z * 0.19])
        upper = _translated(_box(np.array([x * 0.60, y * 0.60, z * 0.38])), [0.0, 0.0, z * 0.31])
        return _merge([lower, upper])
    if shape == "l_prism":
        first = _translated(_box(np.array([x * 0.45, y, z])), [-x * 0.275, 0.0, 0.0])
        second = _translated(_box(np.array([x * 0.55, y * 0.42, z])), [x * 0.225, -y * 0.29, 0.0])
        return _merge([first, second])
    if shape == "cross_prism":
        first = _box(np.array([x, y * 0.42, z]))
        second = _box(np.array([x * 0.42, y, z]))
        cap = _translated(_cylinder(min(x, y) * 0.30, z * 0.25, segments=12), [0.0, 0.0, z * 0.50])
        return _merge([first, second, cap])
    if shape == "twin_cylinder":
        diameter = min(y * 0.62, z)
        left = _translated(_cylinder(diameter, z), [-x * 0.23, 0.0, 0.0])
        right = _translated(_cylinder(diameter, z), [x * 0.23, 0.0, 0.0])
        bridge = _translated(_box(np.array([x * 0.46, y * 0.25, z * 0.22])), [0.0, 0.0, z * 0.22])
        return _merge([left, right, bridge])
    raise ValueError(f"unsupported target shape: {shape}")


def _rotation_z(degrees: float) -> np.ndarray:
    radians = math.radians(degrees)
    cosine, sine = math.cos(radians), math.sin(radians)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def _look_at(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    forward = target - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right_norm = np.linalg.norm(right)
    # A nadir view has no yaw implied by look-at.  Use the map +X direction
    # as the deterministic optical-right convention rather than emitting NaNs.
    if right_norm < 1e-9:
        right = np.array([1.0, 0.0, 0.0])
    else:
        right /= right_norm
    down = np.cross(forward, right)
    return np.vstack((right, down, forward))


def _trajectory(name: str, frame: int, count: int, height_range: list[float]) -> np.ndarray:
    phase = 2.0 * np.pi * frame / count
    if name == "train_orbit_west":
        xy = (1.55 + 0.48 * math.cos(phase), 2.45 + 0.68 * math.sin(phase))
    elif name == "train_orbit_east":
        xy = (2.45 + 0.50 * math.cos(phase + 0.4), 2.43 + 0.65 * math.sin(phase + 0.4))
    elif name == "train_diagonal":
        xy = (1.25 + 1.50 * frame / max(1, count - 1), 1.48 + 1.72 * frame / max(1, count - 1))
    elif name == "val_figure_eight":
        xy = (2.0 + 0.72 * math.sin(phase), 2.35 + 0.70 * math.sin(2.0 * phase))
    elif name == "test_perimeter_arc":
        arc = -2.35 + 4.7 * frame / max(1, count - 1)
        xy = (2.0 + 0.97 * math.cos(arc), 2.35 + 1.08 * math.sin(arc))
    else:
        raise ValueError(f"unsupported trajectory: {name}")
    height = float(height_range[0]) + (float(height_range[1]) - float(height_range[0])) * (0.5 + 0.5 * math.sin(phase * 0.73 + 0.2))
    return np.array([xy[0], xy[1], height], dtype=float)


def _bbox(mask: np.ndarray, object_id: int) -> tuple[list[int], int]:
    ys, xs = np.where(mask == object_id)
    if len(xs) == 0:
        return [0, 0, 0, 0], 0
    return [int(xs.min()), int(ys.min()), int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1)], int(len(xs))


def _camera_yaml(camera: dict) -> str:
    k = np.asarray(camera["K"], dtype=float)
    return "\n".join(["%YAML:1.0", f"Camera.fx: {k[0, 0]}", f"Camera.fy: {k[1, 1]}", f"Camera.cx: {k[0, 2]}", f"Camera.cy: {k[1, 2]}",
                      *[f"Camera.{name}: {value}" for name, value in zip(("k1", "k2", "p1", "p2", "k3"), camera["D"])],
                      "Camera.RGB: 0", "ORBextractor.nFeatures: 1800", "ORBextractor.scaleFactor: 1.2", "ORBextractor.nLevels: 8", "ORBextractor.iniThFAST: 12", "ORBextractor.minThFAST: 5", ""])


def _scene_color(name: str) -> tuple[int, int, int]:
    """Resolve colours used by the planning scene, with neutral fallbacks."""
    extra = {
        "road_white": (226, 226, 216), "glass": (76, 139, 164),
        "beige": (188, 166, 125), "blue_gray": (91, 118, 137),
    }
    return tuple(extra.get(name, COLOR_RGB.get(name, COLOR_RGB["concrete"])))


def _static_scene_meshes(scene: dict, target_source_ids: set[str]) -> list[tuple[np.ndarray, np.ndarray, tuple[int, int, int]]]:
    """Convert the authored sandbox context into unlabeled metric render meshes.

    The ten legacy objects replaced by the catalog proxies are intentionally
    excluded.  Background meshes use object_id=0 and never enter BOP/COCO
    labels, so the dataset remains a target-detector/pose dataset.
    """
    scale = 0.001 if scene.get("unit") == "mm" else 1.0
    _, deck = _meters_scene(scene)
    meshes: list[tuple[np.ndarray, np.ndarray, tuple[int, int, int]]] = []

    for road in scene.get("roads", []):
        size = np.asarray(road["size"], dtype=float) * scale
        height = float(road.get("height", 2.0)) * scale
        center = np.asarray(road["center"], dtype=float) * scale
        vertices, faces = _box(np.array([size[0], size[1], height]))
        vertices += np.array([center[0], center[1], deck + height / 2.0])
        meshes.append((vertices, faces, _scene_color("road")))

    for item in _meters_scene(scene)[0]:
        if item["id"] in target_source_ids:
            continue
        mesh = _cylinder_mesh(item["size"]) if item.get("shape") == "cylinder" else _box_mesh(item["size"])
        meshes.append((mesh[0] + item["position"], mesh[1], _scene_color(item.get("color", "concrete"))))

    marking_z = deck + 0.005
    for marking in scene.get("road_markings", []):
        color = _scene_color(marking.get("color", "road_white"))
        if marking["kind"] == "lane_dashes":
            start, end = np.asarray(marking["start"], dtype=float) * scale, np.asarray(marking["end"], dtype=float) * scale
            delta, distance = end - start, float(np.linalg.norm(end - start))
            unit = delta / distance
            dash, gap = float(marking["dash_length"]) * scale, float(marking["gap"]) * scale
            count = int(math.floor((distance + gap) / (dash + gap)))
            for index in range(count):
                length = min(dash, distance - index * (dash + gap))
                if length <= 0:
                    continue
                center = start + unit * (index * (dash + gap) + length / 2.0)
                size = np.array([length, float(marking["width"]) * scale, 0.004]) if abs(unit[0]) > abs(unit[1]) else np.array([float(marking["width"]) * scale, length, 0.004])
                vertices, faces = _box(size); vertices += np.array([center[0], center[1], marking_z])
                meshes.append((vertices, faces, color))
        elif marking["kind"] == "crosswalk":
            center = np.asarray(marking["center"], dtype=float) * scale
            count, spacing = int(marking["count"]), float(marking["spacing"]) * scale
            for index in range(count):
                offset = (index - (count - 1) / 2.0) * spacing
                if marking["axis"] == "x":
                    size, location = np.array([float(marking["bar_length"]) * scale, float(marking["bar_width"]) * scale, 0.004]), center + np.array([0.0, offset])
                else:
                    size, location = np.array([float(marking["bar_width"]) * scale, float(marking["bar_length"]) * scale, 0.004]), center + np.array([offset, 0.0])
                vertices, faces = _box(size); vertices += np.array([location[0], location[1], marking_z])
                meshes.append((vertices, faces, color))

    for row in scene.get("tree_rows", []):
        start, end = np.asarray(row["start"], dtype=float) * scale, np.asarray(row["end"], dtype=float) * scale
        for center in np.linspace(start, end, int(row["count"])):
            tree_height = float(row["height"]) * scale
            vertices, faces = _frustum(0.085, 0.030, tree_height, segments=10)
            vertices += np.array([center[0], center[1], deck + tree_height / 2.0])
            meshes.append((vertices, faces, (42, 108, 49)))

    for group in scene.get("traffic_light_groups", []):
        for xy in group.get("positions", []):
            vertices, faces = _box(np.array([0.026, 0.026, 0.125]))
            vertices += np.array([xy[0] * scale, xy[1] * scale, deck + 0.0625])
            meshes.append((vertices, faces, (40, 42, 38)))

    for group in scene.get("cone_groups", []):
        center = np.asarray(group["center"], dtype=float) * scale
        for angle in np.linspace(0.0, 2.0 * np.pi, int(group["count"]), endpoint=False):
            xy = center + float(group["radius"]) * scale * np.array([math.cos(angle), math.sin(angle)])
            cone_height = float(group["height"]) * scale
            vertices, faces = _frustum(float(group["diameter"]) * scale, 0.006, cone_height, segments=10)
            vertices += np.array([xy[0], xy[1], deck + cone_height / 2.0])
            meshes.append((vertices, faces, (234, 106, 30)))
    return meshes


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="simulation/target_detection_dataset_v01.yaml")
    parser.add_argument("--out", required=True)
    parser.add_argument("--yolo-assets", choices=("symlink", "copy"), default="symlink", help="copy assets when a Windows process must read the dataset")
    args = parser.parse_args()
    config_path, output = Path(args.config), Path(args.out)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to overwrite existing dataset: {output}")
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    catalog_path = Path(config["target_catalog"])
    catalog = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
    scene_path = Path(config["scene"])
    scene = yaml.safe_load(scene_path.read_text(encoding="utf-8"))
    scale = 0.001 if catalog.get("unit") == "mm" else 1.0
    targets = []
    for raw in catalog["targets"]:
        size = np.asarray(raw["size_mm"], dtype=float) * scale
        mesh = _target_mesh(raw["shape"], size)
        rotation_map_target = _rotation_z(float(raw["yaw_deg"]))
        targets.append({**raw, "size_m": size, "mesh": mesh, "position_m": np.asarray(raw["position_mm"], dtype=float) * scale,
                        "rotation_map_target": rotation_map_target})
    static_scene = _static_scene_meshes(scene, {str(target["source_object_id"]) for target in targets})
    camera = config["camera"]
    width, height = int(camera["image_width_px"]), int(camera["image_height_px"])
    matrix = np.asarray(camera["K"], dtype=float)
    rng = np.random.default_rng(int(config["render"]["seed"]))
    landmarks = np.column_stack((rng.uniform(0.15, 3.85, int(config["render"]["texture_landmark_count"])),
                                rng.uniform(0.15, 4.55, int(config["render"]["texture_landmark_count"])),
                                rng.uniform(0.01, 0.30, int(config["render"]["texture_landmark_count"]))))
    landmark_colors = rng.integers(35, 235, size=(len(landmarks), 3), dtype=np.uint8)
    landmark_patterns = rng.integers(0, 2, size=(len(landmarks), 7, 7), dtype=np.uint8)
    output.mkdir(parents=True)
    (output / "camera.yaml").write_text(_camera_yaml(camera), encoding="utf-8")
    (output / "calibration").mkdir()
    (output / "calibration" / "camera_info.json").write_text(json.dumps(camera, indent=2) + "\n", encoding="utf-8")
    (output / "models").mkdir()
    model_info = {}
    for target in targets:
        vertices, faces = target["mesh"]
        _write_ply(output / "models" / f"obj_{int(target['object_id']):06d}.ply", vertices * 1000.0, faces)
        low, high = (vertices * 1000.0).min(0), (vertices * 1000.0).max(0)
        model_info[str(target["object_id"])] = {"diameter": float(np.max(np.linalg.norm(vertices[:, None] - vertices[None, :], axis=2)) * 1000.0),
                                                  "min_x": float(low[0]), "min_y": float(low[1]), "min_z": float(low[2]),
                                                  "size_x": float(high[0] - low[0]), "size_y": float(high[1] - low[1]), "size_z": float(high[2] - low[2]),
                                                  "symmetry": target["symmetry"], "class_name": target["class_name"], "source_object_id": target["source_object_id"]}
    (output / "models" / "models_info.json").write_text(json.dumps(model_info, indent=2) + "\n", encoding="utf-8")
    (output / "target_classes.json").write_text(json.dumps({"categories": [{"id": int(t["object_id"]), "name": t["class_name"], "source_object_id": t["source_object_id"], "shape": t["shape"], "proxy_cad": False} for t in targets]}, indent=2) + "\n", encoding="utf-8")

    all_camera_rows, all_target_rows, split_manifest = [], [], {"split_by": "trajectory_id", "splits": {}}
    timestamp_base_ns, annotation_id, image_id = 1_780_000_000_000_000_000, 1, 1
    sample_step_ns = int(1e9 / float(config["platform"]["sample_rate_hz"]))
    for split, split_config in config["splits"].items():
        split_root = output / ("train_pbr" if split == "train" else split) / "000001"
        for directory in (split_root / "rgb", split_root / "mask", split_root / "mask_visib", split_root / "labels"):
            directory.mkdir(parents=True, exist_ok=True)
        scene_gt, scene_info, scene_camera = {}, {}, {}
        coco = {"info": {"dataset": config["dataset_id"], "split": split}, "images": [], "annotations": [],
                "categories": [{"id": int(t["object_id"]), "name": t["class_name"], "supercategory": "sandbox_target"} for t in targets]}
        trajectory_ids = list(split_config["trajectory_ids"])
        split_manifest["splits"][split] = {"trajectory_ids": trajectory_ids, "frame_count": 0}
        frame_number = 0
        for trajectory_id in trajectory_ids:
            for frame_in_trajectory in range(int(split_config["frames_per_trajectory"])):
                sequence = frame_number
                position = _trajectory(trajectory_id, frame_in_trajectory, int(split_config["frames_per_trajectory"]), config["platform"]["camera_height_m"])
                rotation_camera_map = _look_at(position, np.asarray(config["platform"]["look_at_map_m"], dtype=float))
                image = np.full((height, width, 3), tuple(config["render"]["background_rgb"]), dtype=np.uint8)
                image[:, :] = tuple(config["render"]["ground_rgb"])
                points_camera = (rotation_camera_map @ (landmarks - position).T).T
                valid = points_camera[:, 2] > 0.1
                homogeneous = points_camera @ matrix.T
                pixels = homogeneous[:, :2] / homogeneous[:, 2:3]
                visible = np.where(valid & (pixels[:, 0] >= 5) & (pixels[:, 0] < width - 5) & (pixels[:, 1] >= 5) & (pixels[:, 1] < height - 5))[0]
                for index in visible[np.argsort(points_camera[visible, 2])[::-1]]:
                    x, y = np.rint(pixels[index]).astype(int)
                    cv2.rectangle(image, (x - 4, y - 4), (x + 4, y + 4), tuple(int(v) for v in landmark_colors[index]), -1)
                    for py in range(7):
                        for px in range(7):
                            if landmark_patterns[index, py, px]:
                                image[y - 3 + py, x - 3 + px] = (20, 20, 20)
                instance_mask = np.zeros((height, width), dtype=np.uint16)
                for vertices, faces, color in static_scene:
                    _draw_mesh(image, instance_mask, vertices, faces, color, position, rotation_camera_map, matrix)
                frame_gt, frame_info, yolo_lines = [], [], []
                timestamp = timestamp_base_ns + (sum(len(s["trajectory_ids"]) * int(s["frames_per_trajectory"]) for s in list(config["splits"].values())[:list(config["splits"]).index(split)]) + sequence) * sample_step_ns
                for target in targets:
                    local_vertices, faces = target["mesh"]
                    world_vertices = (target["rotation_map_target"] @ local_vertices.T).T + target["position_m"]
                    _draw_mesh(image, instance_mask, world_vertices, faces, tuple(target["color_rgb"]), position, rotation_camera_map, matrix, int(target["object_id"]))
                    rotation_camera_target = rotation_camera_map @ target["rotation_map_target"]
                    translation_camera_target = rotation_camera_map @ (target["position_m"] - position)
                    bbox, pixel_count = _bbox(instance_mask, int(target["object_id"]))
                    frame_gt.append({"obj_id": int(target["object_id"]), "cam_R_m2c": rotation_camera_target.reshape(-1).round(12).tolist(), "cam_t_m2c": (translation_camera_target * 1000.0).round(8).tolist()})
                    frame_info.append({"bbox_obj": bbox, "bbox_visib": bbox, "px_count_all": pixel_count, "px_count_visib": pixel_count, "visib_fract": 1.0 if pixel_count else 0.0})
                    target_mask = np.where(instance_mask == int(target["object_id"]), 255, 0).astype(np.uint8)
                    cv2.imwrite(str(split_root / "mask" / f"{sequence:06d}_{int(target['object_id']) - 1:06d}.png"), target_mask)
                    cv2.imwrite(str(split_root / "mask_visib" / f"{sequence:06d}_{int(target['object_id']) - 1:06d}.png"), target_mask)
                    if pixel_count:
                        center_x, center_y = bbox[0] + bbox[2] / 2.0, bbox[1] + bbox[3] / 2.0
                        yolo_lines.append(f"{int(target['object_id']) - 1} {center_x / width:.8f} {center_y / height:.8f} {bbox[2] / width:.8f} {bbox[3] / height:.8f}")
                        coco["annotations"].append({"id": annotation_id, "image_id": image_id, "category_id": int(target["object_id"]), "bbox": bbox, "area": pixel_count, "iscrowd": 0,
                                                    "mask_file": f"mask/{sequence:06d}_{int(target['object_id']) - 1:06d}.png", "trajectory_id": trajectory_id})
                        annotation_id += 1
                    all_target_rows.append({"split": split, "trajectory_id": trajectory_id, "sequence": sequence, "sample_timestamp_ns": timestamp, "object_id": int(target["object_id"]),
                                            "target_x_m": target["position_m"][0], "target_y_m": target["position_m"][1], "target_z_m": target["position_m"][2],
                                            **{f"map_R_target_{r}{c}": target["rotation_map_target"][r, c] for r in range(3) for c in range(3)}})
                file_name = f"{sequence:06d}.png"
                cv2.imwrite(str(split_root / "rgb" / file_name), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                (split_root / "labels" / f"{sequence:06d}.txt").write_text("\n".join(yolo_lines) + ("\n" if yolo_lines else ""), encoding="utf-8")
                scene_gt[str(sequence)], scene_info[str(sequence)] = frame_gt, frame_info
                scene_camera[str(sequence)] = {"cam_K": matrix.reshape(-1).tolist(), "depth_scale": 1.0}
                coco["images"].append({"id": image_id, "file_name": f"rgb/{file_name}", "width": width, "height": height, "sequence": sequence, "trajectory_id": trajectory_id, "sample_timestamp_ns": timestamp})
                all_camera_rows.append({"split": split, "trajectory_id": trajectory_id, "sequence": sequence, "sample_timestamp_ns": timestamp,
                                        "camera_x_m": position[0], "camera_y_m": position[1], "camera_z_m": position[2],
                                        **{f"R_map_camera_{r}{c}": rotation_camera_map[r, c] for r in range(3) for c in range(3)}})
                image_id += 1
                frame_number += 1
        split_manifest["splits"][split]["frame_count"] = frame_number
        (split_root / "scene_gt.json").write_text(json.dumps(scene_gt, indent=2) + "\n", encoding="utf-8")
        (split_root / "scene_gt_info.json").write_text(json.dumps(scene_info, indent=2) + "\n", encoding="utf-8")
        (split_root / "scene_camera.json").write_text(json.dumps(scene_camera, indent=2) + "\n", encoding="utf-8")
        (split_root / "instances_coco.json").write_text(json.dumps(coco, indent=2) + "\n", encoding="utf-8")
        # Ultralytics/YOLO derives labels by replacing /images/ with /labels/.
        # Relative symlinks avoid duplicating the 3280x2464 RGB corpus.
        yolo_split = "train" if split == "train" else split
        yolo_images = output / "detector_yolo" / "images" / yolo_split
        yolo_labels = output / "detector_yolo" / "labels" / yolo_split
        yolo_images.mkdir(parents=True, exist_ok=True); yolo_labels.mkdir(parents=True, exist_ok=True)
        for image in coco["images"]:
            frame_name = Path(image["file_name"]).name
            source_image = split_root / image["file_name"]
            source_label = split_root / "labels" / f"{Path(frame_name).stem}.txt"
            for source, destination in ((source_image, yolo_images / frame_name), (source_label, yolo_labels / f"{Path(frame_name).stem}.txt")):
                if args.yolo_assets == "copy":
                    shutil.copy2(source, destination)
                elif not destination.exists():
                    destination.symlink_to(os.path.relpath(source, destination.parent))
    truth = output / "ground_truth"; truth.mkdir()
    def write_csv(path: Path, rows: list[dict]) -> None:
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    write_csv(truth / "camera_pose_map.csv", all_camera_rows)
    write_csv(truth / "target_pose_map.csv", all_target_rows)
    split_manifest["no_trajectory_overlap"] = len(set().union(*[set(item["trajectory_ids"]) for item in split_manifest["splits"].values()])) == sum(len(item["trajectory_ids"]) for item in split_manifest["splits"].values())
    (output / "split_manifest.json").write_text(json.dumps(split_manifest, indent=2) + "\n", encoding="utf-8")
    (output / "detector_yolo" / "dataset.yaml").write_text(yaml.safe_dump({
        "path": ".", "train": "images/train", "val": "images/val", "test": "images/test",
        "names": {int(target["object_id"]) - 1: target["class_name"] for target in targets},
    }, sort_keys=False), encoding="utf-8")
    (output / "dataset_manifest.json").write_text(json.dumps({"dataset_id": config["dataset_id"], "config": str(config_path), "target_catalog": str(catalog_path), "scene": str(scene_path), "scene_id": scene.get("scene_id"), "image_size": [width, height],
                                                              "calibration_status": camera["calibration_status"], "detector_contract": config["detector_contract"], "yolo_assets": args.yolo_assets,
                                                              "truth_not_for_inference": True, "limitations": ["Target geometry is designed parameterized proxy geometry, not surveyed CAD.", "Scene context is an authored metric proxy based on sandbox_scene.yaml, not a photorealistic reconstruction.", "Camera K is the confirmed Gazebo IMX219 CameraInfo; hardware calibration remains required before physical precision claims."]}, indent=2) + "\n", encoding="utf-8")
    print(f"generated trajectory-disjoint dataset at {output}")


if __name__ == "__main__":
    main()
