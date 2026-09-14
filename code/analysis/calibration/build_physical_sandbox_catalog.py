"""Expand the frozen physical sandbox ground-truth catalog.

The legacy YAML contains semantic rows and groups.  This tool expands them to
individual trees, zebra stripes, traffic lights and cones after applying the
measured table-scale conversion. The user accepted this registered layout as
the project's operational ground truth; image/layout provenance remains in
every record and is distinct from an independent tape/laser survey.
"""

import argparse
import json
import math
from pathlib import Path

import yaml


def _point_mm_to_m(point, sx, sy):
    return [float(point[0]) * 0.001 * sx, float(point[1]) * 0.001 * sy]


def _size_mm_to_m(size, sx, sy):
    return [float(size[0]) * 0.001 * sx, float(size[1]) * 0.001 * sy]


def _expand_marking(marking, sx, sy):
    """Return one record per road-marking primitive in physical-map XY."""
    source_id = marking["id"]
    common = {
        "category": "road_marking",
        "source_id": source_id,
        "color": marking.get("color", "road_white"),
        "xy_status": "operational_ground_truth_xy__layout_registered",
        "legacy_confidence": marking.get("confidence", "C"),
    }
    if marking["kind"] == "crosswalk":
        x, y = marking["center"]
        count = int(marking["count"])
        axis = marking["axis"]
        spacing = float(marking["spacing"])
        records = []
        for index in range(count):
            offset = (index - (count - 1) / 2.0) * spacing
            position = [x, y + offset] if axis == "x" else [x + offset, y]
            size = ([marking["bar_length"], marking["bar_width"]]
                    if axis == "x" else [marking["bar_width"], marking["bar_length"]])
            records.append({
                **common,
                "id": f"{source_id}-{index + 1:02d}",
                "kind": "crosswalk_stripe",
                "position_xy_m": _point_mm_to_m(position, sx, sy),
                "size_xy_m": _size_mm_to_m(size, sx, sy),
            })
        return records
    if marking["kind"] != "lane_dashes":
        return []
    start, end = marking["start"], marking["end"]
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    ux, uy = dx / length, dy / length
    dash, gap = float(marking["dash_length"]), float(marking["gap"])
    count = int((length + gap) // (dash + gap))
    records = []
    for index in range(count):
        distance = index * (dash + gap) + dash / 2.0
        if distance + dash / 2.0 > length:
            break
        position = [start[0] + ux * distance, start[1] + uy * distance]
        size = ([dash, marking["width"]] if abs(dx) >= abs(dy)
                else [marking["width"], dash])
        records.append({
            **common,
            "id": f"{source_id}-{index + 1:02d}",
            "kind": "lane_dash",
            "position_xy_m": _point_mm_to_m(position, sx, sy),
            "size_xy_m": _size_mm_to_m(size, sx, sy),
        })
    return records


def _expand_trees(row, sx, sy):
    start, end, count = row["start"], row["end"], int(row["count"])
    # The legacy rendering used 80 mm and 100/130 mm rows.  This classification
    # is only a reversible mapping to the user's measured short/tall classes.
    height_class = "short" if float(row["height"]) < 100.0 else "tall"
    height_m = 0.06 if height_class == "short" else 0.12
    result = []
    for index in range(count):
        ratio = index / max(count - 1, 1)
        point = [start[0] + ratio * (end[0] - start[0]),
                 start[1] + ratio * (end[1] - start[1])]
        result.append({
            "id": f"{row['id']}-{index + 1:02d}",
            "category": "tree",
            "position_xy_m": _point_mm_to_m(point, sx, sy),
            "height_m": height_m,
            "height_class": height_class,
            "height_status": "mapped_to_user_measured_tree_class",
            "xy_status": "operational_ground_truth_xy__layout_registered",
            "legacy_confidence": row.get("confidence", "C"),
        })
    return result


def _expand_traffic_lights(survey, sx, sy):
    """Expand the six photo-labelled junctions to high/low lamp instances."""
    offsets = survey["corner_offsets_mm"]
    heights = survey["height_classes_m"]
    records = []
    for junction in survey["junctions"]:
        center = junction["legacy_center_mm"]
        for corner, offset in offsets.items():
            anchor = [center[0] + offset[0], center[1] + offset[1]]
            for height_class in ("high", "low"):
                records.append({
                    "id": f"{junction['id']}-{corner.upper()}-{height_class.upper()}",
                    "category": "traffic_light",
                    "junction_id": junction["id"],
                    "corner": corner,
                    "corner_anchor_xy_m": _point_mm_to_m(anchor, sx, sy),
                    "height_m": float(heights[height_class]),
                    "height_class": height_class,
                    "height_status": "user_measured_and_photo_classified",
                    "xy_status": "operational_ground_truth_xy__photo_registered",
                    "photo": junction["photo"],
                })
    return records


def build_catalog(scene, reference, traffic_survey):
    """Build the complete small-object catalog from loaded YAML documents."""
    scale = reference["legacy_layout_xy_transform"]["scale"]
    sx, sy = float(scale["x"]), float(scale["y"])
    items = []

    for road in scene.get("roads", []):
        items.append({
            "id": road["id"], "category": "road",
            "center_xy_m": _point_mm_to_m(road["center"], sx, sy),
            "legacy_size_xy_m": _size_mm_to_m(road["size"], sx, sy),
            "width_status": "use_measured_road_reference_before_evaluation",
            "xy_status": "operational_ground_truth_xy__layout_registered",
            "legacy_confidence": road.get("confidence", "C"),
        })

    for marking in scene.get("road_markings", []):
        items.extend(_expand_marking(marking, sx, sy))
    for row in scene.get("tree_rows", []):
        items.extend(_expand_trees(row, sx, sy))

    items.extend(_expand_traffic_lights(traffic_survey, sx, sy))

    for group in scene.get("cone_groups", []):
        center = group["center"]
        for index in range(int(group["count"])):
            angle = 2.0 * math.pi * index / int(group["count"])
            point = [center[0] + float(group["radius"]) * math.cos(angle),
                     center[1] + float(group["radius"]) * math.sin(angle)]
            items.append({
                "id": f"{group['id']}-{index + 1:02d}", "category": "traffic_cone",
                "position_xy_m": _point_mm_to_m(point, sx, sy),
                "legacy_height_m": float(group["height"]) * 0.001,
                "height_status": "legacy_only_not_user_measured",
                "xy_status": "operational_ground_truth_xy__layout_registered",
                "legacy_confidence": group.get("confidence", "C"),
            })

    for item in scene.get("objects", []):
        record = {
            "id": item["id"], "category": item["category"],
            "subtype": item.get("subtype"),
            "position_xy_m": _point_mm_to_m(item["position"][:2], sx, sy),
            "legacy_footprint_xy_m": _size_mm_to_m(item["size"][:2], sx, sy),
            "legacy_height_m": float(item["size"][2]) * 0.001,
            "xy_status": "operational_ground_truth_xy__layout_registered",
            "legacy_confidence": item.get("confidence", "C"),
        }
        if item["category"] == "tank":
            record.update({"height_m": 0.16, "height_status": "user_measured_tank_height"})
        else:
            record["height_status"] = "needs_instance_to_measured_description_match"
        items.append(record)

    category_counts = {}
    for item in items:
        category_counts[item["category"]] = category_counts.get(item["category"], 0) + 1
    return {
        "schema_version": 1,
        "coordinate_frame": reference["coordinate_frame"],
        "status": "operational_ground_truth_user_accepted",
        "truth_basis": "user-accepted physical dimensions, photo classification, and registered layout",
        "source_layout": "simulation/sandbox_scene.yaml",
        "photo_evidence": "user_sandbox_overview_2026-09-07",
        "category_counts": category_counts,
        "items": items,
    }


def _map_xy(item):
    """Return the one map XY representation used by a catalog item."""
    for key in ("position_xy_m", "center_xy_m", "corner_anchor_xy_m"):
        value = item.get(key)
        if isinstance(value, list) and len(value) == 2:
            return [float(value[0]), float(value[1])]
    raise ValueError(f"catalog item {item.get('id', '<unknown>')} has no map XY")


def build_model_reference_registry(catalog):
    """Create the persistent instance registry used to collect visual references.

    This registry intentionally contains geometry and capture work items only.
    A ground-truth coordinate is not an image descriptor and must never be
    presented as an already-trained visual reference.
    """
    roles = {
        "road": "context_landmark",
        "road_marking": "map_landmark",
        "tree": "context_landmark",
        "traffic_light": "candidate_target",
        "traffic_cone": "candidate_target",
        "industrial_yard": "context_landmark",
        "tank": "candidate_target",
        "building": "candidate_target",
    }
    instances = []
    for item in catalog["items"]:
        xy = _map_xy(item)
        height = item.get("height_m")
        instances.append({
            "instance_id": item["id"],
            "category": item["category"],
            "role": roles[item["category"]],
            "map_position_m": [xy[0], xy[1], 0.0],
            "height_m": float(height) if height is not None else None,
            "geometry_source": item["xy_status"],
            "reference_capture": {
                "state": "pending",
                "required_views": ["nadir", "oblique_north", "oblique_south"],
                "asset_paths": [],
            },
        })
    return {
        "schema_version": 1,
        "registry_id": "physical_sandbox_model_reference_registry_v01",
        "coordinate_frame": catalog["coordinate_frame"]["name"],
        "unit": "m",
        "ground_truth_catalog": "physical_sandbox_ground_truth_v01.json",
        "ground_truth_status": catalog["status"],
        "weight_status": "existing_generic_weights_reusable_no_sandbox_finetune_recorded",
        "reference_status": "geometry_registered_reference_images_pending",
        "instances": instances,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--traffic-survey", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--registry-output", type=Path,
                        help="Optional model reference registry derived from the catalog")
    args = parser.parse_args(argv)
    scene = yaml.safe_load(args.scene.read_text(encoding="utf-8"))
    reference = yaml.safe_load(args.reference.read_text(encoding="utf-8"))
    traffic_survey = yaml.safe_load(args.traffic_survey.read_text(encoding="utf-8"))
    catalog = build_catalog(scene, reference, traffic_survey)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(catalog, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if args.registry_output:
        args.registry_output.parent.mkdir(parents=True, exist_ok=True)
        args.registry_output.write_text(
            json.dumps(build_model_reference_registry(catalog), indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")


if __name__ == "__main__":
    main()
