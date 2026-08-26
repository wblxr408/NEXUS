#!/usr/bin/env python3
"""Generate a lightweight OBJ, Gazebo SDF and Web JSON from sandbox_scene.yaml."""
from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path

import yaml


COLORS = {
    "road": (0.08, 0.09, 0.10), "silver": (0.55, 0.58, 0.60),
    "orange": (0.95, 0.35, 0.05), "purple": (0.45, 0.18, 0.55),
    "blue_red": (0.08, 0.35, 0.75), "green": (0.10, 0.55, 0.22),
    "yellow": (0.90, 0.72, 0.08), "white": (0.82, 0.84, 0.84),
    "white_yellow": (0.95, 0.84, 0.50), "beige": (0.72, 0.62, 0.42),
    "glass": (0.20, 0.55, 0.72), "blue_gray": (0.25, 0.35, 0.45),
    "grass": (0.12, 0.45, 0.12), "tree": (0.08, 0.42, 0.12),
    "concrete": (0.46, 0.50, 0.49), "road_white": (0.93, 0.93, 0.88),
    "road_yellow": (0.92, 0.68, 0.12), "traffic_black": (0.03, 0.03, 0.03),
}


class Obj:
    def __init__(self):
        self.v = []
        self.faces = []
        self.mats = []

    def vertex(self, p):
        self.v.append(p)
        return len(self.v)

    def box(self, center, size, material):
        x, y, z = center
        sx, sy, sz = [a / 2 for a in size]
        ids = [self.vertex((x + dx * sx, y + dy * sy, z + dz * sz))
               for dx, dy, dz in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
        for f in [(0,1,2,3),(4,7,6,5),(0,4,5,1),(1,5,6,2),(2,6,7,3),(4,0,3,7)]:
            self.faces.append(([ids[i] for i in f], material))

    def cylinder(self, center, size, material, n=20):
        x, y, z = center
        r = min(size[0], size[1]) / 2
        h = size[2] / 2
        bot = [self.vertex((x + r * math.cos(2*math.pi*i/n), y + r * math.sin(2*math.pi*i/n), z-h)) for i in range(n)]
        top = [self.vertex((x + r * math.cos(2*math.pi*i/n), y + r * math.sin(2*math.pi*i/n), z+h)) for i in range(n)]
        for i in range(n):
            j = (i + 1) % n
            self.faces.append(([bot[i], bot[j], top[j], top[i]], material))
        self.faces.append((bot, material)); self.faces.append((list(reversed(top)), material))

    def write(self, path):
        used = []
        for _, m in self.faces:
            if m not in used: used.append(m)
        with path.open("w", encoding="utf-8") as f:
            f.write("# NEXUS sandbox generated OBJ; units: m\n")
            f.write(f"mtllib {path.with_suffix('.mtl').name}\n")
            for x, y, z in self.v: f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
            for m in used: f.write(f"usemtl {m}\n")
            current = None
            for face, m in self.faces:
                if current != m: f.write(f"usemtl {m}\n"); current = m
                f.write("f " + " ".join(map(str, face)) + "\n")
        mtl = path.with_suffix(".mtl")
        with mtl.open("w", encoding="utf-8") as f:
            for name, rgb in COLORS.items():
                f.write(f"newmtl {name}\nKd {' '.join(map(str, rgb))}\n\n")


def model_xml(obj):
    x, y, z = obj["position"]
    sx, sy, sz = obj["size"]
    color = obj.get("color", "white")
    geom = f"<cylinder><radius>{sx/2}</radius><length>{sz}</length></cylinder>" if obj.get("shape") == "cylinder" else f"<box><size>{sx} {sy} {sz}</size></box>"
    return f'''<model name="{obj['id']}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose><link name="link"><collision name="collision"><geometry>{geom}</geometry></collision><visual name="visual"><geometry>{geom}</geometry><material><ambient>{' '.join(map(str, COLORS.get(color, COLORS['white'])))} 1</ambient><diffuse>{' '.join(map(str, COLORS.get(color, COLORS['white'])))} 1</diffuse></material></visual></link></model>'''


def imx219_uav_xml(profile):
    """Return the stationary Gazebo camera carrier used for visual integration."""
    camera = profile["camera"]
    uav = profile["uav"]
    x, y, z = uav["spawn_pose_m"]
    hfov = math.radians(camera["horizontal_fov_deg"])
    return f'''<model name="{uav['model_name']}">
  <static>true</static>
  <pose>{x} {y} {z} 0 0 0</pose>
  <link name="base_link">
    <visual name="body"><geometry><box><size>0.34 0.34 0.08</size></box></geometry><material><ambient>0.12 0.12 0.14 1</ambient><diffuse>0.12 0.12 0.14 1</diffuse></material></visual>
    <visual name="rotor_x"><pose>0.25 0 0 0 0 0</pose><geometry><cylinder><radius>0.13</radius><length>0.012</length></cylinder></geometry><material><ambient>0.06 0.06 0.07 1</ambient><diffuse>0.06 0.06 0.07 1</diffuse></material></visual>
    <visual name="rotor_y"><pose>0 0.25 0 0 0 0</pose><geometry><cylinder><radius>0.13</radius><length>0.012</length></cylinder></geometry><material><ambient>0.06 0.06 0.07 1</ambient><diffuse>0.06 0.06 0.07 1</diffuse></material></visual>
    <visual name="rotor_neg_x"><pose>-0.25 0 0 0 0 0</pose><geometry><cylinder><radius>0.13</radius><length>0.012</length></cylinder></geometry><material><ambient>0.06 0.06 0.07 1</ambient><diffuse>0.06 0.06 0.07 1</diffuse></material></visual>
    <visual name="rotor_neg_y"><pose>0 -0.25 0 0 0 0</pose><geometry><cylinder><radius>0.13</radius><length>0.012</length></cylinder></geometry><material><ambient>0.06 0.06 0.07 1</ambient><diffuse>0.06 0.06 0.07 1</diffuse></material></visual>
    <sensor name="imx219" type="camera">
      <pose>0 0 -0.06 0 1.57079632679 0</pose>
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>{camera['update_rate_hz']}</update_rate>
      <camera name="imx219">
        <horizontal_fov>{hfov:.12f}</horizontal_fov>
        <image><width>{camera['image_width_px']}</width><height>{camera['image_height_px']}</height><format>{camera['pixel_format']}</format></image>
        <clip><near>{camera['clip_near_m']}</near><far>{camera['clip_far_m']}</far></clip>
      </camera>
      <plugin name="nexus_imx219_camera" filename="libgazebo_ros_camera.so">
        <ros><namespace>/nexus/camera</namespace></ros>
        <camera_name>imx219</camera_name>
        <frame_name>{camera['frame_id']}</frame_name>
      </plugin>
    </sensor>
  </link>
  <plugin name="nexus_uav_ground_truth" filename="libgazebo_ros_p3d.so">
    <ros><namespace>/nexus/gazebo/uav</namespace></ros>
    <body_name>base_link</body_name>
    <update_rate>30</update_rate>
  </plugin>
</model>'''


def gazebo_world_xml(models, profile):
    """Keep Gazebo and Web views on the same generated sandbox geometry."""
    return f'''<?xml version="1.0"?>
<sdf version="1.7">
  <world name="nexus_sandbox_imx219">
    <gravity>0 0 -9.81</gravity>
    <scene><ambient>0.7 0.7 0.7 1</ambient><background>0.7 0.8 0.9 1</background></scene>
    <physics name="ode" type="ode"><real_time_update_rate>1000</real_time_update_rate><max_step_size>0.001</max_step_size></physics>
    {''.join(models)}
    {imx219_uav_xml(profile)}
  </world>
</sdf>
'''


def boxes_for_marking(marking, deck):
    """Expand semantic road markings to thin, z-up boxes."""
    h = marking.get("height", 0.004)
    z = deck + marking.get("road_height", 0.002) + h / 2 + 0.0005
    color = marking.get("color", "road_white")
    base = {
        "category": "road_marking", "shape": "box", "color": color,
        "confidence": marking.get("confidence", "C"),
        "source_id": marking["id"],
    }
    kind = marking["kind"]
    if kind == "crosswalk":
        x, y = marking["center"]
        count = marking["count"]
        bar_length = marking["bar_length"]
        bar_width = marking["bar_width"]
        spacing = marking["spacing"]
        axis = marking["axis"]
        for index in range(count):
            offset = (index - (count - 1) / 2) * spacing
            # `axis` denotes the direction of each zebra stripe. The bars are
            # laid out across the crossing by offsetting the perpendicular axis.
            size = [bar_length, bar_width, h] if axis == "x" else [bar_width, bar_length, h]
            position = [x, y + offset, z] if axis == "x" else [x + offset, y, z]
            yield {**base, "id": f"{marking['id']}-{index + 1:02d}", "position": position, "size": size}
    elif kind in {"lane_dashes", "edge_line"}:
        x1, y1 = marking["start"]
        x2, y2 = marking["end"]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        ux, uy = dx / length, dy / length
        width = marking["width"]
        if kind == "edge_line":
            yield {**base, "id": marking["id"], "position": [(x1 + x2) / 2, (y1 + y2) / 2, z], "size": [length if abs(dx) >= abs(dy) else width, width if abs(dx) >= abs(dy) else length, h]}
            return
        dash = marking["dash_length"]
        gap = marking["gap"]
        count = int((length + gap) // (dash + gap))
        for index in range(count):
            distance = index * (dash + gap) + dash / 2
            if distance + dash / 2 > length:
                break
            size = [dash, width, h] if abs(dx) >= abs(dy) else [width, dash, h]
            yield {**base, "id": f"{marking['id']}-{index + 1:02d}", "position": [x1 + ux * distance, y1 + uy * distance, z], "size": size}
    else:
        raise ValueError(f"unsupported road marking kind: {kind}")


def overlaps_xy(a, b):
    ax, ay = a["position"][:2]; asx, asy = a["size"][:2]
    bx, by = b["center"]; bsx, bsy = b["size"]
    # Exact edge contact is valid (island/sidewalk/road share a boundary).
    eps = 1e-9
    return abs(ax - bx) * 2 < asx + bsx - eps and abs(ay - by) * 2 < asy + bsy - eps


def validate_layout(scene):
    """Keep scenery out of driveable roadway; markings are intentionally excluded."""
    roads = [road for road in scene.get("roads", []) if road.get("render", True)]
    blocked = {"building", "tank", "industrial_yard", "sidewalk"}
    collisions = [
        f"{item['id']} overlaps {road['id']}"
        for item in scene.get("objects", []) if item.get("category") in blocked
        for road in roads if overlaps_xy(item, road)
    ]
    if collisions:
        raise ValueError("buildings/sidewalks on road: " + "; ".join(collisions))


def validate_crosswalks(markings):
    """Reject overlapping zebra bars before they reach OBJ/SDF/Web output."""
    bars = [item for item in markings if item.get("source_id", "").startswith("CW-")]
    for index, left in enumerate(bars):
        lx, ly, _ = left["position"]; lsx, lsy, _ = left["size"]
        for right in bars[index + 1:]:
            if left["source_id"] == right["source_id"]:
                continue
            rx, ry, _ = right["position"]; rsx, rsy, _ = right["size"]
            if abs(lx - rx) * 2 < lsx + rsx and abs(ly - ry) * 2 < lsy + rsy:
                raise ValueError(f"crosswalk bars overlap: {left['id']} / {right['id']}")


def scene_in_meters(scene):
    """Convert the authored CAD table (mm) to renderer/Gazebo coordinates (m)."""
    if scene.get("unit", "m") != "mm":
        return scene
    result = copy.deepcopy(scene)
    def s(value):
        return value * 0.001 if isinstance(value, (int, float)) else value
    result["dimensions"] = {key: s(value) for key, value in result["dimensions"].items()}
    result["central_core"]["min_xy"] = [s(v) for v in result["central_core"]["min_xy"]]
    result["central_core"]["size_xy"] = [s(v) for v in result["central_core"]["size_xy"]]
    result["central_core"]["center_xy"] = [s(v) for v in result["central_core"]["center_xy"]]
    result["outer_boundary"]["polygon_xy"] = [[s(v) for v in point] for point in result["outer_boundary"]["polygon_xy"]]
    for road in result.get("roads", []):
        road["center"] = [s(v) for v in road["center"]]
        road["size"] = [s(v) for v in road["size"]]
        for key in ("height",):
            if key in road: road[key] = s(road[key])
    for marking in result.get("road_markings", []):
        for key in ("center", "start", "end"):
            if key in marking: marking[key] = [s(v) for v in marking[key]]
        for key in ("width", "dash_length", "gap", "bar_length", "bar_width", "spacing", "height", "road_height"):
            if key in marking: marking[key] = s(marking[key])
    for item in result.get("objects", []):
        item["position"] = [s(v) for v in item["position"]]
        item["size"] = [s(v) for v in item["size"]]
    for row in result.get("tree_rows", []):
        row["start"] = [s(v) for v in row["start"]]
        row["end"] = [s(v) for v in row["end"]]
        row["height"] = s(row["height"])
    for ring in result.get("tree_rings", []):
        ring["center"] = [s(v) for v in ring["center"]]
        for key in ("radius_x", "radius_y", "height"):
            if key in ring: ring[key] = s(ring[key])
    for group in result.get("traffic_light_groups", []):
        group["center"] = [s(v) for v in group["center"]]
        if "positions" in group:
            group["positions"] = [[s(v) for v in point] for point in group["positions"]]
    for group in result.get("cone_groups", []):
        group["center"] = [s(v) for v in group["center"]]
        for key in ("radius", "height", "diameter"):
            if key in group: group[key] = s(group[key])
    result["source_unit"] = "mm"
    result["unit"] = "m"
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="sandbox_scene.yaml")
    ap.add_argument("--out", default=".")
    ap.add_argument("--camera-profile", default=Path(__file__).with_name("imx219_gazebo_camera.yaml"))
    args = ap.parse_args()
    authored_scene = yaml.safe_load(Path(args.scene).read_text(encoding="utf-8"))
    camera_profile = yaml.safe_load(Path(args.camera_profile).read_text(encoding="utf-8"))
    scene = scene_in_meters(authored_scene)
    validate_layout(scene)
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    obj = Obj()
    width = scene["dimensions"]["width_x"]; length = scene["dimensions"]["length_y"]; deck = scene["dimensions"]["deck_height"]
    obj.box((width/2, length/2, deck/2), (width, length, deck), "grass")
    objects = [{"id": "sandbox_deck", "category": "sandbox", "shape": "box", "position": [width/2, length/2, deck/2], "size": [width, length, deck], "color": "grass", "confidence": scene.get("dimensions_confidence", "A")}]
    for road in scene.get("roads", []):
        if road.get("render", True) is False:
            continue
        x, y = road["center"]; sx, sy = road["size"]; h = road.get("height", 0.002)
        item = {"id": road["id"], "category": "road", "shape": "box", "position": [x, y, deck+h/2], "size": [sx, sy, h], "color": "road", "confidence": road.get("confidence")}
        objects.append(item); obj.box(item["position"], item["size"], "road")
    marking_objects = []
    for marking in scene.get("road_markings", []):
        for item in boxes_for_marking(marking, deck):
            marking_objects.append(item)
            objects.append(item); obj.box(item["position"], item["size"], item["color"])
    validate_crosswalks(marking_objects)
    for item in scene.get("objects", []):
        item = dict(item)
        if scene.get("object_z_mode", "center") == "base":
            item["position"] = [item["position"][0], item["position"][1], deck + item["position"][2] + item["size"][2] / 2]
        objects.append(item)
        (obj.cylinder if item.get("shape") == "cylinder" else obj.box)(item["position"], item["size"], item.get("color", "white"))
    for row in scene.get("tree_rows", []):
        x1, y1 = row["start"]; x2, y2 = row["end"]; count = row["count"]
        for i in range(count):
            t = i / max(count - 1, 1); x = x1 + (x2-x1)*t; y = y1 + (y2-y1)*t
            item = {"id": f"{row['id']}-{i+1:02d}", "category": "tree", "shape": "cylinder", "position": [x, y, deck + row["height"]/2], "size": [0.035, 0.035, row["height"]], "color": "tree", "confidence": row.get("confidence")}
            objects.append(item); obj.cylinder(item["position"], item["size"], "tree", n=8)
    for ring in scene.get("tree_rings", []):
        x, y = ring["center"]; count = ring.get("count", 16)
        rx, ry = ring.get("radius_x", 0.4), ring.get("radius_y", 0.4)
        for index in range(count):
            angle = 2 * math.pi * index / count
            height = ring.get("height", 0.10)
            item = {"id": f"{ring['id']}-{index + 1:02d}", "category": "tree", "shape": "cylinder", "position": [x + rx * math.cos(angle), y + ry * math.sin(angle), deck + height / 2], "size": [0.035, 0.035, height], "color": "tree", "confidence": ring.get("confidence", "C")}
            objects.append(item); obj.cylinder(item["position"], item["size"], "tree", n=8)
    for group in scene.get("traffic_light_groups", []):
        positions = group.get("positions")
        if positions is None:
            x, y = group["center"]
            positions = [[x - 0.025, y - 0.025], [x + 0.025, y - 0.025], [x - 0.025, y + 0.025], [x + 0.025, y + 0.025]]
        for index, point in enumerate(positions, 1):
            item = {"id": f"{group['id']}-{index}", "category": "traffic_light", "shape": "box", "position": [point[0], point[1], deck + 0.09], "size": [0.03, 0.03, 0.18], "color": "traffic_black", "confidence": group.get("confidence", "C")}
            objects.append(item); obj.box(item["position"], item["size"], "traffic_black")
    for group in scene.get("cone_groups", []):
        x, y = group["center"]; count = group.get("count", 8); radius = group.get("radius", 0.30)
        for index in range(count):
            angle = 2 * math.pi * index / count
            item = {"id": f"{group['id']}-{index + 1:02d}", "category": "traffic_cone", "shape": "cylinder", "position": [x + radius * math.cos(angle), y + radius * math.sin(angle), deck + group.get("height", 0.05) / 2], "size": [group.get("diameter", 0.025), group.get("diameter", 0.025), group.get("height", 0.05)], "color": "orange", "confidence": group.get("confidence", "C")}
            objects.append(item); obj.cylinder(item["position"], item["size"], "orange", n=8)
    obj.write(out / "sandbox.obj")
    models = [model_xml(x) for x in objects]
    sdf = '<?xml version="1.0"?>\n<sdf version="1.7"><world name="sandbox">' + ''.join(models) + '</world></sdf>\n'
    (out / "sandbox.sdf").write_text(sdf, encoding="utf-8")
    (out / "nexus_sandbox_imx219.world").write_text(
        gazebo_world_xml(models, camera_profile), encoding="utf-8")
    runtime = {"scene_id": scene["scene_id"], "scene_mode": scene["scene_mode"], "frame": scene["coordinate_frame"], "unit": scene["unit"], "source_unit": scene.get("source_unit", scene["unit"]), "dimensions": scene["dimensions"], "central_core": scene["central_core"], "outer_boundary": scene["outer_boundary"], "objects": objects, "uwb_anchors": scene.get("uwb_anchors", []), "model": "sandbox.obj"}
    (out / "sandbox_scene.json").write_text(json.dumps(runtime, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"generated {len(objects)} objects -> {out}")


if __name__ == "__main__":
    main()
