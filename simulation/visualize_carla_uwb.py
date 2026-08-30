#!/usr/bin/env python3
"""Save an overhead CARLA view with UWB anchors and algorithm trajectories."""

from __future__ import annotations

import argparse
import json
import math
import queue
import struct
import sys
import zlib
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "code" / "analysis"))
sys.path.insert(0, str(REPO_ROOT / "simulation"))

from algorithms.router import register_default_algorithms
from run_carla_uwb_reproduction import (
    ALGORITHMS,
    _anchors,
    _collect_carla_trajectory,
    _spawn_uav,
    _run_algorithms,
)


COLORS = {
    "uwb.matlab.trilateration": (255, 180, 0),
    "uwb.matlab.multilateration": (255, 80, 80),
    "uwb.matlab.taylor": (80, 255, 80),
    "uwb.matlab.ekf": (220, 100, 255),
    "uwb.matlab.ukf": (80, 220, 255),
    "uwb.awesome_uwb": (255, 255, 255),
}


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--warmup-frames", type=int, default=20)
    parser.add_argument("--fixed-delta-seconds", type=float, default=0.1)
    parser.add_argument("--sigma-m", type=float, default=0.05)
    parser.add_argument("--random-seed", type=int, default=42)
    parser.add_argument("--anchor-half-width-m", type=float, default=15.0)
    parser.add_argument("--anchor-half-height-m", type=float, default=12.0)
    parser.add_argument("--anchor-heights-m", default="0.5,4.5,0.5,4.5")
    parser.add_argument("--uav-altitude-m", type=float, default=3.0)
    parser.add_argument("--camera-height-m", type=float, default=48.0)
    parser.add_argument("--camera-fov-degrees", type=float, default=90.0)
    parser.add_argument(
        "--draw-debug",
        action="store_true",
        help="also leave the colored traces in the live CARLA world",
    )
    parser.add_argument("--image-width", type=int, default=1600)
    parser.add_argument("--image-height", type=int, default=900)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPO_ROOT / "outputs" / "carla_uwb_reproduction",
    )
    return parser.parse_args(argv)


def _world_location(carla, origin, xyz):
    return carla.Location(
        x=origin.x + float(xyz[0]), y=origin.y + float(xyz[1]), z=origin.z + float(xyz[2]),
    )


def _draw_scene(world, carla, origin, anchors, positions, estimate_rows):
    debug = world.debug
    for anchor in anchors:
        location = carla.Location(
            x=origin.x + anchor.position_m[0],
            y=origin.y + anchor.position_m[1],
            z=origin.z + anchor.position_m[2],
        )
        debug.draw_point(location, size=0.8, color=carla.Color(255, 220, 0), life_time=120.0)
        debug.draw_string(
            location + carla.Location(z=1.0), anchor.id,
            draw_shadow=True, color=carla.Color(255, 220, 0), life_time=120.0,
        )

    truth_points = [
        _world_location(carla, origin, point)
        for point in positions
    ]
    for previous, current in zip(truth_points, truth_points[1:]):
        debug.draw_line(previous, current, thickness=0.28,
                        color=carla.Color(0, 255, 255), life_time=120.0)
    for point in truth_points[::10]:
        debug.draw_point(point, size=0.45, color=carla.Color(0, 255, 255), life_time=120.0)

    by_algorithm = {algorithm: [] for algorithm in ALGORITHMS}
    for row in estimate_rows:
        if int(row["valid"]):
            by_algorithm[row["algorithm"]].append(
                _world_location(carla, origin, (
                    row["estimate_x_m"], row["estimate_y_m"], row["estimate_z_m"]
                ))
            )
    for algorithm, points in by_algorithm.items():
        if len(points) < 2:
            continue
        r, g, b = COLORS[algorithm]
        for previous, current in zip(points, points[1:]):
            debug.draw_line(previous, current, thickness=0.15,
                            color=carla.Color(r, g, b), life_time=120.0)

    uav = truth_points[-1]
    debug.draw_string(uav + carla.Location(z=1.3), "UAV / base_link", draw_shadow=True,
                      color=carla.Color(255, 255, 255), life_time=120.0)
    arm = 2.0
    for offset in (carla.Location(x=-arm), carla.Location(x=arm),
                   carla.Location(y=-arm), carla.Location(y=arm)):
        debug.draw_line(uav, uav + offset, thickness=0.16,
                        color=carla.Color(255, 255, 255), life_time=120.0)
    legend_origin = origin + carla.Location(x=-30.0, y=-24.0, z=3.0)
    debug.draw_string(legend_origin, "cyan: CARLA ground truth", draw_shadow=True,
                      color=carla.Color(0, 255, 255), life_time=120.0)
    for index, algorithm in enumerate(ALGORITHMS):
        r, g, b = COLORS[algorithm]
        label = algorithm.removeprefix("uwb.matlab.")
        if algorithm == "uwb.awesome_uwb":
            label = "awesome_uwb range graph"
        debug.draw_string(
            legend_origin + carla.Location(z=-0.65 * (index + 1)), label,
            draw_shadow=True, color=carla.Color(r, g, b), life_time=120.0,
        )


def _capture_overhead(world, carla, origin, args, output_path):
    blueprints = world.get_blueprint_library()
    camera_bp = blueprints.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(args.image_width))
    camera_bp.set_attribute("image_size_y", str(args.image_height))
    camera_bp.set_attribute("fov", str(args.camera_fov_degrees))
    camera = world.spawn_actor(
        camera_bp,
        carla.Transform(
            carla.Location(x=origin.x, y=origin.y, z=origin.z + args.camera_height_m),
            carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0),
        ),
    )
    images = queue.Queue()
    camera.listen(images.put)
    try:
        world.tick()
        image = images.get(timeout=10.0)
        image.save_to_disk(str(output_path))
        return image.frame, int(image.width), int(image.height), bytes(image.raw_data)
    finally:
        camera.stop()
        camera.destroy()


def _annotate_image_cv2(image_path, args, anchors, positions, estimate_rows, metrics):
    """Overlay world-coordinate traces because CARLA debug primitives are not
    included in an offscreen RGB sensor image.
    """
    try:
        import cv2
    except ImportError:
        print("CARLA image captured; install OpenCV in the active environment to annotate it")
        return
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot read captured CARLA image: {image_path}")
    height, width = image.shape[:2]
    world_width = 2.0 * args.camera_height_m
    world_height = world_width * height / width

    def pixel(x, y):
        return int(round(width / 2.0 + y / world_width * width)), int(
            round(height / 2.0 + x / world_height * height)
        )

    def visible(point):
        return (
            -world_height / 2.0 <= point[0] <= world_height / 2.0
            and -world_width / 2.0 <= point[1] <= world_width / 2.0
        )

    def polyline(points, color, thickness):
        clipped = [pixel(float(point[0]), float(point[1])) for point in points if visible(point)]
        for previous, current in zip(clipped, clipped[1:]):
            cv2.line(image, previous, current, color, thickness, cv2.LINE_AA)

    panel = image.copy()
    cv2.rectangle(panel, (24, 24), (470, 270), (18, 24, 28), -1)
    cv2.addWeighted(panel, 0.82, image, 0.18, 0.0, image)
    cv2.putText(image, "CARLA / UAV UWB OVERHEAD", (44, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.82, (245, 245, 245), 2, cv2.LINE_AA)
    cv2.putText(image, "platform: UAV / base_link", (44, 88),
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(image, "cyan = CARLA ground truth", (44, 116),
                cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255, 255, 0), 1, cv2.LINE_AA)

    truth = positions[:, :2]
    polyline(truth, (255, 255, 0), 5)
    for anchor in anchors:
        point = pixel(anchor.position_m[0], anchor.position_m[1])
        cv2.circle(image, point, 11, (0, 220, 255), -1, cv2.LINE_AA)
        cv2.circle(image, point, 15, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(image, anchor.id, (point[0] + 12, point[1] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2, cv2.LINE_AA)

    by_algorithm = {algorithm: [] for algorithm in ALGORITHMS}
    for row in estimate_rows:
        if int(row["valid"]):
            by_algorithm[row["algorithm"]].append((row["estimate_x_m"], row["estimate_y_m"]))
    for algorithm, trace in by_algorithm.items():
        rgb = COLORS[algorithm]
        polyline(trace, (rgb[2], rgb[1], rgb[0]), 2)

    uav_point = pixel(float(truth[-1, 0]), float(truth[-1, 1]))
    cv2.drawMarker(image, uav_point, (255, 255, 255), cv2.MARKER_CROSS, 36, 4, cv2.LINE_AA)
    cv2.circle(image, uav_point, 10, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(image, "UAV", (uav_point[0] + 14, uav_point[1] + 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.66, (255, 255, 255), 2, cv2.LINE_AA)

    for index, algorithm in enumerate(ALGORITHMS):
        rgb = COLORS[algorithm]
        color = (rgb[2], rgb[1], rgb[0])
        label = algorithm.removeprefix("uwb.matlab.")
        if algorithm == "uwb.awesome_uwb":
            label = "awesome_uwb graph"
        cv2.line(image, (44, 144 + index * 20), (72, 144 + index * 20), color, 3, cv2.LINE_AA)
        cv2.putText(image, f"{label}: RMSE {metrics[algorithm]['rmse_m']:.3f} m",
                    (82, 149 + index * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                    color, 1, cv2.LINE_AA)
    cv2.imwrite(str(image_path), image)


_FONT_5X7 = {
    " ": (0, 0, 0, 0, 0, 0, 0),
    "A": (14, 17, 17, 31, 17, 17, 17),
    "B": (30, 17, 17, 30, 17, 17, 30),
    "C": (15, 16, 16, 16, 16, 16, 15),
    "D": (30, 17, 17, 17, 17, 17, 30),
    "E": (31, 16, 16, 30, 16, 16, 31),
    "F": (31, 16, 16, 30, 16, 16, 16),
    "G": (15, 16, 16, 23, 17, 17, 15),
    "H": (17, 17, 17, 31, 17, 17, 17),
    "I": (31, 4, 4, 4, 4, 4, 31),
    "J": (7, 2, 2, 2, 18, 18, 12),
    "K": (17, 18, 20, 24, 20, 18, 17),
    "L": (16, 16, 16, 16, 16, 16, 31),
    "M": (17, 27, 21, 21, 17, 17, 17),
    "N": (17, 25, 21, 19, 17, 17, 17),
    "O": (14, 17, 17, 17, 17, 17, 14),
    "P": (30, 17, 17, 30, 16, 16, 16),
    "Q": (14, 17, 17, 17, 21, 18, 13),
    "R": (30, 17, 17, 30, 20, 18, 17),
    "S": (15, 16, 16, 14, 1, 1, 30),
    "T": (31, 4, 4, 4, 4, 4, 4),
    "U": (17, 17, 17, 17, 17, 17, 14),
    "V": (17, 17, 17, 17, 17, 10, 4),
    "W": (17, 17, 17, 21, 21, 27, 17),
    "X": (17, 17, 10, 4, 10, 17, 17),
    "Y": (17, 17, 10, 4, 4, 4, 4),
    "Z": (31, 1, 2, 4, 8, 16, 31),
    "0": (14, 17, 19, 21, 25, 17, 14),
    "1": (4, 12, 4, 4, 4, 4, 14),
    "2": (14, 17, 1, 2, 4, 8, 31),
    "3": (30, 1, 1, 14, 1, 1, 30),
    "4": (2, 6, 10, 18, 31, 2, 2),
    "5": (31, 16, 16, 30, 1, 1, 30),
    "6": (14, 16, 16, 30, 17, 17, 14),
    "7": (31, 1, 2, 4, 8, 8, 8),
    "8": (14, 17, 17, 14, 17, 17, 14),
    "9": (14, 17, 17, 15, 1, 1, 14),
    "/": (1, 2, 2, 4, 8, 8, 16),
    "-": (0, 0, 0, 31, 0, 0, 0),
    "_": (0, 0, 0, 0, 0, 0, 31),
    ".": (0, 0, 0, 0, 0, 12, 12),
    ":": (0, 12, 12, 0, 12, 12, 0),
    "=": (0, 31, 0, 31, 0, 0, 0),
    "|": (4, 4, 4, 4, 4, 4, 4),
    "(": (2, 4, 8, 8, 8, 4, 2),
    ")": (8, 4, 2, 2, 2, 4, 8),
    "+": (0, 4, 4, 31, 4, 4, 0),
    "?": (14, 17, 1, 2, 4, 0, 4),
}


def _png_chunk(kind, payload):
    data = kind + payload
    return (
        struct.pack(">I", len(payload))
        + data
        + struct.pack(">I", zlib.crc32(data) & 0xFFFFFFFF)
    )


def _write_rgba_png(path, width, height, rgba):
    """Write an RGBA PNG using only the Python standard library."""
    stride = width * 4
    scanlines = b"".join(
        b"\x00" + bytes(rgba[row * stride:(row + 1) * stride])
        for row in range(height)
    )
    header = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    encoded = (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", header)
        + _png_chunk(b"IDAT", zlib.compress(scanlines, level=6))
        + _png_chunk(b"IEND", b"")
    )
    Path(path).write_bytes(encoded)


def _blend_pixel(rgba, width, height, x, y, color, alpha=255):
    if x < 0 or y < 0 or x >= width or y >= height:
        return
    offset = (y * width + x) * 4
    if alpha >= 255:
        rgba[offset:offset + 4] = bytes((*color, 255))
        return
    inverse = 255 - alpha
    rgba[offset] = (color[0] * alpha + rgba[offset] * inverse) // 255
    rgba[offset + 1] = (color[1] * alpha + rgba[offset + 1] * inverse) // 255
    rgba[offset + 2] = (color[2] * alpha + rgba[offset + 2] * inverse) // 255
    rgba[offset + 3] = 255


def _draw_rect(rgba, width, height, left, top, right, bottom, color, alpha=255):
    for y in range(max(0, int(top)), min(height, int(bottom) + 1)):
        for x in range(max(0, int(left)), min(width, int(right) + 1)):
            _blend_pixel(rgba, width, height, x, y, color, alpha)


def _draw_disc(rgba, width, height, center, radius, color, alpha=255):
    cx, cy = int(round(center[0])), int(round(center[1]))
    radius = max(0.5, float(radius))
    radius_sq = (radius + 0.5) ** 2
    for y in range(int(math.floor(cy - radius)), int(math.ceil(cy + radius)) + 1):
        for x in range(int(math.floor(cx - radius)), int(math.ceil(cx + radius)) + 1):
            if (x - cx) ** 2 + (y - cy) ** 2 <= radius_sq:
                _blend_pixel(rgba, width, height, x, y, color, alpha)


def _draw_line(rgba, width, height, start, end, color, thickness=1, alpha=255):
    x0, y0 = float(start[0]), float(start[1])
    x1, y1 = float(end[0]), float(end[1])
    distance = max(abs(x1 - x0), abs(y1 - y0))
    steps = max(1, int(math.ceil(distance)))
    radius = max(0.5, float(thickness) / 2.0)
    for index in range(steps + 1):
        fraction = index / steps
        _draw_disc(
            rgba, width, height,
            (x0 + fraction * (x1 - x0), y0 + fraction * (y1 - y0)),
            radius, color, alpha,
        )


def _draw_text(rgba, width, height, origin, text, color, scale=1, alpha=255):
    """Draw compact ASCII labels without requiring cv2 or Pillow."""
    cursor = int(origin[0])
    top = int(origin[1])
    scale = max(1, int(scale))
    for character in str(text).upper():
        glyph = _FONT_5X7.get(character, _FONT_5X7["?"])
        for row, bits in enumerate(glyph):
            for column in range(5):
                if bits & (1 << (4 - column)):
                    _draw_rect(
                        rgba, width, height,
                        cursor + column * scale,
                        top + row * scale,
                        cursor + (column + 1) * scale - 1,
                        top + (row + 1) * scale - 1,
                        color, alpha,
                    )
        cursor += 6 * scale


def _annotate_image(image_path, args, anchors, positions, estimate_rows, metrics,
                    raw_bgra, width, height):
    """Overlay local-frame traces on the RGB capture without extra packages.

    CARLA's debug primitives are not included in an offscreen RGB sensor image,
    so the same coordinates are projected into the top-down camera image here.
    The encoder and drawing primitives intentionally use only the standard
    library; this keeps the CARLA venv independent of OpenCV/Pillow.
    """
    expected = width * height * 4
    if len(raw_bgra) != expected:
        raise RuntimeError(
            f"unexpected CARLA image buffer: got {len(raw_bgra)}, expected {expected}"
        )
    rgba = bytearray(expected)
    for index in range(0, expected, 4):
        blue, green, red, alpha = raw_bgra[index:index + 4]
        rgba[index:index + 4] = bytes((red, green, blue, alpha))

    horizontal_fov = math.radians(args.camera_fov_degrees)
    world_width = 2.0 * args.camera_height_m * math.tan(horizontal_fov / 2.0)
    world_height = world_width * height / width

    def pixel(x, y):
        return (
            int(round(width / 2.0 + y / world_width * width)),
            int(round(height / 2.0 + x / world_height * height)),
        )

    def polyline(points, color, thickness):
        projected = [pixel(float(point[0]), float(point[1])) for point in points]
        for previous, current in zip(projected, projected[1:]):
            _draw_line(rgba, width, height, previous, current,
                       (8, 16, 20), thickness + 3, 220)
            _draw_line(rgba, width, height, previous, current, color, thickness, 255)

    def text(origin, value, color, scale):
        shadow = (origin[0] + scale, origin[1] + scale)
        _draw_text(rgba, width, height, shadow, value, (5, 10, 12), scale, 230)
        _draw_text(rgba, width, height, origin, value, color, scale, 255)

    truth = positions[:, :2]
    polyline(truth, (0, 255, 255), 5)
    for point in truth[::10]:
        marker = pixel(float(point[0]), float(point[1]))
        _draw_disc(rgba, width, height, marker, 5, (8, 16, 20), 230)
        _draw_disc(rgba, width, height, marker, 3, (0, 255, 255), 255)

    scale = max(1, min(width, height) // 450)
    for anchor in anchors:
        point = pixel(anchor.position_m[0], anchor.position_m[1])
        _draw_disc(rgba, width, height, point, 15, (8, 16, 20), 235)
        _draw_disc(rgba, width, height, point, 11, (255, 220, 0), 255)
        text((point[0] + 16, point[1] - 9), anchor.id, (255, 245, 170), scale)

    by_algorithm = {algorithm: [] for algorithm in ALGORITHMS}
    for row in estimate_rows:
        if int(row["valid"]):
            by_algorithm[row["algorithm"]].append(
                (float(row["estimate_x_m"]), float(row["estimate_y_m"]))
            )
    for algorithm, trace in by_algorithm.items():
        if len(trace) >= 2:
            polyline(trace, COLORS[algorithm], 2)

    uav_point = pixel(float(truth[-1, 0]), float(truth[-1, 1]))
    _draw_line(rgba, width, height,
               (uav_point[0] - 19, uav_point[1]),
               (uav_point[0] + 19, uav_point[1]), (8, 16, 20), 7, 230)
    _draw_line(rgba, width, height,
               (uav_point[0], uav_point[1] - 19),
               (uav_point[0], uav_point[1] + 19), (8, 16, 20), 7, 230)
    _draw_line(rgba, width, height,
               (uav_point[0] - 17, uav_point[1]),
               (uav_point[0] + 17, uav_point[1]), (255, 255, 255), 3, 255)
    _draw_line(rgba, width, height,
               (uav_point[0], uav_point[1] - 17),
               (uav_point[0], uav_point[1] + 17), (255, 255, 255), 3, 255)
    text((uav_point[0] + 21, uav_point[1] + 5), "UAV", (255, 255, 255), scale)

    margin = max(16, 20 * scale)
    panel_width = min(width - 2 * margin, max(440, int(width * 0.34)))
    line_height = 11 * scale
    panel_height = margin + (8 + len(ALGORITHMS)) * line_height + 10 * scale
    panel_height = min(height - 2 * margin, panel_height)
    _draw_rect(rgba, width, height, margin, margin,
               margin + panel_width, margin + panel_height, (8, 16, 22), 218)
    _draw_rect(rgba, width, height, margin, margin,
               margin + panel_width, margin + 2 * scale, (0, 220, 220), 255)
    text_x = margin + 12 * scale
    text_y = margin + 9 * scale
    text((text_x, text_y), "CARLA / UAV UWB OVERHEAD", (245, 250, 250), scale)
    text_y += 10 * scale
    text((text_x, text_y), "PLATFORM: UAV / BASE_LINK", (215, 225, 228), scale)
    text_y += 10 * scale
    text((text_x, text_y), "3D RANGES | LOCAL FRAME X/Y/Z (M)", (215, 225, 228), scale)
    text_y += 10 * scale
    text((text_x, text_y), "CYAN = CARLA GROUND TRUTH", (0, 255, 255), scale)
    text_y += 10 * scale
    text((text_x, text_y),
         f"ANCHOR Z: {min(a.position_m[2] for a in anchors):.1f}-{max(a.position_m[2] for a in anchors):.1f} M",
         (255, 220, 0), scale)
    text_y += 11 * scale
    for algorithm in ALGORITHMS:
        color = COLORS[algorithm]
        _draw_line(rgba, width, height,
                   (text_x, text_y + 3 * scale),
                   (text_x + 13 * scale, text_y + 3 * scale), color, 3 * scale)
        label = algorithm.removeprefix("uwb.matlab.")
        if algorithm == "uwb.awesome_uwb":
            label = "awesome_uwb graph"
        label = f"{label}: RMSE {metrics[algorithm]['rmse_m']:.3f} m"
        text((text_x + 18 * scale, text_y), label, color, max(1, scale - 1))
        text_y += line_height

    # The camera projection maps +X down and +Y right in this image.
    axis_x = width - margin - 80 * scale
    axis_y = height - margin - 50 * scale
    _draw_line(rgba, width, height, (axis_x, axis_y),
               (axis_x, axis_y + 22 * scale), (255, 150, 100), 3 * scale)
    _draw_line(rgba, width, height, (axis_x, axis_y),
               (axis_x + 22 * scale, axis_y), (150, 255, 150), 3 * scale)
    text((axis_x - 4 * scale, axis_y + 25 * scale), "X", (255, 150, 100), scale)
    text((axis_x + 25 * scale, axis_y - 4 * scale), "Y", (150, 255, 150), scale)
    _write_rgba_png(image_path, width, height, rgba)


def main(argv=None):
    args = _parse_args(argv)
    if args.frames < 10 or args.image_width < 320 or args.image_height < 240:
        raise ValueError("frames must be >= 10 and image dimensions must be at least 320x240")
    if args.camera_height_m <= 0.0:
        raise ValueError("camera-height-m must be positive")
    if not 30.0 <= args.camera_fov_degrees <= 120.0:
        raise ValueError("camera-fov-degrees must be between 30 and 120")
    try:
        import carla
    except ImportError as exc:
        raise RuntimeError("run with the Python environment containing CARLA 0.9.16") from exc

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    if client.get_client_version() != client.get_server_version():
        raise RuntimeError("CARLA client/server versions do not match")
    world = client.get_world()
    if "sandbox-v29" not in world.get_map().name.lower():
        world = client.load_world("sandbox-v29")
    map_name = world.get_map().name
    if "sandbox-v29" not in map_name.lower():
        raise RuntimeError(f"CARLA loaded unexpected map: {map_name}")
    original_settings = world.get_settings()
    uav = None
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    image_path = output_dir / "carla_uwb_overhead.png"
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = args.fixed_delta_seconds
        world.apply_settings(settings)
        register_default_algorithms()
        uav = _spawn_uav(world, carla, args.uav_altitude_m)
        positions, timestamps, velocities, frame_ids, path_length, origin = _collect_carla_trajectory(
            world, uav, args, carla
        )
        anchors = _anchors(args)
        from generators.range_simulation import simulate_ranges
        frames = simulate_ranges(
            anchors, positions, timestamps, sigma_m=args.sigma_m,
            random_seed=args.random_seed, tag_id="carla_uav_uwb_tag", dimensions=3,
        )
        metrics, estimate_rows = _run_algorithms(
            frames, positions, timestamps,
            initial_xyz=(0.0, 0.0, max(args.uav_altitude_m * 0.7, 1.0)),
        )
        frame, image_width, image_height, raw_bgra = _capture_overhead(
            world, carla, origin, args, image_path
        )
        _annotate_image(
            image_path, args, anchors, positions, estimate_rows, metrics,
            raw_bgra, image_width, image_height,
        )
        if args.draw_debug:
            # Debug primitives are useful in a live CARLA client, but can be
            # rendered as black quads by this offscreen UE build. Keep them
            # opt-in so the saved RGB image remains clean by default.
            _draw_scene(world, carla, origin, anchors, positions, estimate_rows)
    finally:
        if uav is not None:
            uav.destroy()
        world.apply_settings(original_settings)

    metadata = {
        "carla_map": map_name,
        "carla_frame": frame,
        "carla_local_origin_world_m": [
            float(origin.x), float(origin.y), float(origin.z)
        ],
        "image": str(image_path),
        "image_size": [args.image_width, args.image_height],
        "camera_height_m": args.camera_height_m,
        "camera_fov_degrees": args.camera_fov_degrees,
        "camera_projection": "top-down pitch=-90 yaw=0; +X down, +Y right",
        "uav_path_length_m": path_length,
        "uav_altitude_m": args.uav_altitude_m,
        "tracked_entity": "uav_platform_base_link",
        "carla_uav_actor": {
            "blueprint": "static.prop.mobile",
            "role_name": "nexus_uav",
            "visual_mesh": "small non-vehicle prop; white UAV marker is overlaid",
        },
        "range_model": "euclidean_3d",
        "dimensions": 3,
        "anchors": [anchor.__dict__ for anchor in anchors],
        "positions_xyz": positions[:, :3].tolist(),
        "estimate_rows": estimate_rows,
        "metrics": metrics,
    }
    with (output_dir / "carla_uwb_overhead.json").open("w", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(f"CARLA overhead image: {image_path}")
    print(f"CARLA map: {metadata['carla_map']}")
    print(f"UAV path length: {path_length:.3f} m")
    print("CARLA_UWB_OVERHEAD_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
