"""Collect diverse checkerboard views from the Pi camera-only TCP service."""

import argparse
import hashlib
import json
from pathlib import Path
import socket
import struct
import time

import cv2
import numpy as np


def receive_exact(channel, count):
    result = bytearray()
    while len(result) < count:
        chunk = channel.recv(count - len(result))
        if not chunk:
            raise ConnectionError("camera stream ended")
        result.extend(chunk)
    return bytes(result)


def receive_frame(channel):
    metadata_size, image_size = struct.unpack("!II", receive_exact(channel, 8))
    if not 0 < metadata_size <= 65536 or not 0 < image_size <= 16_000_000:
        raise ValueError("invalid camera packet size")
    metadata = json.loads(receive_exact(channel, metadata_size))
    encoded = receive_exact(channel, image_size)
    image = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (metadata["height"], metadata["width"]):
        raise ValueError("invalid camera image")
    return metadata, encoded, image


def corners_for(image, board):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(gray, board, cv2.CALIB_CB_NORMALIZE_IMAGE)
    return corners if found else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.1.143")
    parser.add_argument("--port", type=int, default=14552)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=180)
    parser.add_argument("--columns", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-mm", type=float, default=25)
    parser.add_argument("--append", action="store_true",
                        help="append only new, distinct valid views to an existing capture directory")
    args = parser.parse_args()
    board = (args.columns, args.rows)
    saved = []
    if args.output.exists():
        if not args.append:
            raise FileExistsError(f"capture directory already exists: {args.output}")
        for path in sorted(args.output.glob("view_*.jpg")):
            metadata = json.loads(path.with_suffix(".json").read_text())
            if (tuple(metadata.get("board_inner_corners", ())) != board
                    or float(metadata.get("square_size_m", 0.0)) != args.square_mm / 1000):
                raise ValueError(f"existing capture specification differs: {path.name}")
            image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
            corners = None if image is None else corners_for(image, board)
            if corners is None:
                raise ValueError(f"existing saved view no longer has a detectable board: {path.name}")
            saved.append(corners.reshape(-1, 2) / np.array([image.shape[1], image.shape[0]]))
    else:
        args.output.mkdir(parents=True)
    started = time.monotonic()
    last_checked = 0.0
    frames = 0
    with socket.create_connection((args.host, args.port), timeout=5) as channel:
        while time.monotonic() - started < args.seconds:
            metadata, encoded, image = receive_frame(channel)
            frames += 1
            if time.monotonic() - last_checked < 1.0:
                continue
            last_checked = time.monotonic()
            corners = corners_for(image, board)
            preview = image.copy()
            if corners is not None:
                cv2.drawChessboardCorners(preview, board, corners, True)
            cv2.imwrite(str(args.output / "preview.jpg"), cv2.resize(preview, (820, 616)))
            if corners is None:
                print(f"frame={frames} board=not_found saved={len(saved)}", flush=True)
                continue
            # Reject repeated stationary views; compare normalized point displacement.
            points = corners.reshape(-1, 2) / np.array([image.shape[1], image.shape[0]])
            if any(np.sqrt(np.mean((points - old) ** 2)) < .025 for old in saved):
                continue
            name = f"view_{len(saved):03d}"
            (args.output / (name + ".jpg")).write_bytes(encoded)
            metadata.update(board_inner_corners=list(board), square_size_m=args.square_mm / 1000,
                            image_sha256=hashlib.sha256(encoded).hexdigest())
            (args.output / (name + ".json")).write_text(json.dumps(metadata, indent=2) + "\n")
            saved.append(points)
            print(f"SAVED {name} total={len(saved)}", flush=True)
    print(f"CAPTURE_FINISHED frames={frames} diverse_board_views={len(saved)}", flush=True)


if __name__ == "__main__":
    main()
