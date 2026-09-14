"""Reuse the installed WSL CUDA stack without importing its incompatible NumPy 2."""

import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=Path("/root/nexus_envs/gdr_net_packages"))
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parents[2] / "data/processed/2026-09-05_gpu_inference_v01/runtime")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    packages = ("torch", "torchgen", "functorch", "nvidia", "triton", "filelock", "fsspec",
                "jinja2", "markupsafe", "mpmath", "networkx", "sympy", "typing_extensions.py")
    sources = {}
    for name in packages:
        source = (args.source / name).resolve(strict=True)
        target = args.output / name
        if target.exists() or target.is_symlink():
            if not target.is_symlink() or target.resolve() != source:
                raise ValueError(f"refusing to replace existing runtime entry: {target}")
        else:
            target.symlink_to(source, target_is_directory=source.is_dir())
        sources[name] = str(source)
    (args.output.parent / "runtime_sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print("GPU runtime references prepared; system NumPy/Pillow remain in use")


if __name__ == "__main__":
    main()
