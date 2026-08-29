# CARLA 0.9.16 Linux UWB reproduction

This is the verified Linux path for the current GPU container. CARLA 0.9.16
uses UE 4.26, whose Linux desktop RHI is Vulkan. Do not pass `-opengl`: UE
shows a modal warning and switches to Vulkan. `-nullrhi` is also unsuitable
for this build because CARLA releases render resources during startup.

## Server

The current container exposes the NVIDIA character devices but no usable DRM
render node. A headless NVIDIA X server on `:8` is therefore used to initialize
the matching NVIDIA Vulkan ICD; CARLA itself renders offscreen.

```bash
cd /root/autodl-tmp/NEXUS
CARLA_ROOT=/root/autodl-tmp/carla_linux \
NVIDIA_LIB_DIR=/root/autodl-tmp/nvidia-vendor-580 \
NVIDIA_ICD=/root/autodl-tmp/nvidia_icd_580.json \
CARLA_DISPLAY=:8 \
simulation/carla/run_carla_server_linux.sh
```

The server is ready when TCP port 2000 listens. This build starts on its
default `Carla/Maps/Town10HD_Opt`; the UWB scripts then perform one explicit
client-side load of `CustomMaps/sandbox-v29/sandbox-v29` and verify the returned
map name. Do not repeatedly call `load_world` from multiple clients.

## `gz_parts` and `sandbox-v29`

`/root/autodl-tmp/gz_parts` is not a UWB measurement dataset. It is a Git LFS
split archive of the custom CARLA build (`CARLA_294096eb1-dirty.tar.gz`, 297
parts). After reconstruction and extraction, the relevant map is installed at:

```text
/root/autodl-tmp/carla_linux/CarlaUE4/Content/CustomMaps/sandbox-v29/
```

The directory contains the Unreal map assets `sandbox-v29.umap` and
`sandbox-v29.uexp`, the tiled map assets, and the road description
`OpenDrive/sandbox-v29.xodr`. The `.umap`/`.uexp` assets are the rendered 3D
scene; the OpenDRIVE file supplies road topology. `Town10HD_Opt` is only the
server startup default and is not the NEXUS experiment map. The reproduction
scripts explicitly load and verify `sandbox-v29` before generating any UWB
observations.

## CARLA smoke test

```bash
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
/root/autodl-tmp/carla-venv/bin/python \
simulation/carla_smoke_test.py --host 127.0.0.1 --port 2000
```

Acceptance requires `SMOKE_TEST_OK` and a non-empty
`/tmp/carla_smoke_frame.png` produced by a CARLA RGB camera.

## UWB reproduction

```bash
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
/root/autodl-tmp/carla-venv/bin/python \
simulation/run_carla_uwb_reproduction.py \
  --host 127.0.0.1 --port 2000 --frames 100 --sigma-m 0.05
```

The experiment runs the five adapters from
`cliansang/positioning-algorithms-for-uwb-matlab` and the UWB-only range graph
from `qxiaofan/awesome-uwb-localization`. CARLA actor Ground Truth generates
ranges and is used for evaluation only. All six routes consume non-coplanar
3D anchor coordinates and return XYZ estimates. Algorithm-facing input is
stored in `observations.csv`; truth is stored separately in `ground_truth.csv`.

## Annotated visualization

```bash
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
/root/autodl-tmp/carla-venv/bin/python \
simulation/visualize_carla_uwb.py \
  --host 127.0.0.1 --port 2000 --frames 100 --camera-height-m 48
```

The saved PNG is a top-down projection of the 3D CARLA scene. The ranges and
metrics remain 3D; the projection only makes the map and XY trace readable in a
single image. The annotator uses the standard library and does not require
OpenCV or Pillow.
