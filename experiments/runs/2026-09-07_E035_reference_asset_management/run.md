# E035：实体参考图资产登记链路

## 状态

VALIDATED（参考图资产管理软件）。没有在本运行中采集或伪造任何实体参考图。

## 实现

`code/analysis/calibration/register_reference_assets.py` 以 E030 的不可变实例注册表为
输入，要求每个采集项显式提供：稳定 `instance_id`、要求的视角、图像路径、采集批次和
相机标定 ID。它写出新的注册表 revision，而不改写用户接受的真值基线。

每一个登记后的资产包含路径、字节数、SHA-256、视角、标定 ID 和采集批次；只有一个
实例的 `nadir / oblique_north / oblique_south` 三个要求视角齐备，状态才从 `pending`
或 `partial` 变为 `captured`。未知实例、未要求的视角、空/不支持的文件、重复视角及
试图覆盖基准注册表均会拒绝。

## 验证

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=code/analysis \
  python3 -m pytest code/analysis/test/test_reference_asset_registry.py \
    code/analysis/test/test_physical_sandbox_catalog.py -q
# 7 passed

python3 -m py_compile code/analysis/calibration/register_reference_assets.py
```

## 真机采集时使用

相机恢复供电后，先将每个实拍文件保存到对应运行目录的 `data/raw/` 外部位置，再从同一
目录生成含 `capture_session_id`、`calibration_id` 与 `captures` 的 JSON manifest，执行：

```bash
PYTHONPATH=code/analysis python3 code/analysis/calibration/register_reference_assets.py \
  --registry experiments/runs/2026-09-07_E030_physical_sandbox_metric_reference/physical_sandbox_model_reference_registry_v01.json \
  --capture-manifest <本次实拍目录>/capture_manifest.json \
  --output <本次实拍目录>/physical_sandbox_model_reference_registry_capture_v01.json
```

该流程只管理视觉参考数据；不读取飞控串口、不发送 MAVLink 或飞行控制命令。
