# 视觉算法适配位

预留以下外部算法的源码/适配器目录；不要把源码、权重或大模型文件直接提交到本仓库：

```text
third_party/vision/
├── pvnet/           # PVNet（待取得版本、许可证和权重）
├── gdr_net/         # GDR-Net（待取得版本、许可证和权重）
└── foundationpose/  # FoundationPose（当前单目边界下仅作扩展）
```

来源：`docs/2026-08-25_plan_markerless_visual_simulation_preparation.md` 及
`docs/2026-08-24_research_markerless_visual_target_localization.md`。

适配器必须把结果转换为 `AlgorithmResult`，并保留算法名、版本、权重和坐标/单位信息；未接入前路由会明确报告 unavailable，不会用仿真真值代替算法结果。
