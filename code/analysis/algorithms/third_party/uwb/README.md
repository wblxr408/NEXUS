# UWB 算法适配位

外部资料对应目录：

```text
third_party/uwb/
├── awesome_uwb_localization/              # qxiaofan/awesome-uwb-localization
└── positioning_algorithms_for_uwb_matlab/ # cliansang/positioning-algorithms-for-uwb-matlab
```

当前不下载外部仓库。来源登记：

- https://github.com/qxiaofan/awesome-uwb-localization
- https://github.com/cliansang/positioning-algorithms-for-uwb-matlab

MATLAB 或外部 Python 实现接入后，先转换为统一的锚点坐标（m）、测距（m）和 `AlgorithmResult`，再在 `router.py` 注册。目录名仅表示来源，不代表算法已经复现或精度已验证。
