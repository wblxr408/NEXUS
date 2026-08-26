# 离线分析

按 `uwb/`、`vision/`、`calibration/`、`fusion/`、`evaluation/` 分类。脚本输入必须来自 `data/metadata/` 登记的数据集，输出写入对应实验运行目录。

`test_samples/` 是例外：只放明确标注为 `test_sample` 的功能验证输入，运行结果应写入临时目录，不能作为实验数据或精度证据。其回放入口为 `replay_cli.py`。

可切换算法的统一注册表位于 `algorithms/router.py`。仿真和离线回放通过算法全名选择实现，算法核心不得读取仿真真值；外部算法适配位置和来源见 `algorithms/third_party/`。
