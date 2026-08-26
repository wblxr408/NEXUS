# 可切换定位算法

这里是仿真回放和 ROS2 适配器共用的算法选择边界。算法通过完整名称选择，格式为
`<family>.<algorithm>`。本目录只规定位置、输入输出契约和路由，不实现具体算法。

## 算法实现位置

视觉算法放在 `vision/` 的对应子目录，UWB 算法放在 `uwb/` 的对应子目录。每个子目录
都有一个 README 说明来源和接入要求，后续由算法负责人添加源码、配置和测试。

## 外部算法位置

外部源码不随本仓库自动下载。适配说明和来源登记在：

- `third_party/vision/README.md`
- `third_party/uwb/README.md`

加入实际实现后，调用 `register_algorithm(name, family, runner, source)` 注册 runner；
仿真、回放和将来的前端只传递算法名称，路由会把同一输入交给选中的实现。

```python
from algorithms import register_algorithm

register_algorithm("vision.pvnet", "vision", run_pvnet, source="team implementation")
```

`run_pvnet` 必须接收仿真/回放输入并返回 `AlgorithmResult`。路由不提供真值，
也不缓存或替换其他算法的结果。
