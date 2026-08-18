# 数据目录

`raw/` 原始采集（不提交）；`interim/` 中间转换；`processed/` 可复现处理结果；`metadata/` 设备、单位、采样率、坐标系和校验信息。

每个数据集使用 `YYYY-MM-DD_<source>_<scenario>_vNN` 命名，并在元数据中记录来源、许可、时间范围、运行编号和 sha256。
