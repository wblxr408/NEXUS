# Claude Code 项目入口

本仓库的统一协作约束在 [AGENTS.md](AGENTS.md)。任何 Claude Code 会话开始时，先阅读：

1. `AGENTS.md`
2. `docs/PROJECT_INDEX.md`
3. 与当前任务相关的目录 `README.md`

不要把本文件当作第二套规则。它只负责把 Claude Code 路由到同一套仓库规范；新的长期规则应写入 `AGENTS.md` 或对应的 `docs/` 事实来源，并通过决策记录说明原因。

## 实现取向

围绕单一目标快速验证定位主链路。避免无必要的防御性代码、抽象和工程细节；允许在可回滚、可记录的实验范围内采用大胆实现。只保留安全、数据完整性、坐标/单位一致性和结果可解释性所必需的检查。

## 常用入口

- 代码：`code/README.md`
- 实验：`experiments/README.md`
- 记录：`docs/records/README.md`
- 答辩：`defense/README.md`
- 变更决策：`docs/decisions/README.md`
