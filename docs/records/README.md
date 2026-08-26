# 项目记录

记录是事实层，不是宣传材料。当前执行入口是 [当前执行基线](../2026-08-25_plan_current_execution_baseline.md)；历史日记录全部保留。每条新增记录必须包含日期、参与者、环境/设备、输入、操作、证据路径、结论和未决项。

- `daily/`：保留每日事实记录，文件名 `YYYY-MM-DD_daily_<topic>.md`；算法和硬件结论优先写入 `experiments/runs/`，但不删除既有记录。
- `meetings/`：会议纪要，文件名 `YYYY-MM-DD_meeting_<topic>.md`
- `issues/`：阻塞和风险，文件名 `YYYY-MM-DD_issue_<topic>.md`
- `evidence-index.md`：把实测证据映射到规划目标和答辩页

推荐直接复制 `templates/daily-log.md`、`templates/meeting-note.md` 或 `templates/issue-log.md`。
