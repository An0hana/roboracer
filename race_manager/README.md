# race_manager

比赛状态机占位包。

- 订阅：`/perception/obstacles`、`/state_estimation/odom`
- 发布：`/race_manager/state`
- 当前行为：按固定频率发布 `INIT`，不执行跟随、超车或故障切换。
