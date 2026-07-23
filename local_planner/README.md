# local_planner

局部轨迹规划占位包。

- 订阅：`/perception/obstacles`、`/state_estimation/odom`
- 发布：`/planner/local_trajectory`
- 当前行为：收到障碍消息后发布空的无效轨迹，不执行样条规划。
