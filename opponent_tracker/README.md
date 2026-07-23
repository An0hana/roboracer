# opponent_tracker

感知与对手跟踪占位包。

- 订阅：`/scan`、`/state_estimation/odom`
- 发布：`/perception/obstacles`
- 当前行为：每帧雷达数据发布一个空的障碍物数组，不执行检测或跟踪。
