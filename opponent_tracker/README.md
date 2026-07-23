# opponent_tracker

单帧激光障碍检测包。

- 订阅：`/scan`、`/state_estimation/odom`、`/map`
- 发布：`/perception/obstacles`
- 算法：距离过滤、自适应断点聚类、PCA有向包围盒、静态地图墙体过滤
- 坐标：按雷达时间戳通过TF转换到参数`target_frame`
- 频率：定时处理最新雷达帧，由`publish_rate_hz`控制，默认50 Hz

当前不做跨帧关联、稳定ID、速度估计或动态目标分类；这些属于下一阶段。
