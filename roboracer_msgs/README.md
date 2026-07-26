# roboracer_msgs

团队自研感知、规划与比赛状态节点之间的公共 ROS 2 消息。

## 坐标与单位

- 所有位置使用米，速度使用米每秒，加速度使用米每二次方秒。
- 航向角使用弧度，曲率使用每米。
- 数组消息的 `header.frame_id` 默认使用 `map`。
- 所有节点必须保留源数据时间戳，不得用当前时间掩盖过期数据。

## 消息

- `TrackedObstacle`：对手、软赛道边界或静态障碍的状态与分类。
- `TrackedObstacleArray`：同一时刻的障碍物集合。
- `TrajectoryPoint`：带速度、曲率和赛道宽度的轨迹点。
- `Trajectory`：全局参考线或局部避障轨迹。
- `RaceState`：比赛战术与故障状态。

完整话题、TF、QoS 和职责约定见仓库根目录 `INTERFACES.md`。
