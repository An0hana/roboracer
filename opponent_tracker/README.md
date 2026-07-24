# opponent_tracker

激光对手检测与跨帧跟踪包。

- 订阅：`/scan`、`/state_estimation/odom`、`/map`
- 发布：`/perception/obstacles_measurement`（雷达时刻）、
  `/perception/obstacles`（当前时刻预测）、`/perception/obstacle_markers`
- 算法：距离过滤、自适应断点聚类、PCA有向包围盒、静态地图墙体过滤
- 跟踪：最近邻关联、恒速Kalman滤波、稳定ID、速度估计、短时丢失保持
- 航向：速度切换迟滞、连续低通和最大角速度约束，避免低速转弯跳变
- 定位：仿真可按已知车体尺寸补偿几何中心；实车未知车型可关闭固定尺寸
- 时间：保留原始测量状态，并以恒速模型高频预测到当前控制时刻
- 坐标：按雷达时间戳通过TF转换到参数`target_frame`
- 频率：定时处理最新雷达帧，由`publish_rate_hz`控制，默认50 Hz
- 可视化：RViz按实车最小尺寸显示高亮轮廓框、ID、速度箭头和历史轨迹

当前采用轻量级多目标最近邻关联，比赛中若出现密集交叉目标，再升级为更强的数据关联。

仿真启动：

```bash
ros2 launch opponent_tracker opponent_tracker.launch.py \
  odom_topic:=/ego_racecar/odom
```

尺寸模式：

- 仿真：`use_known_opponent_size:=true`，使用`opponent_length/width`拟合车体中心。
- 实车未知车型：`use_known_opponent_size:=false`，输出多帧平滑后的观测尺寸，
  不执行固定尺寸中心补偿，避免车型不同导致系统偏差。
