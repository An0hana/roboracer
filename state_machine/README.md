# state_machine

ROS节点名：`state_machine`。

面向直接控制MPPI的双层比赛状态机。

- 安全层：`INIT / READY / FAULT / STOP`
- 行为层：`RACING / TRAILING / OVERTAKE`
- 订阅：里程计、实时局部Costmap、跟踪障碍物
- 发布：`/state_machine/state`、`/state_machine/state_marker`
- 不订阅局部规划轨迹，不再使用独立`RETURN`状态
- 超车结束后用1.5秒连续横向偏置衰减回归赛线
- 比较实时墙体与赛线边界，输出连续`track_confidence`
- 赛道可信度降低时自动降低速度/赛线权重并提高安全权重

仿真由完整自适应链路统一启动：

```bash
ros2 launch mppi_controller adaptive_mppi_sim.launch.py max_speed:=2.0
```
