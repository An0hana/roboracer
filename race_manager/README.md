# race_manager

最小分层比赛状态机。

- 订阅：`/perception/obstacles`、`/state_estimation/odom`、
  `/planner/local_trajectory`
- 发布：`/race_manager/state`、`/race_manager/state_marker`
- 状态：`INIT -> GLOBAL_TRACK -> TRAILING -> OVERTAKE -> RETURN`
- 降级：输入超时或轨迹无效时进入`FAULT`并置位`fallback_ftg`
- 防抖：状态最短保持时间、切换确认时间和故障恢复确认；启动阶段允许轨迹短暂
  无效，持续超过恢复确认时间才进入`FAULT`

仿真启动：

```bash
ros2 launch race_manager race_manager.launch.py \
  odom_topic:=/ego_racecar/odom
```
