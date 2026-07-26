# local_planner

基于全局赛车线的局部Frenet候选轨迹规划包。

- 订阅：`/perception/obstacles`、`/state_estimation/odom`
- 发布：`/planner/local_trajectory`、`/planner/candidate_markers`
- 候选：按赛道宽度自适应生成最多7个横向位置，并组合3个规划时域和3个速度档位（最多63条）
- 轨迹：Frenet坐标中的五次横向多项式，最终转换为XY
- 检查：投影距离、Frenet奇点、赛道边界、曲率和动态障碍碰撞
- 选择：横向偏移、曲率、障碍净空、速度、时域和方案切换代价

仿真启动：

```bash
ros2 launch local_planner local_planner.launch.py \
  odom_topic:=/ego_racecar/odom
```

RViz添加`MarkerArray`，话题选择`/planner/candidate_markers`：

- 绿色粗线：最终选择，文字显示偏移、时域、速度档和代价
- 蓝色细线：其他有效候选
- 红色线：越界、碰撞或曲率不合格候选
- 车旁汇总文字：候选总数以及有效、碰撞、越界等数量
