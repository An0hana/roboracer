# local_planner

基于全局赛车线的局部Frenet候选轨迹规划包。

- 订阅：`/perception/obstacles`、`/state_estimation/odom`、可选的
  `/race_manager/state`
- 发布：`/planner/local_trajectory`、`/planner/candidate_markers`
- 候选：按赛道宽度自适应生成最多7个横向位置，并组合3个变道完成距离和3个
  速度档位（最多63条）；所有候选统一检查到最大5米，短候选不能逃避远端碰撞
- 轨迹：满足起终点位置、横向斜率和二阶导数六项边界条件的五次多项式；
  起点与后轴中心位置、车辆航向连续，终点保持目标横向位置，不自动回中
- 投影：同时检查距离和车辆前进方向，避免投影到反向赛线
- 检查：Frenet奇点、赛道边界、曲率和动态障碍矩形碰撞；车体矩形由后轴中心
  前移至几何中心
- 跟车：无安全绕行轨迹时，在制动距离允许的前提下输出中线减速停车轨迹；
  障碍已进入制动距离时仍输出无效轨迹并交由安全层紧急处理
- 选择：横向偏移、曲率、障碍净空、速度、时域和方案切换代价
- 状态约束：`OVERTAKE`只采样指定侧，`RETURN`只采样回归赛车线；未启动
  `race_manager`时保持独立规划能力

仿真启动：

```bash
ros2 launch local_planner local_planner.launch.py \
  odom_topic:=/ego_racecar/odom
```

RViz添加`MarkerArray`，话题选择`/planner/candidate_markers`：

- 绿色粗线：最终选择，文字显示偏移、时域、速度档和代价
- 蓝色细线：其他有效几何候选
- 红色线：越界、碰撞或曲率不合格候选
- 速度档仍分别参与动态碰撞预测，但同一几何轨迹只绘制一次
- 车旁汇总文字：候选总数、不同几何数量以及有效、碰撞、越界等数量
