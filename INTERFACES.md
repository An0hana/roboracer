# RoboRacer 接口约定

本文档冻结感知、定位、规划、控制与安全模块之间的第一版接口。算法实现可以独立变化，但不得在未同步团队的情况下修改这些接口。

## 话题

| 话题 | 消息类型 | 提供者 | 使用者 | 默认频率 / QoS |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR / 模拟器 | 感知、Safety | 传感器频率，SensorDataQoS |
| `/state_estimation/odom` | `nav_msgs/msg/Odometry` | 定位模块 | 感知、规划、MPPI、Safety | 50 Hz，Reliable KeepLast(5) |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 建图模块 | 定位、MPPI | Reliable + TransientLocal |
| `/perception/obstacles` | `roboracer_msgs/msg/TrackedObstacleArray` | 本地感知 | 规划、MPPI | 40–50 Hz，Reliable KeepLast(1) |
| `/planner/local_trajectory` | `roboracer_msgs/msg/Trajectory` | 局部规划 | MPPI | 20–50 Hz，Reliable KeepLast(1) |
| `/race_manager/state` | `roboracer_msgs/msg/RaceState` | 比赛状态机 | 规划、MPPI、Safety | 10–50 Hz，Reliable KeepLast(1) |
| `/control/mppi_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | MPPI | Safety | 20 Hz，Reliable KeepLast(1) |
| `/ackermann_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | Safety | 实车VESC接口 | 50 Hz，Reliable KeepLast(1) |

仿真阶段允许将 MPPI、感知和规划节点的 `odom_topic` 参数设为 `/ego_racecar/odom`，不要求伪造 `/state_estimation/odom`。实车部署时统一切换到 `/state_estimation/odom`。

## TF

```text
map -> odom -> base_link -> laser
```

- `map`：全局地图与赛线坐标系。
- `odom`：连续局部里程计坐标系。
- `base_link`：车辆后轴中心。
- `laser`：LiDAR坐标系。
- `base_link -> laser` 的六自由度外参测量后填写。

## 时间与失效

- 所有输出保留源消息时间戳。
- 不得用“当前时间”重新标记过期定位或感知数据。
- 默认状态超时为 0.10 秒，雷达超时为 0.15 秒。
- 消息非有限、坐标系错误或超时均视为无效；Safety负责最终停车。

## 赛线 CSV

固定列顺序：

```text
x,y,v_ref,yaw,s,width_left,width_right,curvature
```

- 坐标系：`map`
- 距离：米
- 速度：米每秒
- 航向：弧度
- 曲率：每米
- 赛线必须闭合，`s` 单调递增。

## 职责

建图负责：

- `/map`
- `/state_estimation/odom`
- `map -> odom -> base_link -> laser` TF
- 地图文件、定位参数与最优赛线文件

算法开发负责：

- `/perception/obstacles`
- `/planner/local_trajectory`
- `/race_manager/state`
- `/control/mppi_cmd`
- MPPI与Safety接口

共同确认：

- 赛线CSV格式
- `base_link -> laser` 外参
- 时间戳、更新频率、失效阈值与实车标定值
