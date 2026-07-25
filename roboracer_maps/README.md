# roboracer_maps

团队共享的赛道地图与参考轨迹资源包。

```text
maps/<track_name>/
├── <track_name>.pgm
├── <track_name>.yaml
├── traj_race_cl.csv
└── raceline.csv
```

`traj_race_cl.csv`保留原始优化器输出，`raceline.csv`为算法使用的标准格式。

`maps/racetrack_1_5x`是米制尺寸统一放大1.5倍的开发测试副本，原始
`maps/racetrack`保持不变。副本中的坐标、里程和赛道宽度放大1.5倍，
曲率缩小为原来的三分之二。

转换命令：

```bash
ros2 run roboracer_maps convert_optimizer_raceline.py \
  traj_race_cl.csv raceline.csv \
  --max-speed 4.0 --max-lateral-acceleration 4.0
```
