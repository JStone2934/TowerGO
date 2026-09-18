# 接口约定

| 接口 | 类型 | 发布/消费 |
|---|---|---|
| /livox/lidar_front、/livox/lidar_back | PointCloud2 | 模拟器发布；扫描处理和局部代价地图消费 |
| /scan | LaserScan | 扫描处理发布；SLAM/AMCL/指令检查消费 |
| /odom | Odometry | 模拟器发布；SLAM/Nav2 消费 |
| /cmd_vel_nav | Twist | Nav2 或脚本路径发布；command_guard 消费 |
| /cmd_vel | Twist | command_guard 发布；仅模拟器消费 |
| /map | OccupancyGrid | 建图时 SLAM 发布，导航时 map_server 发布 |
| /navigate_to_pose | NavigateToPose Action | 导航任务 API |
| /initialpose | PoseWithCovarianceStamped | 外部提供 AMCL 初始位姿 |
| /diagnostics | DiagnosticArray | 扫描/控制健康度 |
| /ground_truth/odom、/ground_truth/collision | Odometry、Bool | 模拟器发布；仅实验评分消费 |

双雷达点云各自处于 lidar_front/lidar_back，外参仅在 scan_processor 中变换一次。厂商实际 livox_frame 尚不能直接沿用。

TF：建图用 SLAM、导航用 AMCL 发布 map→odom；模拟器发布 odom→base_link；模拟静态模型发布 base_link→lidar_front/back。无 world→map 真值变换输入导航。

`command_guard` 拒绝非有限值和非零横移/升降/roll/pitch 指令，限制 vx∈[0,0.2] m/s，wz∈[-0.3,0.3] rad/s；命令或扫描超过 0.5 s、时间倒退时输出零。模拟器另外按墙钟检查命令超时。暂停模拟时保持位置不动。

SDK 状态解析保留 header 时间、position/velocity 数组和原始 frame_id，不发布 /odom 或 TF。硬件 backend 参数取值不是 sim 时立即报错。

二维scan保留前雷达的真实射线原点；双雷达模式中后雷达仅在前雷达已观测射线上补充更近的障碍，不扩展该射线的已知自由空间。后雷达原始点云仍独立参与Nav2局部代价地图。该保守策略避免凹角处虚拟原点错误清空墙面。
