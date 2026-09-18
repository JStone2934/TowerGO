# 导航

使用 experiment navigation 加载地图，先验证AMCL初始化收敛，再通过标准NavigateToPose依次执行三个目标。默认NavFn+DWB，速度仅前进和转向。

局部代价地图分别读取前后原始点云并保留观测原点。虚拟scan用于SLAM/AMCL，合成双雷达存在遮挡/自由空间近似，须由对照实验评估。

任务取消使用Action取消接口；不调用整机stop_robot、restart_robot或move_to_home。

DWB外层增加RotationShimController：先对准新路径再跟踪，避免0.3rad/s低角速度下原地转向采样停滞；线速度和角速度上限不变。

使用ObstacleFootprint critic检查矩形完整轮廓，代价地图轮廓额外padding=0.06m，以覆盖栅格和定位误差；碰撞评分仍使用原始0.8×0.7m轮廓。
