# TowerGO 离线 SLAM 与导航工作区

完整使用说明：[src/astribot_navigation/README.md](src/astribot_navigation/README.md)。

已实现五个 ROS2 包：bringup、sensors、bridge、api、sim，提供合成双雷达建图、AMCL 定位、Nav2 闭环导航和自动实验工具。

所有运行均在独立 ARM64 ROS2 Humble 容器中进行，Domain=125，不连接真实底盘，不导入 Astribot SDK。

```bash
cd /home/astribot/TowerGO/workspace/src/astribot_navigation
BASE_IMAGE=public.ecr.aws/docker/library/ros:humble-ros-base bash scripts/container.sh build
bash scripts/container.sh run bash /towergo/src/astribot_navigation/scripts/test_unit.sh
```

代码在 src/astribot_navigation，数据及验收报告在 data。容器内 data 映射为 /data。
最终验证状态以 data/reports/IMPLEMENTATION_STATUS.md 及各实验 report.json 为准；合成数据结果不代表真机精度。
