# TowerGO 离线 SLAM 与导航实验

本项目针对 ROS2 Humble / ARM64，提供 **合成双雷达 → 二维建图 → 保存地图 → AMCL 重新定位 → Nav2 闭环导航**。所有运行入口仅用于离线实验，没有真机控制后端，不导入或实例化 Astribot SDK。

## 状态与边界

实际验收结果以 `workspace/data/reports/` 的报告为准。源码存在、构建成功、单元测试通过均不等同于建图或导航验收通过。尚未通过的实验会返回非零退出码，报告保留错误而不标成成功。
模拟尺寸、外参、噪声都是实验参数，不是机器人实测值。当前射线模型为平面静态环境，不模拟 MID360 非重复扫描、逐点运动畸变、真实 IMU、轮滑及硬件制动。因此合成数据成绩不能替代真机验证。

## 构建与隔离运行

在机器人终端执行：

```bash
cd /home/astribot/TowerGO/workspace/src/astribot_navigation
BASE_IMAGE=public.ecr.aws/docker/library/ros:humble-ros-base bash scripts/container.sh build
bash scripts/container.sh run bash /towergo/src/astribot_navigation/scripts/test_unit.sh
```

Docker Hub 访问异常时可使用上面的官方镜像镜像源；不关闭 TLS 校验。首次构建需要下载依赖。构建使用 sudo 访问 Docker，不修改用户组或系统 ROS。
运行容器固定 `ROS_DOMAIN_ID=125`、`--network none`、独立 IPC，不挂载设备或 SDK；与实际机器人的 Domain 25 隔离。所有节点在同一个容器内通过 loopback 通信。

工作区 `data` 映射为容器 `/data`；不要把主机地图路径原样传进容器。容器源码位于 `/towergo/src/astribot_navigation`。修改宿主源码后需要重新 build 更新镜像。

## 一次完整实验

```bash
# 建图、保存栅格地图和序列化位姿图，保留合成录包
bash scripts/container.sh run ros2 run astribot_nav_sim experiment mapping \
  --scenario room --seed 42 --scan-source front --record \
  --output /data/reports/room-map

# 加载上一步地图，测试初始误差收敛，并执行三个导航目标
bash scripts/container.sh run ros2 run astribot_nav_sim experiment navigation \
  --scenario room --seed 42 --noise \
  --map /data/reports/room-map/map.yaml --output /data/reports/room-nav

# 取消任务、传感器中断、暂停恢复、堵塞路径
bash scripts/container.sh run ros2 run astribot_nav_sim experiment faults \
  --scenario room --map /data/reports/room-map/map.yaml \
  --output /data/reports/room-faults

# 三个场景、单/双雷达建图、三个随机种子，正常导航共27个目标
bash scripts/container.sh run python3 /towergo/src/astribot_navigation/scripts/run_suite.py \
  --output /data/reports/suite --rate 2
```

默认模拟速度为 2 倍；主机繁忙时使用 `--rate 1`。加速不能牺牲 TF、传感器和控制器处理能力。完整实验可能需要数十分钟。每次使用新的输出目录，避免把旧地图误认为新实验结果。

## 启动入口

在容器内：

```bash
ros2 launch astribot_nav_bringup offline_mapping.launch.py scenario:=room scan_source:=dual
ros2 launch astribot_nav_bringup offline_navigation.launch.py map:=/data/maps/office/map.yaml
ros2 run astribot_nav_api navigate 3.0 0.0 0.0
```

导航 launch 不自动发送初始位姿；实验工具会向 `/initialpose` 注入已知起点偏差，然后检查 AMCL 收敛。手工启动时须自行提供初始位姿。
参数：`scenario:=room|corridor|obstacles`、`seed:=42|43|44`、`scan_source:=front|dual`、`noise:=true|false`、`rate:=2.0`。

## 录包回放

只支持带标准 `/odom`、`odom → base_link` 和雷达静态外参的已整理输入；不把未确认语义的厂商关节状态自动当成里程计。

```bash
ros2 run astribot_nav_bridge prepare_bag /data/reports/room-map/raw_bag /data/bags/room-input
ros2 launch astribot_nav_bringup replay_mapping.launch.py bag:=/data/bags/room-input scan_source:=front
```

`prepare_bag` 保留传感器、里程计及评分真值，移除控制话题和旧地图 TF。原录包不变。每次重新回放应重启整个 replay launch，清除 SLAM 状态；暂停/恢复不改变时间。禁止用 `--loop` 的时间回跳复用已有 SLAM 地图。
回放可检查建图和定位输入；录包轨迹不响应新速度命令，因此不是导航控制闭环。

## 模块与接口

- `astribot_nav_sensors`：C++ 点云变换/过滤；双帧最大时间差 20 ms，拒绝旧帧，不使用最新 TF 代替采样时刻 TF。
- `astribot_nav_bridge`：验证 Twist、限制速度、数据超时停车；只读厂商状态解析，始终标记坐标语义未确认。
- `astribot_nav_sim`：50 Hz 运动模型，10 Hz 双点云，固定种子，碰撞真值和实验评分。
- `astribot_nav_api`：标准 `NavigateToPose` 发送、反馈、结果、取消。
- `astribot_nav_bringup`：统一 launch、SLAM、AMCL、NavFn、DWB 参数。

详见 [接口约定](docs/interfaces.md)、[实验验收](docs/experiments.md)、[SDK 核查](docs/sdk-audit.md)。

## 报告与验收

每个实验目录包含 `report.json`、`launch.log`、`truth.csv`、`estimated.csv`、依赖版本；建图实验另存地图、位姿图，可选原始合成录包。`summary.json` 汇总全套结果。
轨迹评分使用同一时间戳的 TF 与真值，只使用固定初始坐标，不做结束后拟合。地图评分仅对已观测占据栅格计算到真实墙段的距离，不把未观测区域算作建图成功。

## 真机接入前仍需验证

底盘反馈的轴/单位/参考系、实际底盘控制模式、控制权获取与释放、指令超时停车、雷达外参是否已应用、时间同步以及实测轮廓。默认禁止硬件后端，不能用 SDK 强制控制权绕过这些检查。
