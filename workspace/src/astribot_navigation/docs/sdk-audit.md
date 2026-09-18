# Astribot SDK 接口核查

SDK 路径：/home/astribot/astribot_sdk_aarch64
基线提交：37c83d03bffc3e02b66401547fb817e1eed73954。

- astribot_client.py:38 的 Astribot 构造会初始化 AstribotInterface。
- astribot_interface.py:104 在构造期 acquire_control_rights；high_control_rights=False 不代表只读，在已有控制者时仍会交互询问，在服务不存在时会建立控制权服务。
- SDK 已有 executor/thread；不应重复 spin 同一个 SDK 节点。
- examples/202、203 使用 250 Hz 速度积分后下发位置；不能把 Twist 直接当位置。
- set_joints_velocity 接口存在，106 示例仅为夹爪，无法证明底盘支持。
- get_current_joints_position/velocity 与 get_desired_* 不同，后者不能用于实测里程计。
- RobotJointState 包含 header、mode、name、position、velocity、acceleration、torque；历史底盘数据的 name/frame_id 为空。
- 现有约35秒历史底盘记录几乎静止，不足以确定三维坐标定义。
- 底层 AstribotFunction 在 .so 中，本项目不反推未公开语义，也不修改 SDK。
- stop_robot 为整机接口；999 示例还调用 move_to_home 和 restart_robot，不适合作为导航取消任务的实现。

后续真机方案必须先单独验证底盘控制、反馈、超时和控制权，再增加硬件适配。离线项目没有导入 SDK，不申请实际控制权。
