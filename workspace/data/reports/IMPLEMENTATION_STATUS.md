# TowerGO 实现与验证状态

完整矩阵：通过

- 建图：12/12 个实验通过（预期12）。
- 导航：27/27 个目标通过（预期27）。
- 异常实验：1/1 组通过（预期1）。
- 建图位置 RMSE 范围：0.0000–0.0225 m。
- 建图墙面误差 P95 最大值：0.0669 m。
- 导航终点位置误差最大值：0.1050 m。
- 导航终点角度误差最大值：4.19°。
- 双向地图审计：墙面覆盖率最低 100.00%；错误自由空间比例最大 0.00%。

## 单元与集成测试

.................                                                        [100%]
17 passed in 8.06s

录包暂停、恢复、完整重启验证：通过。

公共导航 CLI 验证：通过。

SDK 状态历史录包：8759 条消息解析完成，坐标语义未确认，未发布 TF。

详细结果见 acceptance-summary.json、各实验 report.json、map-coverage-audit.json 和 launch.log。早期调试失败记录保留，不计入最终验收矩阵。

全部运行均为隔离容器中的合成实验，不导入 SDK，不控制真实机器人。模拟精度不能代表实机精度。

公共导航 CLI 中断验证：通过，Ctrl+C 后 0.22 s 模拟停车并保持静止。

交付镜像：`towergo-offline:humble`，ID `sha256:2c8c00741ed67e64064561090390040830e7ef7a0b31a1dd8505f8156da07e57`。

交付地图：`/home/astribot/TowerGO/workspace/data/maps/{room,corridor,obstacles}/map.yaml`，同目录含 SLAM Toolbox 序列化状态及来源记录。
