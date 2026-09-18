# 建图

运行 README 中的 experiment mapping。路径由模拟里程计驱动的脚本控制，真值只用于评分。scan_source=front为基线，dual为合成双雷达对照。

生成map.yaml/图像和posegraph文件；后者用于继续SLAM。不要把只保存二维图像称为可继续原位姿图建图。

回放前必须prepare_bag，清除旧map→odom和控制话题。只接纳已有验证标准里程计的合成输入；厂商状态不能自动转换。
