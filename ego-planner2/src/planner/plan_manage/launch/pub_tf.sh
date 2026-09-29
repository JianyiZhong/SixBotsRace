#!/bin/bash
# ROS 静态TF发布脚本：让 world / map / odom 三个坐标系完全重合
# 原理：发布两个零偏移的静态TF变换，无平移、无旋转

# ===================== 固定配置（无需修改） =====================
# 加载ROS Noetic环境
source /opt/ros/noetic/setup.bash

echo "======== 开始发布 TF：world ↔ map ↔ odom 完全重合 ========"
echo "平移：x=0 y=0 z=0"
echo "旋转：roll=0 pitch=0 yaw=0"
echo "========================================================="

# 发布第一个TF：world → map （零偏移）
# 参数：x y z yaw pitch roll 父坐标系 子坐标系 发布频率(Hz)
rosrun tf static_transform_publisher 0 0 0 0 0 0 world map 10 &

# 发布第二个TF：map → odom （零偏移）
rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 10 &

# 保持脚本运行（静态TF需要持续发布）
wait
