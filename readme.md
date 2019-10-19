# 1. 概述

管廊巡检项目, 室外移动机器人自主移动, 项目中编写的一系列ROS包

# 2. 分类

## 2.1 管廊巡检项目

- autolabor底盘驱动, 启动文件 (前两个不是我写的): [autolabor_description](./autolabor_description/); [autolabor_pro1_driver](./autolabor_pro1_driver/); [autolabor_teleop](./autolabor_teleop/)

- 线性CCD巡线控制(已弃用): [ccd_camera](./ccd_camera); [move_base_lane](./move_base_lane/)

- 走廊中路控制: [coord_target_planner](./coord_target_planner); [coord_move_base](./coord_move_base/)

- PC远程客户端(暂不公开~): [MonitorPlatform](https://github.com/yhfeng1995/MonitorPlatform/)

## 2.2 室外移动机器人自主移动


- 金智达底盘驱动, 启动文件: [neurobot_description](./neurobot_description/); [neurobot_bringup](./neurobot_bringup/)

- 手机偏航角接收程序: [nav_goal_lh](./nav_goal_lh)

- 简单坐标系转换(odom->gps), EKF启动文件: [neurobot_data_proprocess](./neurobot_data_proprocess/)

- 未知环境下避障导航: [rolling_window](./rolling_window/); [obstacle_avoidance](./obstacle_avoidance/)
