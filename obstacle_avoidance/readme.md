# 1. 简介

基于平面激光雷达传感器, 在未知环境下控制机器人绕过障碍物到达指定目标点

![](./misc/20190929-complex_symmetric.gif)

可执行节点有两个:

- `simple_obstacle_stop_node`: 独立的一个小节点, 通过读取激光雷达扫描数据获得障碍物的距离, 执行减速或停止的控制.

- `obstacle_avoidance_node`: 基于滚动窗口选择子目标点, 以及局部代价地图和动态窗口实现机器人控制的节点, 根据`move_base`的`SimpleActionlibServer`修改而来. 滚动窗口参考文献[1]实现, 具体算法实现在`rolling_window`功能包中.

# 2. 依赖

- [rolling_window](../rolling_window/)
- [navigation_stack](https://github.com/ros-planning/navigation/tree/kinetic-devel)
- [turtlebot3_simulation](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)

# 3. 使用

## 3.1 turtlebot3仿真

首先打开turtlebot3 gazebo, 搭建障碍物环境, 比如放几个方块(尽量不要和底盘前进中线对称, 现在程序还有点问题)

```
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
```

启动避障程序:

```
roslaunch obstacle_avoidance turtlebot3_simulation.launch 
```

在rviz下选择目标点, 开始前进

# 4. 存在问题

- [Half DONE] `rolling_window`部分情况的振荡问题, 出现的情况包括但不限于: 障碍物尺寸过大扫描不完整, 凹状障碍物..

# 5. 参考文献

[1] 胡远航. 未知环境下自主移动机器人避障研究[D].哈尔滨工程大学,2013.