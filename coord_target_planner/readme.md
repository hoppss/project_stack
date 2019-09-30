# 测试

静止

```$xslt
roslaunch turtlebot3_gazebo turtlebot3_coord.launch
roslaunch turtlebot3_navigation turtlebot3_coord.launch map_file:=/home/yonghui/map.yaml
roslaunch coord_target_planner planner_demo.launch
```

# 目前进度

完成:

- 根据global costmap找到前向走廊中线path
- 对找到的前向中线使用三次Bezier spline进行拟合平滑

待完成:

- spline拟合加上RANSAC
- 对每次拟合的spline进行卡尔曼滤波
- 实现RA*算法