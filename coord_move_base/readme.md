# 运行

仅查看仿真规划路径:

```c++
roslaunch turtlebot3_gazebo turtlebot3_coord.launch
roslaunch coord_target_plan plan_demo.launch
```

仿真控制效果:

```c++
roslaunch turtlebot3_gazebo turtlebot3_coord.launch
roslaunch turtlebot3_navigation turtlebot3_coord_nav.launch map_file:=/home/yonghui/map.yaml
```

新添加的相关参数文件:

- `coord_move_base_params.yaml`: 主要是控制时候PID控制器的一些参数
- `target_planner_params.yaml`: 中间线规划的一些参数, 扫描范围等