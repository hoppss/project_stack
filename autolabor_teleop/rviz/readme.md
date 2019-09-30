# 0. 发布停转指令

```
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
```

# 1. 查看hokuyo是否工作正常

```
roslaunch autolabor_pro1_driver driver_hokuyo.launch
rosrun rviz rviz -d `rospack find autolabor_teleop`/rviz/autolabor_pro1_model.rviz
```

# 2. gmapping建图

```
roslaunch autolabor_pro1_driver driver_hokuyo.launch
roslaunch autolabor_teleop autolabor_gmapping.launch
rosrun rviz rviz -d `rospack find autolabor_teleop`/rviz/autolabor_pro1_gmapping.rviz
roslaunch autolabor_pro1_driver keyboard_move.launch
rosrun map_server map_saver -f map
```

如果使用rosbag建图的话:
```
roslaunch autolabor_pro1_driver driver_hokuyo.launch
roslaunch autolabor_teleop autolabor_gmapping.launch
rosrun rviz rviz -d `rospack find autolabor_teleop`/rviz/autolabor_pro1_gmapping.rviz
rosbag play 2019-05-06-20-09-17.bag
rosrun map_server map_saver -f map
```
在Rviz中需要把fix frame切换成odom, /scan才能正常显示

# 3. 定位导航

```
roslaunch autolabor_teleop autolabor_teleop.launch
rosrun rviz rviz -d `rospack find autolabor_teleop`/rviz/autolabor_pro1_amcl.rviz
```
