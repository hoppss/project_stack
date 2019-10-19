1.把nav_goal包放在ROS工作空间下并catkin_make编译

2.roscore

3.运行rosrun nav_goal nav_goal_node.py 可以看到输出导航点序列的坐标以及导航点个数

4.运行rosrun nav_goal current_gps_node2.py 并链接手机，即可看到手机GPS和IMU数据