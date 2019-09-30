//
// Created by yonghui on 19-9-17.
//

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include "nav_simulator.h"
#include "move_base_obstacle.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoidance_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    tf::TransformListener tf;
    obstacle_avoidance::NavSimulator sim;
    obstacle_avoidance::MoveBaseObstacle move_base(tf);
    ros::spin();
}