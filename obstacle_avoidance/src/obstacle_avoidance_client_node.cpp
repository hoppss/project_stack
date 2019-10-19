//
// Created by yonghui on 19-10-8.
//

#include <ros/ros.h>
#include <ros/console.h>

#include "obstacle_avoidance_client.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoidance_client_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    obstacle_avoidance::ObstacleAvoidanceClient obs_client;

    ros::NodeHandle private_nh("~");
    double rate;
    private_nh.param("rate", rate, 10.0);
    ros::Rate r(rate);
    while (private_nh.ok())
    {
        obs_client.spinOnce();
        ros::spinOnce();
        r.sleep();
    }
}