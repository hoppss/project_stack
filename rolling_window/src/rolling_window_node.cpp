//
// Created by yonghui on 19-9-19.
//

#include <ros/console.h>
#include "rolling_window.h"
#include "utilities.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rolling_window_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    tf::TransformListener tf;
    ros::NodeHandle nh;

    rolling_window::RollingWindow rw(tf, "rolling_window");
    geometry_msgs::PoseStamped global_goal, local_goal;
    rolling_window::setIdentityPoseStamped(global_goal, "odom");
    global_goal.pose.position.x = 5;
    rw.setGlobalGoal(global_goal);

    ros::Rate r(10);
    while (nh.ok())
    {
        rw.solveLocalGoal(local_goal);
        r.sleep();
        ros::spinOnce();
    }
}