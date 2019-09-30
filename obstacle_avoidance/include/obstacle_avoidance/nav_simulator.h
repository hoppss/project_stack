//
// Created by yonghui on 19-9-17.
//

#ifndef OBSTACLE_AVOIDANCE_POOR_ODOM_SIMULATOR_H
#define OBSTACLE_AVOIDANCE_POOR_ODOM_SIMULATOR_H

#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <string>
#include <nav_msgs/Odometry.h>

using namespace std;

namespace obstacle_avoidance
{
    class NavSimulator
    {
    public:
        NavSimulator();

    protected:
        /**
         * @brief broadcast tf tree world->odom after received odometry message
         * @param odom_msgs
         */
        void odomCb(const nav_msgs::OdometryConstPtr &odom_msgs);

        double getWhiteNoise(double range);

        // pub and sub
        tf2_ros::TransformBroadcaster br_;
        ros::Subscriber odom_sub_;

        // param
        string global_frame_;
        string odom_frame_;

        bool is_noise_;
        double rand_x_;
        double rand_y_;
        double rand_yaw_;
    };
}

#endif //OBSTACLE_AVOIDANCE_POOR_ODOM_SIMULATOR_H
