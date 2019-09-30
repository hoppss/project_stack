//
// Created by yonghui on 19-9-17.
//

#include "nav_simulator.h"
#include <geometry_msgs/TransformStamped.h>
#include <cstdlib>

namespace obstacle_avoidance
{
    NavSimulator::NavSimulator()
    {
        ros::NodeHandle private_nh("~");
        private_nh.param("global_frame", global_frame_, string("world"));
        private_nh.param("odom_frame", odom_frame_, string("odom"));
        private_nh.param("is_noise", is_noise_, true);
        private_nh.param("rand_x", rand_x_, 0.5);
        private_nh.param("rand_y", rand_y_, 0.5);
        private_nh.param("rand_yaw", rand_yaw_, 1./(2*M_PI));

        ros::NodeHandle nh;
        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, &NavSimulator::odomCb, this);
    }


    void NavSimulator::odomCb(const nav_msgs::OdometryConstPtr &odom_msgs)
    {
        double wo_x = 0.0, wo_y = 0.0, wo_yaw = 0.0;
        if (is_noise_)
        {
            wo_x = getWhiteNoise(rand_x_);
            wo_y = getWhiteNoise(rand_y_);
            wo_yaw = getWhiteNoise(rand_yaw_);
        }

        tf::Quaternion q = tf::createQuaternionFromYaw(wo_yaw);
        geometry_msgs::TransformStamped trans_msgs;
        trans_msgs.header.stamp = ros::Time::now();
        trans_msgs.header.frame_id = global_frame_;
        trans_msgs.child_frame_id = odom_frame_;
        trans_msgs.transform.translation.x = wo_x;
        trans_msgs.transform.translation.y = wo_y;
        trans_msgs.transform.translation.z = 0.0;
        trans_msgs.transform.rotation.x = q.x();
        trans_msgs.transform.rotation.y = q.y();
        trans_msgs.transform.rotation.z = q.z();
        trans_msgs.transform.rotation.w = q.w();
        br_.sendTransform(trans_msgs);
    }


    double NavSimulator::getWhiteNoise(double range)
    {
        double noise = (double)rand() / RAND_MAX;
        noise = range * noise - range / 2;
        return noise;
    }
}