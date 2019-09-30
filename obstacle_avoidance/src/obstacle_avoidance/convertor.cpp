//
// Created by yonghui on 19-8-30.
//

#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include "convertor.h"

namespace obstacle_avoidance
{
    double Convertor::getYaw(const geometry_msgs::PoseStamped &pose)
    {
        return angles::normalize_angle(tf::getYaw(pose.pose.orientation));
    }


    double Convertor::getYaw(const tf::Stamped<tf::Pose> &pose)
    {
        return angles::normalize_angle(tf::getYaw(pose.getRotation()));
    }


    void Convertor::setOrientation(geometry_msgs::PoseStamped &pose, double angle)
    {
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    }
}