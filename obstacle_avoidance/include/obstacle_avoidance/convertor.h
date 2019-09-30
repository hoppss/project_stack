//
// Created by yonghui on 19-8-30.
//

#ifndef COORD_LOCAL_PLANNER_CONVERTOR_H
#define COORD_LOCAL_PLANNER_CONVERTOR_H

#include <vector>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

namespace obstacle_avoidance
{
    class Convertor
    {
    public:
        static double getYaw(const geometry_msgs::PoseStamped &pose);

        static double getYaw(const tf::Stamped<tf::Pose> &pose);

        static void setOrientation(geometry_msgs::PoseStamped &pose, double angle);
    };
}

#endif //COORD_LOCAL_PLANNER_CONVERTOR_H
