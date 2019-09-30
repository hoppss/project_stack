//
// Created by yonghui on 17-10-16.
//

#ifndef PROJECT_CONTROLLER_H
#define PROJECT_CONTROLLER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
//#include <tf/transform_listener.h>

namespace ccd_camera
{
    class Controller
    {
    public:
        Controller() = default;
        Controller(float fkp, float fKi, float fKd, float fLinearX, float fErrSumMax=0.5, float fErrSumMin=-0.5);
//    void reconfigCallBack(ccd_camera::pid_Config &config, uint32_t level);
        void updateParams(float fkp, float fKi, float fKd, float fLinearX, float fErrSumMax=0.5, float fErrSumMin=-0.5);
        float output(float fError);
//    void setKp(const float &fKp);
//    void setKi(const float &fKi);
//    void setKd(const float &fKd);
//    void setCmdLinearX(const float &fCmdLinearX);
//    float getCmdLinearX();

    public:
        float mfKp;
        float mfKi;
        float mfKd;
        float mfCmdLinearX;
//    float mfErrLast1;
//    float mfErrLast2;
//    float mfOutputLast;
        float mfErrSumMax;
        float mfErrSumMin;
        float mfErrSum;
        float mfErrLast;
    };
}


#endif //PROJECT_CONTROLLER_H
