//
// Created by yonghui on 19-5-20.
//

#ifndef CCD_CAMERA_CCD_CAMERA_NAV_H
#define CCD_CAMERA_CCD_CAMERA_NAV_H

#include <mutex>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include "ccd_camera/pid_Config.h"
#include "controller.h"

class CCDCameraNav
{
public:
    CCDCameraNav();
    ~CCDCameraNav();
    void NavCmdVelCycle();

protected:
    void NavCmdVellCallBack(const geometry_msgs::Twist::ConstPtr &msgNavCmdVel);
    void StartGoalFlagCallBack(const std_msgs::Bool::ConstPtr &bStartGoal);
    //void reconfigCallBack(ccd_camera::pid_Config &config, uint32_t level);

    ros::NodeHandle nh_;
    Controller clt_;
    CCDCameraSerial serial_;
    ros::Subscriber subNavCmdVel_;
    ros::Publisher pubCltCmdVel_;
    ros::Publisher pubCurrPosErr_;
    std::mutex mMutexGoalStart_;
    int mnMidPos_;
    float mfMaxAngle_;
    bool mbStartGoal_;
    bool mbRequestStop_;
};

#endif //CCD_CAMERA_CCD_CAMERA_NAV_H
