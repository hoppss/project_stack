//
// Created by yonghui on 19-5-20.
//
#include <cmath>
#include <ros/param.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
#include "ccd_camera/ccd_camera_serial.h"
#include "ccd_camera/ccd_camera_nav.h"
#include "../include/ccd_camera/ccd_camera_nav.h"

using namespace std;

CCDCameraNav::CCDCameraNav():
mbStartGoal_(false), mbRequestStop_(false)
{
    // get params
    string strPortName;
    float fKp;
    float fKi;
    float fKd;
    float fXCmdVel;
    float fMaxAngle;
    ros::NodeHandle private_nh_("~");
    private_nh_.param("port", strPortName, std::string("/dev/ccd"));
    private_nh_.param("mid_pos", mnMidPos_, 64);
    private_nh_.param("Kp", fKp, 0.0f);
    private_nh_.param("Ki", fKi, 0.0f);
    private_nh_.param("Kd", fKd, 0.0f);
    private_nh_.param("linear_x", fXCmdVel, 0.1f);
    private_nh_.param("max_angle", mfMaxAngle_, 0.5f);
//    ros::param::get("~port", strPortName);
//    ros::param::get("~mid_pos", mnMidPos_);
//    ros::param::get("~Kp", fKp);
//    ros::param::get("~Ki", fKi);
//    ros::param::get("~Kd", fKd);
//    ros::param::get("~linear_x", fXCmdVel);  // will it be used?
//    ros::param::get("~max_angle", mfMaxAngle_);

    // initialize ccd serial_ and PID controller
    ROS_ERROR("Kp: %f, Ki: %f, Kd: %f", fKp, fKi, fKd);
    serial_.initialize(strPortName.c_str());
    clt_.updateParams(fKp, fKi, fKd, fXCmdVel, mfMaxAngle_, -mfMaxAngle_);

    // publish and subscribe
    pubCltCmdVel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pubCurrPosErr_ = nh_.advertise<std_msgs::Int32>("curr_pos_err", 1);
//    subNavCmdVel_ = nh_.subscribe<geometry_msgs::Twist>("nav_cmd_vel", 1, &CCDCameraNav::NavCmdVellCallBack, this);
    subNavCmdVel_ = nh_.subscribe<std_msgs::Bool>("nav_cmd_vel", 1, &CCDCameraNav::StartGoalFlagCallBack, this);

    // TODO: reconfigure
//    dynamic_reconfigure::Server<ccd_camera::pid_Config> server;
//    dynamic_reconfigure::Server<ccd_camera::pid_Config>::CallbackType f;
//    f = boost::bind(&Controller::reconfigCallBack, &clt, _1, _2);
//    server.setCallback(f);
}


CCDCameraNav::~CCDCameraNav()
{
}



void CCDCameraNav::NavCmdVellCallBack(const geometry_msgs::Twist::ConstPtr &msgNavCmdVel)
{
    if (msgNavCmdVel->linear.x==0 && msgNavCmdVel->linear.y==0 && msgNavCmdVel->angular.z==0)
    {
        ROS_INFO("zero cmd vel!");
        pubCltCmdVel_.publish(msgNavCmdVel);
    }
    // ccd camera
    //! ccd port is not open
    if (!serial_.isPortOpen())
    {
        ROS_WARN("CCD camera serial_ open fail...");
//        std_msgs::Int32 msgCurrPosErr;
//        msgCurrPosErr.data = 0;
//        pubCurrPosErr_.publish(msgCurrPosErr);
//        pubCltCmdVel_.publish(msgNavCmdVel);
        return;
    }
    float fNavLinearX = msgNavCmdVel->linear.x;
    float fNavAngleZ = msgNavCmdVel->angular.z;
    // TODO: better solution judge obstacle
//    if (fNavLinearX > 0.05)
    {
        int nCurrPos = serial_.recv_buffer();
        if (nCurrPos >= 0)
        {
            ROS_WARN("middle: %4d | position: %4d\n", mnMidPos_, nCurrPos);
            //! calculate PID angle adjustment
            float fCmdAngle = clt_.output(nCurrPos - mnMidPos_);
            geometry_msgs::Twist msgCltCmdVel;
            msgCltCmdVel.linear.x = clt_.mfCmdLinearX;  // TODO: linear x should decrease if error is large
            msgCltCmdVel.linear.y = 0.;
            msgCltCmdVel.linear.z = 0.;
            msgCltCmdVel.angular.x = 0;
            msgCltCmdVel.angular.y = 0.;
            msgCltCmdVel.angular.z = - fCmdAngle;
            //! publish position error and control
            std_msgs::Int32 msgCurrPosErr;
            msgCurrPosErr.data = (nCurrPos - mnMidPos_);
            pubCurrPosErr_.publish(msgCurrPosErr);
            pubCltCmdVel_.publish(msgCltCmdVel);
        }
        else
        {
            ROS_WARN("something wrong with CCD output: %4d\n", nCurrPos);

        }
    }
}


void CCDCameraNav::StartGoalFlagCallBack(const std_msgs::Bool::ConstPtr &bStartGoal)
{
    mbStartGoal_ = bStartGoal->data;
    //! send stop request
    if (!mbStartGoal_)
    {
        mbRequestStop_ = true;
    }
    ROS_ERROR("Change start goal status: %d", int(mbStartGoal_));
}


void CCDCameraNav::NavCmdVelCycle()
{
    ros::Rate r(20);
    while (ros::ok())
    {
        if (mbStartGoal_ && !mbRequestStop_)
        {
            int nCurrPos = serial_.recv_buffer();
            if (nCurrPos >= 0)  // 0 and 128 are abnormal detect
            {
                float fYCmdVelAngle = clt_.output(nCurrPos - mnMidPos_);
                // ROS_INFO("Current cmd vel Y: %f", fYCmdVelAngle);
                ROS_INFO("middle: %4d | position: %4d\n", mnMidPos_, nCurrPos);
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = clt_.mfCmdLinearX;
                cmd_vel.linear.y = 0.;
                cmd_vel.linear.z = 0;
                cmd_vel.angular.x = 0;
                cmd_vel.angular.y = 0;
                cmd_vel.angular.z = max(-mfMaxAngle_, min(mfMaxAngle_, -fYCmdVelAngle));
                pubCltCmdVel_.publish(cmd_vel);
                std_msgs::Int32 currPosErrMsg;
                currPosErrMsg.data = (nCurrPos - mnMidPos_);
                pubCurrPosErr_.publish(currPosErrMsg);
            }
        }
        //! deal with stop request
        else if (mbRequestStop_)
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.z = 0;
            pubCltCmdVel_.publish(cmd_vel);
            mbRequestStop_ = false;
        }
        ros::spinOnce();
    }
}


/*
void CCDCameraNav::reconfigCallBack(ccd_camera::pid_Config &config, uint32_t level)
{
    ros::param::set("~Kp", config.Kp);
    ros::param::set("~Ki", config.Ki);
    ros::param::set("~Kd", config.Kd);
    ros::param::set("~linear_x", config.LinearX);
    ROS_WARN("Kp: %f, Ki %f, Kd %f, LinearX %f", config.Kp, config.Ki, config.Kd, config.LinearX);
    clt_.updateParams(config.Kp, config.Ki, config.Kd, config.LinearX, clt_.mfErrSumMax, clt_.mfErrSumMin);
}
*/
