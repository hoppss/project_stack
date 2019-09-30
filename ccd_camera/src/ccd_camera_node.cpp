//
// Created by yonghui on 19-4-19.
//

#include <cmath>
#include <string>
#include <ros/ros.h>
#include <ros/param.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include "ccd_camera_serial.h"
#include "ccd_camera_nav.h"
#include "controller.h"
using namespace std;

void simple_test()
{
//    ros::init(argc, argv, "ccd_camera_node");
    // change log level to debug
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::NodeHandle nh;
    string strPortName;
    int nMidPos;
    float fKp;
    float fKi;
    float fKd;
    float fXCmdVel;
    float fMaxAngle;
    ros::param::get("~port", strPortName);
    ros::param::get("~mid_pos", nMidPos);
    ros::param::get("~Kp", fKp);
    ros::param::get("~Ki", fKi);
    ros::param::get("~Kd", fKd);
    ros::param::get("~linear_x", fXCmdVel);
    ros::param::get("~max_angle", fMaxAngle);
    CCDCameraSerial serialler(strPortName.c_str());
    if (!serialler.isPortOpen())
    {
        return;
    }
    Controller clt(fKp, fKi, fKd, fXCmdVel, fMaxAngle, -fMaxAngle);
//    dynamic_reconfigure::Server<ccd_camera::pid_Config> server;
//    dynamic_reconfigure::Server<ccd_camera::pid_Config>::CallbackType f;
//    f = boost::bind(&Controller::reconfigCallBack, &clt, _1, _2);
//    server.setCallback(f);
    ros::Publisher pubCmdVel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Publisher pubCurrPosErr = nh.advertise<std_msgs::Int32>("curr_pos_err", 10);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        int nCurrPos = serialler.recv_buffer();
		if (nCurrPos >= 0)  // 0 and 128 are abnormal detect
		{
			float fYCmdVelAngle = clt.output(nCurrPos - nMidPos);
        	// ROS_INFO("Current cmd vel Y: %f", fYCmdVelAngle);
			ROS_INFO("middle: %4d | position: %4d\n", nMidPos, nCurrPos);
			geometry_msgs::Twist cmd_vel;
        	cmd_vel.linear.x = clt.mfCmdLinearX;
        	cmd_vel.linear.y = 0.;
        	cmd_vel.linear.z = 0;
        	cmd_vel.angular.x = 0;
        	cmd_vel.angular.y = 0;
        	cmd_vel.angular.z = max(-fMaxAngle, min(fMaxAngle, -fYCmdVelAngle));
        	pubCmdVel.publish(cmd_vel);
        	std_msgs::Int32 currPosErrMsg;
        	currPosErrMsg.data = (nCurrPos - nMidPos);
        	pubCurrPosErr.publish(currPosErrMsg);
		}
        ros::spinOnce();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ccd_camera_node");
    bool bNav;
    ros::param::get("~use_nav", bNav);
    if (bNav)
    {
        simple_test();
    }
    else
    {
        CCDCameraNav nav;
        nav.NavCmdVelCycle();
//        ros::spin();
    }
    return 0;
}
