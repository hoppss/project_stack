//
// Created by yonghui on 19-9-25.
//

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Float32.h>
#include <angles/angles.h>
using namespace message_filters;

#include <eigen3/Eigen/Dense>
#include <vector>
#include <vector>
using namespace std;

#include "tic_toc.h"

class Odom2GPS
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry> OdomSyncPolicy;

    Odom2GPS();

    /**
     * @brief Odometry transform callback.
     * If less than wait_init_time_, add as initialize sample,
     * else if each the wait_init_time_, calculate the original point,
     * then, we will publish transformed odometry result.
     *
     * @param yaw_msg
     * @param gps_meas_msg
     */
    void odomCb(const nav_msgs::OdometryConstPtr &gps_msg,
                const nav_msgs::OdometryConstPtr &yaw_msg,
                const nav_msgs::OdometryConstPtr &odom_msg);

protected:
    /**
     * @brief Very stupid initialize method,
     * just use the mean of static samples as the original point
     */
    void setOrigin();

    void odom2gps(const nav_msgs::Odometry &yaw_msg, const nav_msgs::Odometry &odom_msg,
                  nav_msgs::Odometry &odom_gps_msg);

    /**
     * @brief Test function, to check if transformed data is normal
     * @param odom_msgs
     */
    void checkOdomDelta(const nav_msgs::Odometry &odom_msgs);

    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    ros::Publisher odom_gps_pub_;
//    ros::Subscriber imu_sync_sub_;
//    ros::Subscriber odom_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> gps_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> yaw_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    Synchronizer<OdomSyncPolicy > sync_;
    string global_frame_;
    string odom_frame_;
    bool use_debug_;

    // initialize control flag
    int init_count_;
    double wait_init_time_;
    bool init_complete_;
    TicToc tic_toc_;

    // initialize message
    vector<tf::Vector3> init_samples_;  // odom_x, odom_y, yaw
    tf::Vector3 init_pose_;  // odom_gps_x, odom_gps_y, yaw
    Eigen::Matrix3d Two_;

    // check delta
    ros::Publisher odom_x_pub_;
    ros::Publisher odom_y_pub_;
    ros::Publisher odom_yaw_pub_;
//    double last_odom_x_;
//    double last_odom_y_;
//    double last_odom_yaw_;
    tf::Vector3 last_pose_;
};


Odom2GPS::Odom2GPS() :
private_nh_("~"), nh_(),
gps_sub_(nh_, "gps_meas", 100),
yaw_sub_(nh_, "yaw_odom", 100),
odom_sub_(nh_, "odom", 100),
sync_(OdomSyncPolicy(10), gps_sub_, yaw_sub_, odom_sub_),
init_count_(0),
wait_init_time_(30),
init_complete_(false),
use_debug_(true),
last_pose_(-1, -1, -1)
{
    private_nh_.param("global_frame", global_frame_, string("world"));
    private_nh_.param("odom_frame", odom_frame_, string("odom"));
    private_nh_.param("wait_init_time", wait_init_time_, 30.0);
    private_nh_.param("use_debug", use_debug_, true);
    odom_gps_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_gps", 10);
    sync_.registerCallback(boost::bind(&Odom2GPS::odomCb, this, _1, _2, _3));

    // check delta
    odom_x_pub_ = nh_.advertise<std_msgs::Float32>("odom_gps/x", 10);
    odom_y_pub_ = nh_.advertise<std_msgs::Float32>("odom_gps/y", 10);
    odom_yaw_pub_ = nh_.advertise<std_msgs::Float32>("odom_gps/yaw", 10);
}


void Odom2GPS::odomCb(const nav_msgs::OdometryConstPtr &gps_msg,
                      const nav_msgs::OdometryConstPtr &yaw_msg,
                      const nav_msgs::OdometryConstPtr &odom_msg)
{
    // start the node
    if (init_count_==0)
        tic_toc_.tic();

    // check initializing complete
    double duration = tic_toc_.toc();
    if (!init_complete_ && duration > wait_init_time_)
    {
        init_complete_ = true;
        setOrigin();
        ROS_INFO("Odom transform initialization complete!");
        ROS_INFO("Odom initial pose: x=%12.4f, y=%12.4f, yaw=%7.4f", init_pose_[0], init_pose_[1], init_pose_[2]*180/M_PI);
        return;
    }

    // initializing...
    if (!init_complete_)
    {
//        double x = odom_msg->pose.pose.position.x;
//        double y = odom_msg->pose.pose.position.y;
        double x = gps_msg->pose.pose.position.x;
        double y = gps_msg->pose.pose.position.y;
        double yaw = -yaw_msg->twist.twist.angular.z * M_PI / 180.0 + M_PI_2;
        tf::Vector3 init_msgs(x, y, yaw);
        init_samples_.push_back(init_msgs);
        init_count_++;
        if (use_debug_)
            ROS_DEBUG("Initialize time=%9.4f, sample_x=%12.4f, sample_y=%12.4f, sample_yaw=%9.4f",
                      duration, x, y, yaw);
        return;
    }

    // initialize, publish gps odometry
    nav_msgs::Odometry odom_gps_msg;
    odom2gps(*yaw_msg, *odom_msg, odom_gps_msg);
    odom_gps_msg.header.frame_id = global_frame_;
    if (use_debug_)
    {
        double x_o = odom_gps_msg.pose.pose.position.x;
        double y_o = odom_gps_msg.pose.pose.position.y;
        double yaw_o = tf::getYaw(odom_gps_msg.pose.pose.orientation);
        ROS_DEBUG("Publish odom: odom_x=%9.4f, odom_y=%9.4f, odom_yaw=%9.4f", x_o, y_o, yaw_o*180/M_PI);
        ROS_DEBUG("Original data: odom_x=%9.4f, odom_y=%9.4f, odom_yaw=%9.4f",
                  odom_msg->pose.pose.position.x-init_pose_[0],
                  odom_msg->pose.pose.position.y-init_pose_[1],
                  yaw_msg->twist.twist.angular.z-90-init_pose_[2]*180/M_PI);
    }
    odom_gps_pub_.publish(odom_gps_msg);

    checkOdomDelta(odom_gps_msg);
}


void Odom2GPS::setOrigin()
{
    init_pose_.setZero();
    for (int i=0; i<init_samples_.size(); i++)
    {
//        init_pose_[0] += init_samples_[i][0];
//        init_pose_[1] += init_samples_[i][1];
        init_pose_[0] = 0.;
        init_pose_[1] = 0.;
        init_pose_[2] += init_samples_[i][2];  //! range [0, 360], avoid initializing facing to North!
    }
    init_pose_[0] /= init_samples_.size();
    init_pose_[1] /= init_samples_.size();
    init_pose_[2] /= init_samples_.size();
//    init_pose_[2] += M_PI_2;

    Eigen::Matrix3d Two;
    Two << cos(init_pose_[2]), -sin(init_pose_[2]), init_pose_[0],
            sin(init_pose_[2]),  cos(init_pose_[2]), init_pose_[1],
                           0.0,                 0.0,           1.0;
    Two_ = Two;
}


void Odom2GPS::odom2gps(const nav_msgs::Odometry &yaw_msg, const nav_msgs::Odometry &odom_msg,
                        nav_msgs::Odometry &odom_gps_msg)
{
    double x = odom_msg.pose.pose.position.x;
    double y = odom_msg.pose.pose.position.y;
    double yaw = tf::getYaw(odom_msg.pose.pose.orientation);
    Eigen::Matrix3d Twi;
    Twi << cos(yaw), -sin(yaw),   x,
           sin(yaw),  cos(yaw),   y,
                0.0,       0.0, 1.0;
    Eigen::Matrix3d Toi = Two_ * Twi;
    double x_o = Toi(0, 2);
    double y_o = Toi(1, 2);
    double yaw_o = atan2(Toi(1,0), Toi(0,0));
    odom_gps_msg = odom_msg;
    odom_gps_msg.header.frame_id = global_frame_;
    odom_gps_msg.pose.pose.position.x = x_o;
    odom_gps_msg.pose.pose.position.y = y_o;
    odom_gps_msg.pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw_o);
    odom_gps_msg.pose.pose.orientation = q;

}


void Odom2GPS::checkOdomDelta(const nav_msgs::Odometry &odom_msgs)
{
    // extract current data
    double curr_x = odom_msgs.pose.pose.position.x;
    double curr_y = odom_msgs.pose.pose.position.y;
    double curr_yaw = tf::getYaw(odom_msgs.pose.pose.orientation);

    // publish transformed odom data
    std_msgs::Float32 rel_odom_x, rel_odom_y, rel_odom_yaw;
    rel_odom_x.data = curr_x - init_pose_[0];
    rel_odom_y.data = curr_y - init_pose_[1];
    rel_odom_yaw.data = angles::normalize_angle(curr_yaw - init_pose_[2]);
    odom_x_pub_.publish(rel_odom_x);
    odom_y_pub_.publish(rel_odom_y);
    odom_yaw_pub_.publish(rel_odom_yaw);

    // if last odom data exist
    if (last_pose_[0] < 0 || last_pose_[1] < 0)
    {
        last_pose_[0] = curr_x;
        last_pose_[1] = curr_y;
        last_pose_[2] = curr_yaw;
        return;
    }

    double dx = fabs(curr_x - last_pose_[0]);
    double dy = fabs(curr_y - last_pose_[1]);
    double dyaw = angles::normalize_angle(curr_yaw - last_pose_[2]);
    if (dx > 1)
        ROS_ERROR("ODOM X axis suffers from dramatic change: dx=%7.4f", dx);
    if (dy > 1)
        ROS_ERROR("ODOM Y axis suffers from dramatic change: dy=%7.4f", dy);
    if (dx > 1 || dy > 1)
    {
        ROS_ERROR("Problem last pose: x=%7.4f, y=%7.4f, yaw=%7.4f",
                last_pose_[0],
                last_pose_[1],
                last_pose_[2]);
        ROS_ERROR("Problem pose: x=%7.4f, y=%7.4f, yaw=%7.4f",
                curr_x,
                curr_y,
                curr_yaw);
    }

    // update last odom
    last_pose_[0] = curr_x;
    last_pose_[1] = curr_y;
    last_pose_[2] = curr_yaw;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_odom_transform");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    Odom2GPS got;
    ros::spin();
}