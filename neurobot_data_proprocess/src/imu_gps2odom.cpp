//
// Created by yonghui on 19-9-25.
//

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace message_filters;

#include <eigen3/Eigen/Dense>
#include <vector>
#include <vector>
using namespace std;

#include "tic_toc.h"

class ImuGps2Odom
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry> OdomSyncPolicy;

    ImuGps2Odom(tf::TransformListener &tf);

protected:
    /**
     * @brief GPS odometry transform callback.
     * If less than wait_init_time_, add as initialize sample,
     * else if each the wait_init_time_, calculate the original point,
     * then, we will publish transformed odometry result.
     *
     * @param imu_msg
     * @param gps_meas_msg
     */
    void gpsOdomTransformCb(const sensor_msgs::ImuConstPtr &imu_msg, const nav_msgs::OdometryConstPtr &gps_meas_msg);

    /**
     * @brief debug callback, publish imu relative yaw
     *
     * @param imu_msg
     */
    void imuRelYawCb(const sensor_msgs::ImuConstPtr &imu_msg);

    /**
     * @brief Very stupid initialize method,
     * just use the mean of static samples as the original point
     */
    void setOrigin();

    /**
     * @brief Tranform the direction of imu to gps frame
     *
     * @param imu_msg
     * @param yaw
     * @return
     */
    bool getGpsFrameYaw(const sensor_msgs::ImuConstPtr &imu_msg, double &yaw);

    void gps2odom(const double &yaw, const nav_msgs::Odometry &gps_meas_msg, nav_msgs::Odometry &odom_msg);

    tf::TransformListener &tf_;
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    ros::Subscriber imu_yaw_sub_;
    ros::Publisher imu_yaw_pub_;
    ros::Publisher gps_odom_pub_;
//    ros::Subscriber imu_sync_sub_;
//    ros::Subscriber gps_meas_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sync_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> gps_meas_sub_;
    Synchronizer<OdomSyncPolicy > sync_;

    string global_frame_;
    string base_frame_;
    string imu_frame_;
    string gps_frame_;

    bool use_debug_;

    // initialize control flag
    int init_count_;
    double wait_init_time_;
    bool init_complete_;
    TicToc tic_toc_;

    // initialize message
    vector<tf::Vector3> init_samples_;  // gps_x, gps_y, yaw
    tf::Vector3 init_pose_;  // gps_x, gps_y, yaw
    Eigen::Matrix3d Tow_;
};


ImuGps2Odom::ImuGps2Odom(tf::TransformListener &tf) :
tf_(tf),
private_nh_("~"), nh_(),
imu_sync_sub_(nh_, "imu_data", 10),
gps_meas_sub_(nh_, "gps_meas", 10),
sync_(OdomSyncPolicy(100), imu_sync_sub_, gps_meas_sub_),
init_count_(0),
wait_init_time_(30),
init_complete_(false)
{
    private_nh_.param("global_frame", global_frame_, string("map"));
    private_nh_.param("base_frame", base_frame_, string("base_link"));
    private_nh_.param("imu_frame", imu_frame_, string("imu"));
    private_nh_.param("gps_frame", gps_frame_, string("vo"));
    private_nh_.param("wait_init_time", wait_init_time_, 30.0);
    private_nh_.param("use_debug", use_debug_, true);
    gps_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_gps", 10);
    sync_.registerCallback(boost::bind(&ImuGps2Odom::gpsOdomTransformCb, this, _1, _2));
    if (use_debug_)
    {
        ROS_INFO("Publish imu relative yaw information");
        imu_yaw_sub_ = nh_.subscribe("imu_data", 100, &ImuGps2Odom::imuRelYawCb, this);
        imu_yaw_pub_ = private_nh_.advertise<std_msgs::Float32>("rel_yaw", 100);
    }
}


void ImuGps2Odom::gpsOdomTransformCb(const sensor_msgs::ImuConstPtr &imu_msg,
                                          const nav_msgs::OdometryConstPtr &gps_meas_msg)
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
        ROS_INFO("GPS odom transform initialization complete!");
        ROS_INFO("GPS initial pose: x=%12.4f, y=%12.4f, yaw=%7.4f", init_pose_[0], init_pose_[1], init_pose_[2]*180/M_PI);
    }

    double yaw = 0.0;
    if (!getGpsFrameYaw(imu_msg, yaw))
        return;

    // initializing...
    if (!init_complete_)
    {
        double x = gps_meas_msg->pose.pose.position.x;
        double y = gps_meas_msg->pose.pose.position.y;
        tf::Vector3 init_msgs(x, y, yaw);
        init_samples_.push_back(init_msgs);
        init_count_++;
        if (use_debug_)
        {
//            ROS_DEBUG("Initialize time=%9.4f, sample_x=%12.4f, sample_y=%12.4f, sample_yaw=%9.4f",
//                      duration, x, y, yaw*180/M_PI);
            double r, p, y;
            tf::Quaternion q;
            tf::quaternionMsgToTF(imu_msg->orientation, q);
            tf::Matrix3x3(q).getRPY(r, p, y);
            ROS_DEBUG("Initialize time=%9.4f, sample_roll=%9.4f, sample_pitch=%9.4f, sample_yaw=%9.4f",
                      duration, r, p, y);
        }

        return;
    }

    // initialize, publish gps odometry
    nav_msgs::Odometry odom_msg;
//    gps2odom(yaw, *gps_meas_msg, odom_msg);
    gps2odom(yaw, *gps_meas_msg, odom_msg);
    odom_msg.header.frame_id = global_frame_;
    if (use_debug_)
    {
        double x_o = odom_msg.pose.pose.position.x;
        double y_o = odom_msg.pose.pose.position.y;
        double yaw_o = tf::getYaw(odom_msg.pose.pose.orientation);
        ROS_DEBUG("Publish odom: odom_x=%9.4f, odom_y=%9.4f, odom_yaw=%9.4f", x_o, y_o, yaw_o*180/M_PI);
        ROS_DEBUG("Original data: odom_x=%9.4f, odom_y=%9.4f, odom_yaw=%9.4f",
                  gps_meas_msg->pose.pose.position.x-init_pose_[0],
                  gps_meas_msg->pose.pose.position.y-init_pose_[1],
                  (yaw-init_pose_[2])*180/M_PI);
    }
    gps_odom_pub_.publish(odom_msg);
}


void ImuGps2Odom::imuRelYawCb(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (!init_complete_)
        return;

    double yaw;
    if (!getGpsFrameYaw(imu_msg, yaw))
        return;

    std_msgs::Float32 rel_yaw_msg;
    rel_yaw_msg.data = (yaw-init_pose_[2])*180/M_PI;
    imu_yaw_pub_.publish(rel_yaw_msg);
}


void ImuGps2Odom::setOrigin()
{
    init_pose_.setZero();
    for (int i=0; i<init_samples_.size(); i++)
    {
        init_pose_[0] += init_samples_[i][0];
        init_pose_[1] += init_samples_[i][1];
        init_pose_[2] += init_samples_[i][2];  //! range [0, 360], avoid initializing facing to North!
    }
    init_pose_[0] /= init_samples_.size();
    init_pose_[1] /= init_samples_.size();
    init_pose_[2] /= init_samples_.size();
//    init_pose_[2] += M_PI_2;

    Eigen::Matrix3d Two_;
    Two_ << cos(init_pose_[2]), -sin(init_pose_[2]), init_pose_[0],
            sin(init_pose_[2]),  cos(init_pose_[2]), init_pose_[1],
                           0.0,                 0.0,           1.0;
    Tow_ = Two_.inverse();
}


bool ImuGps2Odom::getGpsFrameYaw(const sensor_msgs::ImuConstPtr &imu_msg, double &yaw)
{
    // transform imu direction into gps frame
    geometry_msgs::QuaternionStamped q_imu, q_base_imu;
    q_imu.header.stamp = imu_msg->header.stamp;
    q_imu.header.frame_id = imu_msg->header.frame_id;
    q_imu.quaternion = imu_msg->orientation;
//    try
//    {
//        tf_.transformQuaternion(gps_frame_, q_imu, q_base_imu);
//    }
//    catch (tf::TransformException &ex)
//    {
//        ROS_ERROR("Fail to transform the direction of imu into %s frame: %s", gps_frame_.c_str(), ex.what());
//        return false;
//    }
    q_base_imu = q_imu;

    // get gps frame yaw direction
    yaw = tf::getYaw(q_base_imu.quaternion)-M_PI*1.25;
    return true;
}


void ImuGps2Odom::gps2odom(const double &yaw, const nav_msgs::Odometry &gps_meas_msg, nav_msgs::Odometry &odom_msg)
{
    double x = gps_meas_msg.pose.pose.position.x;
    double y = gps_meas_msg.pose.pose.position.y;
    double yaw_odom = tf::getYaw(gps_meas_msg.pose.pose.orientation);

    Eigen::Matrix3d Twi;
    Twi << cos(yaw_odom), -sin(yaw_odom),   x,
           sin(yaw_odom),  cos(yaw_odom),   y,
                0.0,       0.0, 1.0;
    Eigen::Matrix3d Toi = Tow_ * Twi;
    double x_o = Toi(0, 2);
    double y_o = Toi(1, 2);
    double yaw_o = atan2(Toi(1,0), Toi(0,0));
    odom_msg = gps_meas_msg;
    odom_msg.pose.pose.position.x = x_o;
    odom_msg.pose.pose.position.y = y_o;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_o);
    odom_msg.pose.covariance.data()[0] = 0.01;
    odom_msg.pose.covariance.data()[7] = 0.01;
    odom_msg.pose.covariance.data()[15] = 99999.0;
    odom_msg.pose.covariance.data()[35] = 0.01;
    if (use_debug_)
    {
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw_o);  // TODO: Change
        odom_msg.pose.pose.orientation = q;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_odom_transform");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    tf::TransformListener tf;
    ImuGps2Odom got(tf);
    ros::spin();
}