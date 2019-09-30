//
// Created by yonghui on 19-9-25.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <string>
using namespace std;

class CombineOdomStamped
{
public:
    CombineOdomStamped();

    void combineOdomCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

    void gpsMeasCb(const nav_msgs::OdometryConstPtr &msg);

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::NodeHandle ekf_nh_;
    ros::NodeHandle gps_nh_;
    ros::Publisher combine_odom_pub_;
    ros::Subscriber combine_odom_sub_;
    ros::Publisher gps_odom_pub_;
    ros::Subscriber gps_meas_sub_;
    string ekf_name_;
    string global_frame_;
    bool init_combine_;
    double init_x_;
    double init_y_;
    bool init_gps_;
    double init_gps_x_;
    double init_gps_y_;
};

CombineOdomStamped::CombineOdomStamped() :
private_nh_("~"), init_combine_(true), gps_nh_("gps_meas"), init_x_(0.0), init_y_(0.0), init_gps_(true), init_gps_x_(0.0), init_gps_y_(0.0)
{
    private_nh_.param("ekf_node_name", ekf_name_, string("robot_pose_ekf"));
    private_nh_.param("global_frame", global_frame_, string("odom"));
    ekf_nh_ = ros::NodeHandle(ekf_name_);
    combine_odom_pub_ = ekf_nh_.advertise<nav_msgs::Odometry>("odom", 10);
    combine_odom_sub_ = ekf_nh_.subscribe("odom_combined", 10, &CombineOdomStamped::combineOdomCb, this);
    gps_odom_pub_ = gps_nh_.advertise<nav_msgs::Odometry>("odom", 10);
    gps_meas_sub_ = nh_.subscribe("gps_meas", 10, &CombineOdomStamped::gpsMeasCb, this);
}


void CombineOdomStamped::combineOdomCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    nav_msgs::Odometry odom_msgs;
    odom_msgs.header = msg->header;
    odom_msgs.pose = msg->pose;
    odom_msgs.twist.twist.linear.x = 0.0;
    odom_msgs.twist.twist.linear.y = 0.0;
    odom_msgs.twist.twist.linear.z = 0.0;
    odom_msgs.twist.twist.angular.x = 0.0;
    odom_msgs.twist.twist.angular.y = 0.0;
    odom_msgs.twist.twist.angular.z = 0.0;
    if (init_combine_)
    {
        init_x_ = odom_msgs.pose.pose.position.x;
        init_y_ = odom_msgs.pose.pose.position.y;
        init_combine_ = false;
    }
    odom_msgs.pose.pose.position.x -= init_x_;
    odom_msgs.pose.pose.position.y -= init_y_;
    combine_odom_pub_.publish(odom_msgs);
}


void CombineOdomStamped::gpsMeasCb(const nav_msgs::OdometryConstPtr &msg)
{
    nav_msgs::Odometry odom_msg = *msg;
    if (init_gps_)
    {
        init_gps_x_ = odom_msg.pose.pose.position.x;
        init_gps_y_ = odom_msg.pose.pose.position.y;
        init_gps_ = false;
    }
    odom_msg.header.frame_id = global_frame_;
    odom_msg.pose.pose.position.x -= init_gps_x_;
    odom_msg.pose.pose.position.y -= init_gps_y_;
    gps_odom_pub_.publish(odom_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "combine_odom_stamped");
    CombineOdomStamped cos;
    ros::spin();
}