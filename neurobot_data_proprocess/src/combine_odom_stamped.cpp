//
// Created by yonghui on 19-9-25.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <string>
#include <std_msgs/Float32.h>

using namespace std;

class CombineOdomStamped
{
public:
    CombineOdomStamped();

    void combineOdomCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

    /**
     * @brief This publish topic is only for visualize gps trajectory
     * @param msg
     */
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
    ros::Publisher gps_x_pub_;
    ros::Publisher gps_y_pub_;
    string ekf_name_;
    string global_frame_;
    bool init_combine_;
    double init_x_;
    double init_y_;
    bool init_gps_;
    double init_gps_x_;
    double init_gps_y_;

    double last_ekf_x_;
    double last_ekf_y_;
    double last_gps_x_;
    double last_gps_y_;
};

CombineOdomStamped::CombineOdomStamped() :
private_nh_("~"), init_combine_(true), gps_nh_("gps_meas"),
init_x_(0.0), init_y_(0.0), init_gps_(true),
init_gps_x_(0.0), init_gps_y_(0.0),
last_ekf_x_(-1), last_ekf_y_(-1),
last_gps_x_(-1), last_gps_y_(-1)
{
    private_nh_.param("ekf_node_name", ekf_name_, string("robot_pose_ekf"));
    private_nh_.param("global_frame", global_frame_, string("world"));
    ekf_nh_ = ros::NodeHandle(ekf_name_);
    combine_odom_pub_ = ekf_nh_.advertise<nav_msgs::Odometry>("odom", 10);
    combine_odom_sub_ = ekf_nh_.subscribe("odom_combined", 10, &CombineOdomStamped::combineOdomCb, this);
    gps_odom_pub_ = gps_nh_.advertise<nav_msgs::Odometry>("odom", 10);
    gps_meas_sub_ = nh_.subscribe("gps_meas", 10, &CombineOdomStamped::gpsMeasCb, this);
    gps_x_pub_ = gps_nh_.advertise<std_msgs::Float32>("relative_x", 10);
    gps_y_pub_ = gps_nh_.advertise<std_msgs::Float32>("relative_y", 10);
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
//    odom_msgs.pose.pose.position.x -= init_x_;
//    odom_msgs.pose.pose.position.y -= init_y_;
    combine_odom_pub_.publish(odom_msgs);

    // check delta
    if (last_ekf_x_ > 0 && last_ekf_y_ > 0)
    {
        double dx = fabs(odom_msgs.pose.pose.position.x - last_ekf_x_);
        double dy = fabs(odom_msgs.pose.pose.position.y - last_ekf_y_);
        if (dx > 2)
            ROS_ERROR("EKF X axis suffers from dramatic change! dx=%7.4f", dx);
        if (dy > 2)
            ROS_ERROR("EKF Y axis suffers from dramatic change! dy=%7.4f", dy);
    }
    last_ekf_x_ = odom_msgs.pose.pose.position.x;
    last_ekf_y_ = odom_msgs.pose.pose.position.y;

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
//    odom_msg.pose.pose.position.x -= init_gps_x_;
//    odom_msg.pose.pose.position.y -= init_gps_y_;
    gps_odom_pub_.publish(odom_msg);

    // check delta
    if (last_gps_x_ > 0 && last_gps_y_ > 0)
    {
        if (fabs(msg->pose.pose.position.x-last_gps_x_) > 2)
            ROS_ERROR("GPS X axis suffers from dramatic change!");
        if (fabs(msg->pose.pose.position.y-last_gps_y_) > 2)
            ROS_ERROR("GPS Y axis suffers from dramatic change!");
    }
    last_gps_x_ = msg->pose.pose.position.x;
    last_gps_y_ = msg->pose.pose.position.y;
    std_msgs::Float32 rel_x, rel_y;
    rel_x.data = msg->pose.pose.position.x - init_gps_x_;
    rel_y.data = msg->pose.pose.position.y - init_gps_y_;
    gps_x_pub_.publish(rel_x);
    gps_y_pub_.publish(rel_y);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "combine_odom_stamped");
    CombineOdomStamped cos;
    ros::spin();
}