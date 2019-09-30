//
// Created by yonghui on 19-9-16.
//

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <vector>
using namespace std;

class StopController
{
public:
    enum ControlState{FORWARD, SLOWDOWN, STOP};

    /**
     * @brief construct function
     */
    StopController();

    /**
     * @brief subscribe /scan topic, detect obstacle forward
     * for message like turtlebot3, whose angle ranges from 0 to 2 pi,
     * we need to change the angle range and resort vectors
     *
     * @param scan_msgs
     */
    void laserScanCb(const sensor_msgs::LaserScanConstPtr &scan_msgs);

    /**
     * @brief publish twist message in one control step, three state:
     * forward, slow down, stop
     *
     */
    void controlOnce();

    /**
     * @brief Get control rate
     *
     * @return
     */
    int getControlRate();

protected:
    /**
     * @brief angle of scan message returned by turtlebot3 ranging from 0 to 2pi,
     * we should change it to [-pi, pi]
     *
     * @param scan_msgs
     * @param new_msgs
     * @return
     */
    bool prepareScanMsg(const sensor_msgs::LaserScanConstPtr &scan_msgs, sensor_msgs::LaserScan &new_msgs);

    /**
     * @brief get the index of target angle in scan vector
     *
     * @param scan_msgs
     * @param target_angle
     * @return
     */
    int getScanRangesIndex(const sensor_msgs::LaserScan &scan_msgs, double target_angle);

    /**
     * @brief publish twist message
     *
     * @param linear_vel
     * @param angular_vel
     */
    void publishCmdVel(double linear_vel=0.0, double angular_vel=0.0);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // pub and sub
    ros::Publisher vel_pub_;
    ros::Publisher prepared_scan_pub_;
    ros::Subscriber scan_sub_;

    // obstacle detect params
    double base_radius_;  ///<@brief base radius
    double slow_dis_;  ///<@brief start slow down distance before obstacle
    double stop_dis_;  ///<@brief immediately stop distance before obstacle
    double slow_detect_angle_;  ///<@brief slow down detect upper angle range
    double stop_detect_angle_;  ///<@brief stop detect upper angle range

    // velocity slope
    double linear_vel_up_;  ///<@brief max linear vel
    double linear_vel_low_;  ///<@brief min linear vel
    double delta_linear_vel_;  ///<@brief linear vel slow down step
    double slow_down_time_;  ///<@brief slow down slope time
    int control_rate_;  ///<@brief control rate

    // publish vel
    double linear_vel_;  ///< @brief publish linear vel
    double angular_vel_;  ///<@brief publish angular vel

    ControlState control_state_;  ///<@brief control flag, slow down or stop if detect obstacle forward
};


StopController::StopController() :
nh_(), private_nh_("~"),
base_radius_(0.0), slow_dis_(0.0), stop_dis_(0.0),
linear_vel_up_(0.0), linear_vel_low_(0.0), delta_linear_vel_(0.0), slow_down_time_(0.0),
control_rate_(0), linear_vel_(0.0), angular_vel_(0.0), control_state_(FORWARD)
{
    private_nh_.param<double>("base_radius", base_radius_, 0.5);
    private_nh_.param<double>("slow_dis", slow_dis_, 2.5);
    private_nh_.param<double>("stop_dis", stop_dis_, 1.0);
    private_nh_.param<double>("linear_vel_up", linear_vel_up_, 0.1);
    private_nh_.param<double>("linear_vel_low", linear_vel_low_, 0.01);
    private_nh_.param<double>("slow_down_time", slow_down_time_, 0.5);
    private_nh_.param<int>("control_rate", control_rate_, 20);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    prepared_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("prepared_scan", 10);
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 10, &StopController::laserScanCb, this);

    // calculate detect range
    slow_detect_angle_ = atan2(base_radius_, slow_dis_);
    stop_detect_angle_ = atan2(base_radius_, stop_dis_);
    ROS_DEBUG_STREAM("slow detect upper range=" << slow_detect_angle_ << ", stop detect upper range=" << stop_detect_angle_);

    // calculate linear change step
    delta_linear_vel_ = (linear_vel_up_ - linear_vel_low_) * slow_down_time_ / control_rate_;
    ROS_DEBUG_STREAM("delta linear vel = " << delta_linear_vel_);
}


void StopController::laserScanCb(const sensor_msgs::LaserScanConstPtr &scan_msgs)
{
    ROS_INFO("---");
    if (stop_dis_ > slow_dis_)
    {
        ROS_ERROR("stop distance is smaller than slow distance");
        control_state_ = STOP;
        return;
    }

    sensor_msgs::LaserScan prepared_msgs;
    if (prepareScanMsg(scan_msgs, prepared_msgs))
    {
        // TODO: LOG
        ROS_DEBUG("prepare scan message");
        prepared_scan_pub_.publish(prepared_msgs);
    }

    const vector<float> &scan_ranges = prepared_msgs.ranges;
    int slow_idx_st = getScanRangesIndex(prepared_msgs, -slow_detect_angle_);
    int slow_idx_ed = getScanRangesIndex(prepared_msgs, slow_detect_angle_);
    int stop_idx_st = getScanRangesIndex(prepared_msgs, -stop_detect_angle_);
    int stop_idx_ed = getScanRangesIndex(prepared_msgs, stop_detect_angle_);

    ROS_DEBUG_STREAM("slow idx: st=" << slow_idx_st << ", ed=" << slow_idx_ed);
    ROS_DEBUG_STREAM("stop idx: st=" << stop_idx_st << ", ed=" << stop_idx_ed);

    for (int i=stop_idx_st; i<=stop_idx_ed; i++)
    {
        if (scan_ranges[i] < stop_dis_)
        {
            ROS_INFO("control state change to STOP");
            control_state_ = STOP;
            return;
        }
        if (i>=slow_idx_st && i<=slow_idx_ed && scan_ranges[i] < slow_dis_)
        {
            ROS_INFO("control state change to SLOWDOWN");
            control_state_ = SLOWDOWN;
            return;
        }
    }

    if (control_state_ != FORWARD)
        ROS_INFO("control state change to FORWARD");
    control_state_ = FORWARD;
}


void StopController::controlOnce()
{
    switch (control_state_)
    {
        case FORWARD:
        {
            linear_vel_ = min(linear_vel_up_, linear_vel_ + delta_linear_vel_);
            publishCmdVel(linear_vel_);
            break;
        }
        case SLOWDOWN:
        {
            linear_vel_ = max(linear_vel_low_, linear_vel_ - delta_linear_vel_);
            publishCmdVel(linear_vel_);
            break;
        }
        case STOP:
        {
            linear_vel_ = 0.0;
            publishCmdVel();
            break;
        }
    }
}


int StopController::getControlRate()
{
    return control_rate_;
}


bool StopController::prepareScanMsg(const sensor_msgs::LaserScanConstPtr &scan_msgs, sensor_msgs::LaserScan &new_msgs)
{
    new_msgs = *scan_msgs;
    double min_angle = scan_msgs->angle_min;
    double max_angle = scan_msgs->angle_max;

    // don't need any prepare
    if (max_angle < M_PI + 1e-4)
        return false;

    // [0, 2pi] => [-pi, pi]
    new_msgs.angle_max -= M_PI;
    new_msgs.angle_min -= M_PI;

    // find new start index, like at -M_PI
    int idx_st = 0;
    double curr_angle = min_angle;
    double delta = scan_msgs->angle_increment;
    while (curr_angle < M_PI)
    {
        curr_angle += delta;
        idx_st++;
    }

    // new vector
    for (int i=idx_st; i<scan_msgs->ranges.size(); i++)
    {
        new_msgs.ranges[i-idx_st] = scan_msgs->ranges[i];
        new_msgs.intensities[i-idx_st] = scan_msgs->intensities[i];
    }
    int ed_idx = scan_msgs->ranges.size() - idx_st;
    for (int i=0; i<idx_st; i++)
    {
        new_msgs.ranges[i+ed_idx] = scan_msgs->ranges[i];
        new_msgs.intensities[i+ed_idx] = scan_msgs->intensities[i];
    }

    return true;
}


int StopController::getScanRangesIndex(const sensor_msgs::LaserScan &scan_msgs, double target_angle)
{
    double min_angle = scan_msgs.angle_min;
    double max_angle = scan_msgs.angle_max;
    double delta = scan_msgs.angle_increment;
//    ROS_ERROR_STREAM("min angle=" << min_angle << ", max angle=" << max_angle);
//    ROS_ERROR_STREAM("delta=" << delta);
    return (target_angle - min_angle) / delta;
}


void StopController::publishCmdVel(double linear_vel, double angular_vel)
{
    geometry_msgs::Twist t;
    t.linear.x = linear_vel;
    t.linear.y = 0.0;
    t.linear.z = 0.0;
    t.angular.x = 0.0;
    t.angular.y = 0.0;
    t.angular.z = angular_vel;
    vel_pub_.publish(t);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_obstacle_stop_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::NodeHandle nh;
    StopController clt;
    ros::Rate r(clt.getControlRate());

    while (nh.ok())
    {
        clt.controlOnce();
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}