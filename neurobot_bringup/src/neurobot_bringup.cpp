//
// Created by yonghui on 19-9-12.
//

#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#define WHEEL_R 0.16  // 轮距半径
#define BASE_R 0.32*1.63      // 轮距半径
//#define WHEEL_R 0.139  // 轮距半径
//#define BASE_R 0.47  // 轮距半径
#define MOTOR_RATE 40  // 电机减速比
#define MAX_MOTO_SPEED 1000 // 电机最大转速 / 串口输入速度最大数据
#define LINEAR_SPEED_FACTOR 600
#define ANGULAR_SPPED_FACTOR 500
#define LINEAR_SPEED_RATE 1.33
#define ANGULAR_SPEED_RATE 0.833
#define VEL_X_COV
#define VEL_YAW_COV

using namespace std;
typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;


class NeuRobotDriver
{
public:
    NeuRobotDriver();

    ~NeuRobotDriver();

    /**
     * @brief 初始化函数
     * @return
     */
    bool initialize();

    /**
     * @brief Simple test receiving data from port
     */
    void testInquired();

protected:
    /**
     * @brief 根据订阅cmd_vel消息, 修改发布到串口的控制速度
     * @param cmd
     */
    void cmdVelCb(const geometry_msgs::TwistConstPtr &cmd);

    void sendSpeedCb(const ros::TimerEvent &);

    void inquireOdom(const ros::TimerEvent &);

    void inquireSensor(const ros::TimerEvent &);

    void sendOdom(string &s_data);

    void parseMsgs();


protected:
    // frame id
    string port_name_;
    string odom_frame_;
    string base_frame_;

    // frequency
    int baud_rate_;
    int control_rate_;
    int odom_rate_;
    int sensor_rate_;

    // pub, sub and timer
    bool use_tf_;
    bool use_control_;
    ros::Publisher odom_pub_;
    ros::Publisher odom_vel_pub_;
    ros::Subscriber cmd_sub_;
    ros::Timer send_speed_timer;
    ros::Timer inquire_odom_timer;
    ros::Timer inquire_sensor_timer;
    tf2_ros::TransformBroadcaster tb_;

    // port
    serial_port_ptr port_;  // boost::asio::serial_port 指针
    boost::asio::io_service io_service_;  // io_service对象是使用boost::asio库的必需要有的对象。
    boost::system::error_code ec_;  // 错误码

    // parse msg thread
    boost::mutex twist_mutex_;
    bool parse_flag_;

    // current message
    geometry_msgs::Twist current_twist_msgs_;
    geometry_msgs::TransformStamped current_trans_msgs_;
    nav_msgs::Odometry current_odom_msgs_;
    geometry_msgs::Vector3Stamped lr_wheel_msgs_;

    // odom message
    ros::Time last_time_;
    geometry_msgs::Twist last_vel_;
    double accumulate_x_;
    double accumulate_y_;
    double accumulate_th_;
};


NeuRobotDriver::NeuRobotDriver() :
port_name_("/dev/ttyUSB0"), odom_frame_("odom"), base_frame_("base_link"),
baud_rate_(11520), control_rate_(20), sensor_rate_(30),
parse_flag_(false), accumulate_x_(0), accumulate_y_(0), accumulate_th_(0)
{}


NeuRobotDriver::~NeuRobotDriver()
{
    parse_flag_ = false;
    if (port_) {
        port_->cancel();
        port_->close();
        port_.reset();
    }
    io_service_.stop();
    io_service_.reset();
}


bool NeuRobotDriver::initialize()
{
    // frame id
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("port_name", port_name_, std::string("/dev/ttyUSB0"));
    private_nh.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
    private_nh.param<std::string>("base_frame", base_frame_, std::string("base_link"));

    // frequency
    private_nh.param<int>("baud_rate", baud_rate_, 115200);
    private_nh.param<int>("control_rate", control_rate_, 30);
    private_nh.param<int>("odom_rate", odom_rate_, 30);
    private_nh.param<int>("sensor_rate", sensor_rate_, 5);

    // flag
    private_nh.param<bool>("use_tf", use_tf_, false);
    private_nh.param<bool>("use_control", use_control_, true);

    // pub and sub
    ros::NodeHandle nh;
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
    odom_vel_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("odom_vel", 10);
    cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &NeuRobotDriver::cmdVelCb, this);
    if (use_control_)
        send_speed_timer = nh.createTimer(ros::Duration(1.0 / control_rate_), &NeuRobotDriver::sendSpeedCb, this);
    inquire_odom_timer = nh.createTimer(ros::Duration(1.0 / odom_rate_), &NeuRobotDriver::inquireOdom, this);
//    inquire_sensor_timer = nh.createTimer(ros::Duration(1.0 / sensor_rate_), &NeuRobotDriver::inquireSensor, this);

    // open port
    if (port_) {
        ROS_ERROR("error : port is already opened...");
        return false;
    }
    port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
    port_->open(port_name_, ec_);
    if (ec_) {
        ROS_ERROR_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << ec_.message().c_str());
        return false;
    }
    // option settings...
    port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    port_->set_option(boost::asio::serial_port_base::character_size(8));
    port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    // begin parse msgs
    boost::thread parse_thd(boost::bind(&NeuRobotDriver::parseMsgs, this));
    return true;
}


void NeuRobotDriver::cmdVelCb(const geometry_msgs::TwistConstPtr &cmd)
{
    twist_mutex_.lock();
    current_twist_msgs_ = *cmd.get();
    twist_mutex_.unlock();
}


void NeuRobotDriver::sendSpeedCb(const ros::TimerEvent &)
{
    double linear_vel = current_twist_msgs_.linear.x;
    double angular_vel = current_twist_msgs_.angular.z;

    //! 差速模型左右电机转速
//    int left_wheel_w = 60 * MOTOR_RATE * (linear_vel + BASE_R * angular_vel) / (2 * M_PI * WHEEL_R);
//    int right_wheel_w = 60 * MOTOR_RATE * (linear_vel - BASE_R * angular_vel) / (2 * M_PI * WHEEL_R);

    //! 线速度, 角速度控制
    int left_wheel_w = LINEAR_SPEED_FACTOR * LINEAR_SPEED_RATE * linear_vel;
    int right_wheel_w = ANGULAR_SPPED_FACTOR * ANGULAR_SPEED_RATE * angular_vel;

    left_wheel_w = max(min(left_wheel_w, MAX_MOTO_SPEED), -MAX_MOTO_SPEED);
    right_wheel_w = -max(min(right_wheel_w, MAX_MOTO_SPEED), -MAX_MOTO_SPEED);

    // build buffer
    string buffer_str = string("!M ") + to_string(left_wheel_w) + string(" ") + to_string(right_wheel_w) + "\r";
    char *buffer = (char *)buffer_str.data();
    int cnt = buffer_str.size();

    // write port
    boost::asio::write(*port_.get(), boost::asio::buffer(buffer, cnt), ec_);
    if (ec_)
    {
        ROS_ERROR_STREAM("send vel cmd error: port read fail, e=" << ec_.message().c_str());
        return;
    }
    ROS_DEBUG("---");

    ROS_DEBUG_STREAM("send port motor speed: left=" << left_wheel_w << ", right=" << right_wheel_w);
    ROS_DEBUG_STREAM("send motor speed cmd=" << string(buffer, buffer+cnt).c_str());
}


void NeuRobotDriver::inquireOdom(const ros::TimerEvent &)
{
    string buffer_str("?S\r");
    int cnt = buffer_str.size();
    char *buffer = (char *)buffer_str.data();
    boost::asio::write(*port_.get(), boost::asio::buffer(buffer, cnt), ec_);
    if (ec_)
        ROS_ERROR_STREAM("inquire odom error: port read fail, e=" << ec_.message().c_str());
}


void NeuRobotDriver::inquireSensor(const ros::TimerEvent &)
{}


void NeuRobotDriver::sendOdom(string &s_data)
{
    // read left and right wheel linear vel from port
    ROS_DEBUG_STREAM(s_data.c_str());
    int colon_idx = s_data.find(':');
    int left_motor_speed = atoi(string(s_data.begin(), s_data.begin()+colon_idx).c_str());
    int right_motor_speed = atoi(string(s_data.begin()+colon_idx+1, s_data.end()).c_str());
    double left_vel = -left_motor_speed * (2 * M_PI * WHEEL_R) / (60 * MOTOR_RATE);
    double right_vel = right_motor_speed * (2 * M_PI * WHEEL_R) / (60 * MOTOR_RATE);
//    ROS_ERROR_STREAM("return odom left velocity=" << left_motor_speed << ", right velocity=" << right_motor_speed << ", idx=" << colon_idx);
    ROS_DEBUG_STREAM("return odom left velocity=" << left_vel << ", right velocity=" << right_vel);

    // get dt
    ros::Time current_time = ros::Time::now();
    double dt = current_time.toSec() - last_time_.toSec();
    last_time_ = current_time;

    // calculate robot vel
    double current_linear_vel = (left_vel + right_vel) / 2;
    double current_angular_vel = (right_vel - left_vel) / (2 * BASE_R);
    ROS_DEBUG_STREAM("current parse vel: linear=" << current_linear_vel << ", angular=" << current_angular_vel);

    // middle calculus
//    double cal_linear = (last_vel_.linear.x + current_linear_vel) / 2;
//    double cal_angular = (last_vel_.angular.z + current_angular_vel) / 2;
    // eular calculus
    double cal_linear = current_linear_vel;
    double cal_angular = current_angular_vel;

    double delta_dis = cal_linear * dt;
    double delta_th = cal_angular * dt;
    // in robot frame
    double delta_x = cos(delta_th) * delta_dis;
    double delta_y = -sin(delta_th) * delta_dis;
    ROS_DEBUG_STREAM("delta distance: dx=" << delta_x << ", dy=" << delta_y << ", delta_th=" << delta_th);

    // update odom
    accumulate_x_ = accumulate_x_ + cos(accumulate_th_)*delta_x - sin(accumulate_th_)*delta_y;
    accumulate_y_ = accumulate_y_ + sin(accumulate_th_)*delta_x + cos(accumulate_th_)*delta_y;
    accumulate_th_ += delta_th;
    ROS_DEBUG_STREAM("odom accumulate: x=" << accumulate_x_ << ", y=" << accumulate_y_ << ", th=" << accumulate_th_);
    tf::Quaternion q = tf::createQuaternionFromYaw(accumulate_th_);

    // update odometry
    // update tf tree
    if (use_tf_)
    {
        current_trans_msgs_.header.stamp = current_time;
        current_trans_msgs_.header.frame_id = odom_frame_;
        current_trans_msgs_.child_frame_id = base_frame_;
        current_trans_msgs_.transform.translation.x = accumulate_x_;
        current_trans_msgs_.transform.translation.y = accumulate_y_;
        current_trans_msgs_.transform.translation.z = 0.0;

        current_trans_msgs_.transform.rotation.x = q.x();
        current_trans_msgs_.transform.rotation.y = q.y();
        current_trans_msgs_.transform.rotation.z = q.z();
        current_trans_msgs_.transform.rotation.w = q.w();
        tb_.sendTransform(current_trans_msgs_);
    }

    // last time vel
    last_vel_.linear.x = current_linear_vel;
    last_vel_.linear.y = 0.0;
    last_vel_.linear.z = 0.0;
    last_vel_.angular.x = 0.0;
    last_vel_.angular.y = 0.0;
    last_vel_.angular.z = current_angular_vel;
    // publish odom
    current_odom_msgs_.header.stamp = current_time;
    current_odom_msgs_.header.frame_id = odom_frame_;
    current_odom_msgs_.pose.pose.position.x = accumulate_x_;
    current_odom_msgs_.pose.pose.position.y = accumulate_y_;
    current_odom_msgs_.pose.pose.position.z = 0.0;
    current_odom_msgs_.pose.pose.orientation.x = q.x();
    current_odom_msgs_.pose.pose.orientation.y = q.y();
    current_odom_msgs_.pose.pose.orientation.z = q.z();
    current_odom_msgs_.pose.pose.orientation.w = q.w();
    current_odom_msgs_.pose.covariance[0]  = 1e-3;  // x cov
    current_odom_msgs_.pose.covariance[7]  = 1e-3;  // y cov
    current_odom_msgs_.pose.covariance[14] = 1e6;  // z cov
    current_odom_msgs_.pose.covariance[21] = 1e6;  // roll cov
    current_odom_msgs_.pose.covariance[28] = 1e6;  // pitch cov
    current_odom_msgs_.pose.covariance[35] = 1e-3;   //yaw cov

    current_odom_msgs_.twist.twist = last_vel_;
    current_odom_msgs_.twist.covariance[0]  = 1e-2;  // x cov
    current_odom_msgs_.twist.covariance[7]  = 1e10;  // y cov
    current_odom_msgs_.twist.covariance[14] = 1e10;  // z cov
    current_odom_msgs_.twist.covariance[21] = 1e10;  // roll cov
    current_odom_msgs_.twist.covariance[28] = 1e10;  // pitch cov
    current_odom_msgs_.twist.covariance[35] = 5e-2;   //yaw cov
    odom_pub_.publish(current_odom_msgs_);
    // publish left and right wheel velocity
    lr_wheel_msgs_.header.stamp = current_time;
    lr_wheel_msgs_.header.frame_id = odom_frame_;
    lr_wheel_msgs_.vector.x = left_vel / WHEEL_R;
    lr_wheel_msgs_.vector.y = right_vel / WHEEL_R;
    lr_wheel_msgs_.vector.z = 0.0;
    odom_vel_pub_.publish(lr_wheel_msgs_);
}


void NeuRobotDriver::parseMsgs()
{
    // read port buffer
    char buffer[255];
    int cnt = 0;
    memset(buffer, 0, 255);

    // run loop
    parse_flag_ = true;
    while (parse_flag_)
    {
        boost::asio::read(*port_.get(), boost::asio::buffer(&buffer[cnt], 1), ec_);
        if (ec_)
        {
            ROS_ERROR_STREAM("error: port read fail, e=" << ec_.message().c_str());
            memset(buffer, 0, 255);
            cnt = 0;
            continue;
        }

        if (buffer[cnt] == '\r')
        {
            // clear buffer
            string buffer_str(buffer, buffer+cnt);
            memset(buffer, 0, 255);
            cnt = 0;

            ROS_DEBUG("---");
            ROS_DEBUG_STREAM("receive message: buffer=\"" << buffer_str.c_str() << "\"");

            // inquire message
            if (buffer_str[0] == '?')
                continue;

            // no return message
            if (buffer_str.find("+") == 0)
            {
                ROS_DEBUG("receive no return message");
                continue;
            }

            // BS return message
            if (buffer_str.find("BS=") == 0)
            {
                ROS_DEBUG_STREAM("receive BS return message=\"" << buffer_str.c_str() << "\"");
                continue;
            }

            // F return message
            if (buffer_str.find("F=") == 0)
            {
                ROS_DEBUG_STREAM("receive F return message=\"" << buffer_str.c_str() << "\"");
                string s_data(buffer_str.begin()+2, buffer_str.end());
                sendOdom(s_data);
                continue;
            }

            // S return message
            if (buffer_str.find("S=") == 0)
            {
                ROS_DEBUG_STREAM("receive S return message=\"" << buffer_str.c_str() << "\"");
                string s_data(buffer_str.begin()+2, buffer_str.end());
                sendOdom(s_data);
                continue;
            }

            ROS_WARN_STREAM("Unparsed return message=\"" << buffer_str.c_str() << "\"");
        }
        else
        {
            cnt++;
        }
    }
}


void NeuRobotDriver::testInquired()
{
    string buffer_str("?S\r");
    int cnt = buffer_str.size();
    char *buffer = (char *)buffer_str.data();
    boost::asio::write(*port_.get(), boost::asio::buffer(buffer, cnt), ec_);
    if (ec_)
        ROS_ERROR_STREAM("error: port read fail, e=" << ec_.message().c_str());
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "neurobot_bringup");

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    NeuRobotDriver driver;

    if (driver.initialize())
    {
        ros::Rate r(10);
        ros::NodeHandle nh;
//        while (nh.ok())
//        {
//            driver.testInquired();
//            r.sleep();
//            ros::spinOnce();
//        }
        ros::spin();
    }

    return 0;
}