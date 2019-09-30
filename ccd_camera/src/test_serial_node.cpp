//
// Created by yonghui on 19-6-11.
//

#include <ccd_camera_serial.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/lockfree/queue.hpp>
#include <chrono>

#define MID_SIGINAL 86
#define LEFT_SIGNAL 25
#define RIGHT_SIGNAL 59

enum Status
{
    NO_RECV = 0,
    MIDDLE = 1,
    LEFT = 2,
    RIGHT = 3,
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_serial_node");
    //! public topic
    ros::NodeHandle nh;
    ros::Publisher pubSerial = nh.advertise<std_msgs::Int32>("serial", 100);
    //! open serial
    ros::NodeHandle private_nh("~");
    std::string port;
    int rate;
    private_nh.param("port", port, std::string("/dev/ttyUSB0"));
    private_nh.param("baud_rate", rate, 300);
    ROS_INFO("port name: %s", port.c_str());
    ROS_INFO("baud_rate: %d", rate);
    // open and check
    CCDCameraSerial serial(port.c_str(), rate);
    if (!serial.isPortOpen())
    {
        ROS_ERROR("Port open fail!");
        return -1;
    }
    //! start read thread
    boost::thread td(boost::bind(&CCDCameraSerial::run, &serial, nh));
    std::chrono::steady_clock::time_point st = std::chrono::steady_clock::now();
    //! read queue
    ros::Rate r(20);
    Status eCurrent = Status::NO_RECV;
    Status eLast = Status::NO_RECV;
    while (nh.ok())
    {
        int buf;
        if (!serial.q.empty() && serial.q.pop(buf))
        {
            std_msgs::Int32 msg;
            msg.data = buf;
            ROS_INFO("Receive Serial Output: %d", buf);
            pubSerial.publish(msg);
            //TODO: Control the base and stop
            switch (buf)
            {
                case MID_SIGINAL:
                    // TODO: publish straight move
                    eCurrent = Status::MIDDLE;
                    break;
                case LEFT_SIGNAL:
                    // TODO: publish left turn move
                    eCurrent = Status::LEFT;
                    break;
                case RIGHT_SIGNAL:
                    // TODO: publish right turn move
                    eCurrent = Status::RIGHT;
                    break;
                default:
                    // deal as no receive
                    eCurrent = Status::NO_RECV;
                    break;
            }
        }
        else
        {
            eCurrent = Status::NO_RECV;
        }
        // read serial time out
        if (eCurrent==Status::NO_RECV)
        {
            std::chrono::steady_clock::time_point ed = std::chrono::steady_clock::now();
            if ((ed-st).count()/1e9 > 1)
            {
                ROS_WARN("Receive Serial Output Time Out!");
                st = ed;
                // TODO: rotate left and right to receive signal
                switch (eLast)
                {
                    case NO_RECV:
                        // TODO: uncertain roatate direction
                        break;
                    case MIDDLE:
                        // TODO: uncertain rotate direction
                        break;
                    // rotate too left
                    case LEFT:
                        // TODO: continue to rotate left until recrive next signal
                        // use while...
                        break;
                    // rotate too right
                    case RIGHT:
                        // TODO: continue to rotate right until receive next signal
                        // use while...
                        break;
                }
            }
        }
        else
        {
            st = std::chrono::steady_clock::now();
            eLast = eCurrent;  // record last status
        }
        r.sleep();
    }
    serial.stop(td);
    return 0;
}
