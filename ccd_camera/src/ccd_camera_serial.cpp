#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "ccd_camera_serial.h"
#include <ros/ros.h>


CCDCameraSerial::CCDCameraSerial():
mcstrPortName(NULL)
{

}


CCDCameraSerial::CCDCameraSerial(const char *cstrPortName, int baud_rate):
    mcstrPortName(cstrPortName), mnBaudRate(baud_rate), q(100)
{
    // choose baud rate
    switch (mnBaudRate)
    {
        case 300:
            mnBaudRate = B300;
            break;
        case 115200:
            mnBaudRate = B115200;
            break;
        default:
            ROS_WARN("Serial has been suspend, please reopen the node...");
            mnBaudRate = B0;
            break;
    }
    initialize(cstrPortName);
}


void CCDCameraSerial::initialize(const char *cstrPortName)
{
    int wlen;
    mcstrPortName = cstrPortName;
//    fd = open(mcstrPortName, O_RDWR | O_NOCTTY | O_SYNC);
    fd = open(mcstrPortName, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (fd < 0) {
        ROS_ERROR("Error opening %s: %s\n", mcstrPortName, strerror(errno));
        mbOpenPortFlag = false;
    }
    else
    {
        //! 波特率: mnSpeed, 字长: 8bits, 停止位: 1bit
        set_interface_attribs(fd, mnBaudRate);
//        set_mincount(fd, 1);

        /* simple output */
        wlen = write(fd, "Hello!\n", 7);
        if (wlen != 7) {
            ROS_ERROR("Error from write: %d, %d\n", wlen, errno);
            mbOpenPortFlag = false;
        }
        else
        {
            tcdrain(fd);    /* delay for output */
            mbOpenPortFlag = true;
        }
    }
}


bool CCDCameraSerial::isPortOpen()
{
    return mbOpenPortFlag;
}

int CCDCameraSerial::set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        ROS_ERROR("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    //! 设置波特率
    cfsetospeed(&tty, (speed_t)speed);  // 输出波特率
    cfsetispeed(&tty, (speed_t)speed);  // 输入波特率

    //! 控制模式标志, 制定终端硬件控制信息
    tty.c_cflag |= (CLOCAL | CREAD);    // 忽略调制解调器的状态
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         // 字长=8bits
    tty.c_cflag &= ~PARENB;     // 不使用奇偶校验
    tty.c_cflag &= ~CSTOPB;     // 设置一个停止位
    tty.c_cflag &= ~CRTSCTS;    // 不使用硬件流控制

    //! 输入输出本地设置
    // 关闭输入标志处理
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    // 关闭本地, 终端编辑处理, 非规范模式
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    // 关闭输出标志处理
    tty.c_oflag &= ~OPOST;
    // 只在阻塞模式下才会起作用
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    // ! 应用设置
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ROS_ERROR("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void CCDCameraSerial::set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        ROS_ERROR("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;  // 0.5s timeout

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        ROS_ERROR("Error tcsetattr: %s\n", strerror(errno));
}


int CCDCameraSerial::recv_buffer()
{
    /* simple noncanonical input */
        unsigned char buf;
        int rdlen;
        rdlen = read(fd, &buf, sizeof(buf));
        if (rdlen > 0) {
	        ROS_INFO("read successful: %d", buf);
			return buf;
        } else if (rdlen < 0) {
//            ROS_ERROR("Error from read: %d: %s", rdlen, strerror(errno));
			return -1;
        }
        /* repeat read to get full message */
}


void CCDCameraSerial::run(ros::NodeHandle &nh)
{
    if (!isPortOpen())
    {
        ROS_ERROR("Port open fail!");
        return;
    }
    ros::Rate r(20);
    while (nh.ok())
    {
        int buf = recv_buffer();
        if (buf > -1)
        {
            q.push(buf);
        }
        r.sleep();
    }
}


void CCDCameraSerial::stop(boost::thread &td)
{
    close(fd);
    td.join();
}