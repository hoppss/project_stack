//
// Created by yonghui on 19-4-19.
//

#ifndef CCD_CAMERA_NODE_CCD_CAMERA_SERIAL_H
#define CCD_CAMERA_NODE_CCD_CAMERA_SERIAL_H

#include <termios.h>
#include <boost/thread.hpp>
#include <boost/lockfree/queue.hpp>
#include <ros/node_handle.h>

class CCDCameraSerial
{
public:
    CCDCameraSerial();
    explicit CCDCameraSerial(const char *cstrPortName, int baud_rate=B115200);
    void initialize(const char *cstrPortName);
    int recv_buffer();
    bool isPortOpen();
    void run(ros::NodeHandle &nh);
    void stop(boost::thread &th);

    boost::lockfree::queue<int, boost::lockfree::fixed_sized<true> > q;

protected:
    int set_interface_attribs(int fd, int speed);
    void set_mincount(int fd, int mcount);

    const char *mcstrPortName;
    bool mbOpenPortFlag;
    int fd;
    int mnBaudRate;
};

#endif //CCD_CAMERA_NODE_CCD_CAMERA_SERIAL_H
