#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "ccd_camera/ccd_camera_serial.h"
#include <ros/ros.h>

namespace ccd_camera
{
    CCDCameraSerial::CCDCameraSerial():
            mcstrPortName(NULL)
    {

    }


    CCDCameraSerial::CCDCameraSerial(const char *cstrPortName):
            mcstrPortName(cstrPortName)
    {
        initialize(cstrPortName);
    }


    void CCDCameraSerial::initialize(const char *cstrPortName)
    {
        int wlen;
        mcstrPortName = cstrPortName;
        fd = open(mcstrPortName, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) {
            ROS_ERROR("Error opening %s: %s\n", mcstrPortName, strerror(errno));
            mbOpenPortFlag = false;
        }
        else
        {
            /*baudrate 115200, 8 bits, no parity, 1 stop bit */
            set_interface_attribs(fd, B115200);
            //set_mincount(fd, 0);                /* set to pure timed read */

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

        cfsetospeed(&tty, (speed_t)speed);
        cfsetispeed(&tty, (speed_t)speed);

        tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;         /* 8-bit characters */
        tty.c_cflag &= ~PARENB;     /* no parity bit */
        tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
        tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

        /* setup for non-canonical mode */
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;

        /* fetch bytes as they become available */
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;

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
        tty.c_cc[VTIME] = 5;        /* half second timer */

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
            ROS_ERROR("Error from read: %d: %s", rdlen, strerror(errno));
            return -1;
        }
        /* repeat read to get full message */
    }

}

