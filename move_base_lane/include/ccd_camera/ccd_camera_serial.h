//
// Created by yonghui on 19-4-19.
//

#ifndef CCD_CAMERA_NODE_CCD_CAMERA_SERIAL_H
#define CCD_CAMERA_NODE_CCD_CAMERA_SERIAL_H

namespace ccd_camera
{
    class CCDCameraSerial
    {
    public:
        CCDCameraSerial();
        explicit CCDCameraSerial(const char *cstrPortName);
        void initialize(const char *cstrPortName);
        int recv_buffer();
        bool isPortOpen();

    protected:
        int set_interface_attribs(int fd, int speed);
        void set_mincount(int fd, int mcount);

        const char *mcstrPortName;
        bool mbOpenPortFlag;
        int fd;
    };
}



#endif //CCD_CAMERA_NODE_CCD_CAMERA_SERIAL_H
