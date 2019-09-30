#include "ccd_camera/controller.h"
#include <algorithm>
using namespace std;

namespace ccd_camera
{
    Controller::Controller(float fkp, float fKi, float fKd, float fCmdLinearX, float fErrSumMax, float fErrSumMin):
            mfKp(fkp), mfKi(fKi), mfKd(fKd), mfCmdLinearX(fCmdLinearX),
//    mfErrLast1(0), mfErrLast2(0), mfOutputLast(0)
            mfErrSumMax(fErrSumMax), mfErrSumMin(fErrSumMin),
            mfErrSum(0.), mfErrLast(0.)
    {
    }


//void Controller::reconfigCallBack(ccd_camera::pid_Config &config, uint32_t level)
//{
//    mfKp = config.Kp;
//    mfKi = config.Ki;
//    mfKd = config.Kd;
//    mfCmdLinearX = config.LinearX;
//    ros::param::set("~Kp", config.Kp);
//    ros::param::set("~Ki", config.Ki);
//    ros::param::set("~Kd", config.Kd);
//    ros::param::set("~linear_x", config.LinearX);
//    ROS_WARN("Kp: %f, Ki %f, Kd %f, LinearX %f", config.Kp, config.Ki, config.Kd, config.LinearX);
//}


    void Controller::updateParams(float fkp, float fKi, float fKd, float fLinearX, float fErrSumMax, float fErrSumMin)
    {
        mfKp = fkp;
        mfKi = fKi;
        mfKd = fKd;
        mfCmdLinearX = fLinearX;
        mfErrSumMax = fErrSumMax;
        mfErrSumMin = fErrSumMin;
    }


    float Controller::output(float fError)
    {
//    float fdOutput = mfKp*(fError-mfErrLast1) + mfKi*fError + mfKd*(fError-2*mfErrLast1+mfErrLast2);
//    float fCurrOutput = fdOutput + mfOutputLast;
//    mfOutputLast = fCurrOutput;
//    mfErrLast2 = mfErrLast1;
//    mfErrLast1 = fError;
        mfErrSum += fError;
        mfErrSum = min(mfErrSumMax, max(mfErrSumMin, mfErrSum));
        float fOut = mfKp*fError + mfKi*mfErrSum + mfKd*(fError-mfErrLast);
        mfErrLast = fError;
        ROS_INFO("Kp: %f | Ki: %f | Kd: %f | Lx: %f | Current output: %f", mfKp, mfKi, mfKd, mfCmdLinearX, fOut);
        return fOut;
    }


//void Controller::setKp(const float &fKp)
//{
//    mfKp = fKp;
//}
//
//
//void Controller::setKi(const float &fKi)
//{
//    mfKi = fKi;
//}
//
//
//void Controller::setKd(const float &fKd)
//{
//    mfKd = fKd;
//}
//
//
//void Controller::setCmdLinearX(const float &fCmdLinearX)
//{
//    mfCmdLinearX = fCmdLinearX;
//}
//
//float Controller::getCmdLinearX()
//{
//    return mfCmdLinearX;
//}
}

