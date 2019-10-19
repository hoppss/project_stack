
#include <iostream>
#include "pid.h"
 
using namespace std;

void  Pid_control::PID_init()
{
    pid.SetAngle = 0.0;
    pid.ActualAngle = 0.0;
    pid.err = 0.0;
    pid.err_last = 0.0;  //上一个
    pid.err_next = 0.0;  //上上一个
    pid.Kp = 0.2;
    pid.Ki = 0.015;
    pid.Kd = 0.2;
}
 
float Pid_control::PID_realize(float base_yaw, float curr_yaw)
{
    pid.SetAngle = base_yaw;

    pid.err_last = pid.err_next;
    pid.err_next = pid.err;
    pid.err = pid.SetAngle - curr_yaw;
    float incrementAngle = pid.Kp*(pid.err - pid.err_next) + pid.Ki*pid.err + pid.Kd*(pid.err - 2 * pid.err_next + pid.err_last);
    pid.ActualAngle += incrementAngle;
    pid.err_last = pid.err_next;
    pid.err_next = pid.err;
    return incrementAngle;
}
