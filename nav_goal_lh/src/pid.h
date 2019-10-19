#ifndef _PID_H_
#define _PID_H_
 
typedef struct _pid{
float SetAngle;
float ActualAngle;
float err;
float err_next;
float err_last;
float Kp, Ki, Kd;
}Pid;
 
 
class Pid_control
{
public: 
    void PID_init();
    float PID_realize(float base_yaw, float curr_yaw);

private:
    int index;
    Pid pid;
};
#endif

