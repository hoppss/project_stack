//
// Created by fs on 19-8-28.
//
#include<iostream>
#include<iomanip>
#include <queue>
#include <map>
#include <fstream>
#include <sstream>
#include <mutex>
#include <stdio.h>
#include <thread>
#include <condition_variable>
#include <sstream>
#include <vector>
#include <math.h>
#include "fusion.hpp"
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_goal_lh/current_gps.h"
#include "getgps.h"
#include "nav_goal_lh/position.h"
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "gps_common/GPSFix.h"
#include "noise_kalman.h"
#include "gps_transform.h"
#include "pid.h"
#include "time.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>

/////键盘控制相关头文件
#include <termios.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <boost/thread/thread.hpp>

#define EARTH_RADIUS            6378.137
#define MIN_D 1
#define CLOCKS_PER_SEC ((clock_t)1000) 

using namespace std;
using namespace Eigen;
std::condition_variable con;

std::queue<nav_goal_lh::current_gps::ConstPtr> gps_buf;
std::queue<nav_msgs::Odometry::ConstPtr> yaw_buf;
//double nav_goal_list[100];
// std::vector<double> nav_goal_list={123.412582, 41.762672,123.412598, 41.762760,123.412727, 41.762745,123.412697, 41.762661,123.412582, 41.762672};
//std::vector<double> nav_goal_list={123.412582, 41.762646, 123.412598, 41.762737, 123.412712, 41.762737, 123.412697, 41.762653, 123.412582, 41.762646};
//std::vector<double> nav_goal_list={123.4137758, 41.7627915, 123.4135875, 41.7628008, 123.4136020, 41.7628795, 123.4137900, 41.7628680, 123.4137758, 41.7627915};
std::vector<double> nav_goal_list={123.413303317, 41.76271615, 123.41316735, 41.762717217, 123.413185167, 41.762813417, 123.413318767, 41.762812267, 123.413303317, 41.76271615};

//        {123.414650, 41.761059,123.414520, 41.761086};食堂
        //{123.415553,41.760326,123.415688, 41.760342};操场
//        {123.412529,41.762539,123.412529,41.762630,123.412468,41.762630};综合楼后门
//std::vector<double> nav_goal_list= {123.413185,41.762028,123.413414,41.761963};综合楼前门
//        {123.412552,41.762783,123.412567,41.762932,123.412758,41.762932,123.412743,41.762783};
int goal_num=5;
double EE_,NN_,YAW_,yaw_filter,YAW_rate,YAW_global;
double curr_odom,curr_odom_x,curr_odom_y,curr_linear_x;
Eigen::Quaterniond now_angle_;


std::mutex m_buf;

ros::Publisher robot_control_pub;

int flag_nav_goal=1;
int flag_control=0;

////////////////////---------------keyboard control code start------------/////////////////////
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57


class SmartCarKeyboardTeleopNode
{
private:
    double walk_vel_;
    double run_vel_;
    double yaw_rate_;
    double yaw_rate_run_;


public:
    SmartCarKeyboardTeleopNode()
    {
        pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        ros::NodeHandle n_private("~");
        n_private.param("walk_vel", walk_vel_, 0.2);
        n_private.param("run_vel", run_vel_, 1.0);
        n_private.param("yaw_rate", yaw_rate_, 1.0);
        n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);
    }
    geometry_msgs::Twist cmdvel_;
    ros::NodeHandle n_;
    ros::Publisher pub_;

    ~SmartCarKeyboardTeleopNode() { }
    void keyboardLoop();

    void stopRobot()
    {
        cmdvel_.linear.x = 0.0;
        cmdvel_.angular.z = 0.0;
        pub_.publish(cmdvel_);
    }
};

SmartCarKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

////////////////////---------------keyboard control code start------------/////////////////////

double kfyaw(double yaw)
{
    KF kf(1,0.1);
    VectorXd yawv,yawc;
    yawv.resize(1);
    yawv<<yaw;
    yawc=kf.kalman(yawv);
    yaw_filter=yawc(0);
    return yaw_filter;
}


/*******
//原始方法：获取GPS定位模块当前的经纬度
void chatterCallback(const nav_goal_lh::current_gps::ConstPtr& msg)
{
  //  ROS_INFO("curent position and orientation:[%f][%f][%f]",msg->EE,msg->NN,msg->yaw_);
  //  std::cout<<" chatterCallback run"<<std::endl;
    m_buf.lock();
//    gps_buf.push(msg);
  //  wgs84togcj02(msg->EE,msg->NN,EE_,NN_);
    EE_ = msg->EE;
    NN_ = msg->NN;
    // cout<<"EE is: "<<EE_<<endl;
    // cout<<"NN is: "<<NN_<<endl;
    m_buf.unlock();
    con.notify_one();
}
*******/

//ros包自带的读取GPS
void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    //  ROS_INFO("curent position and orientation:[%f][%f][%f]",msg->EE,msg->NN,msg->yaw_);
    //  std::cout<<" chatterCallback run"<<std::endl;
    m_buf.lock();
//    gps_buf.push(msg);
    //  wgs84togcj02(msg->EE,msg->NN,EE_,NN_);
    EE_ = msg->longitude;//经度
    NN_ = msg->latitude;//维度
    // cout<<"EE is: "<<EE_<<endl;
    // cout<<"NN is: "<<NN_<<endl;
    m_buf.unlock();
    con.notify_one();
}


//获取手机当前的全局偏航角yaw
void chatterCallback0(const nav_msgs::Odometry::ConstPtr& msg)
{
  //  ROS_INFO("curent position and orientation:[%f][%f][%f]",msg->EE,msg->NN,msg->yaw_);
  //  std::cout<<" chatterCallback run"<<std::endl;
    m_buf.lock();
    yaw_buf.push(msg);
    YAW_global = msg->twist.twist.angular.z;
  //  YAW_global = YAW_global+45-150;
    if (YAW_global<0)
        YAW_global = YAW_global+360;
if (YAW_global>360)
        YAW_global = YAW_global-360;
//    YAW_global=kfyaw(YAW_global);
 //    cout<<"Yaw: "<< YAW_<<endl;
    m_buf.unlock();
    con.notify_one();

}


//获取当前的经纬度以及全局偏航角yaw
// void chatterCallback(const nav_goal::current_gps::ConstPtr& msg)
// {
  //  ROS_INFO("curent position and orientation:[%f][%f][%f]",msg->EE,msg->NN,msg->yaw_);
  //  std::cout<<" chatterCallback run"<<std::endl;
//     m_buf.lock();
//     gps_buf.push(msg);
//     EE_ = msg->EE;
//     NN_ = msg->NN;
//     YAW_ = msg->yaw_;
// //     if(YAW_<0)
//         YAW_=YAW_+360;
//     kfyaw(YAW_);
//     m_buf.unlock();
//     con.notify_one();

// }


////获取导航点序列
void chatterCallback1(const nav_goal_lh::position::ConstPtr& msg)
{
  //  ROS_INFO("I heard nav_goal number:[%d]", msg->n);
  //  std::cout<< "chatterCallconst nav_msgs::Odometry::ConstPtr& msgback2 run"<<std::endl;
    m_buf.lock();
    if(flag_nav_goal)
    {
          nav_goal_list = {123.412613, 41.762646, 123.412613, 41.762794, 123.412758, 41.762788, 123.412743, 41.762638};
          goal_num = 4;
          cout<<"nav_goal_list: "<<endl;
//        nav_goal_list = msg->EN;
//        goal_num = msg->n;
//        for(int i=0;i<goal_num;i=i+2)
//        {
//            double aa,bb;
//            gcj02towgs84(nav_goal_list[i],nav_goal_list[i+1],aa,bb);
//            nav_goal_list[i]=aa;
//            nav_goal_list[i+1]=bb;
//        }

        flag_nav_goal = 0;
    }
    m_buf.unlock();
    con.notify_one();

}



/// 获取imu信息
void odomCallback3(const sensor_msgs::Imu::ConstPtr& msg)
 {
   m_buf.lock();
   YAW_rate=msg->angular_velocity.z;
//   if(msg->orientation.z>0)
//       YAW_global=msg->orientation.z;
//   else
//       YAW_global=360+msg->orientation.z;
   m_buf.unlock();
   con.notify_one();
 }



//获取底盘的里程计信息，并结合IMU计算主方向上的位移，作为是否到达目标点的条件之一
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //  std::cout << "odom information about pose is:" <<  msg->pose << std::endl;
    curr_odom = msg->pose.pose.position.x;//错的，目前先这样写便于编译，后续读取机器人里程计并结合IMU计算主方向上的位移作为判断到达目标点的条件之一
    curr_odom_x = msg->pose.pose.position.x;
    curr_odom_y = msg->pose.pose.position.y;
    now_angle_.x()=msg->pose.pose.orientation.x;
    now_angle_.y()=msg->pose.pose.orientation.y;
    now_angle_.z()=msg->pose.pose.orientation.z;
    now_angle_.w()=msg->pose.pose.orientation.w;
    curr_linear_x = msg->twist.twist.linear.x;


}

//两经纬点之间的距离（纬度，经度）
double get_distance(double lat1, double lng1, double lat2, double lng2)
{
    double radLat1 = lat1 * M_PI / 180.0;   //角度1˚ = π / 180
    double radLat2 = lat2 * M_PI / 180.0;   //角度1˚ = π / 180
    double a = radLat1 - radLat2;//纬度之差
    double b = lng1 * M_PI / 180.0 - lng2* M_PI / 180.0;  //经度之差
    double dst = 2 * asin((sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2))));
    dst = dst * EARTH_RADIUS;
    dst = round(dst * 10000) / 10000;
    return dst;
}

//计算两经纬点之间的角度123.412552,41.762783,123.412567,41.762932,123.412758,41.762932,123.412743,41.762783};
int get_angle(double lat1, double lng1, double lat2, double lng2)
{

    double x = lat2 - lat1;//t d
    double y = lng2 - lng1;//z y
    double angle=-1;
    if (y == 0 && x > 0) angle = 0;
    if (y == 0 && x < 0) angle = 180;
    if(x ==0 && y > 0) angle = 90;
    if(x == 0 && y < 0) angle = 270;
    if (angle == -1)
    {
        double dislat = get_distance(lat1, lng2, lat2, lng2);
        double dislng = get_distance(lat2, lng1, lat2, lng2);
        if (x > 0 && y > 0) angle = atan2(dislng, dislat) / M_PI * 180;
        if (x < 0 && y > 0) angle = atan2(dislat, dislng) / M_PI * 180+90;
        if (x < 0 && y < 0) angle = atan2(dislng, dislat) / M_PI * 180 + 180;
        if (x > 0 && y < 0) angle = atan2(dislat, dislng) / M_PI * 180 + 270;
//        cout<<"dislat dislon  "<<dislat<<"  "<<dislng<<endl;
//        cout<<"atan2(dislng, dislat)  "<<atan2(dislng, dislat) / M_PI * 180<<endl;
    }

    return angle;
}


double get_angle_odom(double y1, double x1, double y2, double x2)
{
    double delta_x=x2-x1;
    double delta_y=y2-y1;
    return atan2(delta_y, delta_x)/ M_PI * 180;
}

double get_distance_odom(double y1, double x1, double y2, double x2)
{
    double delta_x=x2-x1;
    double delta_y=y2-y1;
    return sqrt(delta_x*delta_x+delta_y*delta_y);
}



//机器人的运动策略，目前先简单的前进方向给个匀速，旋转通过当前的方向与主方向的夹角进行调节
float ki_data;
geometry_msgs::Twist controlVel_;
void robot_control(double curr_yaw, double base_yaw, double speed)
{

#if    0
    float kp=0.001,ki=0.0001;
    ki_data += (curr_yaw-base_yaw) ;
    if(ki_data> 1000)
       ki_data=1000;
    if(ki_data<-1000)
       ki_data=-1000;
   controlVel_.angular.z = -kp*(curr_yaw-base_yaw) - ki*ki_data;
   if(controlVel_.angular.z>0.1)
     controlVel_.angular.z = 0.1;
   if(controlVel_.angular.z<-0.1)
   controlVel_.angular.z=-0.1;
#endif   // 15:08  李衡重新改写前留存







   if(abs(curr_yaw-base_yaw)>50)
   {controlVel_.angular.z = 0.1;}
   else
   {
//    Pid_control Pid;
//    Pid.PID_init();


    if (abs(curr_yaw-base_yaw)<1)
    {
//        controlVel_.angular.z += Pid_control::PID_realize(base_yaw, curr_yaw );
        if(controlVel_.angular.z>0.1)
           controlVel_.angular.z = 0.1;
        if(controlVel_.angular.z<-0.1)
           controlVel_.angular.z=-0.1;
        
    }
   }




//    if(abs(curr_yaw-base_yaw)>2)
//        controlVel_.angular.z=0.1;
//    else
//        controlVel_.angular.z=0;

    controlVel_.linear.x = speed;
    controlVel_.linear.y = 0;
    controlVel_.linear.z = 0;
   if (speed!=0)
    controlVel_.angular.z=0;

    controlVel_.linear.x = speed;
    controlVel_.linear.y = 0;
    controlVel_.linear.z = 0;
    if(!flag_control)
    {
        controlVel_.angular.z = 0;
      //  controlVel_.angular.z = 0;
        controlVel_.linear.x = 0;
        controlVel_.linear.y = 0;
        controlVel_.linear.z = 0;
    }
    if(speed==0)
        cout << "zhengzai jiaodu jiaozheng " << endl;
    else
        cout << "zhengzai yundong " << endl;
    robot_control_pub.publish(controlVel_);
}


double judge(double curr_yaw,double base_yaw)
{
    double r0=abs(curr_yaw-base_yaw);
    double r1=abs(curr_yaw-base_yaw+360);
    double r2=abs(curr_yaw-base_yaw-360);
    double r=min(min(r0,r1),r2);
    if(r==r0) return curr_yaw-base_yaw;
    if(r==r1) return curr_yaw-base_yaw+360;
    if(r==r2) return curr_yaw-base_yaw-360;
}




void robot_control1(double curr_yaw, double base_yaw, double speed)
{


      float kp=0.005;
//    ki_data += (curr_yaw-base_yaw) ;
//    if(ki_data> 1000)
//        ki_data=1000;
//    if(ki_data<-1000)
//        ki_data=-1000;
cout<<"aaa  "<<controlVel_.angular.z<<" , "<<judge(curr_yaw,base_yaw)<<","<<curr_yaw<<","<<base_yaw<<endl;
controlVel_.angular.z = kp*judge(curr_yaw,base_yaw);
//        controlVel_.angular.z = 0.1;
    if(controlVel_.angular.z<-0.1)
        controlVel_.angular.z=-0.1;
    if(controlVel_.angular.z>0.1)
        controlVel_.angular.z=0.1;
//    if(curr_yaw-base_yaw>2.5)
//        controlVel_.angular.z=0.1;
//    if(curr_yaw-base_yaw<-1)
//        controlVel_.angular.z=-0.1;

    controlVel_.linear.x = speed;
    controlVel_.linear.y = 0;
    controlVel_.linear.z = 0;
    if (speed!=0)
        controlVel_.angular.z=0;

    controlVel_.linear.x = speed;
    controlVel_.linear.y = 0;
    controlVel_.linear.z = 0;
    if(!flag_control)
    {
        controlVel_.angular.z = 0;
        //  controlVel_.angular.z = 0;
        controlVel_.linear.x = 0;
        controlVel_.linear.y = 0;
        controlVel_.linear.z = 0;
    }
    if(speed==0)
        cout << "zhengzai jiaodu jiaozheng " << endl;
    else
        cout << "zhengzai yundong " << endl;
    robot_control_pub.publish(controlVel_);
}













Eigen::VectorXd fusion(double dt,double yaw_rate,double sita,double s,double lat,double lon);
DataPoint data_point;
Fusion fusions(0.5,0.5,0.1,0.05,0.001,0.05,0.1,0.0,0.0,false);//yaw_rate/180*M_PI,sita/180*M_PI,s,lat,lon;
int count_=0;
double first_lat,first_lon;
Eigen::VectorXd fusion(double dt,double yaw_rate,double sita,double s,double lat,double lon)
{
    VectorXd raw_data(5);
    raw_data<<yaw_rate/180*M_PI,sita/180*M_PI,s,lat,lon;
    if(count_==0) { first_lat=raw_data[3]; first_lon=raw_data[4];count_++;}
    data_point.set(dt,1,raw_data);
    Eigen::VectorXd state0;
    state0=fusions.process(data_point);
    return state0;
}

//根据组合导航定位系统（分配给师弟们完成）得到的当前位置判断是否到达目标点
bool is_arrived_gps(double EE, double NN, double YAW, double curr_odom, double nav_goal_E, double nav_goal_N)
{







//



//	FILE *outfile=fopen("measure.txt","a+");
//	if(!outfile) cout<<"error read file!"<<endl;
//	fprintf(outfile,"%.7lf,%.7lf,%.7lf,%.7lf,%.7lf\n",YAW_rate,YAW_global,curr_linear_x*dt, EE, NN);
//	fclose(outfile);

//    now_la = EE;
//    now_lon = NN;

    double Threshold=0.000015;
    std::cout<<"caculate lat: "<<EE<<" caculate lon: "<<NN<<std::endl;
std::cout<<"different lat: "<<abs(EE-nav_goal_E)<<" different lon: "<<abs(NN-nav_goal_N)<<std::endl;
    if(abs(EE-nav_goal_E)<Threshold&&abs(NN-nav_goal_N)<Threshold)//这个地方留的师弟们做的定位的接口。判断（当前的位置信息与目标点进行比较）
        return 1;
    else
        return 0;
}

//根据里程计算出的在主方向上的位移判断是否到达目标点
bool is_arrived_odom(double distance, double current_dis)
{
    if (abs(distance-current_dis)<MIN_D)
        return 1;
    else
        return 0;
}


double get_now_angle(Eigen::Quaterniond angle)
{
    double q_x,q_y,q_z,q_w;
    q_x=angle.x();
    q_y=angle.y();
    q_z=angle.z();
    q_w=angle.w();

    double tan_angle=atan2(2*q_z*q_w-2*q_x*q_y,2*q_x*q_x+2*q_w*q_w-1);
    return tan_angle;
}
//Pid_control Pid;
int times, trans_flag;
//double nav_goal_list[8] = {0.0,0.0,1.5513,-0.0078,1.5503,-0.0062,1.6015,-1.8892};
//主线程
void process()
{

    // 2019-09-16  15:55 wyp
//    Pid.PID_init();

   // goal_num = 4;
    while(true)//到达目的地停止
    {
        ROS_INFO("----------------the main thread is running---------------");
        ROS_INFO("The navigation goal number is:[%d] the current gps data is [%lf][%lf] and orientation is: [%lf]", goal_num, EE_,NN_,yaw_filter);
        if(goal_num)
        {
            //输出起点和终点，包含小数点后六位
            std::cout << "start point is:" << setiosflags(ios::fixed) << nav_goal_list[0] << "," <<nav_goal_list[1] <<
                      "   end point is:" << nav_goal_list[2*goal_num-2] << "," << nav_goal_list[2*goal_num-1] << std::endl;
          //  double base_angle,base_dis;
            double base_angle,base_dis,now_angle;//,now_dis;
            for(int i=1; i<goal_num; i++)
            {
                int arrive_flag = 0;
    //            base_angle = get_angle(nav_goal_list[2*i-1],nav_goal_list[2*i-2],nav_goal_list[2*i+1],nav_goal_list[2*i]);
//                base_angle = base_angle ;
//                base_angle = get_angle_odom(nav_goal_list[2*i-1],nav_goal_list[2*i-2],nav_goal_list[2*i+1],nav_goal_list[2*i]);
                base_dis = get_distance_odom(nav_goal_list[2*i-1],nav_goal_list[2*i-2],nav_goal_list[2*i+1],nav_goal_list[2*i]);
                if(base_dis<0.00003)
                    continue;
               // now_dis=get_distance_odom(nav_goal_list[2*i-1],nav_goal_list[2*i-2],curr_odom_x,curr_odom_y);
              //  now_angle=get_now_angle(now_angle_);
                 //YAW_= now_angle;
            //    std::cout << "--------base orientation-----" << base_angle << std::endl;
               trans_flag=0;
                while (!arrive_flag)
                {
                    base_angle = get_angle(EE_,NN_,nav_goal_list[2*i+1],nav_goal_list[2*i]);
                    if(abs(YAW_global-base_angle)>2&&trans_flag==0)//如果角度偏差太大，先只调整对齐角度后再前进
                    {
                        trans_flag=0;
                        times = 0;
                        robot_control1(YAW_global,base_angle,0);
                        sleep(1);
                    }
                    else if(abs(YAW_global-base_angle)<=2)
                    {
                        trans_flag=1;
                        times++;
                        if(times>10)
                        {
                           // trans_flag=0; //允许下一次的调整角度
                            robot_control1(YAW_global, base_angle, 0.3);    //直行
                            times = 11;   //允许继续直行
                            sleep(1);
                        }

                    }
                    else
                    {
//                        controlVel_.angular.z = 0;
//                        controlVel_.linear.x = 0;
//                        controlVel_.linear.y = 0;
//                        controlVel_.linear.z = 0;
//                        robot_control_pub.publish(controlVel_);
                        sleep(1);
                    }


                    //if(times>10)


                    std::cout<<"start====================================================================start"<<std::endl;
//                    std::cout << "start position:" << setiosflags(ios::fixed) << nav_goal_list[0] << "," << nav_goal_list[1] << std::endl;
                    std::cout << "current measure position:" << EE_ << ", " << NN_ <<std::endl;
                    std::cout<<"   goal position: " << nav_goal_list[2*i] << ",  " << nav_goal_list[2*i+1] << std::endl;
                //    std::cout << "current yaw:" << yaw_filter << "  current angle:" << YAW_ << "  base angle:" << base_angle << std::endl;
                    std::cout << "  current angle:" << YAW_global << "  base angle:" << base_angle <<"  current angle rate :"<<YAW_rate<< std::endl;
                    if(is_arrived_gps(EE_, NN_, YAW_, curr_odom, nav_goal_list[2*i], nav_goal_list[2*i+1]))//&&is_arrived_odom(base_dis, curr_odom))
                //    if(is_arrived_odom(base_dis,now_dis));
                    {
                        arrive_flag = 1;
                        //goal_num--;
                    }
                    std::cout<<"end====================================================================end"<<std::endl;
                }
            }
        }
      //  sleep(1);
    }
}


//double lost_yaw,lost_yaw_flag,lost_odom_x,lost_odom_y,lost_odom_dx,lost_odom_dy;
double lost_dt, lost_last_lx,lost_last_yaw,lost_odom_ds;
clock_t pre_time=0,now_time=0;

void process1()
{
    double unz_EE,unz_NN;
    // goal_num = 4;
    while(true)//到达目的地停止
    {
        int i;
        ROS_INFO("----------------the main thread is running---------------");
        ROS_INFO("The navigation goal number is:[%d] the current gps data is [%lf][%lf] and orientation is: [%lf]", goal_num, EE_,NN_,yaw_filter);
        if(goal_num)
        {
            //输出起点和终点，包含小数点后六位
            std::cout << "start point is:" << setiosflags(ios::fixed) << nav_goal_list[0] << "," <<nav_goal_list[1] <<
                      "   end point is:" << nav_goal_list[2*goal_num-2] << "," << nav_goal_list[2*goal_num-1] << std::endl;
            //  double base_angle,base_dis;
            double base_angle,base_dis,now_angle;//,now_dis;
            for(i=1; i<goal_num; i++)
            {
                int arrive_flag = 0;
//                base_angle = get_angle(nav_goal_list[2*i-1],nav_goal_list[2*i-2],nav_goal_list[2*i+1],nav_goal_list[2*i]);
//                base_angle = base_angle ;
//                base_angle = get_angle_odom(nav_goal_list[2*i-1],nav_goal_list[2*i-2],nav_goal_list[2*i+1],nav_goal_list[2*i]);
                base_dis = get_distance_odom(nav_goal_list[2*i-1],nav_goal_list[2*i-2],nav_goal_list[2*i+1],nav_goal_list[2*i]);
                if(base_dis<0.00003)
                    continue;
                // now_dis=get_distance_odom(nav_goal_list[2*i-1],nav_goal_list[2*i-2],curr_odom_x,curr_odom_y);
                //  now_angle=get_now_angle(now_angle_);
                //YAW_= now_angle;
                //    std::cout << "--------base orientation-----" << base_angle << std::endl;
                while (!arrive_flag)
                {
		            now_time=clock();
                    double dt=0.1;
                    Eigen::VectorXd state;
//

                    std::cout<<std::endl<<"start====================================================================start"<<std::endl;
                    std::cout << "current measure position:" << EE_ << ", " << NN_ <<std::endl;
//                      state=fusion(dt,YAW_rate,YAW_global,curr_linear_x*dt,NN_,EE_);//缺角速度，轮式里程计
//                      double arc=data_point.get_arc();
//                      double now_la,now_lon,now_YAW;
//                      now_la=state[1]/arc+first_lat;
//                      now_lon=state[0]/arc/cos(now_la*M_PI/180)+first_lon;
//                      now_YAW=state[2];

                    if(NN_==0||EE_==0)
                    {
                        lost_last_lx=curr_linear_x;
                        lost_dt=(double)(now_time-pre_time)/CLOCKS_PER_SEC;
                       lost_odom_ds = lost_last_lx*lost_dt;
		       double xx=cos(YAW_global*M_PI/180)*lost_odom_ds;
		       double yy=sin(YAW_global*M_PI/180)*lost_odom_ds;
		       double arc = 2.0 * M_PI * (6378388.0 + 51)/360.0;
		       unz_NN+=xx/arc+first_lat;
                       unz_EE+=yy/arc/cos(NN_*M_PI/180)+first_lon;

                    }
                    else
                    {
                        unz_EE=EE_;
                        unz_NN=NN_;
                    }
                    


                    base_angle = get_angle(unz_NN,unz_EE,nav_goal_list[2*i+1],nav_goal_list[2*i]);
//                    cout<<"base angle mid :"<<base_angle<<endl;
                    if(!((unz_EE==0&&unz_NN==0)||(unz_EE==100&&unz_NN==100)))
                    {

                        if (abs(YAW_global - base_angle) > 2 && trans_flag == 0)//如果角度偏差太大，先只调整对齐角度后再前进
                        {cout<<"odom "<<curr_linear_x<<endl;
                            trans_flag = 0;
                            times = 0;
                            robot_control1(YAW_global, base_angle, 0);
                            sleep(1);
                        } else if (abs(YAW_global - base_angle) <= 2) {
//                        cout<<"21123sdasdasdad"<<endl;
                            trans_flag = 1;
                            times++;
                            //允许下一次的调整角度
                            {
                                // trans_flag=0; //允许下一次的调整角度
                                robot_control1(YAW_global, base_angle, 1);    //直行
//                            times = 11;   //允许继续直行
                                sleep(1);
                            }

//                        cout<<"times "<<times<<" trans flag "<<trans_flag<<endl;

                        } else {
//                        controlVel_.angular.z = 0;
//                        controlVel_.linear.x = 0;
//                        controlVel_.linear.y = 0;
//                        controlVel_.linear.z = 0;
//                        robot_control_pub.publish(controlVel_);
                            sleep(1);
                        }
                    }
                    if(abs(YAW_global-base_angle)>5)
                    {trans_flag=0;}

                    //if(times>10)



//                    std::cout << "start position:" << setiosflags(ios::fixed) << nav_goal_list[0] << "," << nav_goal_list[1] << std::endl;
                    std::cout << "fusion position:" << unz_EE << ", " << unz_NN <<std::endl;
                    std::cout<<"   goal position: " << nav_goal_list[2*i] << ",  " << nav_goal_list[2*i+1] << std::endl;
                    //    std::cout << "current yaw:" << yaw_filter << "  current angle:" << YAW_ << "  base angle:" << base_angle << std::endl;
                    std::cout << "  current angle:" << YAW_global << "  base angle:" << base_angle <<"  current angle rate :"<<YAW_rate<< std::endl;
                    if(is_arrived_gps(unz_EE, unz_NN, YAW_, curr_odom, nav_goal_list[2*i], nav_goal_list[2*i+1]))//&&is_arrived_odom(base_dis, curr_odom))
                        //    if(is_arrived_odom(base_dis,now_dis));
                    {
                        arrive_flag = 1;
                        //goal_num--;
                    }
                    std::cout<<"end====================================================================end"<<std::endl;
		            pre_time=now_time;
                }
            }
        }
        //  sleep(1);
        if(i==goal_num) {
            controlVel_.angular.z = 0;
            controlVel_.linear.x = 0;
            controlVel_.linear.y = 0;
            controlVel_.linear.z = 0;
            robot_control_pub.publish(controlVel_);
            break;
        }
    }
}






//


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    SmartCarKeyboardTeleopNode tbk;
    boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));
    ros::Rate loop_rate(10);
    robot_control_pub = tbk.n_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);


    ros::Subscriber sub = tbk.n_.subscribe("/gps_pose",1000,chatterCallback);
    ros::Subscriber sub0 = tbk.n_.subscribe("/yaw_odom",1000,chatterCallback0);
//    ros::Subscriber sub1 = tbk.n_.subscribe("nav_goal", 1000, chatterCallback1);
    ros::Subscriber sub2 = tbk.n_.subscribe("odom", 1000, odomCallback);
//     ros::Subscriber sub3 = tbk.n_.subscribe("imu_data", 1000, odomCallback3);
    std::thread measurement_process{process1};
    ros::spin();

    t.interrupt();
    t.join();
    tbk.stopRobot();
    tcsetattr(kfd, TCSANOW, &cooked);

    return 0;
}

int num =0;
void SmartCarKeyboardTeleopNode::keyboardLoop()
{
    char c;
    double max_tv = walk_vel_;
    double max_rv = yaw_rate_;
    bool dirty = false;
    int speed = 0;
    int turn = 0;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    for(;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (dirty == true)
            {
                stopRobot();
                dirty = false;
            }

            continue;
        }

        switch(c)
        {
            case KEYCODE_W:
                max_tv = walk_vel_;
                speed = 1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_S:
                max_tv = walk_vel_;
                speed = -1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_A:
                max_rv = yaw_rate_;
                speed = 0;
                turn = 1;
                dirty = true;
                break;
            case KEYCODE_D:
                max_rv = yaw_rate_;
                speed = 0;
                turn = -1;
                dirty = true;
                break;

            case KEYCODE_Q:
                flag_control = !flag_control;
                break;

            case KEYCODE_W_CAP:
                max_tv = run_vel_;
                speed = 1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_S_CAP:
                max_tv = run_vel_;
                speed = -1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_A_CAP:
                max_rv = yaw_rate_run_;
                speed = 0;
                turn = 1;
                dirty = true;
                break;
            case KEYCODE_D_CAP:
                max_rv = yaw_rate_run_;
                speed = 0;
                turn = -1;
                dirty = true;
                break;
            default:
                max_tv = walk_vel_;
                max_rv = yaw_rate_;
                speed = 0;
                turn = 0;
                dirty = false;
        }

        cmdvel_.linear.x = speed * max_tv;
        cmdvel_.angular.z = turn * max_rv;
        pub_.publish(cmdvel_);
    }
}
