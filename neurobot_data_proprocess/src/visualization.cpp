#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <fstream>

std::ofstream outfile;
int flag;
double gps0_x,gps0_y;
void chatCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
 // outfile << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "\n";
 if(!flag)
 {
   gps0_x = msg->pose.pose.position.x;
   gps0_y = msg->pose.pose.position.y;
   flag = 1;
 }
 else
 {
   outfile << msg->pose.pose.position.x-gps0_x << "," << msg->pose.pose.position.y-gps0_y << "\n";
 }

  ROS_INFO("data is reading-----");
}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"visualization_node");
  ros::NodeHandle n;
  outfile.open("/home/yonghui/gps_odom.txt");
  ros::Subscriber sub = n.subscribe("gps_odom",1000,chatCallback);
  ros::spin();
  return 0;
}
