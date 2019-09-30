//
// Created by yonghui on 19-9-3.
//

#ifndef COORD_MOVE_BASE_MOVE_BASE_LIGHT_H
#define COORD_MOVE_BASE_MOVE_BASE_LIGHT_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_loader.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>

#include <vector>
#include <string>
#include <queue>
#include <boost/shared_ptr.hpp>

#include "controller.h"
using namespace std;

namespace coord_move_base
{
    typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

    enum PidInput {TWIST, ANGLE};

    class MoveBaseLight
    {
    public:
        MoveBaseLight(tf::TransformListener &tf);

        ~MoveBaseLight();

    protected:
        /**
         * @brief 发布停车速度
         */
        void publishZeroVelocity();

        /**
         * @brief 四元数检验, 平面导航要求旋转基本都是偏航角(Yaw)
         * @param q
         * @return
         */
        bool isQuaternionValid(const geometry_msgs::Quaternion& q);

        /**
         * @brief 从goal_pose_msg 的坐标系变换到global frame id下
         * @param goal_pose_msg     输入位姿
         * @param global_pose_msg   输出全局位姿
         * @return
         */
        bool goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg, geometry_msgs::PoseStamped &global_pose_msg);

        /**
         * @brief 更新当前目标点
         * @param move_base_goal 用于更新的目标指针
         * @return
         */
        bool updateCurrentGoal(const move_base_msgs::MoveBaseGoal &move_base_goal, geometry_msgs::PoseStamped &goal);

        /**
         * @brief 计算: 底盘->目标 与 底盘方向 之间的夹角, 用于判断起始转向 / 停止条件
         * @param global_robot_pose
         * @param global_goal_pose
         * @return
         */
        double getGoalDirection(const tf::Stamped<tf::Pose> &global_robot_pose, const tf::Stamped<tf::Pose> &global_goal_pose);


        /**
         * @brief 计算path起始角度的偏航角, 用于计算PID控制器的输入量
         * @param path
         * @return
         */
        double getPathDirection(const vector<geometry_msgs::PoseStamped> &path);

        /**
         * @brief 计算PID控制器的输入量
         */
        double getPidInput(const geometry_msgs::PoseStamped &pose_msg);

        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

        void executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal);

        /**
         * @brief 没有local planner, 直接使用target_planner的路径进行控制
         * @return
         */
        bool executeCycleWoTC();

        void resetState();

        //! listener and server
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::NodeHandle action_nh_;
        ros::NodeHandle simple_nh_;
        ros::NodeHandle pid_nh_;
        tf::TransformListener &tf_;
        MoveBaseActionServer *as_;

        //!topic
        ros::Publisher current_goal_pub_;  ////< @brief topic: /move_base(_node)/current_goal
        ros::Publisher follow_flag_pub_;  ////< @brief topic: /move_base(_node)/follow_flag
        ros::Publisher vel_pub_;  ////< @brief topic: /cmd_vel
        ros::Publisher action_goal_pub_;  ////< @brief topic: /move_base/goal
        ros::Subscriber goal_sub_;  ////< @brief topic: /move_base_simple/goal
        // debug topic
        ros::Publisher angular_z_pub_;  ///<@brief debug topic: /move_base(_node)/pid/angular_z
        ros::Publisher input_pub_;  ///<@brief debug topic: /move_base(_node)/pid/input
        ros::Publisher output_pub_;  ///<@brief debug topic: /move_base(_node)/pid/output

        //! frame id
        std::string global_frame_;  ////< @brief 全局frame
        std::string robot_base_frame_;  ////< @brief 机器人底盘frame

        //! costmap and planner
        costmap_2d::Costmap2DROS *planner_costmap_;  ///<@brief global costmap
        boost::shared_ptr<nav_core::BaseGlobalPlanner> target_planner_;  ///<@brief target planner, 实现global planner插件
        pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> planner_loader_;  ///<@brief global planner pluginlib loader

        //! frame and pose
        tf::Stamped<tf::Pose> global_robot_pose_;  ////< @brief 机器人全局frame下位姿
        tf::Stamped<tf::Pose> current_goal_;  ////< @brief 当前目标路径
        geometry_msgs::PoseStamped goal_pose_;  ///<@brief 目标点位姿
        vector<geometry_msgs::PoseStamped> target_path_;  ///<@brief target planner规划路径
        vector<geometry_msgs::PoseStamped> last_target_path_;  ///<@brief 上一轮的targe_planner规划路径

        //! control
        double control_frequence_;  ////< @brief 循环控制频率
//        double transform_tolerance_;  ////< @brief 允许的最大tf transforms时间差
        double max_linear_vel_;  ////< @brief 最大线速度
        double min_linear_vel_;  ////< @brief 最小线速度
        double max_angular_vel_;  ////< @brief 最大角速度
        double min_angular_vel_;  ////< @brief 最小角速度
        double kp_;  ///<@brief PID P
        double ki_;  ///<@brief PID I
        double kd_;  ///<@brief PID D
        bool is_debug_;  ///@brief 调试标志位
        geometry_msgs::Twist last_cmd_vel_;  ///<@brief last current cmd_vel
        PidInput pid_input_;  ///<@brief PID输入量选择: 角度差输入/角速度差输入
        Controller clt_;  ///<@brief PID controller

        //! state and flag
        bool judge_direct_;  ////< @brief 获得新目标点, 判断底盘移动方向的请求
    };
}

#endif //COORD_MOVE_BASE_MOVE_BASE_LIGHT_H
