//
// Created by yonghui on 19-5-23.
//

#ifndef MOVE_BASE_LANE_MOVE_BASE_LIGHT_H
#define MOVE_BASE_LANE_MOVE_BASE_LIGHT_H

#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include "ccd_camera/ccd_camera_serial.h"
#include "ccd_camera/controller.h"

namespace move_base
{
    typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

    class MoveBaseLight
    {
    public:
        /**
         * @brief 构造函数
         * @param tf    tf监听器
         */
        explicit MoveBaseLight(tf::TransformListener &tf);
        /**
         * @brief 析构函数
         */
        ~MoveBaseLight();
        /**
         * @brief
         * @param goal
         * @return
         */
        bool executeCycle(geometry_msgs::PoseStamped &goal);

    private:
        /**
         * @brief 发布停车速度
         */
        void publishZeroVelocity();
        /**
         * @brief 判断四元数是否正确
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
        bool goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg, geometry_msgs::PoseStamped &global_pose_msg);
        /**
         * @brief 更新当前目标点
         * @param move_base_goal 用于更新的目标指针
         * @return
         */
        bool updateCurrentGoal(const move_base_msgs::MoveBaseGoal &move_base_goal);
        /**
         * @brief 获得底盘在全局坐标系下的位姿
         * @param global_pose 输出的底盘全局位姿
         * @return
         */
        bool getRobotPose(tf::Stamped<tf::Pose> &global_pose);
        /**
         * @brief 获得tf::Pose的yaw角
         * @param pose
         * @return
         */
        double getPoseYaw(const tf::Stamped<tf::Pose> &pose);
        /**
         * @brief 将角度规范化到(-pi, pi)区间中
         * @param angular
         * @return
         */
        double normalizeAngular(double angular);
        /**
         * @brief 计算: 底盘->目标 与 底盘方向 之间的夹角, 用于判断起始转向 / 停止条件
         * @param global_robot_pose
         * @param global_goal_pose
         * @return
         */
        double getGoalDirection(const tf::Stamped<tf::Pose> &global_robot_pose, const tf::Stamped<tf::Pose> &global_goal_pose);
        /**
         * @brief move_goal_simple 主题订阅的回调函数
         * @param msgGoal
         */
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr &pose_goal);
        /**
         * @brief action server 服务回调函数
         * @param move_base_goal
         */
        void executeCB(const move_base_msgs::MoveBaseGoalConstPtr &move_base_goal);
        /**
         * @brief 发布start_follow的一个简单包装
         * @param follow_flag
         */
        void publishFollowFlag(bool follow_flag);
        /**
         * @brief 重置move base的状态, 在到达目标点/当前目标被不可用goal抢占等地方被使用
         */
        void resetState();

        //! listener and server
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::NodeHandle action_nh_;
        ros::NodeHandle simple_nh_;
        tf::TransformListener &tf_;
        MoveBaseActionServer *as_;

        //!topic
        ros::Publisher current_goal_pub_;  ////< @brief /move_base(_node)/current_goal
        ros::Publisher follow_flag_pub_;  ////< @brief /move_base(_node)/follow_flag
        ros::Publisher vel_pub_;  ////< @brief /cmd_vel
        ros::Publisher action_goal_pub_;  ////< @brief /move_base/goal
        ros::Subscriber goal_sub_;  ////< @brief /move_base_simple/goal

        //! param
        std::string global_frame_;  ////< @brief 全局frame
        std::string robot_base_frame_;  ////< @brief 机器人底盘frame
        double control_frequence_;  ////< @brief 循环控制频率
        double transform_tolerance_;  ////< @brief 允许的最大tf transforms时间差
        double max_linear_vel_;  ////< @brief 最大线速度
        double min_linear_vel_;  ////< @brief 最小线速度
        double max_angular_vel_;  ////< @brief 最大角速度
        double min_angular_vel_;  ////< @brief 最小角速度

        //! frame and pose
        tf::Stamped<tf::Pose> global_robot_pose_;  ////< @brief 机器人全局frame下位姿
        tf::Stamped<tf::Pose> current_goal_;  ////< @brief 当前目标路径
//        geometry_msgs::PoseStamped current_goal_;  ////< @brief 当前目标路径

        //! state and flag
        bool judge_direct_;  ////< @brief 获得新目标点, 判断底盘移动方向的请求

        //! ccd_camera

    };
}

#endif //MOVE_BASE_LANE_MOVE_BASE_LIGHT_H
