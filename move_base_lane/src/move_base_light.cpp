//
// Created by yonghui on 19-5-23.
//

#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include "move_base/move_base_light.h"

namespace move_base
{
    MoveBaseLight::MoveBaseLight(tf::TransformListener &tf):
    nh_(), private_nh_("~"), action_nh_("move_base"), simple_nh_("move_base_simple"),
    tf_(tf), as_(NULL)
    {
        ROS_DEBUG("Use MoveBaseLight!");
        //! init actionlib_server
        //! nh_ must be class member, otherwise strange error will happen! (http://wiki.ros.org/cn/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29#CA-4489f75655c29f8fd70e22d2b43906c08f386f4e_10)
        // node_handle, server_name, execute_callback, is_auto_start
        as_ = new MoveBaseActionServer(nh_, "move_base", boost::bind(&MoveBaseLight::executeCB, this, _1), false);

        //! read frame_id
        private_nh_.param("gloabl_frame", global_frame_, std::string("/map"));
        private_nh_.param("robot_base_frame", robot_base_frame_, std::string("/base_footprint"));
        private_nh_.param("control_frequency", control_frequence_, 20.0);
        private_nh_.param("max_linear_vel", max_linear_vel_, 0.5);
        private_nh_.param("min_linear_vel", min_linear_vel_, 0.05);
        private_nh_.param("max_angular_vel", max_angular_vel_, 0.5);
        private_nh_.param("min_angular_vel", min_angular_vel_, 0.05);
        private_nh_.param("transform_tolerance", transform_tolerance_, 0.3);

        //! init pub and sub
        current_goal_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("current_goal", 0);
        follow_flag_pub_ = private_nh_.advertise<std_msgs::Bool>("follow_flag", 1);
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        action_goal_pub_ = action_nh_.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
        goal_sub_ = simple_nh_.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBaseLight::goalCB, this, _1));

        //! start action server
        as_->start();
        ROS_DEBUG("MoveBaseLight initialize complete!");
    }


    MoveBaseLight::~MoveBaseLight()
    {
        if (as_ != NULL)
        {
            delete as_;
        }
    }


    /**
     * @brief 发布停车速度
     */
    void MoveBaseLight::publishZeroVelocity(){
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
    }


    /**
     * @brief 判断四元数是否正确
     * @param q
     * @return
     */
    bool MoveBaseLight::isQuaternionValid(const geometry_msgs::Quaternion& q)
    {
        //first we need to check if the quaternion has nan's or infs
        if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
        {
            ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
            return false;
        }
        return true;
    }


    /**
     * @brief 从goal_pose_msg 的坐标系变换到global frame id下
     * @param goal_pose_msg     输入位姿
     * @param global_pose_msg   输出全局位姿
     * @return
     */
    bool MoveBaseLight::goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg, geometry_msgs::PoseStamped &global_pose_msg)
    {
        tf::Stamped<tf::Pose> goal_pose, global_pose;
        poseStampedMsgToTF(goal_pose_msg, goal_pose);  // 转换到tf::Stamped<tf::Pose>格式

        //just get the latest available transform... for accuracy they should send
        //goals in the frame of the planner
        goal_pose.stamp_ = ros::Time();

        try
        {
            tf_.transformPose(global_frame_, goal_pose, global_pose);
        }
        catch(tf::TransformException& ex)
        {
            ROS_WARN("Failed to transform the goal pose from [%s] into the %s frame: [%s]",
                     goal_pose.frame_id_.c_str(), global_frame_.c_str(), ex.what());
            return false;
        }

        tf::poseStampedTFToMsg(global_pose, global_pose_msg);  // 再从tf::Stamped<tf::Pose>转换到geometry_msgs::PoseStamped
        return true;
    }


    /**
     * @brief 更新当前目标点
     * @param move_base_goal 用于更新的目标指针
     * @return
     */
    bool MoveBaseLight::updateCurrentGoal(const move_base_msgs::MoveBaseGoal &move_base_goal)
    {
        //! 获得goal, 转换到局部坐标系, 发布current_goal
        if (!isQuaternionValid(move_base_goal.target_pose.pose.orientation))
        {
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return false;
        }
        geometry_msgs::PoseStamped goal;
        if (!goalToGlobalFrame(move_base_goal.target_pose, goal))
        {
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because fail goal frame transform");
            return false;
        }
//        current_goal_ = goal;
//        current_goal_pub_.publish(current_goal_);  // -> /move_base/current_goal
        current_goal_pub_.publish(goal);  // -> /move_base/current_goal
        tf::poseStampedMsgToTF(goal, current_goal_);
        return true;
    }


    /**
     * @brief 获得底盘在全局坐标系下的位姿
     * @param global_pose 输出的底盘全局位姿
     * @return
     */
    bool MoveBaseLight::getRobotPose(tf::Stamped<tf::Pose> &global_pose)
    {
        tf::Stamped<tf::Pose> robot_pose;  // 底盘位姿
        tf::Stamped<tf::Pose> goal_pose;
        //! 设置robot_pose为/base_link的原点
        robot_pose.setIdentity();
        robot_pose.frame_id_ = robot_base_frame_;
        robot_pose.stamp_ = ros::Time();
        ros::Time current_time = ros::Time::now();
        //! 监听 /base_link -> /map
        try
        {
            tf_.transformPose(global_frame_, robot_pose, global_pose);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Fail to transform robot pose from [%s] into the frame: [%s]",
                    robot_base_frame_.c_str(), global_frame_.c_str());
            return false;
        }
        //! 检查坐标变换的时间戳没有超时
        if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_)
        {
            ROS_WARN_THROTTLE(1.0,
                              "Transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                              current_time.toSec(), global_pose.stamp_.toSec(), transform_tolerance_);
            return false;
        }
        return true;
    }

    /**
     * @brief 获得tf::Pose的yaw角
     * @param pose
     * @return
     */
    double MoveBaseLight::getPoseYaw(const tf::Stamped<tf::Pose> &pose)
    {
        double roll, pitch, yaw;
        tf::Matrix3x3(pose.getRotation()).getRPY(roll, pitch, yaw);  // TODO: assume range (-pi, pi)
        return yaw;
    }


    /**
     * @brief 将角度规范化到(-pi, pi)区间中
     * @param angular
     * @return
     */
    double MoveBaseLight::normalizeAngular(double angular)
    {
        if (angular < -M_PI)
        {
            angular += 2*M_PI;
        }
        if (angular > M_PI)
        {
            angular -= 2*M_PI;
        }
        return angular;
    }


    /**
     * @brief 计算: 底盘->目标 与 底盘方向 之间的夹角, 用于判断起始转向 / 停止条件
     * @param global_robot_pose
     * @param global_goal_pose
     * @return
     */
    double MoveBaseLight::getGoalDirection(const tf::Stamped<tf::Pose> &global_robot_pose, const tf::Stamped<tf::Pose> &global_goal_pose)
    {
        double diff_x = global_goal_pose.getOrigin().x() - global_robot_pose.getOrigin().x();
        double diff_y = global_goal_pose.getOrigin().y() - global_robot_pose.getOrigin().y();
        double goal_angular = atan2(diff_y, diff_x);  // (-pi, pi)
        double angular = getPoseYaw(global_robot_pose) - goal_angular;
        ROS_DEBUG("goal angular: %f, robot_angular: %f, direction angular: %f"
                , goal_angular, getPoseYaw(global_robot_pose), angular);
        return fabs(normalizeAngular(angular));
    }


    /**
     * @brief move_goal_simple 主题订阅的回调函数
     * @param msgGoal
     */
    void MoveBaseLight::goalCB(const geometry_msgs::PoseStamped::ConstPtr &pose_goal)
    {
        /**
         * MoveBaseActionGoal
         * {
         *      Header header
         *      actionlib_msgs/GoalID goal_id
         *      MoveBaseGoal goal
         * }
         */
        move_base_msgs::MoveBaseActionGoal move_base_goal;
        move_base_goal.header.stamp = ros::Time::now();
        move_base_goal.goal.target_pose = *pose_goal;
        action_goal_pub_.publish(move_base_goal);  // -> /move_base/goal, 注意这里的消息类型是MoveBaseActionGoal
    }

    /**
     * @brief action server 服务回调函数
     * @param move_base_goal
     */
    void MoveBaseLight::executeCB(const move_base_msgs::MoveBaseGoalConstPtr &move_base_goal)
    {
        if (!updateCurrentGoal(*move_base_goal))
        {
            ROS_WARN("Invalid goal!");
            resetState();
            return;
        }
        ROS_DEBUG("Receive goal message successfully!");
        //! 开始循环执行action server callback循环
        ros::Rate rate(control_frequence_);
        ros::NodeHandle n;
        while (n.ok())
        {
            //! 轮询实现查询有关抢占请求的信息, 也就是说判断有goal抢占了, 重新设置goal
            // 抢占允许当前执行的目标被打断, 或者取消
            if (as_->isPreemptRequested())
            {
                //! 判断是否新的可用的goal可以被处理, 更换目标点
                if (as_->isNewGoalAvailable())
                {
                    move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
                    if (!updateCurrentGoal(new_goal))
                    {
//                        ROS_WARN("Invalid goal!");
                        resetState();
                        return;
                    }
                    judge_direct_ = true;
                }
                //! 抢占的目标不可用, 取消当前执行的目标
                else
                {
                    resetState();
                    as_->setPreempted();
                    return;
                }
            }  // 抢占处理结束

            //! 根据目标点判断底盘移动方向
            if (judge_direct_)  // 只有有了新的目标才会去判断移动方向
            {
                //! 首先要停下巡线程序
                publishFollowFlag(false);
                rate.sleep();

                judge_direct_ = false;
                if (!getRobotPose(global_robot_pose_))
                {
                    ROS_WARN("Fail to get goal direction: Get robot glabal pose fail.");
                    resetState();
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal when getting robot global pose in judging start direction");
                    return;
                }
//                tf::Stamped<tf::Pose> goal_pose;
//                tf::poseStampedMsgToTF(current_goal_, goal_pose);
//                double start_angular = getGoalDirection(global_robot_pose_, goal_pose);
                double start_angular = getGoalDirection(global_robot_pose_, current_goal_);

                //! 我们假设地图基本是长直的, 如果目标和底盘方向夹角>pi/2, 考虑掉头
                if (start_angular > M_PI/2)
                {
                    ROS_DEBUG("Start point is in front of goal, base should first turn around!");
                    double last_angular = getPoseYaw(global_robot_pose_);
                    double turn_angular = 0.;
                    bool turn_round_flag = true;
                    while (n.ok() && turn_round_flag && getRobotPose(global_robot_pose_))
                    {
                        double current_angular = getPoseYaw(global_robot_pose_);
                        double delta_angular = normalizeAngular(current_angular - last_angular);
                        turn_angular += delta_angular;
                        last_angular = current_angular;
//                        ROS_DEBUG("Turned around: %f, delta turn around: %f", turn_angular, delta_angular);
                        if (fabs(turn_angular) > M_PI)
                        {
                            publishZeroVelocity();
                            turn_round_flag = false;
                        }
                        else
                        {
                            geometry_msgs::Twist cmd_vel;
                            cmd_vel.linear.x = 0;
                            cmd_vel.linear.y = 0;
                            cmd_vel.linear.z = 0;
                            cmd_vel.angular.x = 0;
                            cmd_vel.angular.y = 0;
                            cmd_vel.angular.z = max_angular_vel_;
                            vel_pub_.publish(cmd_vel);
                        }
                        rate.sleep();
                    }

                    //! 判断是否正常停止
                    if (turn_round_flag)
                    {
                        ROS_WARN("Abnormal stop when turning around.");
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal when turning around");
                        return;
                    }
                    //! 允许巡线程序
                    else
                    {
                        publishFollowFlag(true);
                        ROS_DEBUG("Start ccd camera follow lane!");
                    }
                }  // 掉头循环结束
                //! 方向一致, 直接巡线开始
                else
                {
                    publishFollowFlag(true);
                    ROS_DEBUG("Start ccd camera follow lane!");
                }
            }  // 判断掉头结束
            //! 巡线中
            else
            {
                if (!getRobotPose(global_robot_pose_))
                {
                    ROS_WARN("Fail to get goal direction: Get robot glabal pose fail.");
                    resetState();
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal when getting robot global pose in following");
                    return;
                }
                //! 判断巡线程序是否应该停止
                double goal_angular = getGoalDirection(global_robot_pose_, current_goal_);
                double goal_distance = hypot(
                        global_robot_pose_.getOrigin().x()-current_goal_.getOrigin().x(),
                        global_robot_pose_.getOrigin().y()-current_goal_.getOrigin().y()
                        );
                // TODO: Considering the goal outside feasible range
                if (goal_angular > M_PI/2 && goal_distance < 0.5)
                {
                    resetState();
                    as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
                    return;
                }
                //! 还没有到目标点
                else
                {
                    //! 发布feedback
                    geometry_msgs::PoseStamped robot_pose_msg;
                    tf::poseStampedTFToMsg(global_robot_pose_, robot_pose_msg);
                    move_base_msgs::MoveBaseFeedback feedback;
                    as_->publishFeedback(feedback);
                }

            }
            rate.sleep();
//            resetState();
//            return;  // TODO: only test turn around, should be delete
        }  // server主循环结束

        //! if the node is killed then we'll abort and return
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    }


    /**
     * @brief 发布follow_flag主题的一个简单包装
     * @param follow_flag
     */
    void MoveBaseLight::publishFollowFlag(bool follow_flag)
    {
        std_msgs::Bool follow_flag_msg;
        follow_flag_msg.data = follow_flag;
        follow_flag_pub_.publish(follow_flag_msg);
    }


    /**
     * @brief 重置move base的状态, 在到达目标点/当前目标被不可用goal抢占等地方被使用
     */
    void MoveBaseLight::resetState()
    {
        judge_direct_ = true;
        publishZeroVelocity();
        publishFollowFlag(false);
    }
}