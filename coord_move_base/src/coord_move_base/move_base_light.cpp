//
// Created by yonghui on 19-9-3.
//

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <angles/angles.h>
#include "move_base_light.h"
#include "convertor.h"

namespace coord_move_base
{
    MoveBaseLight::MoveBaseLight(tf::TransformListener &tf):
    nh_(), private_nh_("~"), action_nh_("move_base"), simple_nh_("move_base_simple"), pid_nh_("~/pid"),
    tf_(tf), planner_loader_("nav_core", "nav_core::BaseGlobalPlanner")
    {
        ROS_DEBUG("Use MoveBaseLight!");
        //! init actionlib_server
        //! nh_ must be class member, otherwise strange error will happen!
        //! (http://wiki.ros.org/cn/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29#CA-4489f75655c29f8fd70e22d2b43906c08f386f4e_10)
        // node_handle, server_name, execute_callback, is_auto_start
        as_ = new MoveBaseActionServer(nh_, "move_base", boost::bind(&MoveBaseLight::executeCB, this, _1), false);

        //! controller
        private_nh_.param("control_frequency", control_frequence_, 20.0);
        private_nh_.param("max_linear_vel", max_linear_vel_, 0.5);
        private_nh_.param("min_linear_vel", min_linear_vel_, 0.05);
        private_nh_.param("max_angular_vel", max_angular_vel_, 0.5);
        private_nh_.param("min_angular_vel", min_angular_vel_, 0.05);
//        private_nh_.param("transform_tolerance", transform_tolerance_, 0.3);
        private_nh_.param("kp", kp_, 0.1);
        private_nh_.param("ki", ki_, 0.0);
        private_nh_.param("kd", kd_, 0.0);
//        private_nh_.param<PidInput >("pid_input", pid_input_, TWIST);
        int idx_pid_input;
        private_nh_.param("pid_input", idx_pid_input, 0);
        pid_input_ = PidInput(idx_pid_input);
        private_nh_.param("debug", is_debug_, true);
        clt_ = Controller(kp_, ki_, kd_, max_angular_vel_, -max_angular_vel_);

        //! init pub and sub
        current_goal_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("current_goal", 0);
        follow_flag_pub_ = private_nh_.advertise<std_msgs::Bool>("follow_flag", 1);
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        action_goal_pub_ = action_nh_.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
        goal_sub_ = simple_nh_.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBaseLight::goalCB, this, _1));
        // debug topic
        angular_z_pub_ = pid_nh_.advertise<std_msgs::Float32>("angular_z", 100);
        input_pub_ = pid_nh_.advertise<std_msgs::Float32>("input", 100);
        output_pub_ = pid_nh_.advertise<std_msgs::Float32>("output", 100);

        //! global costmap
        private_nh_.param("global_costmap/gloabl_frame", global_frame_, std::string("/map"));
        private_nh_.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("/base_footprint"));
        planner_costmap_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
        planner_costmap_->pause();

        //! target planner
        string target_planner_name;
        private_nh_.param("base_global_planner", target_planner_name, string("global_planner/CoordTargetPlannerROS"));
        target_planner_ = planner_loader_.createInstance(target_planner_name);
        target_planner_->initialize(planner_loader_.getName(target_planner_name), planner_costmap_);
        planner_costmap_->start();

        //! start action server
        as_->start();
        ROS_DEBUG("MoveBaseLight initialize complete!");
    }


    MoveBaseLight::~MoveBaseLight()
    {
        if (as_ != NULL)
            delete as_;
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
        last_cmd_vel_ = cmd_vel;  // 重置上一周期的控制速度

        // debug
        if (is_debug_)
        {
            std_msgs::Float32 angular_z_msg, input_msg, output_msg;
            angular_z_msg.data = 0.0;
            input_msg.data = 0.0;
            output_msg.data = 0.0;
            angular_z_pub_.publish(angular_z_msg);
            input_pub_.publish(input_msg);
            output_pub_.publish(output_msg);
        }
    }


    /**
     * @brief 四元数检验, 平面导航要求旋转基本都是偏航角(Yaw)
     * @param q
     * @return
     */
    bool MoveBaseLight::isQuaternionValid(const geometry_msgs::Quaternion& q){
        //first we need to check if the quaternion has nan's or infs
        if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
            ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
            return false;
        }

        tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

        //next, we need to check if the length of the quaternion is close to zero
        //! 四元数不能全0
        if(tf_q.length2() < 1e-6){
            ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
            return false;
        }

        //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
        tf_q.normalize();

        tf::Vector3 up(0, 0, 1);

        double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

        //! 平面导航要求偏航角
        if(fabs(dot - 1) > 1e-3){
            ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
            return false;
        }

        return true;
    }


    /**
     * @brief 从goal_pose_msg 的坐标系变换到global frame id下
     *
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
     * @brief 判断当前目标点是否有效,对目标点坐标转换后更新当前目标点
     *
     * @param move_base_goal 用于更新的目标指针
     * @return
     */
    bool MoveBaseLight::updateCurrentGoal(const move_base_msgs::MoveBaseGoal &move_base_goal, geometry_msgs::PoseStamped &goal)
    {
        //! 四元数检验, 平面导航要求旋转基本都是偏航角(Yaw)
        if (!isQuaternionValid(move_base_goal.target_pose.pose.orientation))
        {
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return false;
        }
        //! 转换到全局坐标系, 发布到current_goal
//        geometry_msgs::PoseStamped goal;
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
        double robot_angular = Convertor::getYaw(global_robot_pose);
        double angular =  robot_angular - goal_angular;
        ROS_DEBUG("goal angular: %f, robot_angular: %f, direction angular: %f"
        , goal_angular, robot_angular, angular);
        return fabs(angles::normalize_angle(angular));
    }


    double MoveBaseLight::getPathDirection(const vector<geometry_msgs::PoseStamped> &path)
    {

    }


    /**
     * @brief 计算PID控制器的输入量
     */
    double MoveBaseLight::getPidInput(const geometry_msgs::PoseStamped &pose_msg)
    {
        double input = 0.;
        switch (pid_input_)
        {
            case TWIST:
            {
                //! 计算前后两个path查分角速度, 和当前底盘偏航角速度差
                if (last_target_path_.empty())
                    break;  // 控制开始的第一个周期, 没有角速度上的控制
                int num_point = target_path_.size();  // 计算角度使用轨迹点数量, 不能太长
                double path_angle_diff = 0.;
                for (int i=0; i<num_point; i++)
                    path_angle_diff += angles::normalize_angle(Convertor::getYaw(target_path_[i]) - Convertor::getYaw(last_target_path_[i]));
                input = path_angle_diff * control_frequence_ / num_point - last_cmd_vel_.angular.z;
                break;
            }
            case ANGLE:
            {
                //! 平均前几个轨迹点相对当前底盘方向的偏航角差
                int num_point = std::min(5ul, target_path_.size());  // 计算角度使用轨迹点数量, 不能太长
                double curr_angle = Convertor::getYaw(pose_msg);
                double angle_diff = 0.;
                for (int i=0; i<num_point; i++)
                    angle_diff += angles::normalize_angle(Convertor::getYaw(target_path_[i]) - curr_angle);
                input = angle_diff / num_point;
                break;
            }
            default:
                break;
        }
        return input;
    }


    /**
     * @brief /move_base_simple/goal回调函数, 不需要actionlib client就能直接指定目标导航,
     * rviz界面导航就是发布到这个主题上
     *
     * @param goal
     */
    void MoveBaseLight::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
        ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
        move_base_msgs::MoveBaseActionGoal action_goal;
        action_goal.header.stamp = ros::Time::now();
        action_goal.goal.target_pose = *goal;
        //! 同样是组装一个ActionGoal, 然后通过actionlib publisher发布
        action_goal_pub_.publish(action_goal);
    }


    /**
     * @brief actionlib server的执行函数, 包括了整个导航过程的大循环
     *
     * @param goal
     */
    void MoveBaseLight::executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
    {
        geometry_msgs::PoseStamped goal_msg;

        if (!updateCurrentGoal(*goal, goal_msg))  // 判断新目标有效性, 设置新目标点
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
                    if (!updateCurrentGoal(new_goal, goal_msg))  // 判断新目标有效性, 设置新目标点
                    {
                        ROS_WARN("Invalid goal!");
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
//                publishFollowFlag(false);

                rate.sleep();

                judge_direct_ = false;
                if (!planner_costmap_->getRobotPose(global_robot_pose_))
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
                    double last_angular = Convertor::getYaw(global_robot_pose_);
                    double turn_angular = 0.;
                    bool turn_round_flag = true;

                    //! 用积分的方法判断是否转过180度
                    while (n.ok() && turn_round_flag && planner_costmap_->getRobotPose(global_robot_pose_))
                    {
                        double current_angular = Convertor::getYaw(global_robot_pose_);
                        double delta_angular = angles::normalize_angle(current_angular - last_angular);

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
                }  // 掉头循环结束
            }  // 判断掉头结束
            //! 路径控制
            else
            {
                if (!planner_costmap_->getRobotPose(global_robot_pose_))
                {
                    ROS_WARN("Fail to get goal direction: Get robot glabal pose fail.");
                    resetState();
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal when getting robot global pose in following");
                    return;
                }

                //! 单步控制循环
                bool done = executeCycleWoTC();

                //! 到达判断
                if (done)
                {
                    resetState();
                    as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
                    return;
                }
                else
                {
                    //! 发布feedback
                    geometry_msgs::PoseStamped robot_pose_msg;
                    tf::poseStampedTFToMsg(global_robot_pose_, robot_pose_msg);
                    move_base_msgs::MoveBaseFeedback feedback;
                    as_->publishFeedback(feedback);
                }
            }  // 路径控制结束
            rate.sleep();
        }  // server 主循环结束

        //! if the node is killed then we'll abort and return
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    }


    bool MoveBaseLight::executeCycleWoTC()
    {
        //! 判断已经到达终点
        double goal_angular = getGoalDirection(global_robot_pose_, current_goal_);
        double goal_distance = hypot(
                global_robot_pose_.getOrigin().x()-current_goal_.getOrigin().x(),
                global_robot_pose_.getOrigin().y()-current_goal_.getOrigin().y()
        );
        if (goal_angular > M_PI/2 && goal_distance < 0.5)
        {
            return true;
        }
        //! 还没有到达终点, 做前向控制
        else
        {
            //! 进行一次target planner
            geometry_msgs::PoseStamped robot_pose_msg;
            tf::poseStampedTFToMsg(global_robot_pose_, robot_pose_msg);
            if (!target_planner_->makePlan(robot_pose_msg, robot_pose_msg, target_path_) )
            {
                // 规划失败, 停下来
                publishZeroVelocity();
                return false;
            }

            //! 单次控制周期
            double error = getPidInput(robot_pose_msg);
            double angle_clt = clt_.output(error);
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = max_linear_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = std::max(std::min(angle_clt, max_angular_vel_), -max_angular_vel_);
            vel_pub_.publish(cmd_vel);
            last_cmd_vel_ = cmd_vel;
            last_target_path_ = target_path_;

            // debug
            if (is_debug_)
            {
                std_msgs::Float32 angular_z_msg, input_msg, output_msg;
                angular_z_msg.data = cmd_vel.angular.z;
                input_msg.data = error;
                output_msg.data = angle_clt;
                angular_z_pub_.publish(angular_z_msg);
                input_pub_.publish(input_msg);
                output_pub_.publish(output_msg);
            }
            return false;
        }
    }


    /**
     * @brief 重置move base的状态, 在到达目标点/当前目标被不可用goal抢占等地方被使用
     */
    void MoveBaseLight::resetState()
    {
        judge_direct_ = true;
        publishZeroVelocity();
        target_path_.clear();
        last_target_path_.clear();
    }
}