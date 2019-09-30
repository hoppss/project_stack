//
// Created by yonghui on 19-9-3.
//

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <angles/angles.h>
#include <cmath>
#include "move_base_obstacle.h"
#include "convertor.h"

namespace obstacle_avoidance
{
    MoveBaseObstacle::MoveBaseObstacle(tf::TransformListener &tf):
    nh_(), private_nh_("~"), action_nh_("move_base"), simple_nh_("move_base_simple"),
    tf_(tf), local_loader_("nav_core", "nav_core::BaseLocalPlanner"), goal_init_(true)
    {
        ROS_DEBUG("Use MoveBaseObstacle!");
        //! init actionlib_server
        //! nh_ must be class member, otherwise strange error will happen!
        //! (http://wiki.ros.org/cn/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29#CA-4489f75655c29f8fd70e22d2b43906c08f386f4e_10)
        // node_handle, server_name, execute_callback, is_auto_start
        as_ = new MoveBaseActionServer(nh_, "move_base", boost::bind(&MoveBaseObstacle::executeCB, this, _1), false);

        //! controller
        private_nh_.param("control_frequency", control_frequence_, 20.0);
        private_nh_.param("max_linear_vel", max_linear_vel_, 0.5);
        private_nh_.param("min_linear_vel", min_linear_vel_, 0.05);
        private_nh_.param("max_angular_vel", max_angular_vel_, 0.5);
        private_nh_.param("min_angular_vel", min_angular_vel_, 0.05);
        private_nh_.param("debug", is_debug_, true);
        private_nh_.param("generate_path", is_generate_path_, true);
        private_nh_.param("path_resolution", path_resolution_, 0.05);
//        clt_ = Controller(kp_, ki_, kd_, max_angular_vel_, -max_angular_vel_);

        //! init pub and sub
        current_goal_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("current_goal", 0);
//        follow_flag_pub_ = private_nh_.advertise<std_msgs::Bool>("follow_flag", 1);
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        action_goal_pub_ = action_nh_.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
        goal_sub_ = simple_nh_.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBaseObstacle::goalCB, this, _1));
        // path
        gp_pub_ = nh_.advertise<nav_msgs::Path>("global_path", 1);
        gp_sub_ = nh_.subscribe<nav_msgs::Path>("global_path", 1, &MoveBaseObstacle::globalPathCb, this);

        //! local costmap
        private_nh_.param("local_costmap/gloabl_frame", global_frame_, std::string("world"));
        private_nh_.param("local_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
        local_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
        local_costmap_->pause();

        //! local planner
        string local_planner_name;
        private_nh_.param("base_local_planner", local_planner_name, string("local_planner/DWAPlannerROS"));
        try
        {
            local_planner_ = local_loader_.createInstance(local_planner_name);
            local_planner_->initialize(local_loader_.getName(local_planner_name), &tf_, local_costmap_);
        }
        catch (const pluginlib::PluginlibException &ex)
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner_name.c_str(), ex.what());
            exit(1);
        }
        local_costmap_->start();

        //! rolling window
        private_nh_.param("rw_time_thres", rw_time_thres_, 1.0);
        private_nh_.param("rw_dist_thres", rw_dist_thres_, 0.5);
        rw_ = new rolling_window::RollingWindow(tf_, "rolling_window");

        //! start action server
        as_->start();
        ROS_DEBUG("MoveBaseObstacle initialize complete!");
    }


    MoveBaseObstacle::~MoveBaseObstacle()
    {
        if (as_ != NULL)
            delete as_;

        if (local_costmap_ != NULL)
            delete local_costmap_;

        if (rw_ != NULL)
            delete rw_;
    }


    /**
     * @brief 发布停车速度
     */
    void MoveBaseObstacle::publishZeroVelocity(){
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
    }


    /**
     * @brief 全局规划路径的回调函数
     * @param path_msgs
     */
    void MoveBaseObstacle::globalPathCb(const nav_msgs::PathConstPtr &path_msgs)
    {
        if (!is_generate_path_)
            global_path_ = path_msgs->poses;
    }


    /**
     * @brief 当没有外部程序发布global_plan时候, 需要模拟地图导航结果, 生成直线的规划路径
     * @param start_pose
     * @param end_pose
     * @param path
     * @return
     */
    bool MoveBaseObstacle::generatePath(const geometry_msgs::PoseStamped &start_pose,
                                     const geometry_msgs::PoseStamped &end_pose,
                                     vector<geometry_msgs::PoseStamped> &path)
    {
        ros::Time t = ros::Time::now();
        path.clear();
        path.push_back(start_pose);
        // 计算直线的斜率和角度
        double dx = end_pose.pose.position.x - start_pose.pose.position.x;
        double dy = end_pose.pose.position.y - start_pose.pose.position.y;
        double th = angles::normalize_angle(atan2(dy, dx));
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(th);
        // 梯度模长
        double norm_dx = fabs(dx);
        double norm_dy = fabs(dy);
        // 步进较大的维度使用path_resolution作为步长
        double step_x = norm_dx < norm_dy ? dx/(norm_dy+1e-4)*path_resolution_ : dx/norm_dx*path_resolution_;
        double step_y = norm_dy < norm_dx ? dy/(norm_dx+1e-4)*path_resolution_ : dy/norm_dy*path_resolution_;
        ROS_DEBUG_STREAM("Cacluate global path: step_x=" << step_x << ", step_y=" << step_y);
        // 遍历直线路径上的点 (不带start, 带end)
        double num_step = floor(dx / step_x);  // 根据x方向遍历
        double curr_x = start_pose.pose.position.x;
        double curr_y = start_pose.pose.position.y;
        for (int i=0; i<num_step; i++)
        {
            curr_x += step_x;
            curr_y += step_y;
            geometry_msgs::PoseStamped pose_msgs;
            pose_msgs.header.stamp = t;
            pose_msgs.header.frame_id = global_frame_;
            pose_msgs.pose.position.x = curr_x;
            pose_msgs.pose.position.y = curr_y;
            pose_msgs.pose.position.z = 0.0;
            pose_msgs.pose.orientation = q;
            path.push_back(pose_msgs);
        }
        // 目标点不在path中的话需要单独放进去
        if (fabs(curr_x-end_pose.pose.position.x) > 1e-2 || fabs(curr_y-end_pose.pose.position.y) > 1e-2)
            path.push_back(end_pose);

        // 发布消息
        nav_msgs::Path path_msgs;
        path_msgs.header.stamp = ros::Time::now();
        path_msgs.header.frame_id = global_frame_;
        path_msgs.poses = path;
        gp_pub_.publish(path_msgs);
        return true;
    }


    /**
     * @brief 每次做local planner之前, 都需要对global path进行一次update,
     * 把当前底盘坐标正交投影到global path的直线上, 落在投影点后面的路标点全部删除
     *
     * @param global_pose
     * @param path
     * @return
     */
    bool MoveBaseObstacle::updatePath(const geometry_msgs::PoseStamped &global_pose,
                                   vector<geometry_msgs::PoseStamped> &path)
    {
        // path解算直线方程: (dx)*y + (-dy)*x + (dy*x0-dx*y0) = 0
//        double dx =(path.end()-1)->pose.position.x - path.begin()->pose.position.x;
//        double dy = (path.end()-1)->pose.position.y - path.begin()->pose.position.y;
        double x0 = path.begin()->pose.position.x;
        double y0 = path.begin()->pose.position.y;
        // 不直接计算投影点坐标, 通过计算path上每个点到外点距离, 去掉所有距离变小的点
        double gx = global_pose.pose.position.x;
        double gy = global_pose.pose.position.y;
        vector<geometry_msgs::PoseStamped> copy_path(path.begin(), path.end());
        vector<geometry_msgs::PoseStamped>::iterator iter = copy_path.begin();
        double last_dis = hypot(gx-x0, gy-y0);
        iter++;
        for (; iter!=copy_path.end(); iter++)
        {
            double px = iter->pose.position.x;
            double py = iter->pose.position.y;
            double curr_dis = hypot(gx-px, gy-py);
            if (curr_dis >= last_dis)
                break;
            last_dis = curr_dis;
        }
        // 说明所有路标点都可能在底盘前面
        if (iter == copy_path.begin()+1)
            return true;
        // 说明所有路标点都可能在底盘后面
        if (iter == copy_path.end())
        {
            path.clear();
            return false;
        }
        // 一般情况, 把底盘之前的点删除掉
        path.clear();

        int skip_num = min(2l, copy_path.end()-iter-2);
        path.insert(path.begin(), iter+skip_num, copy_path.end());
        // 发布消息
        nav_msgs::Path path_msgs;
        path_msgs.header.stamp = ros::Time::now();
        path_msgs.header.frame_id = global_frame_;
        path_msgs.poses = path;
        gp_pub_.publish(path_msgs);
        return true;
    }


    /**
     * @brief 四元数检验, 平面导航要求旋转基本都是偏航角(Yaw)
     * @param q
     * @return
     */
    bool MoveBaseObstacle::isQuaternionValid(const geometry_msgs::Quaternion& q){
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
    bool MoveBaseObstacle::goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg, geometry_msgs::PoseStamped &global_pose_msg)
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
    bool MoveBaseObstacle::updateCurrentGoal(const move_base_msgs::MoveBaseGoal &move_base_goal, geometry_msgs::PoseStamped &goal)
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
     *
     * @param global_robot_pose
     * @param global_goal_pose
     * @return
     */
    double MoveBaseObstacle::getGoalDirection(const tf::Stamped<tf::Pose> &global_robot_pose, const tf::Stamped<tf::Pose> &global_goal_pose)
    {
        double diff_x = global_goal_pose.getOrigin().x() - global_robot_pose.getOrigin().x();
        double diff_y = global_goal_pose.getOrigin().y() - global_robot_pose.getOrigin().y();
        double goal_angle = atan2(diff_y, diff_x);  // (-pi, pi)
        double robot_angle = Convertor::getYaw(global_robot_pose);
        double angle =  goal_angle - robot_angle;
        ROS_INFO("goal_angle: %f, robot_angle: %f, direction angle: %f"
        , goal_angle, robot_angle, angle);
        return angles::normalize_angle(angle);
    }


    /**
     * @brief 调用rolling window寻找一次局部子目标, 并算出直线路径
     */
    void MoveBaseObstacle::makePlan()
    {
        geometry_msgs::PoseStamped robot_pose_msg;
        tf::poseStampedTFToMsg(global_robot_pose_, robot_pose_msg);
        global_path_.clear();
        rw_->solveLocalGoal(local_goal_msg_);
        generatePath(robot_pose_msg, local_goal_msg_, global_path_);
    }


    /**
     * @brief /move_base_simple/goal回调函数, 不需要actionlib client就能直接指定目标导航,
     * rviz界面导航就是发布到这个主题上
     *
     * @param goal
     */
    void MoveBaseObstacle::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
        //! 没有外部程序发布global_plan主题时, 自己生成主题并发布
        if (is_generate_path_)
        {
            if (!local_costmap_->getRobotPose(global_robot_pose_))  // TODO: 使用local costmap, 需要使用其他方法获得机器人位置
            {
                ROS_WARN("Fail to get goal direction: Get robot glabal pose fail.");
                resetState();
                as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal when getting robot global pose in judging start direction");
                return;
            }
            geometry_msgs::PoseStamped global_pose;
            tf::poseStampedTFToMsg(global_robot_pose_, global_pose);
            generatePath(global_pose, *goal, global_path_);
        }

        //! 同样是组装一个ActionGoal, 然后通过actionlib publisher发布
        ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
        move_base_msgs::MoveBaseActionGoal action_goal;
        action_goal.header.stamp = ros::Time::now();
        action_goal.goal.target_pose = *goal;
        action_goal_pub_.publish(action_goal);
    }


    /**
     * @brief actionlib server的执行函数, 包括了整个导航过程的大循环
     *
     * @param goal
     */
    void MoveBaseObstacle::executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
    {
        resetState();
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
                    goal_init_ = true;
                }
                //! 抢占的目标不可用, 取消当前执行的目标
                else
                {
                    resetState();
                    as_->setPreempted();
                    return;
                }
            }  // 抢占处理结束

            //! 目标点初始化, 第一次执行需要:
            //! + 让底盘朝目标方向旋转
            //! + 调用一次rolling window
            if (goal_init_)  // 只有有了新的目标才会去判断移动方向
            {
                goal_init_ = false;  // 对于一个目标点, 初始化只需要一次

                //! 获得底盘开始位姿
                if (!local_costmap_->getRobotPose(global_robot_pose_))  // TODO: 使用local costmap, 需要使用其他方法获得机器人位置
                {
                    ROS_WARN("Fail to get goal direction: Get robot glabal pose fail.");
                    resetState();
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal when getting robot global pose in judging start direction");
                    return;
                }

                //! 让底盘朝目标方向旋转
                double target_angle = getGoalDirection(global_robot_pose_, current_goal_);
                double last_angle = Convertor::getYaw(global_robot_pose_);
                double turn_angle = 0.0;
                bool turn_flag = true;
                while (nh_.ok() && turn_flag && local_costmap_->getRobotPose(global_robot_pose_))
                {
                    // 旋转积分
                    double current_angle = Convertor::getYaw(global_robot_pose_);
                    double delta_angle = angles::normalize_angle(current_angle - last_angle);
                    turn_angle += delta_angle;
                    last_angle = current_angle;
                    ROS_DEBUG("Turned around: %f, delta turn around: %f", turn_angle, delta_angle);


                    // 判断旋转是否完成
                    if (fabs(turn_angle) > fabs(target_angle))
                    {
                        ROS_INFO("Initialize goal: turning around complete.");
                        publishZeroVelocity();
                        turn_flag = false;  // 到达指定朝向, 停止旋转
                    }
                    else  // 发布原地旋转的速度
                    {
                        geometry_msgs::Twist cmd_vel;
                        cmd_vel.linear.x = 0;
                        cmd_vel.linear.y = 0;
                        cmd_vel.linear.z = 0;
                        cmd_vel.angular.x = 0;
                        cmd_vel.angular.y = 0;
                        if (target_angle > 0)
                            cmd_vel.angular.z = max_angular_vel_;
                        else
                            cmd_vel.angular.z = -max_angular_vel_;
                        vel_pub_.publish(cmd_vel);
                    }
                    rate.sleep();
                }  // 旋转完成

                //! 转向结束后, 重置rolling window
                // 设置rolling window的全局目标
                rw_->setGlobalGoal(goal_msg);
                ROS_DEBUG("Call rolling window to find a new local goal");
                // 初始化求解第一个子目标点
                rw_->solveLocalGoal(local_goal_msg_);
                ROS_DEBUG_STREAM("Start pose: x=" << global_robot_pose_.getOrigin().x() << ", y=" << global_robot_pose_.getOrigin().y());
                ROS_DEBUG_STREAM("Local goal: x=" << local_goal_msg_.pose.position.x << ", y=" << local_goal_msg_.pose.position.y);
                // 重置计数器
                makeplan_timer_.tic();
                // 更新控制器每次local control的时候都会更新global path
                geometry_msgs::PoseStamped global_pose;
                tf::poseStampedTFToMsg(global_robot_pose_, global_pose);
                std::vector<geometry_msgs::PoseStamped> current_path;
                generatePath(global_pose, local_goal_msg_, current_path);

                // update global path of local controller
                if (!local_planner_->setPlan(current_path))
                {
                    //ABORT and SHUTDOWN COSTMAPS
                    ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                    resetState();
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Fail to pass global path to the local controller");
                    return;
                }
            }  // 目标初始化结束
            //! 路径控制
            else
            {
                if (!local_costmap_->getRobotPose(global_robot_pose_))  // TODO: 使用local costmap, 需要使用其他方法获得机器人位置
                {
                    ROS_WARN("Fail to get goal direction: Get robot glabal pose fail.");
                    resetState();
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal when getting robot global pose in following");
                    return;
                }

                //! 单步控制循环
                bool done = executeCycle();

                //! 到达判断
                if (done)
                    return;
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


    bool MoveBaseObstacle::executeCycle()
    {
        //! 判断已经到达终点
        double goal_angle = getGoalDirection(global_robot_pose_, current_goal_);
        double goal_distance = hypot(
                global_robot_pose_.getOrigin().x()-current_goal_.getOrigin().x(),
                global_robot_pose_.getOrigin().y()-current_goal_.getOrigin().y()
        );
        ROS_DEBUG("Difference to goal: diff_angle=%7.4f, diff_dist=%7.4f", goal_angle, goal_distance);
        if (goal_distance < 0.3)
        {
            resetState();
            as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
            return true;
        }
        //! 还没有到达终点, 做前向控制
        else
        {
            //! 满足一定条件调用rolling_window更新局部子目标点
            double local_goal_dist = hypot(
                    global_robot_pose_.getOrigin().x() - local_goal_msg_.pose.position.x,
                    global_robot_pose_.getOrigin().y() - local_goal_msg_.pose.position.y
                    );
            double local_global_dist = hypot(
                    current_goal_.getOrigin().x() - local_goal_msg_.pose.position.x,
                    current_goal_.getOrigin().y() - local_goal_msg_.pose.position.y
                    );
            vector<geometry_msgs::PoseStamped > current_path;
            if (fail_cnt_ > 0 ||  // 有local plan连续失败控制的记录, 重新进行一次rolling window
                makeplan_timer_.toc() > rw_time_thres_ ||  // 超过一定时间, 重新进行一次rolling window
                ( local_goal_dist < rw_dist_thres_ && local_global_dist > 1e-3)  // 距离子目标距离小于一定距离, 重新进行一次rolling window, 但当子目标就是全局目标的时候, 不再重新更新
            )
//            if (local_goal_dist < rw_dist_thres_)  // 距离子目标距离小于一定距离, 重新进行一次rolling window, 但当子目标就是全局目标的时候, 不再重新更新
            {
                ROS_DEBUG("Call rolling window to find a new local goal");
                rw_->solveLocalGoal(local_goal_msg_);
                ROS_DEBUG_STREAM("Start pose: x=" << global_robot_pose_.getOrigin().x() << ", y=" << global_robot_pose_.getOrigin().y());
                ROS_DEBUG_STREAM("Local goal: x=" << local_goal_msg_.pose.position.x << ", y=" << local_goal_msg_.pose.position.y);
                makeplan_timer_.tic();
                //! 更新控制器每次local control的时候都会更新global path
                geometry_msgs::PoseStamped global_pose;
                tf::poseStampedTFToMsg(global_robot_pose_, global_pose);
                generatePath(global_pose, local_goal_msg_, current_path);

                // update global path of local controller
                if (!local_planner_->setPlan(current_path))
                {
                    //ABORT and SHUTDOWN COSTMAPS
                    ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                    resetState();
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Fail to pass global path to the local controller");
                    return true;
                }
            }

            //! local planner规划出速度
            geometry_msgs::Twist cmd_vel;
            if (local_planner_->computeVelocityCommands(cmd_vel))
            {
                ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                                 cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
                vel_pub_.publish(cmd_vel);
                fail_cnt_ = 0;
                return false;
            }
            else
            {
                // TODO: recovery?
                fail_cnt_++;
                ROS_WARN("Continuous local planner control fail times: %02d\n", fail_cnt_);
                if (fail_cnt_ > 10)
                {
                ROS_ERROR("Local controller cannot get a valid plan, aborting.");
                resetState();
                as_->setAborted(move_base_msgs::MoveBaseResult(), "Local planner fail to get a valid plan");
                return true;
                }
                return false;
            }

        }
    }


    /**
     * @brief 重置move base的状态, 在到达目标点/当前目标被不可用goal抢占等地方被使用
     */
    void MoveBaseObstacle::resetState()
    {
        goal_init_ = true;
        fail_cnt_ = 0;
        publishZeroVelocity();
    }
}