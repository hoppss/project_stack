//
// Created by yonghui on 19-9-3.
//

#ifndef COORD_MOVE_BASE_MOVE_BASE_LIGHT_H
#define COORD_MOVE_BASE_MOVE_BASE_LIGHT_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_loader.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/base_local_planner.h>
#include <rolling_window/rolling_window.h>

#include <vector>
#include <string>
#include <queue>
#include <boost/shared_ptr.hpp>
using namespace std;

//#include "controller.h"
#include <obstacle_avoidance/TransformPoseStamped.h>
#include "tic_toc.h"


namespace obstacle_avoidance
{
    typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

    class MoveBaseObstacle
    {
    public:
        MoveBaseObstacle(tf::TransformListener &tf);

        ~MoveBaseObstacle();

        bool transformPoseStampedCb(TransformPoseStamped::Request &req, TransformPoseStamped::Response &res);

    protected:
        /**
         * @brief Get robot pose using rolling window's interface
         * @param robot_pose
         * @return
         */
        bool getRobotPoseRollingWindow(tf::Stamped<tf::Pose> &robot_pose);

        /**
         * @brief 发布停车速度
         */
        void publishZeroVelocity();

        /**
         * @brief 全局规划路径的回调函数
         * @param path_msgs
         */
        void globalPathCb(const nav_msgs::PathConstPtr &path_msgs);

        /**
         * @brief 当没有外部程序发布global_plan时候, 需要模拟地图导航结果, 生成直线的规划路径
         * @param start_pose
         * @param end_pose
         * @param path
         * @return
         */
        bool generatePath(const geometry_msgs::PoseStamped &start_pose, const geometry_msgs::PoseStamped &end_pose, vector<geometry_msgs::PoseStamped> &path);

        /**
         * @brief 每次做local planner之前, 都需要对global path进行一次update,
         * 把当前底盘坐标正交投影到global path的直线上, 落在投影点后面的路标点全部删除
         *
         * @param global_pose
         * @param path
         * @return
         */
        bool updatePath(const geometry_msgs::PoseStamped &global_pose, vector<geometry_msgs::PoseStamped> &path);

        /**
         * @brief 四元数检验, 平面导航要求旋转基本都是偏航角(Yaw)
         * @param q
         * @return
         */
        bool isQuaternionValid(const geometry_msgs::Quaternion& q);

        /**
         * @brief 从goal_pose_msg 的坐标系变换到global frame id下, 这个函数写的不好,
         * 因为上一个项目用tf::Stamped<tf::Pose>来存储位姿信息, 这个函数跟着保留了下来,
         * 但是在写rolling window的时候发现geometry_msgs::PoseStamped还是更方便一点
         * 现在程序中出现两者混用的情况, 转换来转换去很麻烦, 有时间考虑重构
         *
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
         * @brief 调用rolling window寻找一次局部子目标, 并算出直线路径
         */
        void makePlan();

        /**
         * @brief move_base_simple callback function
         *
         * @param goal
         */
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

        /**
         * @brief simple actionlib server callback function
         *
         * @param goal
         */
        void executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal);

        /**
         * @brief 没有local planner, 直接使用target_planner的路径进行控制
         * @return
         */
        bool executeCycle();

        void resetState();

        //! listener and server
        ros::NodeHandle nh_;  ///<@brief ns: /
        ros::NodeHandle private_nh_;  ///<@brief ns: /move_base(_node)
        ros::NodeHandle action_nh_;  ///<@brief ns: /move_base
        ros::NodeHandle simple_nh_;  ///<@brief ns: /move_base_simple
        tf::TransformListener &tf_;
        MoveBaseActionServer *as_;

        //!topic
        ros::Publisher current_goal_pub_;  ////< @brief topic: /obstacle_avoidance_server/current_goal
        ros::Publisher vel_pub_;  ////< @brief topic: /cmd_vel
        ros::Publisher action_goal_pub_;  ////< @brief topic: /move_base/goal
        ros::Subscriber goal_sub_;  ////< @brief topic: /move_base_simple/goal
        // path
        ros::Publisher gp_pub_;  ///<@brief topic: /obstacle_avoidance_server/global_path
        ros::Subscriber gp_sub_;  ///<@brief topic: /obstacle_avoidance_server/global_path
        // transform pose
        ros::ServiceServer trans_srv_;  ///<@brief topic: /obstacle_avoidance_server/transform_posestamped

        //! frame id
        std::string global_frame_;  ////< @brief 全局frame
        std::string robot_base_frame_;  ////< @brief 机器人底盘frame

        //! costmap and planner
        costmap_2d::Costmap2DROS *local_costmap_;  ///<@brief local costmap
        boost::shared_ptr<nav_core::BaseLocalPlanner> local_planner_;  ///<@brief local planner, 实现local planner插件
        pluginlib::ClassLoader<nav_core::BaseLocalPlanner> local_loader_;  ///<@brief local planner pluginlib loader
        rolling_window::RollingWindow *rw_;  ///<@brief rolling window to detect obstacles and find sub optimal local goal
        TicToc makeplan_timer_;  ///<@brief 定时进行rolling window计时器
        double rw_time_thres_;  ///<@brief 进行一次rolling window的时间间隔
        double rw_dist_thres_;  ///<@brief 距离目标点多近重新进行一次rolling window
        double rw_stop_thres_;  ///<@brief rolling window扫描障碍物停止阈值

        //! frame and pose
        tf::Stamped<tf::Pose> global_robot_pose_;  ////< @brief 机器人全局frame下位姿
        tf::Stamped<tf::Pose> current_goal_;  ////< @brief 当前目标路径
        geometry_msgs::PoseStamped local_goal_msg_;  ///<@brief 局部子目标点位置
        vector<geometry_msgs::PoseStamped> global_path_;  ///<@brief 外部程序给出的全局路径规划

        //! control
        double control_frequence_;  ////< @brief 循环控制频率
        double max_linear_vel_;  ////< @brief 最大线速度
        double min_linear_vel_;  ////< @brief 最小线速度
        double max_angular_vel_;  ////< @brief 最大角速度
        double min_angular_vel_;  ////< @brief 最小角速度

        bool is_debug_;  ///<@brief 调试标志位
        bool is_generate_path_;  ///<@brief 生成path, 正常是从别人程序订阅path, 但是现在没有, 需要自己生成
        double path_resolution_;  ///<@brief path采样点对齐轴上的间隔率, 较短的轴为参考

        //! state and flag
        bool goal_init_;  ////< @brief 获得新目标点的标志位
        int fail_cnt_;  ///<@brief local planner控制失败次数

        //! global no obstacle delay
        bool use_global_delay_;  ///<@brief 是否启用global delay, 这是为了防止底盘绕过障碍物时过早向global Goal移动
        int global_delay_thres_;  ///<@brief global delay的上溢阈值, 控制次数为单位
        int global_delay_cnt_;  ///<@brief 当前连续允许向global goal控制的次数
        geometry_msgs::PoseStamped last_local_goal_;  ///<@brief 记录上一次rolling window / global delay的local goal
        geometry_msgs::Twist last_cmd_vel_;  ///<@brief 记录上一次cmd_vel
    };
}

#endif //COORD_MOVE_BASE_MOVE_BASE_LIGHT_H
