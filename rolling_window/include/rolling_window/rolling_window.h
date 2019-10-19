//
// Created by yonghui on 19-9-19.
//

#ifndef ROLLING_WINDOW_ROLLING_WINDOW_H
#define ROLLING_WINDOW_ROLLING_WINDOW_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <string>
#include <utility>
using namespace std;

#include "obstacle.h"

namespace rolling_window
{
    class RollingWindow
    {
    public:
//        RollingWindow();

        /**
         * @brief Construct function
         *
         * @param tf
         * @param name
         */
        RollingWindow(tf::TransformListener &tf, string name);

        /**
         * @brief Set global goal, before you start use rolling window,
         * you should call this function at least once
         *
         * @param global_goal
         */
        void setGlobalGoal(const geometry_msgs::PoseStamped &global_goal);

        /**
         * @brief One step of rolling window, get the best local goal at
         * current time, user should call this function with certain frequency,
         * util get the global goal
         *
         * @param local_goal Output target goal
         * @return If robot can direct move towards global goal, return true
         * else return false.
         */
        bool solveLocalGoal(geometry_msgs::PoseStamped &local_goal);

        /**
         * @brief Find the minimum distance to obstacle at current time
         * !!!DO NOT USE, BUG EXIST IN THIS FUNCTION!!!
         *
         * @return
         */
        double obstacleMinDist();

        /**
         * @brief coordinate convert: input_frame->output_frame
         *
         * @param input_msg
         * @param output_msg
         * @param output_frame
         * @return
         */
        bool getTargetFramePose(const geometry_msgs::PoseStamped &input_msg, geometry_msgs::PoseStamped &output_msg, const string &output_frame);

        /**
          * @brief coordinate convert: base_link->world
          *
          * @param base_msg
          * @param global_msg
          * @return
          */
        bool getGlobalFramePose(const geometry_msgs::PoseStamped &base_msg, geometry_msgs::PoseStamped &global_msg);

        /**
         * @brief coordinate convert: world->base_link
         *
         * @param global_msg
         * @param base_msg
         * @return
         */
        bool getBaseFramePose(const geometry_msgs::PoseStamped &global_msg, geometry_msgs::PoseStamped &base_msg);

    protected:
        /**
         * @brief Given the edge polar coordinate, calculated with the relative edge goal and half of safe distance.
         * For more specific description please refer to the brief of `findLocalSubOptimalGoal` function.
         * The only difference betwween front and back edge goal is the adding / reducing of delta theta.
         *
         * @param edge_angle
         * @param edge_dist
         * @param is_front
         * @param edge_goal
         * @return
         */
        bool getObstacleEdgeGoal(const double &edge_angle, const double &edge_dist, const bool &is_front,
                                 geometry_msgs::PoseStamped &edge_goal);

        /**
         * @brief Before searching local sub optimal goal, we will first to check
         * if global goal is available , with following steps:
         *
         * 1. get the direction from position of robot to global goal
         *
         * 2. Finding scan points in a given angle range
         *
         * 3. If no scan points in search range, return true
         *
         * @return
         */
        bool checkGlobalGoalAvailable();

        /**
         * @brief Given scan information, split scan points to several different obstacles
         *
         * @param min_angle
         * @param max_angle
         */
        void obstacleDetect( double min_angle, double max_angle);

        /**
         * @@brief Obstacle merge, with following operations
         *
         * 1. pop the pop two obstacles in merged obstacles' stack(back1, back2),
         * if stack size less than two, directly push current obstacle into stack
         *
         * 2. check interval obstacles'(back2, current) start point and end point distance,
         * if need merging, first push back1, then push merged (back2 + current)
         * else, first push back2, then push back1, finally push current
         *
         * @param obstacles
         * @param goal_pose
         */
        void obstacleMerge(vector<Obstacle > &obstacles);

        /**
         *@brief Find the best local goal as the rolling window method's result. Two main task will be completed in this
         * function:
         *
         * A: Generate candidate local goals.Generally. We will consider two main types of local goal when creating
         * candidate goals, which is calculated by the edge points of the obstacles and half of safe
         *  distance. The first front edge point and the last back edge point will be created as the edge local goal.
         *  Besides, if the range between two obstacles is able to pass (has an interval degree), we will also generate
         *  (front and back) obstacle edge goals.
         *
         * B: Traverse all candidate local goals and find the best one. In this step we simply evaluate the sum of
         * distances from local goal to base and goal position. It is more accurate and robust if using A*'s path
         * generation and evaluation method but I am lazy.
         *
         * @param obstacles
         */
        void findLocalSubOptimalGoal(const vector<Obstacle > &obstacles, geometry_msgs::PoseStamped &sub_goal_pose);

        /**
         * @brief scan topic callback function, it only continously update class member scan message
         *
         * @param scan_msgs
         */
        void laserScanCb(const sensor_msgs::LaserScanConstPtr &scan_msgs);


        /**
         * @brief use mark to present detected obstacle, for visualized debug,
         * the position of mark is calculated by the mean of obstacle points'
         * angle and distance
         *
         * @param obstacles
         */
        void publishObstacleMarkers(const vector<Obstacle> &obstacles) const ;

        // pub and sub
        string name_;
        ros::Publisher obs_marker_pub_;
        ros::Publisher subgoal_marker_pub_;
        ros::Subscriber scan_sub_;
        sensor_msgs::LaserScan scan_msg_;  ///<@brief scan message, each spin will be updated by subscribe callback function

        // pose
        tf::TransformListener &tf_;
        double transform_tolerance_;
        string global_frame_;  ///<@brief global frame id
        string base_frame_;  ///<@brief robot base frame id
        geometry_msgs::PoseStamped robot_pose_;  ///<@brief robot pose in global frame
        geometry_msgs::PoseStamped goal_pose_;  ///<@brief global goal in global frame

        // safe distance parameters
        double base_diameter_;  ///<@brief base diameter
        double inflation_radius_;  ///<@brief distance between base and obstacle
        double safe_dist_;  ///<@brief safe distance between obstacles, detect two different obstacle
        double goal_reach_dist_;  ///<@brief if goal is arrived, if sub goal is arrived, we will split it into two different goal

        // different action range threshold
        double obstacle_range_;  ///<@brief Within this range, base should slow down to prepare for the planning
        double min_range_;  ///<@brief Min valid range, scan point within this range will be ignored
//        double plan_range_;  ///<@brief within this range, base should start calling rolling window to plan the local optimal goal
//        double stop_range_;  ///<@brief within this range, obstacle is too close to avoid, base should immediately stop
        double min_angle_;  ///<@brief laser scan min angle
        double max_angle_;  ///<@brief laser scan max angle

        // angle threshold
        double check_goal_range_;  ///<@brief angle range to check if global goal is available
        double dist_cost_scale_;  ///<@brief used in local goal evaluation, weight of local goal distance cost
        double angle_cost_scale_;  ///<@brief used in local goal evaluation, weight of angle difference cost

        // debug
        bool use_debug_;  ///<@brief debug flag

        // detected obstacles
        vector<Obstacle > obstacles_;
        vector<Obstacle > last_obstacles_;
    };
}

#endif //ROLLING_WINDOW_ROLLING_WINDOW_H
