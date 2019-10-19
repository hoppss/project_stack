//
// Created by yonghui on 19-10-8.
//

#ifndef OBSTACLE_AVOIDANCE_OBSTACLE_AVOIDANCE_CLIENT_H
#define OBSTACLE_AVOIDANCE_OBSTACLE_AVOIDANCE_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <queue>
#include <string>
using namespace std;

#include "tic_toc.h"

namespace obstacle_avoidance
{
    class ObstacleAvoidanceClient
    {
    public:
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

        ObstacleAvoidanceClient();

        ~ObstacleAvoidanceClient();

        /**
         * @brief main function, send one nav_path points to the server.
         * For use_file_ mode, after dealing with all goals, node will shutdown afterwards.
         * For not use_file_ mode, node will wait until next nav path is published.
         */
        void spinOnce();

    protected:
        /**
         * @brief nav path topic callback function,
         * such nav path can be acquired from calling remote navigation service of electronic map
         * @param path_msg
         */
        void navPathCb(const nav_msgs::PathConstPtr &path_msg);

        /**
         * @brief load all preset nav path from file,
         * which is convenient for test
         *
         * @param filename
         * @return load nav path successfully
         */
        bool navPathFromFile(const string &filename);

        // ros
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::NodeHandle server_nh_;

        // topic and service
        MoveBaseClient ac_;
        ros::Subscriber nav_path_sub_;
        ros::ServiceClient trans_client_;

        // load nav path from file
        bool use_file_;  ///<@brief whether load preset nav path from file, or subscribe nav path topic
        bool use_base_goal_;  ///<@brief only used under use_file_ mode, whether the goal is under base frame or under gloabl frame
        string nav_path_filename_;  ///<@brief if loading nav path from file, we will read data from this path

        // frame and pose
        string global_frame_;
        string base_frame_;
        std::queue<geometry_msgs::PoseStamped> nav_path_queue_;  ///<@brief queue store nav path points

        // time
        TicToc tic_toc_;
        double timeout_;  ///<@brief no nav path or server response inform interval timeout
    };
}

#endif //OBSTACLE_AVOIDANCE_OBSTACLE_AVOIDANCE_CLIENT_H
