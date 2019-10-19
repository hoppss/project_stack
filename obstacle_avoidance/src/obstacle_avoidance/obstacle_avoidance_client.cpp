//
// Created by yonghui on 19-10-8.
//

#include <tf/transform_datatypes.h>
#include <gps_common/conversions.h>

#include <fstream>
#include <sstream>

#include <obstacle_avoidance/TransformPoseStamped.h>
#include "obstacle_avoidance_client.h"

namespace obstacle_avoidance
{
    ObstacleAvoidanceClient::ObstacleAvoidanceClient() :
    nh_(), private_nh_("~"), server_nh_("obstacle_avoidance_server"),
    ac_("move_base", true), use_file_(true), use_base_goal_(true), timeout_(0.0)
    {
        // waiting for server coming up
        while ( !ac_.waitForServer(ros::Duration(timeout_)) )
            ROS_INFO("No move base server response, waiting for server coming up...");

        if (use_base_goal_)
        {
            trans_client_ = server_nh_.serviceClient<TransformPoseStamped>("transform_posestamped");
            ROS_WARN( "Bind transform PoseStamped service: %s", trans_client_.getService().c_str() );
        }

        private_nh_.param("global_frame", global_frame_, string("odom"));
        private_nh_.param("base_frame", base_frame_, string("base_link"));
        private_nh_.param("use_file", use_file_, true);
        private_nh_.param("use_base_goal", use_base_goal_, true);
        private_nh_.param("filename", nav_path_filename_, string("nav_goal_path.txt"));
        private_nh_.param("timeout", timeout_, 5.0);

        // load path from file
        if (use_file_)
        {
            navPathFromFile(nav_path_filename_);
        }
        // subscribe path from topic
        else
        {
            nav_path_sub_ = nh_.subscribe("nav_path", 1, &ObstacleAvoidanceClient::navPathCb, this);
            ROS_INFO("Subscribe navigation path from topic: %s", nav_path_sub_.getTopic().c_str());
        }

        // clear timer
        tic_toc_.tic();
    }


    ObstacleAvoidanceClient::~ObstacleAvoidanceClient()
    {

    }


    void ObstacleAvoidanceClient::spinOnce()
    {
        // deal with no goal condition
        if (nav_path_queue_.empty())
        {
            // load file empty, deal with all goals
            if (use_file_)
            {
                ROS_INFO("Deal with all navigation points in the file!");
                ros::shutdown();
                return;
            }
            // otherwise, waiting util no path is published
            else
            {
                if (tic_toc_.toc() > timeout_)
                {
                    ROS_INFO("No navigation gaol left, waiting for new navigation path...");
                    tic_toc_.tic();
                }
                return;
            }
        }

        // clear timer if remaining unreached goal
        tic_toc_.tic();

        // pop next goal and send to server
        move_base_msgs::MoveBaseGoal target_goal;
        target_goal.target_pose = nav_path_queue_.front();
        nav_path_queue_.pop();
        ROS_INFO("Start to deal with goal: x=%9.4f, y=%9.4f",
                target_goal.target_pose.pose.position.x,
                target_goal.target_pose.pose.position.y);

        // send to server and get result
        ac_.sendGoal(target_goal);
        ac_.waitForResult();
        if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Reach goal successfully!");
        else
            ROS_WARN("Fail to reach goal for some reason...");

        sleep(5);
    }


    void ObstacleAvoidanceClient::navPathCb(const nav_msgs::PathConstPtr &path_msg)
    {
        for (int i=0; i<path_msg->poses.size(); i++)
        {
            nav_path_queue_.push(path_msg->poses[i]);
            ROS_INFO("Add new navigation path point: no.%02d, x=%9.4f, y=%9.4f",
                     i,
                     path_msg->poses[i].pose.position.x,
                     path_msg->poses[i].pose.position.y);
        }
        ROS_INFO("Subscribe new navigation path, points number: %2d", path_msg->poses.size());
    }


    bool ObstacleAvoidanceClient::navPathFromFile(const string &filename)
    {
        ROS_INFO("Load navigation path from file: %s", filename.c_str());
        fstream fin;
        fin.open(filename.c_str(), ios::in);
        string line_buf;
        int cnt = 0;
        while (getline(fin, line_buf))
        {
            // Read nav goal point longitude and latitude
            double longitude, latitude;
            stringstream ss(line_buf);
            ss >> longitude >> latitude;
            ROS_INFO("longitude=%7.4f, latitude=%7.4f", longitude, latitude);

            geometry_msgs::PoseStamped global_nav_goal;
            global_nav_goal.header.stamp = ros::Time::now();

            // load global nav goal, should convert from GPS to UTM
            if (!use_base_goal_)
            {
                // Convert GPS to UTM
                double northing, easting;
                string zone;
                gps_common::LLtoUTM(latitude, longitude, northing, easting, zone);
                global_nav_goal.header.stamp = ros::Time::now();
                global_nav_goal.header.frame_id = global_frame_;
                global_nav_goal.pose.position.x = easting;
                global_nav_goal.pose.position.y = northing;
                global_nav_goal.pose.position.z = 0.0;
                global_nav_goal.pose.orientation.x = 0.0;
                global_nav_goal.pose.orientation.y = 0.0;
                global_nav_goal.pose.orientation.z = 0.0;
                global_nav_goal.pose.orientation.w = 1.0;
            }
            // load base nav goal, should convert from base frame to global frame
            else
            {
                geometry_msgs::PoseStamped local_nav_goal;
                local_nav_goal.header.stamp = ros::Time::now();
                local_nav_goal.header.frame_id = base_frame_;
                local_nav_goal.pose.position.x = longitude;  // if use_base_frame, longitude is x
                local_nav_goal.pose.position.y = latitude;  // if use_base_frame, latitude is y
                local_nav_goal.pose.position.z = 0.0;
                local_nav_goal.pose.orientation.x = 0.0;
                local_nav_goal.pose.orientation.y = 0.0;
                local_nav_goal.pose.orientation.z = 0.0;
                local_nav_goal.pose.orientation.w = 1.0;

                // call server service, transform to global frame
                TransformPoseStamped srv;
                srv.request.input_pose = local_nav_goal;
                srv.request.target_frame = global_frame_;
                if ( !trans_client_.call(srv) )
                {
                    ROS_WARN("Transform base goal to global fail, base_x=%7.4f, base_y=%7.4f, input_frame=%s, target_frame=%s",
                            local_nav_goal.pose.position.x,
                            local_nav_goal.pose.position.y,
                            local_nav_goal.header.frame_id.c_str(),
                            global_frame_.c_str());
                    continue;
                }
                global_nav_goal = srv.response.output_pose;

            }

            // push to queue
            ROS_INFO("Add new navigation path point: no.%02d, x=%9.4f, y=%9.4f",
                     cnt++,
                     global_nav_goal.pose.position.x,
                     global_nav_goal.pose.position.y);
            nav_path_queue_.push(global_nav_goal);
        }
        ROS_INFO("Navigation path points number: %2d", nav_path_queue_.size());
    }
}