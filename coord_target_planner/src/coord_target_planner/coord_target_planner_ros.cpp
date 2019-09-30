//
// Created by yonghui on 19-8-29.
//

#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include "convertor.h"
#include "coord_target_planner_ros.h"
#include "coord_target_planner.h"

// register plugin, macros paramters: plugin implementation class, plugin base class
PLUGINLIB_EXPORT_CLASS(coord_target_planner::CoordTargetPlannerROS, nav_core::BaseGlobalPlanner)

namespace coord_target_planner
{
    CoordTargetPlannerROS::CoordTargetPlannerROS():
    planner_(NULL), costmap_ros_(NULL), initialized_(false)
    {

    }


    CoordTargetPlannerROS::~CoordTargetPlannerROS()
    {
        if (planner_ != NULL)
            delete planner_;
    }


    void CoordTargetPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        // skip initialized
        if (initialized_)
        {
            ROS_WARN("Target planner has been initialized.");
            return;
        }
        else
            initialized_ = true;

        // for parameters
        ros::NodeHandle private_param_nh("~/" + name);
        private_param_nh.param("angle_smooth_win", angle_smooth_win_, 1);

        costmap_ros_ = costmap_ros;
        costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();

        // target planner
        if (planner_ != NULL)
            delete planner_;
        planner_ = new CoordTargetPlanner(name, costmap);

        // publish path
        ros::NodeHandle private_nh("~/target_planner");
        pth_pub_ = private_nh.advertise<nav_msgs::Path>("path", 1);
        poly_tgt_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>("target_area", 1);
        poly_fs_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>("free_space", 1);
    }


    bool CoordTargetPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                                         const geometry_msgs::PoseStamped& goal,
                                         std::vector<geometry_msgs::PoseStamped>& plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return false;
        }
        return makePlan(start, plan);
    }


    bool CoordTargetPlannerROS::makePlan(const geometry_msgs::PoseStamped &start,
                                         std::vector<geometry_msgs::PoseStamped> &plan)
    {
        double wpx = start.pose.position.x;
        double wpy = start.pose.position.y;
        double wth = Convertor::getYaw(start);
        plan.clear();
        plan.push_back(start);  // start pose is the first pose of plan

        // find target path
        if (!planner_->findTargetPath(wpx, wpy, wth, plan))
        {
            ROS_ERROR("Fail to get a target plan");
            return false;
        }

        // add angle information
        processPosePlan(plan);

        // publish target path
        publishPlan(plan);

        // publish target area
        publishTargetArea();

        // publish free space area
        publishFreeSpaceArea();

        return true;
    }


    void CoordTargetPlannerROS::publishPlan(const vector<geometry_msgs::PoseStamped> &poses)
    {
        nav_msgs::Path path;
        path.header.stamp = poses[0].header.stamp;
        path.header.frame_id = poses[0].header.frame_id;
        path.poses = poses;
        pth_pub_.publish(path);
    }


    void CoordTargetPlannerROS::publishTargetArea()
    {
        geometry_msgs::PolygonStamped poly_msg;

        // add stamp
        ros::Time t = ros::Time::now();
        poly_msg.header.stamp = t;
        poly_msg.header.frame_id = global_frame_;

        planner_->getTargetArea(poly_msg);
        poly_tgt_pub_.publish(poly_msg);
    }


    void CoordTargetPlannerROS::publishFreeSpaceArea()
    {
        geometry_msgs::PolygonStamped poly_msg;

        // add stamp
        ros::Time t = ros::Time::now();
        poly_msg.header.stamp = t;
        poly_msg.header.frame_id = global_frame_;

        planner_->getFreeSpaceArea(poly_msg);
        poly_fs_pub_.publish(poly_msg);
    }


    void CoordTargetPlannerROS::processPosePlan(vector<geometry_msgs::PoseStamped> &pose_plan)
    {
        ros::Time t = ros::Time::now();
        int path_len = pose_plan.size();
        double x0, y0, x1, y1, thi;
        for (int i=0; i<path_len; i++)
        {
            // update header
            pose_plan[i].header.stamp = t;
            pose_plan[i].header.frame_id = global_frame_;

            // Lagrange mean value theorem
            int idx0 = std::max(0, i-angle_smooth_win_);
            int idx1 = std::min(path_len-1, i+angle_smooth_win_);
            x0 = pose_plan[idx0].pose.position.x;
            y0 = pose_plan[idx0].pose.position.y;
            x1 = pose_plan[idx1].pose.position.x;
            y1 = pose_plan[idx1].pose.position.y;
            thi = atan2(y1-y0, x1-x0);
            pose_plan[i].pose.orientation = tf::createQuaternionMsgFromYaw(thi);
        }
    }
}