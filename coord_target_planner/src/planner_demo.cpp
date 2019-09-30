//
// Created by yonghui on 19-8-30.
//

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_loader.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

class PlannerDemo
{
public:
    explicit PlannerDemo(tf::TransformListener &tf):
    tf_(tf),
    planner_name_("global_planner/CoordTargetPlannerROS"),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner")  // base_cls package, base_cls typename
    {
        // global costmap
        costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
        costmap_ros_->pause();

        // create CoordTargetPlannerROS by pluginlib
        global_planner_ = bgp_loader_.createInstance(planner_name_);
        global_planner_->initialize(bgp_loader_.getName(planner_name_), costmap_ros_);

        // start costmap
        costmap_ros_->start();
    }

    ~PlannerDemo()
    {
        delete costmap_ros_;
    }

    bool makePlan()
    {
        if (costmap_ros_->getRobotPose(current_pose_))
        {
            global_path_.clear();
            geometry_msgs::PoseStamped st_msg;
            geometry_msgs::PoseStamped ed_msg;
            tf::poseStampedTFToMsg(current_pose_, st_msg);
            return global_planner_->makePlan(st_msg, ed_msg, global_path_);
        }
        else
        {
            ROS_ERROR("Fail to get robot pose...");
            return false;
        }
    }

protected:
    tf::TransformListener &tf_;
    std::string planner_name_;
    costmap_2d::Costmap2DROS *costmap_ros_;
    boost::shared_ptr<nav_core::BaseGlobalPlanner> global_planner_;
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;

    tf::Stamped<tf::Pose> current_pose_;
    std::vector<geometry_msgs::PoseStamped> global_path_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_demo_node");
    ros::NodeHandle nh;

    // create demo instance
    tf::TransformListener tf_(ros::Duration(10));
    PlannerDemo pd(tf_);

    ros::Rate r(1);
    while (nh.ok())
    {
        ros::Time t1 = ros::Time::now();
        bool success = pd.makePlan();
        ros::Time t2 = ros::Time::now();
        ROS_INFO("Make Plan: %d | Time Cost: %f s", success, (t2-t1).toSec());
        ros::spinOnce();
        r.sleep();
    }
}