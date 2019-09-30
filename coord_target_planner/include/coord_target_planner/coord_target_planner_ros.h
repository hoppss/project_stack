//
// Created by yonghui on 19-8-29.
//

#ifndef COORD_LOCAL_PLANNER_COORD_TARGET_PLANNER_ROS_H
#define COORD_LOCAL_PLANNER_COORD_TARGET_PLANNER_ROS_H

#include <nav_core/base_global_planner.h>
#include "coord_target_planner.h"

namespace coord_target_planner
{
    class CoordTargetPlannerROS: public nav_core::BaseGlobalPlanner
    {
    public:
        CoordTargetPlannerROS();

        ~CoordTargetPlannerROS();

        /**
         * @brief target planner interface, since target planner intrinsically is a larger local planner,
         * it will only find the middle path in the neighbor forward area, gaol is no use
         *
         * @param start planner start robot pose
         * @param goal no use input...
         * @param plan output trajectory
         *
         * @return if find path success
         */
        virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

        bool makePlan(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan);

        virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void publishPlan(const vector<geometry_msgs::PoseStamped> &poses);

        void publishTargetArea();

        void publishFreeSpaceArea();

    protected:
        void processPosePlan(vector<geometry_msgs::PoseStamped> &pose_plan);

        ros::Publisher pth_pub_;
        ros::Publisher poly_tgt_pub_;
        ros::Publisher poly_fs_pub_;

        CoordTargetPlanner *planner_;
        costmap_2d::Costmap2DROS *costmap_ros_;

        bool initialized_;
        string global_frame_;
        int angle_smooth_win_;
    };
}

#endif //COORD_LOCAL_PLANNER_COORD_TARGET_PLANNER_ROS_H
