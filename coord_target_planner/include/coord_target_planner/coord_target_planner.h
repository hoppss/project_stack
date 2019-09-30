//
// Created by yonghui on 19-8-28.
//

#ifndef COORD_LOCAL_PLANNER_COORD_TARGET_PLANNER_H
#define COORD_LOCAL_PLANNER_COORD_TARGET_PLANNER_H

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/trajectory.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <Eigen/Core>
#include <vector>
using namespace std;

namespace coord_target_planner
{
    class CoordTargetPlanner
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CoordTargetPlanner(std::string name, costmap_2d::Costmap2D *global_costmap);

        void setParams(double target_size_x, double target_size_y, double middle_rate, int traj_points_num);

        bool findTargetPath(double px, double py, double th, vector<geometry_msgs::PoseStamped > &plan);

        void getTargetArea(geometry_msgs::PolygonStamped &poly_msg);

        void getFreeSpaceArea(geometry_msgs::PolygonStamped &poly_msg);

    protected:
        /**
         * @brief Given the float-type index, search the neighbor in local costmap
         * @param idx_x float-type index x
         * @param idx_y float-type index y
         * @return cost value
         */
        unsigned char getNeighborCost(double idx_x, double idx_y);

        costmap_2d::Costmap2D *costmap_;

        // target grid member
        double target_size_x_;  ///<@brief target grid x size
        double target_size_y_;  ///<@brief target grid y size, notice that robot is only allowed moving forward,
        ///< target_size_y_ should be half of target_size_x_

        int target_step_x_;  ///<@brief target grid x step, scale to target_size_x
        int target_step_y_;  ///<@brief target grid y step, scale to target_size_y
        double resolution_grid_x_;  ///<@brief x axis costmap resolution  (m/cell)
        double resolution_grid_y_;  ///<@brief y axis costmap resolution (m/cell)

        Eigen::MatrixXd target_sample_x_;  ///<@brief target sample position matrix
        Eigen::MatrixXd target_sample_y_;  ///<@brief target sample position matrix
        Eigen::MatrixXd target_sample_ones_;  ///<@brief add helper ones matrix

        double resolution_traj_;  ///<@brief trajectory resolution (m/cell)
        double middle_rate_;  ///<@brief position preference: LEFT [0.0<----0.5---->1.0] RIGHT
        int traj_points_num_;  ///<@brief discrete sample target trajectory

        // relate to target area
        Eigen::Vector2d top_left_;  ///<@brief top-left point of target area
        Eigen::Vector2d top_right_;  ///<@brief top-right point of target area
        Eigen::Vector2d bottom_right_;  ///<@brief bottom-right point of target area
        Eigen::Vector2d bottom_left_;  ///<@brief bottom-left point of target area

        // free area of target
        vector<geometry_msgs::Point32> left_points_;  ///<@brief target area left free space boundary
        vector<geometry_msgs::Point32> right_points_;  ///<@brief target area right free space boundary
    };
}

#endif //COORD_LOCAL_PLANNER_COORD_TARGET_PLANNER_H
