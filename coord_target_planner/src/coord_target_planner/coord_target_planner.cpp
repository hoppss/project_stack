//
// Created by yonghui on 19-8-28.
//

#include <algorithm>
#include <numeric>
#include <costmap_2d/cost_values.h>
#include "coord_target_planner.h"
#include "spline_fitter.h"

namespace coord_target_planner
{
    CoordTargetPlanner::CoordTargetPlanner(std::string name, costmap_2d::Costmap2D *global_costmap) :
    costmap_(global_costmap),
    target_size_x_(0.0), target_size_y_(0.0), target_step_x_(0), target_step_y_(0),
    resolution_grid_x_(0.0), resolution_grid_y_(0.0), resolution_traj_(0.0),
    middle_rate_(0.5), traj_points_num_(10)
    {
        double target_size_x, target_size_y, middle_rate;
        int traj_points_num;

        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("size_x", target_size_x, 3.);
        private_nh.param("size_y", target_size_y, 3.);
        private_nh.param("middle_rate", middle_rate, 0.5);
        private_nh.param("traj_points_num", traj_points_num, 10);

        setParams(target_size_x, target_size_y, middle_rate, traj_points_num);
    }


    void CoordTargetPlanner::setParams(double target_size_x, double target_size_y, double middle_rate, int traj_points_num)
    {
        // update param
        target_size_x_ = target_size_x;
        target_size_y_ = target_size_y;
        middle_rate_ = middle_rate;
        traj_points_num_ = traj_points_num;

        // update target grid size
        double costmap_resolution = costmap_->getResolution();  // x axis sample resolution = costmap resolution
        resolution_traj_ = traj_points_num_ / target_size_y_;  // y axis sample resolution = trajectory resolution
        target_step_x_ = ceil(target_size_x / costmap_resolution);  // x axis is forward direction
        target_step_y_ = ceil(target_size_y / costmap_resolution);
        resolution_grid_x_ = target_size_x_ / (target_step_x_-1);
        resolution_grid_y_ = target_size_y_ / (target_step_y_-1);

        // create position sample matrix
        target_sample_x_ = Eigen::MatrixXd::Zero(target_step_x_, target_step_y_);  //! Be careful to the order
        target_sample_y_ = Eigen::MatrixXd::Zero(target_step_x_, target_step_y_);  //! Be careful to the order
        target_sample_ones_ = Eigen::MatrixXd::Ones(target_sample_x_.rows(), target_sample_y_.cols());
        for (int i=0; i<target_step_x_; i++)
            for (int j=0; j<target_step_y_; j++)
            {
                target_sample_y_(i,j) = -target_size_y/2 + j*resolution_grid_y_;
                target_sample_x_(i,j) = i*resolution_grid_x_;
            }
//        ROS_WARN_STREAM("Target area x grid: \n" << target_sample_x_);
//        ROS_WARN_STREAM("Target area y grid: \n" << target_sample_y_);

        // update publish container
        left_points_.resize(target_step_x_);
        right_points_.resize(target_step_x_);
    }


    bool CoordTargetPlanner::findTargetPath(double px, double py, double th, vector<geometry_msgs::PoseStamped > &plan)
    {
        // check the validity of the given pose
        unsigned int mpx, mpy;  // coordinate in costmap grid
        ROS_ERROR("Local costmap original: (%f, %f)", costmap_->getOriginX(), costmap_->getOriginY());
        if (!costmap_->worldToMap(px, py, mpx, mpy))
        {
            ROS_WARN_THROTTLE(1.0, "The robot's start position is off the global costmap. "
                                   "Planning will always fail, are you sure the robot has been properly localized?");
            return false;
        }

        // FIXME: Test no information area cost value
//        assert(costmap_->worldToMap(0., 1., mpx, mpy));
//        ROS_ERROR("No information cell value: %d", costmap_->getCost(mpx, mpy));

        // transform sample matrix to current pose
        double sin_th = sin(th);
        double cos_th = cos(th);
        Eigen::MatrixXd curr_sample_x = px*target_sample_ones_ + cos_th*target_sample_x_ - sin_th*target_sample_y_;
        Eigen::MatrixXd curr_sample_y = py*target_sample_ones_ + sin_th*target_sample_x_ + cos_th*target_sample_y_;

        // update target area vertices
        top_left_ << curr_sample_x(target_step_x_-1, target_step_y_-1), curr_sample_y(target_step_x_-1, target_step_y_-1);
        top_right_ << curr_sample_x(target_step_x_-1, 0), curr_sample_y(target_step_x_-1, 0);
        bottom_right_ << curr_sample_x(0, 0), curr_sample_y(0, 0);
        bottom_left_ << curr_sample_x(0, target_step_y_-1), curr_sample_y(0, target_step_y_-1);

        // traversing start from the middle of the corridor
        int curr_middle = target_step_y_ / 2;
        int left_boundary = curr_middle;
        int right_boundary = curr_middle;
        double wpx, wpy;  // coordinate in global frame

        // clear robot center cost
        costmap_->worldToMap(px, py, mpx, mpy);
        costmap_->setCost(mpx, mpy, costmap_2d::FREE_SPACE);

        // two list to describe left and obstacle
        int *left_boundary_list = new int[target_step_x_];
        int *right_boundary_list = new int[target_step_x_];
        int *middle_boundary_list = new int[target_step_x_];

        // TODO: If target planner is slow, may be we should skip some lines
        // find corridor boundary
        for (int i=0; i<target_step_x_; i++)  // out loop traverse y axis
        {
            // find left free space range
            for (;left_boundary<target_step_y_; left_boundary++)
            {
                wpx = curr_sample_x(i, left_boundary);
                wpy = curr_sample_y(i, left_boundary);
                if (costmap_->worldToMap(wpx, wpy, mpx, mpy))
                {
                    unsigned char curr_cost = costmap_->getCost(mpx, mpy);
                    // find obstacle return
                    if (curr_cost==costmap_2d::LETHAL_OBSTACLE ||
                        curr_cost==costmap_2d::NO_INFORMATION ||
                        curr_cost==costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                        break;
                }
                else
                {
                    // find map boundary return
                    ROS_WARN("Target planner grid ut of costmap boundary");
                    break;
                }
            }

            // find right free space range
            for (;right_boundary>=0; right_boundary--)
            {
                wpx = curr_sample_x(i, right_boundary);
                wpy = curr_sample_y(i, right_boundary);
                if (costmap_->worldToMap(wpx, wpy, mpx, mpy))
                {
                    unsigned char curr_cost = costmap_->getCost(mpx, mpy);
                    // find obstacle return
                    if (curr_cost==costmap_2d::LETHAL_OBSTACLE ||
                        curr_cost==costmap_2d::NO_INFORMATION ||
                        curr_cost==costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                        break;
                }
                else
                {
                    // find map boundary return
                    ROS_WARN("Target planner grid out of costmap boundary");
                    break;
                }
            }

            // cannot move forward
            if (left_boundary == right_boundary)
            {
                ROS_WARN("Target planner could not get a vaild path");
                return false;
            }

            // TODO: Seems no use, emmmmm...
            // add left and right boundary
            left_boundary_list[i] = std::min(left_boundary, target_step_y_-1);
            right_boundary_list[i] = std::max(right_boundary, 0);
            // update left & right free space area
            left_points_[i].x = curr_sample_x(i, left_boundary_list[i]);
            left_points_[i].y = curr_sample_y(i, left_boundary_list[i]);
            left_points_[i].z = 0.;
            right_points_[i].x = curr_sample_x(i, right_boundary_list[i]);
            right_points_[i].y = curr_sample_y(i, right_boundary_list[i]);
            right_points_[i].z = 0.;

            // reset start middle position
            curr_middle = (left_boundary + right_boundary) / 2;  // middle point index
            left_boundary = curr_middle;
            right_boundary = curr_middle;

            // add middle path
            middle_boundary_list[i] = curr_middle;
        }

        // TODO: Use RANSAC + Splining
        int interval = (target_step_x_-1) / traj_points_num_;
        if (interval <= 0)
        {
            ROS_ERROR("path sample points excess the range of step_y, "
                      "considering enlarge step_y or reduce path sample points");
            return false;
        }
        // pointers
        int *pt_st = middle_boundary_list;
        int *pt_ed = middle_boundary_list + target_step_x_ -1;
        int *pt_curr;

//        // calculate neighbor middle points mean, simple smooth operation
//        for (int i=interval; i<target_step_x_; i+=interval)
//        {
//            //! simple linear interpolation to get position
//            // deal with limit
//            pt_curr = pt_st + i;
//            int *pt_curr_st = i<interval ? pt_st : pt_curr - interval;
//            int *pt_curr_ed = i>=target_step_x_-interval ? pt_ed : pt_curr + interval;
//
//            // linear interpolation to get position
//            double traj_y = (double)accumulate(pt_curr_st, pt_curr_ed, 0) / (pt_curr_ed - pt_curr_st);
//            wpx = ( curr_sample_x(i, floor(traj_y)) + curr_sample_x(i, ceil(traj_y)) ) / 2;
//            wpy = ( curr_sample_y(i, floor(traj_y)) + curr_sample_y(i, ceil(traj_y)) ) / 2;
//
//            //! simple use current point position
////            wpx = target_sample_x_(i, middle_boundary_list[i]);
////            wpy = target_sample_y_(i, middle_boundary_list[i]);
//
//            // update path
//            geometry_msgs::PoseStamped pose;
//            pose.pose.position.x = wpx;
//            pose.pose.position.y = wpy;
//            pose.pose.position.z = 0.;
//            plan.push_back(pose);
//        }

        //! use spline fitting
        // first point is robot start point
        vector<Eigen::Vector2d> fit_scatters;
        Eigen::Vector2d st_xy;
        st_xy << 0., target_step_y_/2.;
        fit_scatters.push_back(st_xy);
        // we still need to average and smooth the sample point
        for (int i=interval; i<target_step_x_; i+=interval)
        {
            // deal with limit
            pt_curr = pt_st + i;
            int *pt_curr_st = i<interval ? pt_st : pt_curr - interval;
            int *pt_curr_ed = i>=target_step_x_-interval ? pt_ed : pt_curr + interval;

            // linear interpolation to get position
            double traj_y = (double)accumulate(pt_curr_st, pt_curr_ed, 0) / (pt_curr_ed - pt_curr_st);
            Eigen::Vector2d pt;
            pt << i, traj_y;
            fit_scatters.push_back(pt);
        }
        // use sample point to fir spline
        SplineFitter spf;
        Spline sp;
        spf.addPoints(fit_scatters);
        spf.solve(sp);
        // along y axis to project sample point to spline
        for (int i=1; i<fit_scatters.size(); i++)
        {
            int traj_x = (int)fit_scatters[i](0);
            double traj_y;
            sp.computerVerticalDistance(fit_scatters[i], traj_y);
            // linear interpolation to get position
            wpx = ( curr_sample_x(traj_x, floor(traj_y)) + curr_sample_x(traj_x, ceil(traj_y)) ) / 2;
            wpy = ( curr_sample_y(traj_x, floor(traj_y)) + curr_sample_y(traj_x, ceil(traj_y)) ) / 2;
            // update path
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = wpx;
            pose.pose.position.y = wpy;
            pose.pose.position.z = 0.;
            plan.push_back(pose);
        }
//        Eigen::Vector2d final_pt = fit_scatters.back();
//        int traj_x = (int)final_pt(0);
//        double traj_y = final_pt(1);
//        wpx = ( curr_sample_x(traj_x, floor(traj_y)) + curr_sample_x(traj_x, ceil(traj_y)) ) / 2;
//        wpy = ( curr_sample_y(traj_x, floor(traj_y)) + curr_sample_y(traj_x, ceil(traj_y)) ) / 2;
//        // update final point
//        geometry_msgs::PoseStamped pose;
//        pose.pose.position.x = wpx;
//        pose.pose.position.y = wpy;
//        pose.pose.position.z = 0.;
//        plan.push_back(pose);

        delete []left_boundary_list;
        delete []right_boundary_list;
        delete []middle_boundary_list;

        return true;
    }


    void CoordTargetPlanner::getTargetArea(geometry_msgs::PolygonStamped &poly_msg)
    {
        // add points
        geometry_msgs::Point32 p_top_left, p_top_right, p_bottom_right, p_bottom_left;
        p_top_left.x = top_left_(0), p_top_left.y = top_left_(1), p_top_left.z = 0;
        p_top_right.x = top_right_(0), p_top_right.y = top_right_(1), p_top_right.z = 0;
        p_bottom_right.x = bottom_right_(0), p_bottom_right.y = bottom_right_(1), p_bottom_right.z = 0;
        p_bottom_left.x = bottom_left_(0), p_bottom_left.y = bottom_left_(1), p_bottom_left.z = 0;
        poly_msg.polygon.points.push_back(p_top_left);
        poly_msg.polygon.points.push_back(p_top_right);
        poly_msg.polygon.points.push_back(p_bottom_right);
        poly_msg.polygon.points.push_back(p_bottom_left);
    }

    void CoordTargetPlanner::getFreeSpaceArea(geometry_msgs::PolygonStamped &poly_msg)
    {
        for (int i=0; i<left_points_.size(); i++)
            poly_msg.polygon.points.push_back(left_points_[i]);
        for (int i=right_points_.size()-1; i>=0; i--)
            poly_msg.polygon.points.push_back(right_points_[i]);
    }
}