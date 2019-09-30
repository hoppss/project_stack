//
// Created by yonghui on 19-9-19.
//

#include <ros/ros.h>

#include <cfloat>

#include "obstacle.h"
#include "utilities.h"

namespace rolling_window
{
    Obstacle::Obstacle() :
    safe_dist_(1.0)
    {

    }


    Obstacle::Obstacle(double safe_dist) :
    safe_dist_(safe_dist)
    {

    }


    bool Obstacle::empty() const
    {
        return points_.empty();
    }


    size_t Obstacle::size() const
    {
        return points_.size();
    }


    double Obstacle::min_dist() const
    {
        if (empty())
            return -1;

        double min_dist = DBL_MAX;
        for (int i=0; i<points_.size(); i++)
            if (points_[i].second < min_dist)
                min_dist = points_[i].second;
        return min_dist;
    }


    bool Obstacle::front(double &output_angle, double &output_dist, bool verbose) const
    {
        if (empty())
        {
            if (verbose)
                ROS_WARN("Try to visit an empty obstacle!");
            return false;
        }
        output_angle = points_[0].first;
        output_dist = points_[0].second;
        return true;
    }


    bool Obstacle::back(double &output_angle, double &output_dist, bool verbose) const
    {
        if (empty())
        {
            if (verbose)
                ROS_WARN("Try to visit an empty obstacle!");
            return false;
        }
        output_angle = points_.back().first;
        output_dist = points_.back().second;
        return true;
    }


    bool Obstacle::push_back(double input_angle, double input_dist)
    {
        double last_angle, last_dist;

        // if obstacle is empty, directly push back
        if (!back(last_angle, last_dist, false))
        {
            points_.emplace_back(input_angle, input_dist);
            return true;
        }

        // assert push back point with larger angle
        if (input_angle <= last_angle)
        {
            ROS_ERROR_STREAM("Make sure pushing back point with larger angle to the obstacle: "
                             "last_angle=" << last_angle << ", input_angle=" << input_angle);
            return false;
        }

        // judge if the point belong to the obstacle
        double rel_angle = input_angle - last_angle;
        if (cosineFormula(last_dist, input_dist, rel_angle) > safe_dist_)
            return false;

        points_.emplace_back(input_angle, input_dist);
        return true;
    }


    bool Obstacle::merge_back(const rolling_window::Obstacle &obs)
    {
        if (!is_valid_obstacle(obs))
            return false;

        // push back
//        for (int i=0; obstacle.size(); i++)
//            points_.push_back(obstacle.points_[i]);
        points_.insert(points_.end(), obs.points_.begin(), obs.points_.end());

        return true;
    }


    bool Obstacle::mark_point(double &output_angle, double &output_dist, bool verbose) const
    {
        if (empty())
        {
            if (verbose)
                ROS_WARN("Try to visit an empty obstacle!");
            return false;
        }
        output_angle = (points_[0].first + points_.back().first ) / 2;
        output_dist = 0.;
        for (int i=0; i<points_.size(); i++)
            output_dist += points_[i].second;
        output_dist /= points_.size();
        return true;
    }


    bool Obstacle::find_closest(const rolling_window::Obstacle &obs, double &min_dist,
                                double &min_angle1, double &min_dist1, double &min_angle2, double &min_dist2) const
    {
        if (!is_valid_obstacle(obs))
            return false;

        min_dist = DBL_MAX;
        for (int i=0; i<size(); i++)
            for (int j=0; j<obs.size(); j++)
            {
                double rel_angle = obs.points_[j].first - points_[i].first;
                double curr_dist = cosineFormula(obs.points_[j].second, points_[i].second, rel_angle);
                if (curr_dist <= min_dist)
                {
                    min_dist = curr_dist;
                    // angle1 / dist1 is of current(front) obstacle
                    min_angle1 = points_[i].first;
                    min_dist1 = points_[i].second;
                    // angle2 / dist2 is of input(back) obstacle
                    min_angle2 = obs.points_[j].first;
                    min_dist2 = obs.points_[j].second;
                }
            }
        return true;
    }


    bool Obstacle::is_valid_obstacle(const rolling_window::Obstacle &obs) const
    {
        double back_angle, back_dist, front_angle, front_dist;

        // if merge back obstacle is empty, omit it
        if (!obs.front(front_angle, front_dist))
        {
            ROS_WARN("Merge back obstacle is empty.");
            return false;
        }

        // assert pushing back obstacle with larger front angle
        if (back(back_angle, back_dist) && back_angle >= front_angle)
        {
            ROS_ERROR_STREAM("Make sure merging back obstacle with larger front angle to the obstacle: "
                             "back_angle=" << back_angle << ", front_angle" << front_angle);
            return false;
        }

        return true;
    }
}