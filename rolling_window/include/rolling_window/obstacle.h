//
// Created by yonghui on 19-9-19.
//

#ifndef ROLLING_WINDOW_OBSTACLE_H
#define ROLLING_WINDOW_OBSTACLE_H

#include <vector>
using namespace std;

namespace rolling_window
{
    typedef pair<double, double> ObstaclePoint;

    class Obstacle
    {
    public:
        Obstacle();

        Obstacle(double safe_dist);

        bool empty() const ;

        size_t size() const ;

        double min_dist() const ;

        bool front(double &output_angle, double &output_dist, bool verbose = true) const ;

        bool back(double &output_angle, double &output_dist, bool verbose = true) const ;

        bool push_back(double input_angle, double input_dist);

        bool merge_back(const Obstacle &obs);

        bool mark_point(double &output_angle, double &output_dist, bool verbose=true) const ;

        bool find_closest(const Obstacle &obs, double &min_dist,
                          double &min_angle1, double &min_dist1, double &min_angle2, double &min_dist2) const ;

    protected:
        bool is_valid_obstacle(const Obstacle &obs) const ;

        double safe_dist_;  ///<@brief use to judge a new obstacle

        vector<ObstaclePoint> points_;  ///<@brief restore scan points data
    };
}

#endif //ROLLING_WINDOW_OBSTACLE_H
