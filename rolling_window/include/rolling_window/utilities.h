//
// Created by yonghui on 19-9-19.
//

#ifndef ROLLING_WINDOW_UTILITIES_H
#define ROLLING_WINDOW_UTILITIES_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <vector>
#include <string>
using namespace std;

namespace rolling_window
{
    double cosineFormula(double dist1, double dist2, double rel_angle);

    void polar2Descartes(double input_angle, double input_dist, geometry_msgs::PoseStamped &pose_msgs);

    visualization_msgs::Marker createMarker(const geometry_msgs::PoseStamped &pose_msgs, int id, const string &ns,
                                            vector<double> rgba);

    void setIdentityPoseStamped(geometry_msgs::PoseStamped &pose_msg, const string &frame_id);

    int getScanRangesIndex(const sensor_msgs::LaserScan &scan_msgs, double target_angle);

    void prepareScanMsg(const sensor_msgs::LaserScan &scan_msgs, sensor_msgs::LaserScan &new_msgs);

//    void prepareScanMsg(const sensor_msgs::LaserScan &scan_msgs, sensor_msgs::LaserScan &new_msgs,
//                        const double &detect_min_angle, const double &detect_max_angle);
}

#endif //ROLLING_WINDOW_UTILITIES_H
