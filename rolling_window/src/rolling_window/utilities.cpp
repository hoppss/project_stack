//
// Created by yonghui on 19-9-19.
//

#include <angles/angles.h>
#include <tf/transform_datatypes.h>

#include <cmath>

#include "utilities.h"

namespace rolling_window
{
    double cosineFormula(double dist1, double dist2, double rel_angle)
    {
        rel_angle = angles::normalize_angle(rel_angle);
        return sqrt(dist1*dist1 + dist2*dist2 - 2*dist1*dist2*cos(rel_angle));
    }


    void polar2Descartes(double input_angle, double input_dist, geometry_msgs::PoseStamped &pose_msgs)
    {
        input_angle = angles::normalize_angle(input_angle);
        double x = cos(input_angle) * input_dist;
        double y = sin(input_angle) * input_dist;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(input_angle);
        pose_msgs.pose.position.x = x;
        pose_msgs.pose.position.y = y;
        pose_msgs.pose.position.z = 0.0;
        pose_msgs.pose.orientation = q;
    }


    void setIdentityPoseStamped(geometry_msgs::PoseStamped &pose_msg, const string &frame_id)
    {
        pose_msg.header.stamp = ros::Time();
        pose_msg.header.frame_id = frame_id;
        pose_msg.pose.position.x = 0.0;
        pose_msg.pose.position.y = 0.0;
        pose_msg.pose.position.z = 0.0;
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        pose_msg.pose.orientation.w = 1.0;
    }


    visualization_msgs::Marker createMarker(const geometry_msgs::PoseStamped &pose_msgs, int id,
                                                           const string &ns, vector<double> rgba)
    {
        // 生成mark
        visualization_msgs::Marker marker;
        // info
        marker.header = pose_msgs.header;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        // pose
        marker.pose = pose_msgs.pose;
        // scale
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        // color
        marker.color.r = rgba[0];
        marker.color.g = rgba[1];
        marker.color.b = rgba[2];
        marker.color.a = rgba[3];

        return marker;
    }


    void deleteAllMarkers(ros::Publisher &pub)
    {
        visualization_msgs::MarkerArray delete_arr;
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        delete_arr.markers.push_back(delete_marker);
        pub.publish(delete_arr);
    }


    int getScanRangesIndex(const sensor_msgs::LaserScan &scan_msgs, double target_angle)
    {
        double min_angle = scan_msgs.angle_min;
//        double max_angle = scan_msgs.angle_max;
        double delta = scan_msgs.angle_increment;
//    ROS_ERROR_STREAM("min angle=" << min_angle << ", max angle=" << max_angle);
//    ROS_ERROR_STREAM("delta=" << delta);
        return (target_angle - min_angle) / delta;
    }


    void prepareScanMsg(const sensor_msgs::LaserScan &scan_msgs, sensor_msgs::LaserScan &new_msgs)
    {
        new_msgs = scan_msgs;
        double min_angle = scan_msgs.angle_min;
        double max_angle = scan_msgs.angle_max;

        // don't need change the order of ranges and intensities vector
        if (max_angle < M_PI + 1e-4)
            return;

        // [0, 2pi] => [-pi, pi]
        new_msgs.angle_max -= M_PI;
        new_msgs.angle_min -= M_PI;

        // find new start index, like at -M_PI
        int idx_st = 0;
        double curr_angle = min_angle;
        double delta = scan_msgs.angle_increment;
        while (curr_angle < M_PI)
        {
            curr_angle += delta;
            idx_st++;
        }

        // new vector
        for (int i=idx_st; i<scan_msgs.ranges.size(); i++)
        {
            new_msgs.ranges[i-idx_st] = scan_msgs.ranges[i];
            new_msgs.intensities[i-idx_st] = scan_msgs.intensities[i];
        }
        int ed_idx = scan_msgs.ranges.size() - idx_st;
        for (int i=0; i<idx_st; i++)
        {
            new_msgs.ranges[i+ed_idx] = scan_msgs.ranges[i];
            new_msgs.intensities[i+ed_idx] = scan_msgs.intensities[i];
        }
    }


//    void prepareScanMsg(const sensor_msgs::LaserScan &scan_msgs, sensor_msgs::LaserScan &new_msgs,
//                        const double &detect_min_angle, const double &detect_max_angle)
//    {
//        new_msgs = scan_msgs;
////        double min_angle = max<double >(scan_msgs.angle_min, min_angle_);
////        double max_angle = min<double >(scan_msgs.angle_max, max_angle_);
//
//        // don't need change the order of ranges and intensities vector
//        if (scan_msgs.angle_max < M_PI + 1e-4)
//        {
//            // resize scan angle
//            double min_angle = max<double >(scan_msgs.angle_min, detect_min_angle);
//            double max_angle = min<double >(scan_msgs.angle_max, detect_max_angle);
//            int min_idx = getScanRangesIndex(scan_msgs, min_angle);
//            int max_idx = getScanRangesIndex(scan_msgs, max_angle);
//
//            // modify scan message
//            new_msgs.angle_min = min_angle;
//            new_msgs.angle_max = max_angle;
//            new_msgs.ranges.clear();
//            new_msgs.intensities.clear();
//            for (int i=min_idx; i<=max_idx; i++)
//            {
//                new_msgs.ranges.push_back(scan_msgs.ranges[i]);
//                new_msgs.intensities.push_back(scan_msgs.intensities[i]);
//            }
//        }
//        else  // such special angle range is usually used for 360-degree scan
//        {
//            // find new start index, start point and end point, like at -M_PI
//            int idx_st = getScanRangesIndex(scan_msgs, M_PI);
//
//            // new vector, change the order
//            vector<double> tmp_ranges(scan_msgs.ranges.size());
//            vector<double> tmp_intenities(scan_msgs.intensities.size());
//            for (int i=idx_st; i<scan_msgs.ranges.size(); i++)
//            {
//                tmp_ranges[i-idx_st] = scan_msgs.ranges[i];
//                tmp_intenities[i-idx_st] = scan_msgs.intensities[i];
//            }
//            int ed_idx = scan_msgs.ranges.size() - idx_st;
//            for (int i=0; i<idx_st; i++)
//            {
//                tmp_ranges[i+ed_idx] = scan_msgs.ranges[i];
//                tmp_intenities[i+ed_idx] = scan_msgs.intensities[i];
//            }
//
//            // [0, 2pi] => [-pi, pi], then resize scan angle
//            new_msgs.angle_min -= M_PI;
//            new_msgs.angle_max -= M_PI;
//            double min_angle = max<double>(new_msgs.angle_min, detect_min_angle);
//            double max_angle = max<double>(new_msgs.angle_max, detect_max_angle);
//            int min_idx = getScanRangesIndex(new_msgs, min_angle);
//            int max_idx = getScanRangesIndex(new_msgs, max_angle);
//
//            // modify scan message
//            new_msgs.angle_min = min_angle;
//            new_msgs.angle_max = max_angle;
//            new_msgs.ranges.clear();
//            new_msgs.intensities.clear();
//            ROS_ERROR_STREAM("min_idx=" << min_idx << ", max_idx=" << max_idx);
//            for (int i=min_idx; i<=max_idx; i++)
//            {
//                new_msgs.ranges.push_back(tmp_ranges[i]);
//                new_msgs.intensities.push_back(tmp_intenities[i]);
//            }
//        }
//        return;
//    }
}