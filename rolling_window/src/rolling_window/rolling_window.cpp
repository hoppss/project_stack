//
// Created by yonghui on 19-9-19.
//
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>

#include <stack>
#include <cfloat>

#include "rolling_window.h"
#include "utilities.h"

namespace rolling_window
{
//    RollingWindow::RollingWindow()
//    {}


    RollingWindow::RollingWindow(tf::TransformListener &tf, string name) :
    tf_(tf), name_(name)
    {
        ros::NodeHandle param_nh("~/" + name_);
        param_nh.param("global_frame", global_frame_, string("odom"));
        param_nh.param("base_frame", base_frame_, string("base_link"));
        param_nh.param("transform_tolerance", transform_tolerance_, 1.0);
        param_nh.param("base_diameter", base_diameter_, 0.4);
        param_nh.param("inflation_radius", inflation_radius_, 0.2);
        param_nh.param("goal_reach_dist", goal_reach_dist_, 0.5);
        param_nh.param("obstacle_range", obstacle_range_, 3.0);
//        param_nh.param("plan_range", plan_range_, 1.0);
//        param_nh.param("stop_range", stop_range_, 0.5);
        param_nh.param("min_angle", min_angle_, -1.57);
        param_nh.param("max_angle", max_angle_, 1.57);
        param_nh.param("check_goal_angle", check_goal_range_, 0.5236);
        param_nh.param("dist_cost_scale", dist_cost_scale_, 2.0);
        param_nh.param("angle_cost_scale", angle_cost_scale_, 1.0);
        param_nh.param("use_debug", use_debug_, true);
        safe_dist_ = base_diameter_ + 2*inflation_radius_;

        ros::NodeHandle nh, private_nh("~");
        obs_marker_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("obstacle_marker", 10);
        subgoal_marker_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("subgoal_marker", 10);
        scan_sub_ = nh.subscribe("scan", 10, &RollingWindow::laserScanCb, this);
    }


    void RollingWindow::setGlobalGoal(const geometry_msgs::PoseStamped &global_goal)
    {
        goal_pose_ = global_goal;
    }


    void RollingWindow::solveLocalGoal(geometry_msgs::PoseStamped &local_goal)
    {
        ros::Time t_start = ros::Time::now();

        // update global base pose
        geometry_msgs::PoseStamped base_msgs;
        setIdentityPoseStamped(base_msgs, base_frame_);
        getGlobalFramePose(base_msgs, robot_pose_);
        goal_pose_.header.stamp = ros::Time();  // time stamp of global pose should update, otherwise tf tree listener will fail
        ROS_INFO_NAMED(name_, "Robot_pose: [%7.4f, %7.4f]", robot_pose_.pose.position.x, robot_pose_.pose.position.y);
        ROS_INFO_NAMED(name_, "Global pose: [%7.4f, %7.4f]", goal_pose_.pose.position.x, goal_pose_.pose.position.y);

        // detect current obstacles
        vector<double> scan_ranges(scan_msg_.ranges.begin(), scan_msg_.ranges.end());
        obstacleDetect(min_angle_, max_angle_);
        obstacleMerge(obstacles_);

        // if no obstacle, directly return global goal
        // after check goal direction return true, return global goal
        if (obstacles_.empty() || checkGlobalGoalAvailable())
        {
            local_goal = goal_pose_;
            return;
        }

        // find sub optimal goal
        findLocalSubOptimalGoal(obstacles_, local_goal);

        // statistic time
        ros::Time t_end = ros::Time::now();
        ROS_DEBUG_STREAM("Solve local time cost: " << t_end.toSec() - t_start.toSec());
    }


    double RollingWindow::obstacleMinDist()
    {
        if (obstacles_.empty())
            return -1;

        double min_dist = DBL_MAX;
        for (int i=0; obstacles_.size(); i++)
        {
            double curr_dist = obstacles_[i].min_dist();
            if (curr_dist > 0 && curr_dist < min_dist)
                min_dist = curr_dist;
        }
        return min_dist;
    }


    bool RollingWindow::getTargetFramePose(const geometry_msgs::PoseStamped &input_msg,
                                           geometry_msgs::PoseStamped &output_msg, const string &output_frame)
    {
        // geometry_msgs::PoseStamped => tf::Stamped<tf::Pose>
        tf::Stamped<tf::Pose> input_pose, output_pose;
        tf::poseStampedMsgToTF(input_msg, input_pose);

        // pose in input_frame
        ros::Time current_time = ros::Time::now();
        string input_frame = input_msg.header.frame_id;

        // listern transformation between: input_frame->output_frame
        try
        {
            tf_.transformPose(output_frame, input_pose, output_pose);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Fail to transform robot pose from [%s] into the frame: [%s]",
                     input_frame.c_str(), output_frame.c_str());
            return false;
        }

        // 检查坐标变换的时间戳没有超时
        if (current_time.toSec() - output_pose.stamp_.toSec() > transform_tolerance_)
        {
            ROS_WARN_THROTTLE(1.0,
                              "Transform timeout. Current time: %.4f, global_msg stamp: %.4f, tolerance: %.4f",
                              current_time.toSec(), output_pose.stamp_.toSec(), transform_tolerance_);
            return false;
        }

        // tf::Stamped<tf::Pose> => geometry_msgs::PoseStamped
        tf::poseStampedTFToMsg(output_pose, output_msg);
        return true;
    }


    bool RollingWindow::getGlobalFramePose(const geometry_msgs::PoseStamped &base_msg,
                                           geometry_msgs::PoseStamped &global_msg)
    {
        return getTargetFramePose(base_msg, global_msg, global_frame_);
    }


    bool RollingWindow::getBaseFramePose(const geometry_msgs::PoseStamped &global_msg,
                                         geometry_msgs::PoseStamped &base_msg)
    {
        return getTargetFramePose(global_msg, base_msg, base_frame_);
    }


    bool RollingWindow::getObstacleEdgeGoal(const double &edge_angle, const double &edge_dist, const bool &is_front,
                                            geometry_msgs::PoseStamped &edge_goal)
    {
        // Get polar coordination of edge goal
        double dth = atan2(safe_dist_, 2*edge_dist);
        double goal_dist = hypot(safe_dist_/2, edge_dist);
        double goal_angle = edge_angle;
        if (is_front)
            goal_angle -= dth;
        else
            goal_angle += dth;
        geometry_msgs::PoseStamped goal_base_frame;
        polar2Descartes(goal_angle, goal_dist, goal_base_frame);
        goal_base_frame.header.stamp = ros::Time();
        goal_base_frame.header.frame_id = base_frame_;
        getGlobalFramePose(goal_base_frame, edge_goal);
        return true;
    }


    bool RollingWindow::checkGlobalGoalAvailable()
    {
        geometry_msgs::PoseStamped base_goal_msg;
        if (!getBaseFramePose(goal_pose_, base_goal_msg))
        {
            ROS_ERROR("Fail to get global goal pose in base frame.");
            return false;
        }
        double goal_yaw = tf::getYaw(base_goal_msg.pose.orientation);
        int min_idx = getScanRangesIndex(scan_msg_, goal_yaw-check_goal_range_);
        int max_idx = getScanRangesIndex(scan_msg_, goal_yaw+check_goal_range_);
        for (int i=min_idx; i<=max_idx; i++)
        {
            if (scan_msg_.ranges[i] <= obstacle_range_)
                return false;
        }
        return true;
    }


    void RollingWindow::obstacleDetect(double min_angle, double max_angle)
    {
        // swap osbtacle vector
        last_obstacles_ = obstacles_;
        obstacles_.clear();

        // start to detect current obstacles
        vector<double> ranges(scan_msg_.ranges.begin(), scan_msg_.ranges.end());
        int min_idx = getScanRangesIndex(scan_msg_, min_angle);
        int max_idx = getScanRangesIndex(scan_msg_, max_angle);
        double curr_angle = min_angle;
        double delta_angle = scan_msg_.angle_increment;
        for (int i=min_idx; i<max_idx; i++)
        {
            // 如果这个点距离大于反应距离, 忽略
            if (ranges[i] > obstacle_range_)
            {
                curr_angle += delta_angle;
                continue;
            }

            // 如果这是第一个小于反应距离的点, 或者尝试添加到最后一个obstacle发现间隔太大,
            // 直接创建新的obstacle, 并添加到vector中
            if (obstacles_.empty() || !obstacles_.back().push_back(curr_angle, ranges[i]))
            {
                Obstacle obs(safe_dist_);
                obs.push_back(curr_angle, ranges[i]);
                obstacles_.push_back(obs);
            }
            curr_angle += delta_angle;
        }
    }


    void RollingWindow::obstacleMerge(vector<rolling_window::Obstacle> &obstacles)
    {
        stack<Obstacle > merged_obstacle;
        for (int i=0; i< obstacles.size(); i++)
        {
            // 如果已被合并的obstacle数量<2, 可以直接添加
            int merged_size = merged_obstacle.size();
            if (merged_size < 2)
            {
                merged_obstacle.push(obstacles[i]);
                continue;
            }

            // 判断和前前一个obstacle的距离, 如果小于安全距离把前前一个obstacle和现在的合并
            Obstacle obstacle_back1 = merged_obstacle.top();  // 前一个
            merged_obstacle.pop();
            Obstacle obstacle_back2 = merged_obstacle.top();  // 前前一个
            merged_obstacle.pop();
            // 前前一个判断
            double back_angle, back_dist, front_angle, front_dist;
            obstacle_back2.back(back_angle, back_dist);  // skip following obstacle
            obstacles[i].front(front_angle, front_dist);  // current traverse obstacle
            double skip_dist = cosineFormula(back_dist, front_dist, back_angle-front_angle);
            // 需要合并, 先合并, 再把back1压栈, 最后把合并obstacle压栈
            if (skip_dist <= safe_dist_)
            {
                ROS_DEBUG("Obstacle[%02d] is merged.", i);
                obstacle_back2.merge_back(obstacles[i]);
                merged_obstacle.push(obstacle_back1);
                merged_obstacle.push(obstacle_back2);
            }
            // 不需要合并, 按原来顺序压入栈中
            else
            {
                merged_obstacle.push(obstacle_back2);
                merged_obstacle.push(obstacle_back1);
                merged_obstacle.push(obstacles[i]);
            }
        }

        // 合并后的merge栈去更新原来的列表
        list<Obstacle> merged_obs_li;
        while (!merged_obstacle.empty())
        {
            merged_obs_li.push_front(merged_obstacle.top());
            merged_obstacle.pop();
        }
        obstacles = vector<Obstacle>(merged_obs_li.begin(), merged_obs_li.end());

        if (use_debug_)
        {
            ROS_DEBUG("Detect obstacle number: %ld", obstacles_.size());
            publishObstacleMarkers(obstacles_);
        }
    }


    void RollingWindow::findLocalSubOptimalGoal(const vector<rolling_window::Obstacle> &obstacles,
                                                geometry_msgs::PoseStamped &sub_goal_pose)
    {
        // 收集所有可能的局部子目标
        // TODO: 使用堆排序
        vector<geometry_msgs::PoseStamped> candidate_goals;
        for (int i=0; i<obstacles.size(); i++)
        {
            // 如果是第一个点, 会特殊处理开始的边界点,
            // 需要注意的是, 开头的点不一定是角度最小的点, 因此需要做一个判断
            if (i == 0)
            {
                double start_angle, start_dist;
                obstacles[i].front(start_angle, start_dist);
                // 对于后面合并障碍物的特殊处理
                if (obstacles.size() > 1)
                {
                    double next_start_angle, next_start_dist;
                    obstacles[i+1].front(next_start_angle, next_start_dist);
                    if (next_start_angle < start_angle)
                    {
                        start_angle = next_start_angle;
                        start_dist = next_start_dist;
                    }
                }

//                double dth = atan2(safe_dist_ , 2*start_dist);
//                double goal_angle = angles::normalize_angle(start_angle - dth);  // 安全角是减
//                double goal_dist = hypot(safe_dist_, start_dist);
//                geometry_msgs::PoseStamped start_pose_msg, start_global_msg;
//                polar2Descartes(goal_angle, goal_dist, start_pose_msg);
//                start_pose_msg.header.stamp = ros::Time();
//                start_pose_msg.header.frame_id = base_frame_;

                geometry_msgs::PoseStamped start_global_msg;
                getObstacleEdgeGoal(start_angle, start_dist, true, start_global_msg);

                ROS_DEBUG("Start local sub goal [%02d]: [%7.4f, %7.4f]",
                          i, start_global_msg.pose.position.x, start_global_msg.pose.position.y);

                candidate_goals.push_back(start_global_msg);
            }

            // 如果是最后一个点, 会特殊处理结尾的边界点,
            // 和起始点不一样, 我们的添加策略会保证最后一个障碍的结尾点是角度最大的点
            if (i == obstacles.size()-1)
            {
                double end_angle, end_dist;
                obstacles[i].back(end_angle, end_dist);

//                double dth = atan2(safe_dist_ , 2*end_dist);
//                double goal_angle = angles::normalize_angle(end_angle + dth);  // 安全角是加
//                double goal_dist = hypot(safe_dist_, end_dist);
//                geometry_msgs::PoseStamped end_pose_msg, end_global_msg;
//                polar2Descartes(goal_angle, goal_dist, end_pose_msg);
//                end_pose_msg.header.stamp = ros::Time();
//                end_pose_msg.header.frame_id = base_frame_;

                geometry_msgs::PoseStamped end_global_msg;
                getObstacleEdgeGoal(end_angle, end_dist, false, end_global_msg);
                getGlobalFramePose(end_global_msg, end_global_msg);

                ROS_DEBUG("End local sub goal [%02d]: [%7.4f, %7.4f]",
                          i, end_global_msg.pose.position.x, end_global_msg.pose.position.y);
                candidate_goals.push_back(end_global_msg);
                continue;  // 最后一个点了, 后面不要处理了
            }

            // 下面会处理每个障碍物的结尾点, 和下一个障碍物的开始点. 尝试取这两个点之间的中间点,
            // 因为相邻障碍物的边界已经保证了间距大于安全距离, 如果不是太过奇形怪状的障碍的话,
            // 这些点应该都是可以到达的
            double end_angle, end_dist, start_angle, start_dist;
            if (!obstacles[i].back(end_angle, end_dist) ||
                !obstacles[i+1].front(start_angle, start_dist) )
                continue;

            // 如果相邻两个障碍结尾点和起始点相距太小, 甚至重叠, 直接不考虑这个间距点
            // TODO: 这里还需要考虑考虑, 先给个0吧
            if (start_angle - end_angle <= 0)
                continue;

            // 对相邻障碍物使用暴力搜索最邻近点对距离
            double min_dist, min_angle1, min_dist1, min_angle2, min_dist2;
            if (!obstacles[i].find_closest(obstacles[i + 1], min_dist, min_angle1, min_dist1, min_angle2, min_dist2))
                continue;
            if (min_dist < safe_dist_)  // 这里需要判断, 我们不确定最小距离肯定比安全距离大
                continue;
            geometry_msgs::PoseStamped min_pose1, min_pose2, middle_pose, middle_global_pose;
            polar2Descartes(min_angle1, min_dist1, min_pose1);
            polar2Descartes(min_angle2, min_dist2, min_pose2);
            double middle_x = (min_pose1.pose.position.x + min_pose2.pose.position.x) / 2;
            double middle_y = (min_pose1.pose.position.y + min_pose2.pose.position.y) / 2;

            // TODO: 这里需要一个判断, 如果子目标点离底盘太近, 需要拆成两个子目标点,
            //  防止底盘停止在这个子目标点上
            double robot_dist = hypot(middle_x, middle_y);  // TODO: 和下面评价子目标点计算距离一部分重复计算了
            // 中间点子目标距离底盘还有一定距离, 直接可以作为候选目标
            if (robot_dist > goal_reach_dist_)
            {
                middle_pose.header.stamp = ros::Time();
                middle_pose.header.frame_id = base_frame_;
                middle_pose.pose.position.x = middle_x;
                middle_pose.pose.position.y = middle_y;
                middle_pose.pose.position.z = 0.0;
                geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw((start_angle + end_angle)/2);
                middle_pose.pose.orientation = q;
                getGlobalFramePose(middle_pose, middle_global_pose);

                // 添加到候选列表中
                ROS_DEBUG("Middle local sub goal [%02d]: [%7.4f, %7.4f]",
                          i, middle_global_pose.pose.position.x, middle_global_pose.pose.position.y);
                candidate_goals.push_back(middle_global_pose);
            }
            // 中间点目标距离太小, 可能陷在这个点中, 拆分成两个edge local goal
            else
            {
                // 1st back edge
                double back_angle, back_dist;
                obstacles[i].back(back_angle, back_dist);
                geometry_msgs::PoseStamped back_global_msg;
                getObstacleEdgeGoal(back_angle, back_dist, false, back_global_msg);
                ROS_DEBUG("Middle edge back goal [%02d]: [%7.4f, %7.4f]",
                          i, back_global_msg.pose.position.x, back_global_msg.pose.position.y);
                candidate_goals.push_back(back_global_msg);

                // 2nd front edge
                double front_angle, front_dist;
                obstacles[i+1].front(front_angle, front_dist);
                geometry_msgs::PoseStamped front_global_msg;
                getObstacleEdgeGoal(front_angle, front_dist, true, front_global_msg);
                ROS_DEBUG("Middle edge front goal [%02d]: [%7.4f, %7.4f]",
                          i, front_global_msg.pose.position.x, front_global_msg.pose.position.y);
                candidate_goals.push_back(front_global_msg);
            }
        }

        // 候选local goal根据到底盘和目标点的距离之和来排序
        // 评价使用的标准量
        double goal_dist_inv = 1. / hypot(goal_pose_.pose.position.x - robot_pose_.pose.position.x,
                                          goal_pose_.pose.position.y - robot_pose_.pose.position.y);
        double base_angle = tf::getYaw(robot_pose_.pose.orientation);
        // 最优子目标
        int best_idx = 0;
        double best_cost = DBL_MAX;
        // 遍历寻找
        for (int i=0; i<candidate_goals.size(); i++)
        {
            // 距离代价计算
            double curr_dist = hypot(candidate_goals[i].pose.position.x - robot_pose_.pose.position.x,
                                     candidate_goals[i].pose.position.y - robot_pose_.pose.position.y) +  // 子目标点到底盘的距离
                               hypot(candidate_goals[i].pose.position.x - goal_pose_.pose.position.x,
                                     candidate_goals[i].pose.position.y - goal_pose_.pose.position.y);  // 子目标点到全局目标的距离
            double dist_cost = dist_cost_scale_ * curr_dist * goal_dist_inv;
            // 角度代价计算
            double goal_angle = atan2(candidate_goals[i].pose.position.y - robot_pose_.pose.position.y,
                                      candidate_goals[i].pose.position.x - robot_pose_.pose.position.x);
            double angle_cost = angle_cost_scale_ * fabs( angles::normalize_angle(base_angle - goal_angle) );
            double curr_cost = dist_cost + angle_cost;
            ROS_INFO_NAMED(name_, "Local candidate goal: [%7.4f, %7.4f], curr_dist=%7.4f",
                           candidate_goals[i].pose.position.x, candidate_goals[i].pose.position.y, curr_dist);
            if (curr_cost <= best_cost)
            {
                best_idx = i;
                best_cost = curr_cost;
                sub_goal_pose = candidate_goals[i];
            }
        }


        if (use_debug_)
        {
            ROS_INFO_STREAM_NAMED(name_, "Number of candidate local sub optimal goal: " << candidate_goals.size());
            ROS_INFO_NAMED(name_, "Best local sub optimal goal: [%7.4f, %7.4f], best_cost=%7.4f",
                      sub_goal_pose.pose.position.x, sub_goal_pose.pose.position.y, best_cost);

            visualization_msgs::MarkerArray marker_arr;
            for (int i=0; i<candidate_goals.size(); i++)
            {
                visualization_msgs::Marker marker;
                if (i == best_idx)
                    marker = createMarker(candidate_goals[i], i, "sub_goal", {1.0, 1.0, 0.0, 1.0});
                else
                    marker = createMarker(candidate_goals[i], i, "sub_goal", {1.0, 0.0, 0.0, 1.0});
                marker_arr.markers.push_back(marker);
            }
            subgoal_marker_pub_.publish(marker_arr);
        }
    }


    void RollingWindow::laserScanCb(const sensor_msgs::LaserScanConstPtr &scan_msgs)
    {
        // prepare [-pi, pi] scan message
        prepareScanMsg(*scan_msgs, scan_msg_);
    }


    void RollingWindow::publishObstacleMarkers(const vector<Obstacle> &obstacles) const
    {
        visualization_msgs::MarkerArray marker_arr;
        ros::Time t = ros::Time();
        for (int i=0; i<obstacles.size(); i++)
        {
            if (use_debug_)
            {
                double start_angle, start_dist, end_angle, end_dist;
                obstacles[i].front(start_angle, start_dist);
                obstacles[i].back(end_angle, end_dist);
                geometry_msgs::PoseStamped start_msgs, end_msgs;
                polar2Descartes(start_angle, start_dist, start_msgs);
                polar2Descartes(end_angle, end_dist, end_msgs);
                ROS_INFO_NAMED(name_, "Obstacle[%02d]: size=%4ld start=[%7.4f, %7.4f]; end=[%7.4f, %7.4f]",
                          i, obstacles[i].size(), start_msgs.pose.position.x, start_msgs.pose.position.y,
                          end_msgs.pose.position.x, end_msgs.pose.position.y);
            }

            double mark_angle, mark_dist;
            if (!obstacles[i].mark_point(mark_angle, mark_dist))
                continue;

            // 极坐标系转笛卡尔坐标系
            geometry_msgs::PoseStamped point_base_msgs;
            polar2Descartes(mark_angle, mark_dist, point_base_msgs);
            point_base_msgs.header.stamp = t;
            point_base_msgs.header.frame_id = base_frame_;

            // 生成mark
            visualization_msgs::Marker marker = createMarker(point_base_msgs, i, "obstacle", {0.0, 1.0, 0.0, 1.0});

            // 加入list
            marker_arr.markers.push_back(marker);
        }

        // publish
        obs_marker_pub_.publish(marker_arr);
    }

}