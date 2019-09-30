//
// Created by yonghui on 19-9-23.
//

#ifndef OBSTACLE_AVOIDANCE_TIC_TOC_H
#define OBSTACLE_AVOIDANCE_TIC_TOC_H

#include <ros/time.h>

namespace obstacle_avoidance
{
    class TicToc
    {
    public:
        TicToc()
        {
            tic();
        }

        void tic()
        {
            t_start = ros::Time::now();
        }

        double toc()
        {
            ros::Time t_end = ros::Time::now();
            return t_end.toSec() - t_start.toSec();
        }

    protected:
        ros::Time t_start;

    };
}

#endif //OBSTACLE_AVOIDANCE_TIC_TOC_H
