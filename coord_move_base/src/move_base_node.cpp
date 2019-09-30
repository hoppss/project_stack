//
// Created by yonghui on 19-9-3.
//

#include "move_base_light.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_node");
    tf::TransformListener tf(ros::Duration(10));
    coord_move_base::MoveBaseLight mbl(tf);
    ros::spin();
}