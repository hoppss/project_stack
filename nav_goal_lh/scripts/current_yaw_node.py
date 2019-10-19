#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import socket
import rospy
import time
import numpy as np
from nav_msgs.msg import Odometry

def talker():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('192.168.43.121', 8660))
    s.listen(3)
    data, addr = s.accept()
    print (addr)
    try:
        while not rospy.is_shutdown():
            recv_data = data.recv(1024)
            recv_data.decode('utf-8')
            if recv_data.find(b']['):
                recv_data = recv_data.split(b'][')
                if len(recv_data) > 1:
                    if len(recv_data[0]) >= len(recv_data[1]):
                        recv_data = recv_data[0]
                    else:
                        recv_data = recv_data[1]
                    gps_data = recv_data.strip("[]\n ").split(',')
                else:
                    gps_data = recv_data[0].strip("[]\n ").split(',')
            gps_data = np.array(gps_data)
            if len(gps_data) < 8:
                continue
            pub = rospy.Publisher('yaw_odom', Odometry, queue_size=10)
            rospy.init_node('current_gps', anonymous=True)
            current_yaw = Odometry()
            current_time=rospy.get_rostime()
            current_yaw.header.stamp=current_time
            current_yaw.header.frame_id = "phone"
            if float(gps_data[7])<0:
            	yaw_data = float(gps_data[7])+360
            else:
            	yaw_data = float(gps_data[7])
            current_yaw.twist.twist.angular.z = yaw_data
            rate = rospy.Rate(10)
            goal_str =current_yaw
            rospy.loginfo(goal_str)
            pub.publish(goal_str)
            rate.sleep()

    except KeyboardInterrupt:
        s.close()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

