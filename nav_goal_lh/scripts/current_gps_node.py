#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import socket
import rospy
import time
import numpy as np
from nav_goal.msg import current_gps

def talker():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('192.168.43.72', 8660))
    s.listen(3)
    data, addr = s.accept()
    print (addr)
    try:
        while True:
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
            
            current_gps_data = current_gps()
            current_gps_data.EE = float(gps_data[0])
            current_gps_data.NN = float(gps_data[1])
            current_gps_data.acc_x = float(gps_data[2])
            current_gps_data.acc_y = float(gps_data[3])
            current_gps_data.acc_z = float(gps_data[4])
            current_gps_data.pitch_ = float(gps_data[5])
            current_gps_data.roll_ = float(gps_data[6])
            current_gps_data.yaw_ = float(gps_data[7])
            pub = rospy.Publisher('curr_yaw', current_gps, queue_size=10)
            rospy.init_node('current_gps', anonymous=True)
            rate = rospy.Rate(10)
            goal_str =current_gps_data
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

