#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import socket
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from nav_goal.msg import position

def unicode_convert(input):
    if isinstance(input, dict):
        return {unicode_convert(key): unicode_convert(value) for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [unicode_convert(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

def talker():
    rospy.loginfo("runing")
    data = {'key': 'd39dd2133f0789500e8aff516da06935', 'origin': '123.419212, 41.764191',
            'destination': '123.420286,41.76547', 'output': 'JSON'}
    ret = requests.get("https://restapi.amap.com/v3/direction/driving?", params=data)

    ret_json_to_dict = json.loads(ret.text)
    start_point = ret_json_to_dict["route"]["origin"]
    end_point = ret_json_to_dict["route"]["destination"]
    start_point = start_point.split(";")
    end_point = end_point.split(";")
    # rospy.loginfo(start_point)
    load_data = ret_json_to_dict["route"]["paths"][0]["steps"]
    goal = []
    goal.extend(start_point)

    load_num = len(load_data)

    for i in range(0, load_num):
        mstr = load_data[i]["polyline"]
        mlist = mstr.split(";")
        goal.extend(mlist)

    goal.extend(end_point)
    rospy.loginfo(goal)

    goal2 = unicode_convert(goal)
    arr = np.zeros((len(goal2), 2))
    for i in range(0, len(goal2)):
        arr[i] = goal2[i].split(',')
    nav_goal = position()
    nav_goal.n = len(goal2)
    nav_goal.EN = arr.reshape(-1).tolist()
    pub = rospy.Publisher('nav_goal', position, queue_size=10)
    rospy.init_node('navigation_goal', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        goal_str = nav_goal
        rospy.loginfo(goal_str)
        pub.publish(goal_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

