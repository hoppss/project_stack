#!/usr/bin/env python

import socket
import struct
import rospy
from geometry_msgs.msg import Twist

HOST = '0.0.0.0'
PORT = 8000
MOVE_BINDINGS = [(1, 1), (1, 0), (1, -1),
                 (0, 1), (0, 0), (0, -1),
                 (-1, -1), (-1, 0), (-1, 1)]
SPEED = .2
TURN = 1

if __name__ == '__main__':
    rospy.init_node('gui_control')
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST, PORT))
    s.settimeout(1)
    cmd_size = struct.calcsize('I')
    control_speed = 0.
    control_turn = 0.

    try:
        while True:
            try:
                data, addr = s.recvfrom(cmd_size)
                cmd = struct.unpack('I', data)[0]
                print 'Received command: %d' % cmd
            except socket.timeout:
                cmd = 4

            x, th = MOVE_BINDINGS[cmd]
            target_speed = SPEED * x
            target_turn = SPEED * th

            if target_speed > control_speed:
                control_speed = min(target_speed, control_speed + 0.02)
            elif target_speed < control_speed:
                control_speed = max(target_speed, control_speed - 0.02)
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min(target_turn, control_turn + 0.1)
            elif target_turn < control_turn:
                control_turn = max(target_turn, control_turn - 0.1)
            else:
                control_turn = target_turn

            if cmd == 4:
                control_speed = 0
                contorl_turn = 0

            twist = Twist()
            twist.linear.x = control_speed
            twist.linear.y = 0.
            twist.linear.z = 0.
            twist.angular.x = 0.
            twist.angular.y = 0.
            twist.angular.z = control_turn
            pub.publish(twist)
        s.close()
    except Exception, e:
        print e
    finally:
        twist = Twist()
        twist.linear.x = 0.
        twist.linear.y = 0.
        twist.linear.z = 0.
        twist.angular.x = 0.
        twist.angular.y = 0.
        twist.angular.z = 0.
        pub.publish(twist)
