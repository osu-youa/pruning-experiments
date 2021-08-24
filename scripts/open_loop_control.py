#!/usr/bin/env python

import rospy
import socket
import numpy as np
from geometry_msgs.msg import Vector3Stamped, Vector3
from pruning_experiments.srv import ServoWaypoints
from std_srvs.srv import Empty

def handle_servo_request(req):
    pts = req.points
    vel = req.vel
    for pt in pts:
        servo_to_point(pt, vel)
    return 0

def servo_to_point(pt, vel):
    print(pt)
    print(vel)

if __name__ == '__main__':
    rospy.init_node('open_loop_manager')
    servo_start = rospy.ServiceProxy('servo_activate', Empty)
    servo_stop = rospy.ServiceProxy('servo_stop', Empty)
    servo_rewind = rospy.ServiceProxy('servo_rewind', Empty)

    # TODO: Add TF manager and constantly update the tool frame's position

    rospy.Service('servo_waypoints', ServoWaypoints, handle_servo_request)
    rospy.spin()

