#!/usr/bin/env python

import rospy
import socket
import numpy as np
from geometry_msgs.msg import Vector3Stamped, Vector3, PoseStamped, PointStamped
from pruning_experiments.srv import ServoWaypoints
from std_srvs.srv import Empty

last_pt = np.array([0.0, 0.0, 0.0])

def handle_servo_request(req):
    pts = req.points
    vel = req.vel
    servo_start()
    try:
        for pt in pts:
            servo_to_point(pt, vel)
    finally:
        servo_stop()

    return 0

def servo_to_point(pt, vel, slow_thres = 0.02, slow_vel=0.005, stop_thres=0.001):

    start_pt = last_pt.copy()
    goal_pt = pt_to_array(pt)
    init_approach_vec = start_pt - goal_pt
    init_approach_vec = init_approach_vec / np.linalg.norm(init_approach_vec)

    done = False
    while not done:
        command_vec = goal_pt - last_pt
        current_dist = -command_vec.dot(init_approach_vec)
        command_vec /= np.linalg.norm(command_vec)

        if current_dist < stop_thres:
            command_vec = np.zeros(3)
            done = True
        elif current_dist > slow_thres:
            command_vec *= vel
        else:
            limited_vel = vel + (slow_vel - vel) * (1 - current_dist / slow_thres)
            command_vec *= limited_vel

        vec = Vector3Stamped()
        vec.header.frame_id = 'base_link'
        vec.vector = Vector3(*command_vec)
        vel_pub.publish(vec)

def pt_to_array(pt):
    if isinstance(pt, PointStamped):
        pt = pt.point
    return np.array([pt.x, pt.y, pt.z])


def pose_to_pt_array(pose):
    if isinstance(pose, PoseStamped):
        pose = pose.pose

    pt = pose.position
    return np.array([pt.x, pt.y, pt.z])

def update_pose(pose):
    global last_pt
    last_pt = pose_to_pt_array(pose)

if __name__ == '__main__':
    rospy.init_node('open_loop_manager')
    servo_start = rospy.ServiceProxy('servo_activate', Empty)
    servo_stop = rospy.ServiceProxy('servo_stop', Empty)
    servo_rewind = rospy.ServiceProxy('servo_rewind', Empty)
    rospy.Subscriber('tool_pose', PoseStamped, update_pose)
    vel_pub = rospy.Publisher('vel_command', Vector3Stamped, queue_size=1)
    rospy.sleep(1.0)

    rospy.Service('servo_waypoints', ServoWaypoints, handle_servo_request)
    rospy.spin()

