#!/usr/bin/env python
import os.path

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from ur_dashboard_msgs.msg import SafetyMode
from ur_dashboard_msgs.srv import Load

LAST_MODE = 1

def handle_safety_mode(msg):
    mode = msg.mode
    if mode != 1:
        rospy.logwarn('Safety mode status {} detected!'.format(mode))
    global LAST_MODE
    if mode != 1 and mode != LAST_MODE:
        pub.publish(Bool(True))
    LAST_MODE = mode

def handle_load_cutter_install(*_, **__):
    resp = install_srv('2021mockcutter.installation')
    return []

if __name__ == '__main__':
    rospy.init_node('safety_status_manager')

    sub = rospy.Subscriber('/ur_hardware_interface/safety_mode', SafetyMode, handle_safety_mode, queue_size=1)
    pub = rospy.Publisher('/abort', Bool, queue_size=1)
    rospy.Service('/load_cutter_installation', Empty, handle_load_cutter_install)
    install_srv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_installation', Load)

    rospy.spin()
