#!/usr/bin/env python

import rospy
import socket
import numpy as np
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Bool

CONTACT = False
ABORT = False

def handle_contact(msg):
    contact = msg.data
    global CONTACT
    CONTACT = contact

def handle_abort(msg):
    abort = msg.data
    global ABORT
    ABORT = abort

if __name__ == '__main__':

    ADDRESS = 'localhost'
    PORT = 10000

    name = 'vel_command_client'
    rospy.init_node(name)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    address = (ADDRESS, PORT)

    try:
        sock.connect(address)
    except socket.error:
        code = -1
        print(code)
        exit(0)

    def shutdown():
        sock.close()
    rospy.on_shutdown(shutdown)

    # rospy.loginfo('Connected to socket at {}:{}!'.format(*address))

    run_contact_controller = lambda: 5  # Dummy
    pub = rospy.Publisher('/vel_command', Vector3Stamped, queue_size=1)
    rospy.Subscriber('/contact', Bool, handle_contact, queue_size=1)
    rospy.Subscriber('/abort', Bool, handle_abort, queue_size=1)

    code = 0
    while not rospy.is_shutdown():
        if ABORT:
            code = 1
            break
        if CONTACT:
            code = run_contact_controller()
            break

        data = np.zeros(0)
        sock.sendall(data.tostring())
        response = np.fromstring(sock.recv(1024), dtype=np.float64)
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'cutpoint'
        msg.vector = Vector3(*response)
        pub.publish(msg)

    print(code)