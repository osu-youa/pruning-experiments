#!/usr/bin/env python

import rospy
import socket
import numpy as np
from geometry_msgs.msg import Vector3Stamped, Vector3


if __name__ == '__main__':

    ADDRESS = 'localhost'
    PORT = 10000

    name = 'vel_command_client'
    rospy.init_node(name)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    address = (ADDRESS, PORT)

    def shutdown():
        sock.close()
        print('Socket closed...')
    rospy.on_shutdown(shutdown)

    sock.connect(address)
    rospy.loginfo('Connected to socket at {}:{}!'.format(*address))

    pub = rospy.Publisher('/vel_command', Vector3Stamped, queue_size=1)

    while not rospy.is_shutdown():
        data = np.zeros(0)
        sock.sendall(data.tostring())
        response = np.fromstring(sock.recv(1024), dtype=np.float64)
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'cutpoint'
        msg.vector = Vector3(*response)
        pub.publish(msg)
