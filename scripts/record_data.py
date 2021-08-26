#!/usr/bin/env python

import rospy
import socket
import numpy as np
from pruning_experiments.srv import RecordData
from std_srvs.srv import Empty

SAVE_FILE = None

def handle_record_data(req):
    file = req.file

def handle_stop_record_data(*_, **__):
    pass

if __name__ == '__main__':
    rospy.init_node('record_data')

    rospy.Service('record_data', RecordData, handle_record_data)
    rospy.Service('stop_record_data', Empty, handle_stop_record_data)

    rospy.spin()