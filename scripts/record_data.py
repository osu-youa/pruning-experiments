#!/usr/bin/env python

import rospy
import socket
import numpy as np
from pruning_experiments.srv import RecordData
from std_srvs.srv import Empty

from geometry_msgs.msg import WrenchStamped, PoseStamped, Vector3Stamped
import cPickle
from collections import defaultdict
from std_srvs.srv import Empty
from sensor_msgs.msg import CompressedImage
import subprocess, shlex

class DataRecorder(object):

    def __init__(self, to_record, ns=''):
        self.to_record = to_record
        self.active_process = None

        prefix = ''
        if ns:
            prefix = ns.strip('/') + '/'

        rospy.Service(prefix + 'record_data', RecordData, self.start_recording)
        rospy.Service(prefix + 'stop_record_data', Empty, self.stop_recording)

    def stop_recording(self, *_, **__):
        self.active_process.terminate()
        return []

    def start_recording(self, req):

        if self.active_process is not None:
            self.active_process.terminate()

        msg = "rosbag record -O {} ".format(req.file) + ' '.join(self.to_record)
        args = shlex.split(msg)
        self.active_process = subprocess.Popen(args, stderr=subprocess.PIPE, shell=False)
        return []


if __name__ == '__main__':
    rospy.init_node('record_data')
    topics = ['/wrench', '/tool_pose', '/camera/color/image_raw/compressed', '/vel_command']
    recorder = DataRecorder(topics)
    rospy.spin()
