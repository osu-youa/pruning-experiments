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

class DataRecorder(object):

    def __init__(self, ns='', params_to_save=None):
        self.RECORDING = False
        self.ALL_DATA = defaultdict(dict)
        self.save_file = None
        self.params_to_save = params_to_save or []

        prefix = ''
        if ns:
            prefix = ns.strip('/') + '/'

        rospy.Service(prefix + 'record_data', RecordData, self.start_recording)
        rospy.Service(prefix + 'stop_record_data', Empty, self.stop_recording)

    def stop_recording(self, *_, **__):

        if not self.RECORDING:
            return []

        self.RECORDING = False

        for param in self.params_to_save:
            val = rospy.get_param(param, None)
            if val is None:
                rospy.logwarn('Warning! Param {} was not set when recording was started'.format(param))
            self.ALL_DATA[param] = val

        # Save file
        with open(self.save_file, 'wb') as fh:
            cPickle.dump(self.ALL_DATA, fh)
            rospy.loginfo('Saved file to {}'.format(self.save_file))

        self.save_file = None
        self.ALL_DATA = defaultdict(dict)

        return []


    def start_recording(self, req):
        self.save_file = req.file
        self.RECORDING = True
        return []


    def add_subscriber(self, topic, msg_type):
        rospy.Subscriber(topic, msg_type, self.create_subscriber_handler(topic), queue_size=1)


    def create_subscriber_handler(self, name):
        def handler(msg):

            if self.RECORDING:
                while True:
                    try:
                        stamp = msg.header.stamp
                    except AttributeError:
                        stamp = rospy.Time.now()

                    self.ALL_DATA[name][stamp.to_sec()] = msg
                    return


        return handler

    def pop(self):
        to_return = self.ALL_DATA
        self.ALL_DATA = defaultdict(dict)
        return to_return

if __name__ == '__main__':
    rospy.init_node('record_data')
    params_to_save = []
    recorder = DataRecorder(params_to_save=params_to_save)

    # Add your interesting topics here
    # You may consider throttling these topics (i.e. creating a throttled version of the node and subscribing to the throttled node)
    recorder.add_subscriber('/wrench', WrenchStamped)                 # 3 linear force and 3 torques
    recorder.add_subscriber('/camera/color/image_raw/compressed', CompressedImage)
    recorder.add_subscriber('/tool_pose', PoseStamped)

    rospy.spin()