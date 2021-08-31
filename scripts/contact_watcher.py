#!/usr/bin/env python

import rospy
import sys
import numpy as np

from geometry_msgs.msg import WrenchStamped
from pruning_experiments.srv import MadeContact, MadeContactResponse
from std_msgs.msg import Bool

class ContactWatcher():

    def __init__(self):

        # Subscribe to the filtered FT sensor readings
        self.ftsense_sub = rospy.Subscriber('/wrench_filtered', WrenchStamped, self.wrench_callback)

        queue_size = 10
        # Initialize empty queues
        self.z_forces = np.zeros(queue_size)
        self.y_forces = np.zeros(queue_size)
        self.ones = np.ones(queue_size)

        self.f_thresh = 0.75 # N
        rospy.loginfo("Contact watcher waiting for contact.")

    def wrench_callback(self, wrench_msg):
        # Extract forces from the wrench
        w = wrench_msg.wrench.force

        # Update the queue
        self.z_forces = np.append(self.z_forces, w.z)
        self.y_forces = np.append(self.y_forces, w.y)
        self.z_forces = np.delete(self.z_forces, 0)
        self.y_forces = np.delete(self.y_forces, 0)

        # If currently waiting for contact
        z_less_than = (self.z_forces < -self.f_thresh)
        z_more_than = (self.z_forces > self.f_thresh)
        z_outside_mask = (z_less_than | z_more_than)

        y_less_than = (self.y_forces < -self.f_thresh)
        y_more_than = (self.y_forces > self.f_thresh)
        y_outside_mask = (y_less_than | y_more_than)

        num_z_outside = np.sum(self.ones[z_outside_mask])
        num_y_outside = np.sum(self.ones[y_outside_mask])

        if num_y_outside > 5 or num_z_outside > 5:
            contact_pub.publish(Bool(True))

if __name__ == '__main__':

    # Initialize node
    rospy.init_node('contact_watcher')
    contact_pub = rospy.Publisher('contact', Bool, queue_size=1)

    watcher = ContactWatcher()

    rospy.spin()