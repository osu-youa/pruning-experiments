#!/usr/bin/env python

import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Wrench, Vector3Stamped, Vector3, WrenchStamped

class ForceFilter():

    def __init__(self, kernel):
        self.kernel = kernel
	    # subscribe to wrench topic from UR5
        self.wrench_sub = rospy.Subscriber('/wrench', WrenchStamped, self.wrench_callback)
	    # Publish to 'filtered wrench' topic
        self.wrench_pub = rospy.Publisher('/wrench_filtered', WrenchStamped, queue_size=10)
        
        self.new_wrench = WrenchStamped()

        self.f_y_queue = np.zeros(kernel)
        self.f_z_queue = np.zeros(kernel)
        self.m_x_queue = np.zeros(kernel)
        rospy.loginfo("Filter running.")

        self.counter = 0

    def wrench_callback(self, wrench_msg):
        """
        Callback function to deal with incoming wrench messages; publishes filtered wrench 
        """
        # rospy.loginfo("Wrench filter received a wrench message!")

        # Write the wrench_msg into an array
        w = wrench_msg.wrench
        self.f_y_queue = np.append(self.f_y_queue, w.force.y)
        self.f_z_queue = np.append(self.f_z_queue, w.force.z)
        self.m_x_queue = np.append(self.m_x_queue, w.torque.x)
        self.f_y_queue = np.delete(self.f_y_queue, 0)
        self.f_z_queue = np.delete(self.f_z_queue, 0)
        self.m_x_queue = np.delete(self.m_x_queue, 0)

        self.counter +=1 

        if self.counter > self.kernel*2:
            # Filter Fy, Fz, and Mx
            [filt_y, filt_z, filt_Mx] = self.median_filter()
        
            # Set up the wrench output
            self.new_wrench.header = wrench_msg.header
            self.new_wrench.header.stamp = rospy.Time.now()

            # forward the wrenches not used by the controller
            self.new_wrench.wrench = w

            # write filtered wrench data to messsage
            self.new_wrench.wrench.torque.x = filt_Mx
            self.new_wrench.wrench.force.y = filt_y
            self.new_wrench.wrench.force.z = filt_z

            # Publish the new wrench
            self.wrench_pub.publish(self.new_wrench)

    def median_filter(self):
        '''
        Returns the mean value of the data in the queue.
        '''
        filt_y = np.mean(self.f_y_queue)
        filt_z = np.mean(self.f_z_queue)
        filt_Mx = np.mean(self.m_x_queue)
        return filt_y, filt_z, filt_Mx


if __name__ == '__main__':

    # Initialize node
    rospy.init_node('force_filter', argv=sys.argv)

    # input: kernel size
    filter = ForceFilter(51)

    rospy.spin()
