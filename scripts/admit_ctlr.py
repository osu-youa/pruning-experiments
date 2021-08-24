#!/usr/bin/env python

# Hannah Kolano kolanoh@oregonstate.edu
#
# Pruning project: subscribes to wrench data from UR5 and publishes control velocities to cut a branch
#
# Last modified 8/19/2021 by Hannah

import rospy
import sys
import numpy as np

from geometry_msgs.msg import Wrench, Vector3Stamped, Vector3, WrenchStamped
from std_srvs.srv import Empty

class AdmitCtlr():

    def __init__(self, is_connected):
        '''
        Set up subscriber to the force torque sensor
        '''

        # Subscribe to wrist wrench topic
        self.wrench_sub = rospy.Subscriber('/wrench_filtered', WrenchStamped, self.wrench_callback)
        # Publish velocities to robot
        self.vel_pub = rospy.Publisher('/vel_command', Vector3Stamped, queue_size=5)
	    # Set up servoing services
        if is_connected:
    	    servo_activate = rospy.ServiceProxy('/servo_activate', Empty)
            self.servo_stop = rospy.ServiceProxy('/servo_stop', Empty)

        # Set up parameters
        # Desired (reference) wrench
        self.des_wrench = np.array([0, 0, 0, 0, 0, -1.5])
        # Controller gain 
        self.Kf = .05
        # Selection matrix
        self.l = np.diag([1, 0, 0, 0, 1, 1])
        # Velocity limit
        self.vel_lim = 0.01 # 1 cm/s
        # Deadband (for wrench)
        self.f_thresh = 0.25 # 0.2 N (will ignore anything < .2N)
        self.last_dirs = ["stopped", "stopped"]
	self.last_stop_condition = False
        self.is_connected  = is_connected

        self.stop_force_thresh = 0.25
        self.stop_torque_thresh = 0.01

        self.vel = Vector3Stamped()
        self.vel.header.stamp = rospy.Time.now()
        self.vel.header.frame_id = 'tool0_controller'
        self.vel.vector = Vector3(0.0, 0.0, 0.0)
        
        if is_connected:
            servo_activate()
	    
        rospy.loginfo("Finished initializing admit ctlr node.")

    def check_goal_state(self, wrench_vec, stop_f, stop_m):
        '''
        If the wrench is within desired parameters, stop servoing the robot.
        '''
        w_diff = self.des_wrench-wrench_vec
	# rospy.loginfo("moment diff = %0.4f; force y diff: %0.3f", w_diff[0], w_diff[4])

        #if -stop_f < w_diff[4] < stop_f:
        #    y_good = True
        #if -stop_f < w_diff[5] < stop_f:
        #    z_good = True
        #if -stop_m < w_diff[0] < stop_m:
        #    x_good = True
	    # 

        if -stop_f < w_diff[4] < stop_f and -stop_f < w_diff[5] < stop_f and -stop_m < w_diff[0] < stop_m:
            stop_cond = True
            #rospy.loginfo("CONDITIONS MET; STOPPING ROBOT!!!")
            if self.is_connected:
               	self.servo_stop()
        else:
            stop_cond = False
	    	#rospy.loginfo("conditions not met")

        if stop_cond != self.last_stop_condition:
            if stop_cond == True:
                rospy.loginfo("CONDITIONS MET; STOPPING ROBOT!!!")
            else:
                rospy.loginfo("conditions not met; running")
        self.last_stop_condition = stop_cond

    def deadzone(self, wrench_in):
        ''' 
        Implement a "dead zone" for forces
        If the force in is less than the force threshold, sets force to 0
        '''
        wrench_out = wrench_in.copy()
        for ind, f_in in enumerate(wrench_in):
            if f_in > self.f_thresh:
                f_out = f_in-self.f_thresh
            elif f_in < -self.f_thresh:
                f_out = f_in + self.f_thresh
            else:
                f_out = 0
            wrench_out[ind] = f_out
        return wrench_out

    def impose_vel_limit(self, vel):
        if vel > self.vel_lim:
            output_vel = self.vel_lim
        elif vel < -self.vel_lim:
            output_vel = -self.vel_lim
        else:
            output_vel = vel
        return output_vel 

    def show_ctlr_direction(self, vely, velz):
        '''
        Prints in rospy when the controller switches directions.
        '''
        # Check vertical direction
        if vely > 0:
	        dir1_text = "up"
        elif vely < 0:
            dir1_text = "down"
        else:
	        dir1_text = "stopped"

        # Check forward/backward direction
        if velz > 0:
	        dir2_text = "forward"
        elif velz < 0:
	        dir2_text = "backward"
        else:
            dir2_text = "stopped"
        
        these_dirs = [dir1_text, dir2_text]

        if not these_dirs == self.last_dirs:
            rospy.loginfo("Changing direction. \n Moving {} and {}".format(dir1_text, dir2_text))
        self.last_dirs = these_dirs

    def wrench_callback(self, wrench_msg):
        """
        Callback function to deal with incoming wrench messages
        """
        # rospy.loginfo("Force subscriber received a wrench message!")

        # Write the wrench_msg into an array
        w = wrench_msg.wrench
        wrench_vec = np.array([w.torque.x, w.torque.y, w.torque.z, w.force.x, w.force.y, w.force.z])
	    # rospy.loginfo("checking wrench state: {}".format(wrench_vec))
        rospy.logdebug('New wrench. \n Y force: {0} \n Z force: {1} \n X moment: {2}'.format(wrench_vec[4], wrench_vec[5], wrench_vec[0]))
        
        # Admittance controller 
        vel_des = -self.Kf*np.dot(self.l,self.des_wrench-self.deadzone(wrench_vec))

        # Impose the velocity limit
        vel_y_limited = self.impose_vel_limit(vel_des[4])
        vel_z_limited = self.impose_vel_limit(vel_des[5])
	
        # Set up and publish the velocity command
        self.vel.header.stamp = rospy.Time.now()
        self.vel.vector = Vector3(0.0, vel_y_limited, vel_z_limited)
        self.vel_pub.publish(self.vel)
        rospy.logdebug("Published vels: \n {}".format(self.vel.vector))

        # Display human-readable controller directions to the terminal
        self.show_ctlr_direction(vel_y_limited, vel_z_limited)
        
        # rospy.loginfo("checking wrench state: {}".format(wrench_vec))
        #Check for the stop condition
        self.check_goal_state(wrench_vec, self.stop_force_thresh, self.stop_torque_thresh)

if __name__ == '__main__':

    # Initialize node
    rospy.init_node('admit_ctlr', argv=sys.argv)

    ctlr = AdmitCtlr(False)

    rospy.spin()
