#!/usr/bin/env python

# Hannah Kolano kolanoh@oregonstate.edu
#
# Pruning project: subscribes to wrench data from UR5 and publishes control velocities to cut a branch
#
# Last modified 9/2/2021 by Hannah

import rospy
import sys
import numpy as np

from geometry_msgs.msg import Wrench, Vector3Stamped, Vector3, WrenchStamped
from std_srvs.srv import Empty, Trigger

class AdmitCtlr():

    def __init__(self, is_connected):
        '''
        Set up subscriber to the force torque sensor
        '''

        # Subscribe to wrist wrench topic
        self.wrench_sub = rospy.Subscriber('/wrench_filtered', WrenchStamped, self.wrench_callback)
        # Publish velocities to robot
        self.vel_pub = rospy.Publisher('/vel_command', Vector3Stamped, queue_size=5)

        rospy.Service("run_admittance_controller", Trigger, self.handle_run_controller)
        self.activated = False

        # Set up parameters
        # Desired (reference) wrench
        self.des_wrench = np.array([0, 0, 0, 0, 0, -2])
        # Controller gains 

        # self.Kf = .04 # M^-1 (higher number = lower mass)
        self.Kf = np.diag([0., 0., 0., 0., 0.01, 0.1])
        self.Kd = 300 # D (or B, but the damping term)
        self.Kd = np.diag([0., 0., 0., 0., 400, 250])

        # Selection matrix
        self.l = np.diag([1, 0, 0, 0, 1, 1])
        # Velocity limit
        self.vel_lim = 0.01  # 1 cm/s
        # Deadband (for wrench)
        self.f_thresh = 0.2  # 0.2 N (will ignore anything < .2N)
        self.last_dirs = ["stopped", "stopped"]
        self.last_stop_condition = False

        self.global_done = False
        self.is_connected = is_connected

        self.stop_force_thresh = 0.15
        self.stop_torque_thresh = 0.0025
        self.publish_freq = 500.0

        self.vel = Vector3Stamped()
        self.vel_prev = np.array([0, 0, 0, 0, 0, 0])
        self.prev_z_vels = np.zeros(1000)
        self.z_travel_amts = np.ones(10)
        self.vel.header.stamp = rospy.Time.now()
        self.vel.header.frame_id = 'tool0_controller'
        self.vel.vector = Vector3(0.0, 0.0, 0.0)
        self.count = 0

        # set up variables for end condition watching
        self.last_goal_checks = np.zeros(10)

        rospy.loginfo("Finished initializing admit ctlr node.")


    def handle_run_controller(self, *_, **__):

        self.activated = True
        self.global_done = False
        start_time = rospy.Time.now()
        rate = rospy.Rate(100)
        try:
            while True:
                if self.global_done:
                    return_msg = (True, "")
                    break
                if (rospy.Time.now() - start_time).to_sec() > 45.0:
                    return_msg = (False, "Timeout")
                    rospy.logwarn('Admittance controller timed out')
                    break
                rate.sleep()
        finally:
            self.activated = False

        return return_msg

    def check_goal_state(self, wrench_vec, z_vel):
        '''
        If the wrench is within desired parameters, stop servoing the robot.
        '''
        stop_f = self.stop_force_thresh
        stop_m = self.stop_torque_thresh
        w_diff = self.des_wrench-wrench_vec
        self.prev_z_vels = np.roll(self.prev_z_vels, -1)
        self.prev_z_vels[-1] = z_vel
        slow_f = 1.5*stop_f
        slow_m = 1.5*stop_m
        forward_travel = np.sum(self.prev_z_vels)/self.publish_freq*1000
        self.z_travel_amts = np.append(self.z_travel_amts, forward_travel)
        self.z_travel_amts = np.delete(self.z_travel_amts, 0)

        rospy.loginfo_throttle(0.1, "mm of forward travel in 1s: {}".format(np.sum(self.prev_z_vels)/self.publish_freq*1000))

        # How many times did it travel less than .01mm forwards or backwards in the last 10 steps?
        no_travel = ((0.01 > self.z_travel_amts) & (self.z_travel_amts > -.01)).sum()
        within_force_bounds = (-stop_f < w_diff[4] < stop_f) & (-stop_f < w_diff[5] < stop_f) & (-stop_m < w_diff[0] < stop_m)

        rospy.loginfo_throttle(0.1, "moment diff = %0.4f; force y diff: %0.3f; force z diff: %0.3f", w_diff[0], w_diff[4], w_diff[5])

        if no_travel > 7:
            rospy.loginfo("No forward progress...")

        if within_force_bounds:
            rospy.loginfo("Forces within stopping bounds")

        if no_travel > 7 and within_force_bounds:
            rospy.loginfo("NO FORWARD PROGRESS AND WITHIN FORCE BOUNDS; STOPPING ROBOT!!!")
            self.global_done = True

        # if -slow_f < w_diff[4] < slow_f and -slow_f < w_diff[5] < 0 and -slow_m < w_diff[0] < slow_m:
            # rospy.loginfo("close to goal state \n")

       # Check if the forces are within the stop condition threshold, add 1 to queue, else a 0
       #  if -stop_f < w_diff[4] < stop_f and -stop_f < w_diff[5] < stop_f and -stop_m < w_diff[0] < stop_m:
       #      stop_cond = True
       #      self.last_goal_checks = np.append(self.last_goal_checks, 1)
       #      rospy.loginfo("stop conditions met this step; may stop robot")
       #  else:
       #      stop_cond = False
       #      self.last_goal_checks = np.append(self.last_goal_checks, 0)
       #      if stop_cond != self.last_stop_condition:
       #          rospy.loginfo("conditions not met; running")

        # # Get rid of least recent stop condition check in queue
        # self.last_goal_checks = np.delete(self.last_goal_checks, 0)
        #
        # # If the last 10 steps met conditions more than 5 times, stop robot
        # if np.sum(self.last_goal_checks) > 2:
        #     rospy.loginfo("CONDITIONS MET SEVERAL TIMES; STOPPING ROBOT!!!")
        #     if self.is_connected:
        #         self.global_done = True
        #
        # self.last_stop_condition = stop_cond

    def deadzone(self, wrench_in):
        ''' 
        Implement a "dead zone" for forces
        If the force in is less than the force threshold, sets force to 0
        '''
        wrench_out = wrench_in.copy()
        for ind, f_in in enumerate(wrench_in):
            if f_in > self.f_thresh:
                f_out = f_in - self.f_thresh
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
        rospy.logdebug('New wrench. \n Y force: {0} \n Z force: {1} \n X moment: {2}'.format(wrench_vec[4], wrench_vec[5], wrench_vec[0]))
        
        # Admittance controller terms
        # acceleration due to force (M-1 * force_des-force_actual)
        # acc_des_force_term = -self.Kf*np.dot(self.l,self.deadzone(self.des_wrench-wrench_vec))
        acc_des_force_term = np.dot(-self.Kf, np.dot(self.l, self.deadzone(self.des_wrench - wrench_vec)))
        # acceleration due to damping (M-1 * B * x')
        # acc_des_damp_term = -self.Kf*self.Kd*self.vel_prev
        acc_des_damp_term = np.dot(-self.Kf, np.dot(self.Kd, self.vel_prev))

        # New controller -- use the acceleration and the publish rate to update the velocity
        vel_des = self.vel_prev + (1/self.publish_freq)*(acc_des_force_term+acc_des_damp_term)

        vel_y_limited = self.impose_vel_limit(vel_des[4])
        vel_z_limited = self.impose_vel_limit(vel_des[5])

        self.vel_prev = np.array([0, 0, 0, 0, vel_y_limited, vel_z_limited])
	
        # Set up and publish the velocity command
        self.vel.header.stamp = rospy.Time.now()
        self.vel.vector = Vector3(0.0, vel_y_limited, vel_z_limited)
        if self.activated:
            self.vel_pub.publish(self.vel)

        rospy.logdebug("Published vels: \n {}".format(self.vel.vector))

        # Display human-readable controller directions to the terminal
        self.show_ctlr_direction(vel_y_limited, vel_z_limited)

        #Check for the stop condition
        self.check_goal_state(wrench_vec, vel_z_limited)

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('admit_ctlr', argv=sys.argv)

    ctlr = AdmitCtlr(True)

    rospy.spin()
