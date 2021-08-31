#!/usr/bin/env python

# Server that gets a message when contact is made. 
# Last modified by Hannah Kolano 8/23/21

import rospy
import sys

from pruning_imp_ctlr.srv import MadeContact, MadeContactResponse

def callback(request):
    rospy.loginfo("Contact Server received {}".format(request.made_contact))

    return MadeContactResponse(True)

if __name__ == '__main__':

    # Initialize node
    rospy.init_node('contact_server')

    service = rospy.Service('declare_contact', MadeContact, callback)

    rospy.loginfo("Contact Server ready for messages.")

    rospy.spin()