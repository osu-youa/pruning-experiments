#!/usr/bin/env python
import os.path

import rospy
from geometry_msgs.msg import TransformStamped, Point, Pose, PoseStamped
from tf2_ros import TransformListener as TransformListener2, Buffer

def tf_to_pose(tf, keep_header=False):
    header = None
    if isinstance(tf, TransformStamped):
        header = tf.header
        tf = tf.transform
    elif keep_header:
        raise ValueError("Cannot preserve the header of a non-stamped Transform!")
    t = tf.translation
    r = tf.rotation
    pose = Pose()
    pose.position = Point(t.x, t.y, t.z)
    pose.orientation = r
    if not keep_header:
        return pose
    assert header is not None
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header = header
    return pose_stamped


def retrieve_tf(base_frame, target_frame, stamp = rospy.Time()):

    # Retrieves a TransformStamped with a child frame of base_frame and a target frame of target_frame
    # This transform can be applied to a point in the base frame to transform it to the target frame
    success = tf_buffer.can_transform(target_frame, base_frame, stamp, rospy.Duration(0.5))
    if not success:
        rospy.logerr("Couldn't look up transform between {} and {}!".format(target_frame, base_frame))
    tf = tf_buffer.lookup_transform(target_frame, base_frame, stamp)
    return tf


if __name__ == '__main__':

    rospy.init_node('pose_publisher')

    base_frame = rospy.get_param('base_frame')
    tool_frame = rospy.get_param('tool_frame')
    pub_topic = 'tool_pose'

    pub = rospy.Publisher(pub_topic, PoseStamped, queue_size=1)
    tf_buffer = Buffer()
    tf_listener = TransformListener2(tf_buffer)
    rospy.sleep(3.0)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        tf = retrieve_tf(tool_frame, base_frame)
        pose = tf_to_pose(tf, keep_header=True)
        pub.publish(pose)


