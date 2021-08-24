#!/usr/bin/env python
import os.path

import rospy
import socket
import numpy as np
from copy import deepcopy
from geometry_msgs.msg import Vector3Stamped, Vector3, TransformStamped, Transform, Point, Pose, PoseStamped, PointStamped, Quaternion
from tf2_ros import TransformListener as TransformListener2, Buffer
from itertools import product
from tf2_geometry_msgs import do_transform_point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from arm_utils.srv import HandlePosePlan, HandleJointPlan
from sensor_msgs.msg import JointState
import cPickle as pickle

data_folder = os.path.join(os.path.expanduser('~'), 'data', 'icra2022')
base_frame = 'base_link'
tool_frame = 'tool0'
POSE_LIST = []
ACTIVE_POSE = None


# ==========
# UTILS
# ==========

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

def generate_pose_grid(tf, x_offsets = (-0.02, 0, 0.02), y_offsets = (-0.02, 0, 0.02)):
    rez = []
    base_pose = tf_to_pose(tf, keep_header=True)
    for x, y in product(x_offsets, y_offsets):
        pt = PointStamped()
        pt.header = tool_frame
        pt.point = Point(x, y, 0)
        tfed_pt = do_transform_point(pt, tf)
        pose = deepcopy(base_pose)
        pose.pose.position = tfed_pt.point
        rez.append(pose)
    return rez

def save_status():
    output = {
        'poses': POSE_LIST,
        'active_pose': None,
    }
    with open(os.path.join(data_folder, 'status.pickle'), 'wb') as fh:
        pickle.dump(output, fh)

def load_status():
    try:
        with open(os.path.join(data_folder, 'status.pickle'), 'rb') as fh:
            rez = pickle.load(fh)
    except IOError:
        print('No poses have been saved!')
        return
    global POSE_LIST
    global ACTIVE_POSE
    POSE_LIST = rez['poses']
    ACTIVE_POSE = rez['active_pose']

# ==========
# ACTIONS
# ==========

def set_active_pose():
    global POSE_LIST
    global ACTIVE_POSE
    tf = retrieve_tf(tool_frame, base_frame)
    POSE_LIST = generate_pose_grid(tf)
    ACTIVE_POSE = None

    save_status()

def load_pose():
    load_status()
    if POSE_LIST:
        plan_pose_srv(POSE_LIST[0], True)

def next_pose():
    global ACTIVE_POSE
    global POSE_LIST
    if ACTIVE_POSE is None:
        ACTIVE_POSE = 0
    else:
        ACTIVE_POSE = (ACTIVE_POSE + 1) % len(POSE_LIST)

    plan_pose_srv(POSE_LIST[ACTIVE_POSE], True)

def freedrive():
    pass

def level_pose():
    tf = retrieve_tf(tool_frame, base_frame)
    rot = tf.transform.rotation
    eul = list(euler_from_quaternion([rot.x, rot.y, rot.z, rot.w], 'rxyz'))
    eul[0] = -np.pi/2
    rot_new = Quaternion(*quaternion_from_euler(*eul, axes='rxyz'))
    pose = tf_to_pose(tf, keep_header=True)
    pose.pose.orientation = rot_new

    plan_pose_srv(pose, True)

def preview_grid():
    current_joints = rospy.wait_for_message('/joint_states', JointState)
    to_move = POSE_LIST
    if not POSE_LIST:
        tf = retrieve_tf(tool_frame, base_frame)
        to_move = generate_pose_grid(tf)

    for pose in to_move:
        plan_pose_srv(pose, True)

    plan_joints_srv(current_joints, True)

def run_open_loop():
    pass

def run_closed_loop():
    pass

class StopProgramException(Exception):
    pass

def stop_program():
    raise StopProgramException()

if __name__ == '__main__':

    # SETUP
    rospy.init_node('experiment_manager')
    tf_buffer = Buffer()
    tf_listener = TransformListener2(tf_buffer)

    plan_pose_srv = rospy.ServiceProxy('plan_pose', HandlePosePlan)
    plan_joints_srv = rospy.ServiceProxy('plan_joints', HandleJointPlan)


    rospy.sleep(1.0)





    actions = [
        ('Set the active pose', set_active_pose),
        ('Load the previous active pose', load_pose),
        ('Move to next pose', next_pose),
        ('Freedrive to a new pose', freedrive),
        ('Level the existing pose', level_pose),
        ('Preview target grid', preview_grid),
        ('Run open loop controller', run_open_loop),
        ('Run closed loop controller', run_closed_loop),
        ('Quit', stop_program)
    ]


    while True:

        checklist = '\n'.join(['{}) {}'.format(i, msg) for i, (msg, _) in enumerate(actions)])
        if not POSE_LIST:
            status = 'Pose list has not been generated.'
        else:
            if ACTIVE_POSE is None:
                status = 'Currently not at given pose in the pose list.'
            else:
                status = 'Currently at pose {} out of {}.'.format(ACTIVE_POSE + 1, len(POSE_LIST))

        msg = "What would you like to do?\n{}\n{}\nType action here: ".format(status, checklist)

        try:
            action = int(raw_input(msg))
            actions[action][1]()
        except (ValueError, IndexError):
            rospy.logerr('Not a valid action!')
        except StopProgramException:
            break

    print('All done!')


