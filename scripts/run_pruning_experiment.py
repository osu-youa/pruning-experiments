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
from std_srvs.srv import Empty
import cPickle as pickle
from pruning_experiments.srv import ServoWaypoints, RecordData
from functools import partial

data_folder = os.path.join(os.path.expanduser('~'), 'data', 'icra2022')
fwd_velocity = 0.03
POSE_ID = None
POSE_LIST = []
ACTIVE_POSE = None


# ==========
# UTILS
# ==========

def get_file_name(open_loop=False, variant=False):
    if ACTIVE_POSE is None:
        return None
    identifier = open_loop * 2 + variant
    return 'data_{}_{}.pickle'.format(identifier, ACTIVE_POSE)

def record_success():
    print('[TODO]')

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

def pt_to_array(pt):
    if isinstance(pt, PointStamped):
        pt = pt.point

    return np.array([pt.x, pt.y, pt.z])

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
    counter = 0
    while True:
        file = os.path.join(data_folder, 'POSE_{}.pickle'.format(counter))
        if os.path.exists(file):
            counter += 1
            continue
        with open(file, 'wb') as fh:
            pickle.dump(output, fh)
        return counter

def load_status(pose_id):
    file = os.path.join(data_folder, 'POSE_{}.pickle'.format(pose_id))
    try:
        with open(file, 'rb') as fh:
            rez = pickle.load(fh)
    except IOError:
        print('No such pose exists!')
        return
    global POSE_ID
    global POSE_LIST
    global ACTIVE_POSE
    POSE_ID = pose_id
    POSE_LIST = rez['poses']
    ACTIVE_POSE = rez['active_pose']

# ==========
# ACTIONS
# ==========

def set_active_pose():
    global POSE_LIST
    global ACTIVE_POSE
    global POSE_ID
    tf = retrieve_tf(tool_frame, base_frame)
    POSE_LIST = generate_pose_grid(tf)
    ACTIVE_POSE = None
    POSE_ID = save_status()

def load_pose():

    to_load = int(raw_input('Which file ID do you want to load?'))
    load_status(to_load)
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

def run_open_loop(use_miscalibrated = False):

    camera_base = 'tool0'
    if use_miscalibrated:
        camera_base = 'tool0_false'

    # TODO: Start publishing transforms here to link to the camera

    camera_connected = False
    if camera_connected:
        raise NotImplementedError()
    else:
        # If no camera is connected, just run a test motion
        print('No camera connected! Running test motion...')
        final_target = PointStamped()
        final_target.header.frame_id = tool_frame
        final_target.point = Point(0.0, -0.05, 0.15)

    tf = retrieve_tf(final_target.header.frame_id, base_frame)
    final_target = do_transform_point(final_target, tf)

    final_target_array = pt_to_array(final_target)
    intermediate_array = final_target_array - np.array([0.0, 0.01, 0.04])

    waypoints = []
    for array in [intermediate_array, final_target_array]:
        pt = PointStamped()
        pt.header.frame_id = base_frame
        pt.point = Point(*array)
        waypoints.append(pt)

    file_name = get_file_name(open_loop=True, variant=use_miscalibrated)
    if file_name is None:
        print('[!] Warning, your data will not be recorded for this run!')
    else:
        record_data_srv(file_name)

    response = open_loop_srv(waypoints, fwd_velocity)
    print(response)

    if file_name is not None:
        stop_record_data_srv()
    record_success()
    raw_input('Servoing done! Press Enter to rewind...')
    servo_rewind()

def run_closed_loop(use_nn=False):

    # TODO: BOOT UP THE NN SUBPROCESS

    file_name = get_file_name(open_loop=False, variant=not use_nn)
    if file_name is None:
        print('[!] Warning, your data will not be recorded for this run!')
    else:
        record_data_srv(file_name)

    # TODO: DO STUFF

    if file_name is not None:
        stop_record_data_srv()
    record_success()
    raw_input('Servoing done! Press Enter to rewind...')
    servo_rewind()


class StopProgramException(Exception):
    pass

def stop_program():
    raise StopProgramException()

if __name__ == '__main__':

    # SETUP
    rospy.init_node('experiment_manager')
    base_frame = rospy.get_param('base_frame')
    tool_frame = rospy.get_param('tool_frame')

    tf_buffer = Buffer()
    tf_listener = TransformListener2(tf_buffer)

    plan_pose_srv = rospy.ServiceProxy('plan_pose', HandlePosePlan)
    plan_joints_srv = rospy.ServiceProxy('plan_joints', HandleJointPlan)
    open_loop_srv = rospy.ServiceProxy('servo_waypoints', ServoWaypoints)
    record_data_srv = rospy.ServiceProxy('record_data', RecordData)
    stop_record_data_srv = rospy.ServiceProxy('stop_record_data', Empty)
    servo_rewind = rospy.ServiceProxy('servo_rewind', Empty)

    rospy.sleep(1.0)

    actions = [
        ('Quit', stop_program),
        ('Set the active pose', set_active_pose),
        ('Load the previous active pose', load_pose),
        ('Move to next pose', next_pose),
        # ('Freedrive to a new pose', freedrive),
        ('Level the existing pose', level_pose),
        ('Preview target grid', preview_grid),
        ('Run open loop controller', run_open_loop),
        ('Run closed loop controller', run_closed_loop),

    ]


    while True:

        checklist = '\n'.join(['{}) {}'.format(i, msg) for i, (msg, _) in enumerate(actions)])
        if not POSE_LIST:
            status = 'Pose list has not been generated.'
        else:
            if POSE_ID is None:
                prefix = '(UNSAVED POSE)'
            else:
                prefix = 'Pose {}'.format(POSE_ID)

            if ACTIVE_POSE is None:
                status = '{}: Currently not at given pose in the pose list.'.format(prefix)
            else:
                status = '{}: Currently at pose {} out of {}.'.format(prefix, ACTIVE_POSE + 1, len(POSE_LIST))


        msg = "What would you like to do?\n{}\n{}\nType action here: ".format(status, checklist)

        try:
            action = int(raw_input(msg))
            actions[action][1]()
        except (ValueError, IndexError):
            rospy.logerr('Not a valid action!')
        except StopProgramException:
            break

    print('All done!')


