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
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_from_matrix, euler_matrix
from arm_utils.srv import HandlePosePlan, HandleJointPlan
from sensor_msgs.msg import JointState, PointCloud2
from std_srvs.srv import Empty, Trigger
import cPickle as pickle
from pruning_experiments.srv import ServoWaypoints, RecordData
from functools import partial
import subprocess, shlex
from contextlib import contextmanager

data_folder = os.path.join(os.path.expanduser('~'), 'data', 'icra2022')
fwd_velocity = 0.03
POSE_ID = None
POSE_INFO = {}

INTERMEDIATE_OFFSET = np.array([0.0, 0.01, 0.05])
STANDARD_CAMERA_POS = np.array([0.0719, 0.07416, -0.0050 + 0.031 - 0.0248/2])
STANDARD_CAMERA_ROT = np.array([0, -np.radians(30), 0])

# ==========
# UTILS
# ==========

@contextmanager
def subprocess_manager(subproc_msg):
    args = shlex.split(subproc_msg)
    resource = subprocess.Popen(args, stderr=subprocess.PIPE, shell=False)
    try:
        yield resource
    finally:
        resource.terminate()

def get_file_name(open_loop=False, variant=False):
    if POSE_ID is None:
        return None
    identifier = open_loop * 2 + variant
    return os.path.join(data_folder, 'data_{}_{}.pickle'.format(POSE_ID, identifier))

def record_success(file_name, auto_failure=False):

    with open(file_name, 'rb') as fh:
        data = pickle.load(fh)
    if auto_failure:
        in_cutter = False
        dist = None
        print('Run was aborted, marking cutter/distance as a failure...')
    else:
        dist = None
        in_cutter = bool(int(raw_input('Type 1 if branch is in cutter, 0 otherwise: ')))
        if in_cutter:
            dist = float(raw_input('Please measure the branch distance from the cutpoint (cm): '))

    data['success'] = in_cutter
    data['dist'] = dist
    with open(file_name, 'wb') as fh:
        pickle.dump(data, fh)

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

def load_status(pose_id):
    file = os.path.join(data_folder, 'POSE_{}.pickle'.format(pose_id))
    try:
        with open(file, 'rb') as fh:
            rez = pickle.load(fh)
    except IOError:
        print('No such pose exists!')
        return
    global POSE_ID
    global POSE_INFO

    POSE_ID = pose_id
    POSE_INFO = rez

# ==========
# ACTIONS
# ==========

def load_pose():

    pose_files = [x for x in os.listdir(data_folder) if x.startswith('POSE') and x.endswith('.pickle')]
    pose_ids = sorted([int(x.replace('.pickle', '').replace('POSE_', '')) for x in pose_files])

    print('Which file ID do you want to load?')
    print('Available IDs: {}'.format(', '.join(map(str, pose_ids))))
    to_load = int(raw_input('Your choice: '))
    load_status(to_load)
    if POSE_INFO:
        plan_pose_srv(POSE_INFO['base'], True)


def save_pose(pose_info=None):

    if pose_info is None:
        current_pose = rospy.wait_for_message('tool_pose', PoseStamped, timeout=1.0)
        pose_info = {'base': current_pose}

    pose_id = 0
    while True:
        file = os.path.join(data_folder, 'POSE_{}.pickle'.format(pose_id))
        if not os.path.exists(file):
            with open(file, 'wb') as fh:
                pickle.dump(pose_info, fh)
            print('Saved pose ID {} at {}'.format(pose_id, file))
            return pose_id
        pose_id += 1

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

def get_camera_point_if_connected():
    camera_connected = False
    try:
        rospy.wait_for_message('/camera/depth_registered/points', PointCloud2, timeout=1.0)
        camera_connected = True
    except:
        pass

    if camera_connected:
        print('Please click on a point in RViz to go to! (Waiting 30 seconds')
        final_target = rospy.wait_for_message('/clicked_point', PointStamped, timeout=30.0)

    else:
        # If no camera is connected, just run a test motion
        print('No camera connected! Running test motion...')
        final_target = PointStamped()
        final_target.header.frame_id = tool_frame
        final_target.point = Point(0.0, -0.05, 0.15)

    return final_target

def get_camera_tf(noise=None):
    pos = list(STANDARD_CAMERA_POS)

    rot_mat = euler_matrix(*STANDARD_CAMERA_ROT)[:3,:3]
    adjustment_mat = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])
    final_euler = euler_from_matrix(rot_mat.dot(adjustment_mat))
    quat = list(quaternion_from_euler(*final_euler))
    return ' '.join(['{}'] * 7).format(*pos + quat)

def run_open_loop(use_miscalibrated = False):

    if use_miscalibrated:
        noise = None
    else:
        noise = None

    tare_force_sensor()

    tf_str = get_camera_tf(noise=noise)
    with subprocess_manager('rosrun tf static_transform_publisher {} tool0 camera_link 10'.format(tf_str)):

        final_target = get_camera_point_if_connected()
        if final_target.header.frame_id != tool_frame:
            tf = retrieve_tf(final_target.header.frame_id, tool_frame)
            final_target = do_transform_point(final_target, tf)

        final_target_array = pt_to_array(final_target)

        if np.linalg.norm(final_target_array) > 0.4:
            rospy.logwarn('This point seems pretty far ahead! Are you sure this is what you want?')
        intermediate_array = final_target_array - INTERMEDIATE_OFFSET

        tf = retrieve_tf(final_target.header.frame_id, base_frame)
    waypoints = []
    for array in [intermediate_array, final_target_array]:
        pt = PointStamped()
        pt.header.frame_id = final_target.header.frame_id
        pt.point = Point(*array)
        tfed_pt = do_transform_point(pt, tf)
        waypoints.append(tfed_pt)

    file_name = get_file_name(open_loop=True, variant=use_miscalibrated)
    if file_name is None:
        print('[!] Warning, your data will not be recorded for this run!')
    else:
        record_data_srv(file_name)

    response = open_loop_srv(waypoints, fwd_velocity, False).code
    if file_name is not None:
        stop_record_data_srv()
        record_success(file_name)
    raw_input('Servoing done! Press Enter to rewind...')
    servo_rewind()

def run_closed_loop(use_nn=False):

    tare_force_sensor()

    file_name = get_file_name(open_loop=False, variant=not use_nn)
    if use_nn:
        if file_name is None:
            print('[!] Warning, your data will not be recorded for this run!')
        else:
            record_data_srv(file_name)

        code = subprocess.check_output(shlex.split('rosrun pruning_experiments velocity_command_client.py')).strip()
        if code == str(-1):
            rospy.logerr("Neural network failed to start!")
        else:
            code = int(code)

    else:
        # Find the intermediate point and target a straight line
        final_target = get_camera_point_if_connected()
        final_target_array = pt_to_array(final_target)
        if np.linalg.norm(final_target_array) > 0.4:
            rospy.logwarn('This point seems pretty far ahead! Are you sure this is what you want?')
        intermediate_array = final_target_array - INTERMEDIATE_OFFSET
        tf = retrieve_tf(final_target.header.frame_id, base_frame)

        pt = PointStamped()
        pt.header.frame_id = final_target.header.frame_id
        pt.point = Point(*intermediate_array)
        tfed_pt = do_transform_point(pt, tf)
        intermediate_world = pt_to_array(tfed_pt)
        cutter_world = pt_to_array(rospy.wait_for_message('tool_pose', PoseStamped, timeout=1.0).pose.position)
        movement_vec = (intermediate_world - cutter_world)
        movement_vec /= np.linalg.norm(movement_vec)
        actual_target = intermediate_world + 0.04 * movement_vec
        tfed_pt.point = Point(*actual_target)
        waypoints = [tfed_pt]

        if file_name is None:
            print('[!] Warning, your data will not be recorded for this run!')
        else:
            record_data_srv(file_name)
        code = open_loop_srv(waypoints, fwd_velocity, True).code

    rospy.loginfo("Return code: {}".format(code))

    if file_name is not None:
        stop_record_data_srv()
        record_success(file_name)
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
    tare_force_sensor = rospy.ServiceProxy('/ur_hardware_interface/zero_ftsensor', Trigger)

    rospy.sleep(1.0)

    actions = [
        ('Quit', stop_program),
        ('Load the previous active pose', load_pose),
        ('Save current pose', save_pose),
        # ('Freedrive to a new pose', freedrive),
        ('Level the existing pose', level_pose),
        ('Run open loop controller', run_open_loop),
        ('Run open loop controller miscalibrated', partial(run_open_loop, use_miscalibrated=True)),
        ('Run simple closed loop controller', run_closed_loop),
        ('Run NN closed loop controller', partial(run_closed_loop, use_nn=True)),
    ]


    while True:

        checklist = '\n'.join(['{}) {}'.format(i, msg) for i, (msg, _) in enumerate(actions)])
        if POSE_ID is None:
            status = 'No pose is currently loaded.'
        else:
            status = 'Working with Pose {}'.format(POSE_ID)

        msg = "What would you like to do?\n\n{}\n\n{}\n\nType action here: ".format(status, checklist)

        try:
            action = int(raw_input(msg))
            actions[action][1]()
        except (ValueError, IndexError):
            rospy.logerr('Not a valid action!')
        except StopProgramException:
            break

    print('All done!')


