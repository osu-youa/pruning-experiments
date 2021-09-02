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
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_from_matrix, euler_matrix, quaternion_about_axis, quaternion_multiply, quaternion_matrix
from arm_utils.srv import HandlePosePlan, HandleJointPlan
from sensor_msgs.msg import JointState, PointCloud2
from std_srvs.srv import Empty, Trigger
import cPickle as pickle
from pruning_experiments.srv import ServoWaypoints, RecordData
from functools import partial
import subprocess, shlex
from contextlib import contextmanager
from ur_dashboard_msgs.srv import Load

data_folder = os.path.join(os.path.expanduser('~'), 'data', 'icra2022')
fwd_velocity = 0.03
POSE_ID = None
POSE_INFO = {}

APPROACH_OFFSET = 0.15
INTERMEDIATE_OFFSET = np.array([0.0, 0.01, 0.05])
STANDARD_CAMERA_POS = np.array([0.0719, -0.0050 + 0.031 - 0.0248/2, 0.07416])
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

@contextmanager
def record_data(open_loop, variant):
    file_name = get_file_name(open_loop, variant)
    if file_name is None:
        print('[!] Warning, your data will not be recorded for this run!')
    else:
        record_data_srv(file_name)
    try:
        yield
    finally:
        if file_name:
            stop_record_data_srv()
            record_success(file_name)

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

def quat_to_array(quat):
    return np.array([quat.x, quat.y, quat.z, quat.w])

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

def save_pose_from_camera():
    tf_str = get_camera_tf()
    with subprocess_manager('rosrun tf static_transform_publisher {} tool0 camera_link 10'.format(tf_str)):
        pt = get_camera_point_if_connected()
        if pt.header.frame_id != tool_frame:
            tf = retrieve_tf(pt.header.frame_id, tool_frame)
            pt = do_transform_point(pt, tf)
        tool_to_base_tf = retrieve_tf(pt.header.frame_id, base_frame)

    tool_to_base_tf_wrong = deepcopy(tool_to_base_tf)
    pos = pt_to_array(tool_to_base_tf_wrong.transform.translation)
    quat = quat_to_array(tool_to_base_tf_wrong.transform.rotation)
    pos_noise = np.random.uniform(-1, 1, 3)
    pos += pos_noise / np.linalg.norm(pos_noise) * 0.01
    random_rot_axis = np.random.uniform(-1, 1, 3)
    random_rot_axis /= np.linalg.norm(random_rot_axis)
    noise_rot_quat = quaternion_about_axis(np.radians(5), random_rot_axis)
    quat_noise = quaternion_multiply(quat, noise_rot_quat)
    tool_to_base_tf_wrong.transform.translation = Vector3(*pos)
    tool_to_base_tf_wrong.transform.rotation = Quaternion(*quat_noise)

    # Compute the desired approach point orientation
    if rospy.get_param('sim', True):
        normal_vec = np.array([0, -1, 0])
    else:
        normal_vec = np.array([-np.sqrt(2), -np.sqrt(2)], 0)

    z_axis = -normal_vec
    y_axis = np.array([0, 0, 1])
    x_axis = np.cross(y_axis, z_axis)
    rot_array = np.stack([x_axis, y_axis, z_axis], axis=1)

    tool_euler = euler_from_matrix(rot_array)
    tool_quat = quaternion_from_euler(*tool_euler)

    # Use the computed TFs to retrieve the world frame locations
    info_dict = {}
    for tf, key in [(tool_to_base_tf, 'base'), (tool_to_base_tf_wrong, 'noisy')]:
        pt_world = pt_to_array(do_transform_point(pt, tf))
        approach_pt = pt_world + normal_vec * APPROACH_OFFSET
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = base_frame
        approach_pose.pose.position = Point(*approach_pt)
        approach_pose.pose.orientation = Quaternion(*tool_quat)
        info_dict[key] = approach_pose

    save_pose(info_dict)

def freedrive():
    program_load_srv('freedrive.urp')
    program_play_srv()
    try:
        raw_input('Freedrive mode activated! Press Enter when complete.')
    finally:
        program_stop_srv()


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
        print('Please click on a point in RViz to go to! (Waiting 45 seconds)')
        final_target = rospy.wait_for_message('/clicked_point', PointStamped, timeout=45.0)

    else:
        # If no camera is connected, just run a test motion
        print('No camera connected! Running test motion...')
        final_target = PointStamped()
        final_target.header.frame_id = tool_frame
        final_target.point = Point(0.0, -0.05, 0.15)

    return final_target

def get_camera_tf(as_str=True):
    pos = list(STANDARD_CAMERA_POS)

    rot_mat = euler_matrix(*STANDARD_CAMERA_ROT)[:3,:3]
    adjustment_mat = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])
    final_euler = euler_from_matrix(rot_mat.dot(adjustment_mat))
    quat = list(quaternion_from_euler(*final_euler))
    if as_str:
        return ' '.join(['{}'] * 7).format(*pos + quat)
    else:
        return pos, quat

def run_open_loop(use_miscalibrated = False, run_from_current_pose=False):

    success = True
    if not POSE_INFO or run_from_current_pose:
        pass
    elif use_miscalibrated:
        success = plan_pose_srv(POSE_INFO['noisy'], True).success
    else:
        success = plan_pose_srv(POSE_INFO['base'], True).success
    if not success:
        rospy.logerr('Pose failed to plan!')
        return

    tare_force_sensor()

    final_target_array = np.array([0.0, 0.0, APPROACH_OFFSET])
    intermediate_array = final_target_array - INTERMEDIATE_OFFSET

    tf = retrieve_tf(tool_frame, base_frame)
    waypoints = []
    for array in [intermediate_array, final_target_array]:
        pt = PointStamped()
        pt.header.frame_id = tool_frame
        pt.point = Point(*array)
        waypoints.append(do_transform_point(pt, tf))

    with record_data(open_loop=True, variant=use_miscalibrated):
        code = open_loop_srv(waypoints, fwd_velocity, False).code
        rospy.loginfo("Return code: {}".format(code))
    raw_input('Servoing done! Press Enter to rewind...')
    servo_rewind()

def run_closed_loop(use_nn=False):

    tare_force_sensor()
    if POSE_INFO:
        success = plan_pose_srv(POSE_INFO['base'], True).success
        if not success:
            rospy.logerr('Pose failed to plan!')
            return

    with record_data(open_loop=False, variant=not use_nn):

        if use_nn:
            code = subprocess.check_output(shlex.split('rosrun pruning_experiments velocity_command_client.py')).strip()
            if code == str(-1):
                rospy.logerr("Neural network failed to start!")
            else:
                code = int(code)
        else:
            # Find the intermediate point and target a straight line
            final_target_array = np.array([0.0, 0.0, APPROACH_OFFSET])
            intermediate_array = final_target_array - INTERMEDIATE_OFFSET
            pt = PointStamped()
            pt.header.frame_id = tool_frame
            pt.point = Point(*intermediate_array)
            tf = retrieve_tf(tool_frame, base_frame)
            tfed_pt = do_transform_point(pt, tf)
            intermediate_world = pt_to_array(tfed_pt)
            cutter_world = pt_to_array(rospy.wait_for_message('tool_pose', PoseStamped, timeout=1.0).pose.position)
            movement_vec = (intermediate_world - cutter_world)
            movement_vec /= np.linalg.norm(movement_vec)
            actual_target = intermediate_world + 0.04 * movement_vec
            tfed_pt.point = Point(*actual_target)
            waypoints = [tfed_pt]

            code = open_loop_srv(waypoints, fwd_velocity, True).code

        rospy.loginfo("Return code: {}".format(code))

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

    program_load_srv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
    program_play_srv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
    program_stop_srv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)

    rospy.sleep(1.0)

    actions = [
        ('Quit', stop_program),
        ('Load an existing pose', load_pose),
        ('Save current pose', save_pose),
        ('Save pose from camera', save_pose_from_camera),
        ('Freedrive to a new pose', freedrive),
        ('Level the existing pose', level_pose),
        ('Run open loop controller', run_open_loop),
        ('Run open loop controller miscalibrated', partial(run_open_loop, use_miscalibrated=True)),
        ('Run simple closed loop controller', run_closed_loop),
        ('Run NN closed loop controller', partial(run_closed_loop, use_nn=True)),
        ('[DEBUG] Run open loop from current pose', partial(run_open_loop, run_from_current_pose=True)),
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


