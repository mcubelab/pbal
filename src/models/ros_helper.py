import numpy as np
import math
import os

import tf
import rospy
from geometry_msgs.msg import PoseStamped, Pose2D, WrenchStamped, PointStamped


def list2pose_stamped(pose, frame_id="world"):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]
    msg.pose.orientation.x = pose[3]
    msg.pose.orientation.y = pose[4]
    msg.pose.orientation.z = pose[5]
    msg.pose.orientation.w = pose[6]
    return msg

def lookupTransform(homeFrame, targetFrame, listener):
    ntfretry = 100
    retryTime = .05
    for i in range(ntfretry):
        try:
            t = rospy.Time(0)
            (trans, rot) = listener.lookupTransform(targetFrame, homeFrame, t)
            return (trans, rot)
        except:  #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('[lookupTransform] failed to transform')
            print('[lookupTransform] targetFrame %s homeFrame %s, retry %d' %
                  (targetFrame, homeFrame, i))
            rospy.sleep(retryTime)
    return None, None

def rotate_wrench(wrench_source, pose_transform):
    force_source, torque_source = wrenchstamped_2FT(wrench_source)
    T_transform_source = matrix_from_pose(pose_transform)[:3, :3]
    force_transformed = np.matmul(T_transform_source, np.array(force_source))
    torque_transformed = np.matmul(T_transform_source, np.array(torque_source))

    return list2wrench_stamped(force_transformed.tolist() + 
        torque_transformed.tolist())

def wrench_reference_point_change(wrench_source, vector_from_new_ref):
    # wrench source and vector_from_new_ref need in the same frame
    force_source, torque_source = wrenchstamped_2FT(wrench_source)
    torque_transformed = np.array(torque_source) + np.cross(np.array(vector_from_new_ref), 
        np.array(force_source))
    return list2wrench_stamped(force_source + torque_transformed.tolist())


def wrenchstamped_2FT(wrench):

    force = [
        wrench.wrench.force.x,
        wrench.wrench.force.y,
        wrench.wrench.force.z]

    torque = [
        wrench.wrench.torque.x,
        wrench.wrench.torque.y,
        wrench.wrench.torque.z]

    return force, torque

def list2wrench_stamped(wrench, frame_id="base"):

    msg = WrenchStamped()
    msg.header.frame_id = frame_id
    msg.wrench.force.x = wrench[0]
    msg.wrench.force.y = wrench[1]
    msg.wrench.force.z = wrench[2]
    msg.wrench.torque.x = wrench[3]
    msg.wrench.torque.y = wrench[4]
    msg.wrench.torque.z = wrench[5]
    return msg

def wrench_stamped2list(msg):
    force, torque = wrenchstamped_2FT(msg)
    return force + torque

def transform_pose(pose_source, pose_transform):
    T_pose_source = matrix_from_pose(pose_source)
    T_transform_source = matrix_from_pose(pose_transform)
    T_pose_final_source = np.matmul(T_transform_source, T_pose_source)
    pose_final_source = pose_from_matrix(T_pose_final_source, frame_id=pose_source.header.frame_id)
    return pose_final_source

def transform_pose_intermediate_frame(pose_source_frameA, pose_frameB, pose_transform_frameB):
    pose_source_frameB = convert_reference_frame(pose_source_frameA,
                                                  pose_frameB,
                                                  unit_pose(),
                                                  frame_id="frameB")
    pose_source_transformed_frameB = transform_pose(pose_source_frameB,
                                                    pose_transform_frameB)
    pose_source_transformed_frameA = convert_reference_frame(pose_source_transformed_frameB,
                                                  unit_pose(),
                                                  pose_frameB,
                                                  frame_id="frameB")
    return pose_source_transformed_frameA

def transform_body(pose_source_world, pose_transform_target_body):
    #convert source to target frame
    pose_source_body = convert_reference_frame(pose_source_world,
                                                 pose_source_world,
                                                 unit_pose(),
                                                 frame_id="body_frame")
    #perform transformation in body frame
    pose_source_rotated_body = transform_pose(pose_source_body,
                                              pose_transform_target_body)
    # rotate back
    pose_source_rotated_world = convert_reference_frame(pose_source_rotated_body,
                                                         unit_pose(),
                                                         pose_source_world,
                                                         frame_id="yumi_body")
    return pose_source_rotated_world

def pose_stamped2list(msg):
    return [float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
            ]

def list2pose_twod(pose):
    msg = Pose2D()
    msg.x = pose[0]
    msg.y = pose[1]
    msg.theta = pose[2]
    return msg

def list2point_stamped(xyz):
    msg = PointStamped()
    msg.point.x = xyz[0]
    msg.point.y = xyz[1]
    msg.point.z = xyz[2]
    return msg

def point_stamped2list(msg):
    return [msg.point.x, 
        msg.point.y,
        msg.point.z]

def quat2list(quat):
    return[quat.x, quat.y, quat.z, quat.w]

def unit_pose():
    return list2pose_stamped([0,0,0,0,0,0,1])


def convert_reference_frame(pose_source, pose_frame_target, pose_frame_source, frame_id = "work_obj"):
    T_pose_source = matrix_from_pose(pose_source)
    pose_transform_target2source = get_transform(pose_frame_source, pose_frame_target)
    T_pose_transform_target2source = matrix_from_pose(pose_transform_target2source)
    T_pose_target = np.matmul(T_pose_transform_target2source, T_pose_source)
    pose_target = pose_from_matrix(T_pose_target, frame_id=frame_id)
    return pose_target

def pose_from_matrix(matrix, frame_id="world"):
    trans = tf.transformations.translation_from_matrix(matrix)
    quat = tf.transformations.quaternion_from_matrix(matrix)
    pose = list(trans) + list(quat)
    pose = list2pose_stamped(pose, frame_id=frame_id)
    return pose

def get_pose_from_tf_frame(listener, target_frame, source_name):
    trans, quat = listener.lookupTransform(source_name, target_frame, rospy.Time(0))
    pose_target_source = list2pose_stamped(trans+quat,
                                           frame_id=source_name)
    return pose_target_source

def get_transform(pose_frame_target, pose_frame_source):
    """
    Find transform that transforms pose source to pose target
    :param pose_frame_target:
    :param pose_frame_source:
    :return:
    """
    #both poses must be expressed in same reference frame
    T_target_world = matrix_from_pose(pose_frame_target)
    T_source_world = matrix_from_pose(pose_frame_source)
    T_relative_world = np.matmul(T_target_world, np.linalg.inv(T_source_world))
    pose_relative_world = pose_from_matrix(T_relative_world, frame_id=pose_frame_source.header.frame_id)
    return pose_relative_world

def matrix_from_pose(pose):
    pose_list = pose_stamped2list(pose)
    trans = pose_list[0:3]
    quat = pose_list[3:7]
    T = tf.transformations.quaternion_matrix(quat)
    T[0:3,3] = trans
    return T

def initialize_rosbag(topics, exp_name='test'):
    import datetime
    import subprocess
    #Saving rosbag options
    name_of_bag  = str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")) + '-' + exp_name
    dir_save_bagfile = os.environ['CODE_BASE'] + '/data/rosbag_data/'
    rosbag_proc = subprocess.Popen('rosbag record -q -O %s %s' % (name_of_bag, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)

def terminate_rosbag():
    terminate_ros_node('/record')

def terminate_ros_node(s):
    import subprocess
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for term in list_output.split("\n"):
        if (term.startswith(s)):
            os.system("rosnode kill " + term)
            print "rosnode kill " + term


def compute_tip2tcp_offset(listener, pose_tip_push_start, tip_name='/apriltag_tip'):
    # pose_tip_start = list2pose_stamped(pose_list, frame_id='push_start')
    #1. get transforms from tf
    pose_push_start_map = get_pose_from_tf_frame(listener=listener,
                                            target_frame='/push_start',
                                            source_name='/map')

    pose_link_6_tip = get_pose_from_tf_frame(listener=listener,
                                            target_frame='/link_6',
                                            source_name=tip_name)

    #2. Find tip pose in map frame (from push start frame)
    pose_tip_map = convert_reference_frame(pose_source= pose_tip_push_start,
                                         pose_frame_target=unit_pose(),
                                         pose_frame_source=pose_push_start_map,
                                         frame_id='map')

    #3. Find tcp frame (in map) from tip frame (in map)
    pose_tcp_map = convert_reference_frame(pose_source=pose_link_6_tip,
                                                         pose_frame_target=unit_pose(),
                                                         pose_frame_source=pose_tip_map,
                                                         frame_id='map')
    return pose_tcp_map


def transform_tip2tcp(listener,  pose_tip_push_start, tip_name='/apriltag_tip'):
    pose_tcp_map = compute_tip2tcp_offset(listener,  pose_tip_push_start, tip_name)
    pose_map_list = convert_ros2abb(pose_tcp_map)
    return pose_map_list

def quatlist_to_theta(quat_list):
    '''converts fraka quaternion to sagittal plane angle'''
    
    pose_list = [0., 0., 0.] + quat_list
    pose_stamped = list2pose_stamped(pose_list)
    pose_homog = matrix_from_pose(pose_stamped)

    hand_normal_x = pose_homog[0,0]
    hand_normal_z = pose_homog[2,0]

    return -np.arctan2(hand_normal_x, -hand_normal_z)   


def theta_to_quatlist(theta):
    '''converts sagittal plane angle to fraka quaternion'''

    # sin/cos of line contact orientation
    sint, cost = np.sin(theta), np.cos(theta)
    
    # line contact orientation in world frame
    endpoint_orien_mat = np.array([[-sint, -cost, 0], 
        [0, 0, 1], [-cost, sint, 0]])

    # zero position
    endpoint_position = np.array([0., 0., 0.])

    # homogenous transform w.r.t world frame
    endpoint_homog = np.vstack([np.vstack([endpoint_orien_mat.T,  
        endpoint_position]).T, np.array([0., 0., 0., 1.])])

    # pose stamped in world frame
    endpoint_pose = pose_from_matrix(endpoint_homog)

    # pose list in world frame
    endpoint_pose_list = pose_stamped2list(endpoint_pose)

    # return quaternion portion
    return endpoint_pose_list[3:]

