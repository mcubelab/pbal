#!/usr/bin/env python
import os
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, gparentdir)

import collections
import copy
import matplotlib.pyplot as plt
import numpy as np
import pdb
import rospy
import time
import tf
import tf.transformations as tfm

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32MultiArray, Bool, Int32
from visualization_msgs.msg import Marker

import Helpers.ros_helper as rh
import Helpers.timing_helper as th
from Modelling.system_params import SystemParams

from franka_interface import ArmInterface 


def get_hand_orientation_in_base(contact_pose_homog):
    # current orientation
    hand_normal_x = contact_pose_homog[0,0]
    hand_normal_z = contact_pose_homog[2,0]
    return -np.arctan2(hand_normal_x, -hand_normal_z)

def calc_generalized_coordinates(x0, z0, contact_pose, 
    ee_pose_contact_frame_old, torque_boundary_flag, LCONTACT):

    contact_pose_stamped = rh.list2pose_stamped(contact_pose)
    contact_pose_homog = rh.matrix_from_pose(contact_pose_stamped)

    # 2-D unit contact normal in world frame
    e_n = contact_pose_homog[:3, 0]
    n2D = e_n[[0,2]]
    e_n2D = n2D/np.sqrt(np.sum(n2D ** 2))

    # 2-D unit contact tangent in world frame
    e_t = contact_pose_homog[:3, 1]
    t2D = e_t[[0,2]]
    e_t2D = t2D/np.sqrt(np.sum(t2D ** 2))

    # xc and zc (with pivot at 0,0)
    xc, zc = contact_pose[0] - x0, contact_pose[2] - z0

    if torque_boundary_flag == -1:

        # find normal and tangential displacment
        s = e_t2D[0]*xc + e_t2D[1]*zc 
        d = e_n2D[0]*xc + e_n2D[1]*zc 

        # find angle
        tht = get_hand_orientation_in_base(contact_pose_homog)
        tht_hand = tht

        # generalized position
        ee_pos_contact_frame = np.array([d, s, tht])

    elif torque_boundary_flag == 0:
        print("Not in contact")
        ee_pos_contact_frame = copy.deepcopy(ee_pos_contact_frame_old)
        # ee_vel_contact_frame = np.zeros_like(ee_pos_contact_frame)
        tht_hand = ee_pos_contact_frame[-1]

    elif torque_boundary_flag == 1 or torque_boundary_flag == 2:

        d_old, s_old, theta_old = ee_pos_contact_frame_old[0
            ], ee_pos_contact_frame_old[1], ee_pos_contact_frame_old[2]

        if torque_boundary_flag == 1:

            xcontact = xc + 0.5 * LCONTACT * e_t2D[0]
            zcontact = zc + 0.5 * LCONTACT * e_t2D[1]

        if torque_boundary_flag == 2:
            
            xcontact = xc - 0.5 * LCONTACT * e_t2D[0]
            zcontact = zc - 0.5 * LCONTACT * e_t2D[1]

        # angle of hand
        tht_hand = get_hand_orientation_in_base(contact_pose_homog)

        # length of vector from pivot to (xcontact, zcontact)
        lsquared = xcontact ** 2 + zcontact ** 2

        # two possible values for sliding position of contact point
        if lsquared < d_old**2:
            sp_contact = 0
            sm_contact = 0
        else:
            sp_contact = np.sqrt(lsquared - d_old ** 2)
            sm_contact = -np.sqrt(lsquared - d_old ** 2)

        # two possible values for angle of object
        thtp = np.arctan2(xcontact, zcontact) + np.arctan2(sp_contact, 
            np.abs(d_old))
        thtm = np.arctan2(xcontact, zcontact) + np.arctan2(sm_contact, 
            np.abs(d_old))

        # pick correct value of theta and s
        if torque_boundary_flag == 1:

            if thtp > tht_hand and thtm < tht_hand:
                s_contact = sp_contact
                tht, s = thtp, s_contact - 0.5 * LCONTACT
            elif thtp < tht_hand and thtm > tht_hand:
                s_contact = sm_contact
                tht, s = thtm, s_contact - 0.5 * LCONTACT
            else: 
                thtp_err = np.abs(thtp - theta_old)
                thtm_err = np.abs(thtm - theta_old)

                if thtp_err < thtm_err:
                    s_contact = sp_contact
                    tht, s = thtp, s_contact - 0.5 * LCONTACT
                else:
                    s_contact = sm_contact
                    tht, s = thtm, s_contact - 0.5 * LCONTACT

        if torque_boundary_flag == 2:

            if thtp < tht_hand and thtm > tht_hand:
                s_contact = sp_contact
                tht, s = thtp, s_contact + 0.5 * LCONTACT
            elif thtp > tht_hand and thtm < tht_hand:
                s_contact = sm_contact
                tht, s = thtm, s_contact + 0.5 * LCONTACT
            else: 
                thtp_err = np.abs(thtp - theta_old)
                thtm_err = np.abs(thtm - theta_old)

                if thtp_err < thtm_err:
                    s_contact = sp_contact
                    tht, s = thtp, s_contact + 0.5 * LCONTACT
                else:
                    s_contact = sm_contact
                    tht, s = thtm, s_contact + 0.5 * LCONTACT

        # generalized position
        ee_pos_contact_frame = np.array([d_old, s, tht])

    else:
        raise RuntimeError("incorrect torque_boundary_flag value")


    return ee_pos_contact_frame
        
def pivot_xyz_realsense_callback(data):
    global pivot_xyz
    global pivot_xyz_realsense
    global pivot_xyz_estimated
    global pivot_message_realsense_time
    global pivot_message_estimated_time

    pivot_message_realsense_time = time.time()
    pivot_xyz_realsense =  [data.transform.translation.x,
        data.transform.translation.y,
        data.transform.translation.z]

    pivot_xyz = pivot_xyz_realsense

def pivot_xyz_estimated_callback(data):
    global pivot_xyz
    global pivot_xyz_realsense
    global pivot_xyz_estimated
    global pivot_message_realsense_time
    global pivot_message_estimated_time

    pivot_message_estimated_time = time.time()
    pivot_xyz_estimated =  [data.transform.translation.x,
        data.transform.translation.y,
        data.transform.translation.z]
    

    #if there has been no message from the realsense

    if (pivot_xyz_realsense is None) or (time.time()-pivot_message_realsense_time>1.0):
        pivot_xyz = pivot_xyz_estimated

def ee_pose_callback(data):
    global panda_hand_in_base_pose
    panda_hand_in_base_pose = data

def torque_cone_boundary_test_callback(data):
    global torque_boundary_boolean
    torque_boundary_boolean = data.data

def torque_cone_boundary_flag_callback(data):
    global torque_cone_boundary_flag
    torque_cone_boundary_flag = data.data


if __name__ == '__main__':

    node_name = 'gen_coord_estimator'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.estimator_params["RATE"])

    LCONTACT = sys_params.object_params["L_CONTACT_MAX"] # in yaml

    # setting up subscribers
    pivot_xyz, pivot_xyz_realsense, pivot_xyz_estimated = None,None,None
    pivot_message_realsense_time = None
    pivot_message_estimated_time = None

    # subscribers
    pivot_xyz_realsense_sub = rospy.Subscriber("/pivot_frame_realsense", 
        TransformStamped, pivot_xyz_realsense_callback)

    pivot_xyz_estimated_sub = rospy.Subscriber("/pivot_frame_estimated", 
        TransformStamped, pivot_xyz_estimated_callback)

    torque_boundary_boolean = None
    torque_cone_boundary_test_sub = rospy.Subscriber(
        "/torque_cone_boundary_test", Bool, 
        torque_cone_boundary_test_callback)

    torque_cone_boundary_flag = None
    torque_cone_boundary_flag_sub = rospy.Subscriber(
        "/torque_cone_boundary_flag", Int32,
        torque_cone_boundary_flag_callback)

    # subscribe to ee pose data
    panda_hand_in_base_pose =  None
    panda_hand_in_base_pose_sub = rospy.Subscriber(
        '/ee_pose_in_world_from_franka_publisher', PoseStamped, 
        ee_pose_callback, queue_size=1)

    # setting up publishers
    generalized_positions_pub = rospy.Publisher('/generalized_positions', 
        Float32MultiArray, queue_size=10)
    # generalized_velocities_pub = rospy.Publisher('/generalized_velocities', 
    #     Float32MultiArray, queue_size=10)

    # define messages
    position_msg = Float32MultiArray()
    # velocity_msg = , Float32MultiArray()

    # wait for robot pose data
    print("Waiting for robot data")
    while panda_hand_in_base_pose is None:
        pass

    # # make sure subscribers are receiving commands
    # print("Waiting for pivot estimate to stabilize")
    # while pivot_xyz is None:
    #     rospy.sleep(0.1)

    # print("Waiting for torque boundary check")
    # while torque_boundary_boolean is None:
    #     pass

    # print("Waiting for torque boundary check")
    # while torque_cone_boundary_flag is None:
    #     pass


    # queue for computing frequnecy
    time_deque = collections.deque(
        maxlen=sys_params.debug_params['QUEUE_LEN'])

    ee_pos_contact_frame_old = None

    print("Starting to publish sliding velocity/position")
    while not rospy.is_shutdown():

        t0 = time.time()

        if (pivot_xyz is not None) and (torque_boundary_boolean is not None
            ) and (torque_cone_boundary_flag is not None):

            # # face_center franka pose
            # endpoint_pose_franka = arm.endpoint_pose()

            # # face_center list
            # endpoint_pose_list = franka_helper.franka_pose2list(endpoint_pose_franka)
            endpoint_pose_list = rh.pose_stamped2list(
                panda_hand_in_base_pose)

            
            # calculate generalized coordinates
            ee_pos_contact_frame = calc_generalized_coordinates(
                x0 = pivot_xyz[0], 
                z0 = pivot_xyz[1], 
                contact_pose = endpoint_pose_list,
                ee_pose_contact_frame_old = ee_pos_contact_frame_old,
                torque_boundary_flag=torque_cone_boundary_flag,
                LCONTACT=LCONTACT)

            # # face center velocity franka        
            # endpoint_velocity = arm.endpoint_velocity()

            # # face center velocity list
            # endpoint_velocity_list = franka_helper.franka_velocity2list(
                # endpoint_velocity) 

            # # update sliding velocity
            # ee_vel_contact_frame, ee_pos_contact_frame, tht_hand = update_sliding_velocity(
            #     pivot_xyz[0], pivot_xyz[2], endpoint_pose_list,
            #     endpoint_velocity_list, torque_cone_boundary_flag, 
            #     ee_pos_contact_frame_old, LCONTACT, RATE)

            # update messages
            position_msg.data = ee_pos_contact_frame
            # velocity_msg.data = ee_vel_contact_frame
            ee_pos_contact_frame_old = ee_pos_contact_frame

            # publish
            generalized_positions_pub.publish(position_msg)
            # generalized_velocities_pub.publish(velocity_msg)

        # update time deque
        time_deque.append(1000 * (time.time() - t0))   

        # log timing info
        if len(time_deque) == sys_params.debug_params['QUEUE_LEN']:
            rospy.loginfo_throttle(sys_params.debug_params["LOG_TIME"], 
                (node_name + " runtime: {mean:.3f} +/- {std:.3f} [ms]")
                .format(mean=sum(time_deque)/len(time_deque), 
                std=th.compute_std_dev(my_deque=time_deque, 
                    mean_val=sum(time_deque)/len(time_deque))))

            
        rate.sleep()

# def update_sliding_velocity(x0, z0, contact_pose, contact_vel, 
#     torque_boundary_flag, ee_pos_contact_frame_old, LCONTACT, RATE):

#     contact_pose_stamped = rh.list2pose_stamped(contact_pose)
#     contact_pose_homog = rh.matrix_from_pose(contact_pose_stamped)

#     # 2-D unit contact normal in world frame
#     e_n = contact_pose_homog[:3, 0]
#     n2D = e_n[[0,2]]
#     e_n2D = n2D/np.sqrt(np.sum(n2D ** 2))

#     # 2-D unit contact tangent in world frame
#     e_t = contact_pose_homog[:3, 1]
#     t2D = e_t[[0,2]]
#     e_t2D = t2D/np.sqrt(np.sum(t2D ** 2))

#     # xc and zc (with pivot at 0,0)
#     xc, zc = contact_pose[0] - x0, contact_pose[2] - z0

#     if torque_boundary_flag == -1 or ee_pos_contact_frame_old is None:

#         # 2-D angular velocity
#         theta_dot = contact_vel[4]

#         # compute velocity jacobian between world frame (x, z, tht) and contact frame (n, t, tht)
#         velocity_jacobian = np.vstack([np.vstack([e_n2D, e_t2D, np.array([zc, -xc])]).T,
#               np.array([0., 0., 1.])])

#         # compute end effector velocity in contact frame
#         ee_vel_contact_frame = np.linalg.solve(velocity_jacobian, 
#           np.array([contact_vel[0], contact_vel[2], theta_dot]))

#         # find normal and tangential displacment
#         s = e_t2D[0]*xc + e_t2D[1]*zc 
#         d = e_n2D[0]*xc + e_n2D[1]*zc 

#         # find angle
#         tht = get_hand_orientation_in_base(contact_pose_homog)
#         tht_hand = tht

#         # generalized position
#         ee_pos_contact_frame = np.array([d, s, tht])

#     elif torque_boundary_flag == 0:
#         print("Not in contact")
#         ee_pos_contact_frame = copy.deepcopy(ee_pos_contact_frame_old)
#         ee_vel_contact_frame = np.zeros_like(ee_pos_contact_frame)
#         tht_hand = ee_pos_contact_frame[-1]

#     elif torque_boundary_flag == 1 or torque_boundary_flag == 2:

#         d_old, s_old, theta_old = ee_pos_contact_frame_old[0
#             ], ee_pos_contact_frame_old[1], ee_pos_contact_frame_old[2]

#         if torque_boundary_flag == 1:

#             xcontact = xc + 0.5 * LCONTACT * e_t2D[0]
#             zcontact = zc + 0.5 * LCONTACT * e_t2D[1]

#         if torque_boundary_flag == 2:
            
#             xcontact = xc - 0.5 * LCONTACT * e_t2D[0]
#             zcontact = zc - 0.5 * LCONTACT * e_t2D[1]

#         # angle of hand
#         tht_hand = get_hand_orientation_in_base(contact_pose_homog)

#         # length of vector from pivot to (xcontact, zcontact)
#         lsquared = xcontact ** 2 + zcontact ** 2

#         # two possible values for sliding position of contact point
#         if lsquared < d_old**2:
#             sp_contact = 0
#             sm_contact = 0
#         else:
#             sp_contact = np.sqrt(lsquared - d_old ** 2)
#             sm_contact = -np.sqrt(lsquared - d_old ** 2)

#         # two possible values for angle of object
#         thtp = np.arctan2(xcontact, zcontact) + np.arctan2(sp_contact, 
#             np.abs(d_old))
#         thtm = np.arctan2(xcontact, zcontact) + np.arctan2(sm_contact, 
#             np.abs(d_old))

#         # pick correct value of theta and s
#         if torque_boundary_flag == 1:

#             if thtp > tht_hand and thtm < tht_hand:
#                 s_contact = sp_contact
#                 tht, s = thtp, s_contact - 0.5 * LCONTACT
#             elif thtp < tht_hand and thtm > tht_hand:
#                 s_contact = sm_contact
#                 tht, s = thtm, s_contact - 0.5 * LCONTACT
#             else: 
#                 thtp_err = np.abs(thtp - theta_old)
#                 thtm_err = np.abs(thtm - theta_old)

#                 if thtp_err < thtm_err:
#                     s_contact = sp_contact
#                     tht, s = thtp, s_contact - 0.5 * LCONTACT
#                 else:
#                     s_contact = sm_contact
#                     tht, s = thtm, s_contact - 0.5 * LCONTACT

#         if torque_boundary_flag == 2:

#             if thtp < tht_hand and thtm > tht_hand:
#                 s_contact = sp_contact
#                 tht, s = thtp, s_contact + 0.5 * LCONTACT
#             elif thtp > tht_hand and thtm < tht_hand:
#                 s_contact = sm_contact
#                 tht, s = thtm, s_contact + 0.5 * LCONTACT
#             else: 
#                 thtp_err = np.abs(thtp - theta_old)
#                 thtm_err = np.abs(thtm - theta_old)

#                 if thtp_err < thtm_err:
#                     s_contact = sp_contact
#                     tht, s = thtp, s_contact + 0.5 * LCONTACT
#                 else:
#                     s_contact = sm_contact
#                     tht, s = thtm, s_contact + 0.5 * LCONTACT

#         # generalized position
#         ee_pos_contact_frame = np.array([d_old, s, tht])

#         ee_vel_contact_frame = (ee_pos_contact_frame - 
#             ee_pos_contact_frame_old)/RATE

#     else:
#         raise RuntimeError("incorrect torque_boundary_flag value")


#     return ee_vel_contact_frame, ee_pos_contact_frame, tht_hand