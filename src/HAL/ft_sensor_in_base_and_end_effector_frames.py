#!/usr/bin/env python

# This ros node transforms raw wrench data that is measured from the force-torque sensor
# located in the robot palm into various frames relevant frames for the manipulation system

# Inputs:

# ROS topics:

# /netft/netft_data
# wrench measured by force-torque sensor that we have installed in the wrist of the robot
# units are in Newtons (force component) and Newton-Meters (torque component)
# the wrench given is in the frame of the sensor

# '/ee_pose_in_world_from_franka_publisher':
# Cartesian 6 DOF pose of the robot end-effector in the world frame

# ROS Static Transforms:

# /ft_sensor
# This frame is attached to the force torque sensor which is mounted to the robot.
# It is fixed relative to the 6th link of the robot arm (which is how it is computed)
# and is also fixed relative to the robot palm
# the wrench measurement from /netft/netft_data is relative to this frame

# /panda_EE
# This frame is attached to the centroid of the robot palm.
# It is fixed relative to the 6th link of the robot arm (which is how it is computed)
# and is also fixed relative to the force-torque sensor frame.
# This is one of the two frames that we care about for our manipulation system

# base
# This is the world frame, with the origin located at the base of the robot arm
# It is at "ground" level (i.e. on the level of the table the arm is attached to)
# Z corresponds to the vertical direction

# Outputs:

# /ft_sensor_in_base_frame
# wrench at the force torque sensor rotated into the base frame
# Wrench measured by the ATI, multiplied by a rotation matrix (from ATI frame to world)

# /ft_sensor_in_end_effector_frame
# wrench at the force torque sensor rotated into the end effector frame -Neel 7/5/2022
# wrench measured by the ATI, multiplied by a rotation matrix (from ATI frame to end-effector)

# /end_effector_sensor_in_end_effector_frame
# wrench at the end-effector in the end effector coordinates -Neel 7/5/2022
# wrench measured by the ATI, but the torque has been transformed using a different reference point,
# specifically, the origin of the end-effector frame (should be palm center), and using end-effector basis

# /end_effector_sensor_in_base_frame
# wrench at the end-effector in the base coordinates -Neel 7/5/2022
# wrench measured by the ATI, but the torque has been transformed using a different reference point,
# specifically, the origin of the end-effector frame (should be palm center), and using BASE FRAME basis

# /torque_cone_boundary_test
# this is a boolean signalling whether or not the measured wrench is on the boundary of 
# the torque cone that corresponds to contact with the robot palm. This is important because the
# interior of torque cone corresponds to line contact with the palm,
# whereas the boundary of the torque cone admits the possibility of point contact (the palm tilting on one of its edges)
# True -> Interior of torque cone
# False -> Boundary of torque cone OR no contact detected

# /torque_cone_boundary_flag
# this is a higher resolution message than /torque_cone_boundary_test that expounds on exactly what is going on with the torque cone boundary
# the flag is one of four integers:
# (/torque_cone_boundary_test = TRUE):
# -1 -> Interior of torque cone 
# (/torque_cone_boundary_test = FALSE):
# 0 -> No contact detected
# 1 -> torque is on the positive boundary of the torque cone
# 2 -> torque is on the negative boundary of the torque cone

# Note that when computing /torque_cone_boundary_test and /torque_cone_boundary_flag,
# The system is being treated as a planar system

import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import netft_rdt_driver.srv as srv
import numpy as np
import pdb
import rospy
import time
import tf

import Helpers.ros_helper as rh
from Helpers.time_logger import time_logger
from Helpers.ros_manager import ros_manager
from Modelling.system_params import SystemParams

sys_params = SystemParams()
LCONTACT = sys_params.object_params["L_CONTACT_MAX"]                                        # length of the end effector 
NORMAL_FORCE_THRESHOLD = sys_params.estimator_params["NORMAL_FORCE_THRESHOLD_FORCE"]        # Minimum required normal force
TORQUE_BOUNDARY_MARGIN = sys_params.object_params["TORQUE_BOUNDARY_MARGIN"]                 # in yaml
END_EFFECTOR_MASS = sys_params.object_params["END_EFFECTOR_MASS"]                           # mass of end effectors

# calls the ROS service that tares (zeroes) the force-torque sensor
def zero_ft_sensor():
    rospy.wait_for_service('/netft/zero', timeout=0.5)
    zero_ft = rospy.ServiceProxy('/netft/zero', srv.Zero)
    zero_ft()

if __name__ == '__main__':
    
    # initialize node
    node_name = 'ft_sensor_in_base_frame'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.hal_params["RATE"])     


    # Make listener and get ft sensor in hand frame
    listener = tf.TransformListener()

    # ft sensor in the hand frame
    (ft_sensor_in_end_effector_trans, ft_sensor_in_end_effector_rot) = \
        rh.lookupTransform('/ft_sensor', '/panda_EE', listener)

    # Initialize all publishers and subscribers. 
    # See /Helpers/ros_manager for the actual code (mostly book-keeping)
    rm = ros_manager()
    rm.subscribe_to_list(['/netft/netft_data',
                          '/ee_pose_in_world_from_franka_publisher'])
    
    rm.spawn_publisher_list(['/ft_sensor_in_base_frame',
                             '/ft_sensor_in_end_effector_frame',
                             '/end_effector_sensor_in_end_effector_frame',
                             '/end_effector_sensor_in_base_frame',
                             '/torque_cone_boundary_test',
                             '/torque_cone_boundary_flag'])


    # wait until messages have been received from all essential ROS topics before proceeding
    rm.wait_for_necessary_data()

    # panda hand pose in base frame WHEN TARING
    (panda_hand_in_base_trans0, panda_hand_in_base_rot0) = \
        rh.lookupTransform('/panda_EE', 'base', listener)
    rm.panda_hand_in_base_pose0 = rh.list2pose_stamped(panda_hand_in_base_trans0 
        + panda_hand_in_base_rot0, frame_id="base")
    rm.base_z_in_panda_hand0 = rh.matrix_from_pose(
        rm.panda_hand_in_base_pose0)[2, :3]

    # ft sensor pose in end effector frame
    (ft_sensor_in_end_effector_trans, ft_sensor_end_effector_in_base_rot) = \
        rh.lookupTransform('/ft_sensor', '/panda_EE', listener)
    ft_sensor_in_end_effector_pose = rh.list2pose_stamped(ft_sensor_in_end_effector_trans 
        + ft_sensor_end_effector_in_base_rot, frame_id="/panda_EE")
    T_ft_sensor_in_panda_hand = rh.matrix_from_pose(
        ft_sensor_in_end_effector_pose)

    # base frame in base frame
    base_in_base_pose = rh.unit_pose()
    
    # zero sensor
    zero_ft_sensor()
    print("Zeroing sensor")

    # object for computing loop frequnecy
    tl = time_logger(node_name)

    # Run node at rate
    while not rospy.is_shutdown():
        tl.reset()
        rm.unpack_all()


        # ft sensor pose in base frame
        T_panda_hand_in_base = rh.matrix_from_pose(rm.panda_hand_in_base_pose)
        T_ft_sensor_in_base = np.matmul(T_panda_hand_in_base, T_ft_sensor_in_panda_hand)
        ft_sensor_in_base_pose2 = rh.pose_from_matrix(T_ft_sensor_in_base)
        

        # ft wrench reading in end-effector frame
        ft_wrench_in_end_effector_reading = rh.rotate_wrench(rm.ft_wrench_in_ft_sensor, 
            ft_sensor_in_end_effector_pose)
        ft_wrench_in_end_effector_list = rh.wrench_stamped2list(
            ft_wrench_in_end_effector_reading)
        correction = (-rm.base_z_in_panda_hand0 + rm.base_z_in_panda_hand) * 9.81 * END_EFFECTOR_MASS
        ft_wrench_in_end_effector_list[0] += correction[0]
        ft_wrench_in_end_effector_list[1] += correction[1]
        ft_wrench_in_end_effector_list[2] += correction[2]
        ft_wrench_in_end_effector = rh.list2wrench_stamped(ft_wrench_in_end_effector_list)
        ft_wrench_in_end_effector.header.frame_id = "/panda_EE"

        # ft wrench in base frame
        ft_wrench_in_base = rh.rotate_wrench(ft_wrench_in_end_effector, 
            rm.panda_hand_in_base_pose)
        ft_wrench_in_base.header.frame_id = 'base'

        # end effector wrench in end effector frame
        end_effector_wrench_in_end_effector = rh.wrench_reference_point_change(
            ft_wrench_in_end_effector, ft_sensor_in_end_effector_trans)
        end_effector_wrench_in_end_effector.header.frame_id = "/panda_EE"

        # end effector wrench in base frame
        end_effector_wrench_in_base = rh.rotate_wrench(end_effector_wrench_in_end_effector, 
            rm.panda_hand_in_base_pose)
        end_effector_wrench_in_base.header.frame_id = "base"


        #normal force component of the measured wrench at the end-effector (projected onto the planar system we care about)
        normal_force = end_effector_wrench_in_end_effector.wrench.force.x

        #friction force component of the measured wrench at the end-effector (projected onto the planar system we care about)
        friction_force = end_effector_wrench_in_end_effector.wrench.force.y

        #torque of the measured wrench at the end-effector (projected onto the planar system we care about)
        torque =  end_effector_wrench_in_end_effector.wrench.torque.z

        #evaluate torque_boundary_boolean which is output as the rostopic /torque_cone_boundary_test
        #torque_boundary_boolean is TRUE when the measured torque is on the interior of the torque cone (for the planar system)
        #torque_boundary_boolean is FALSE when either the measured torque is on or past one of the boundaries of the torque cone
        #or contact is not detect (i.e. the measured normal_force does not exceed the contact threshold)
        torque_boundary_boolean =  normal_force<-NORMAL_FORCE_THRESHOLD and \
            (np.abs(torque)/np.abs(normal_force))<=(0.5*TORQUE_BOUNDARY_MARGIN*LCONTACT)
   
        
        #initialize the boundary flag
        torque_boundary_flag = None

        if torque_boundary_boolean:
            #if the torque is on the interior of the cone, set the flag to -1
            torque_boundary_flag=-1
        else:
            #otherwise...

            if normal_force>=-NORMAL_FORCE_THRESHOLD:
                #if no contact is detected, set flag to 0 
                torque_boundary_flag=0
            else:
                if torque/np.abs(normal_force)>(
                    0.5*TORQUE_BOUNDARY_MARGIN*LCONTACT):

                    #if torque value is on or past the positive boundary, set flag to 1
                    torque_boundary_flag=1
                if torque/np.abs(normal_force)<-(
                    0.5*TORQUE_BOUNDARY_MARGIN*LCONTACT):

                    #if torque value is on or past the negative boundary, set flag to 2
                    torque_boundary_flag=2


        # publish and sleep
        rm.pub_ft_sensor_in_base_frame(ft_wrench_in_base)
        rm.pub_ft_sensor_in_end_effector_frame(ft_wrench_in_end_effector)
        rm.pub_end_effector_sensor_in_end_effector_frame(end_effector_wrench_in_end_effector)
        rm.pub_end_effector_sensor_in_base_frame(end_effector_wrench_in_base)
        rm.pub_torque_cone_boundary_test(torque_boundary_boolean)
        rm.pub_torque_cone_boundary_flag(torque_boundary_flag)   

        # log timing info
        tl.log_time()

        rate.sleep()