#!/usr/bin/env python
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

    rm = ros_manager()
    rm.subscribe_to_list(['/netft/netft_data',
                          '/ee_pose_in_world_from_franka_publisher'])
    
    rm.spawn_publisher_list(['/ft_sensor_in_base_frame',
                             '/ft_sensor_in_end_effector_frame',
                             '/end_effector_sensor_in_end_effector_frame',
                             '/end_effector_sensor_in_base_frame',
                             '/torque_cone_boundary_test',
                             '/torque_cone_boundary_flag'])



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

        #check to see if we are near the torque boundary of the wrench cone
        #(conditioned on the normal force exceeding a certain amount)
        #return false if we are close to boundary, or there is no normal force
        #return true if we are sufficiently in the interior of the wrench cone
        normal_force = end_effector_wrench_in_end_effector.wrench.force.x
        friction_force = end_effector_wrench_in_end_effector.wrench.force.y
        torque =  end_effector_wrench_in_end_effector.wrench.torque.z


        torque_boundary_boolean =  normal_force<-NORMAL_FORCE_THRESHOLD and \
            (np.abs(torque)/np.abs(normal_force))<=(0.5*TORQUE_BOUNDARY_MARGIN*LCONTACT)
   
        

        torque_boundary_flag = None

        if torque_boundary_boolean:
            torque_boundary_flag=-1
        else:
            if normal_force>=-NORMAL_FORCE_THRESHOLD:
                torque_boundary_flag=0
            else:
                if torque/np.abs(normal_force)>(
                    0.5*TORQUE_BOUNDARY_MARGIN*LCONTACT):
                    torque_boundary_flag=1
                if torque/np.abs(normal_force)<-(
                    0.5*TORQUE_BOUNDARY_MARGIN*LCONTACT):
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