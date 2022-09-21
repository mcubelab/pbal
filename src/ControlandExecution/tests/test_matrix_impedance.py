#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(os.path.dirname(currentdir)))

import matplotlib.pyplot as plt
import rospy
import numpy as np
import time
import Helpers.impedance_mode_helper as IMH
from Helpers.ros_manager import ros_manager

if __name__ == '__main__':


    #this starts the ros node
    rospy.init_node("test_impedance_control_with_matrix")

    #wait function (for one second) so that everything is connected before proceeding
    rospy.sleep(1.0)

    rm = ros_manager()

    rm.subscribe_to_list(['/ee_pose_in_world_from_franka_publisher'])

    my_impedance_mode_helper = IMH.impedance_mode_helper()


    # default [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
    #torque upper
    torque_upper = [40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0]
    # [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
    force_upper = [100.0, 100.0, 100.0, 25.0, 25.0, 25.0]

    my_impedance_mode_helper.initialize_impedance_mode(torque_upper=torque_upper,
                                                       force_upper=force_upper)

    # [4000, 4000, 4000, 400, 120, 400] #[1200, 600, 200, 100, 0, 100]

    #the stiffness values used for the cartesian impedance Fx,Fy,Fz,Tx,Ty,Tz
    #the stiffness matrix is diagonal, we are only defining the diagonal entries
    # IMPEDANCE_STIFFNESS_LIST = [3000, 3000, 3000, 100, 100, 100]
    # IMPEDANCE_STIFFNESS_LIST = [10, 10, 1000, 10, 10, 10]

    #the damping values for the cartesian impedance, also a diagonal matrix
    # IMPEDANCE_DAMPING_LIST = [0.5 * np.sqrt(k) for k in IMPEDANCE_STIFFNESS_LIST] 

    # my_impedance_mode_helper.set_cart_impedance_stiffness(IMPEDANCE_STIFFNESS_LIST,IMPEDANCE_DAMPING_LIST)

    IMPEDANCE_STIFFNESS_TRANS = [[  10.0,    0.0,    0.0],
                                 [  0.0,  1000.0,    0.0],
                                 [  0.0,     0.0,   100.0]]

    IMPEDANCE_STIFFNESS_ROT = [[ 10.0,   0.0,   0.0],
                               [ 0.0,    10.0,   0.0],
                               [ 0.0,    0.0,   10.0]]


    IMPEDANCE_DAMPING_TRANS = [[  3.0,  0.0,  0.0],
                               [  0.0,  30.0,  0.0],
                               [  0.0,  0.0,  3.0]]

    IMPEDANCE_DAMPING_ROT = [[ 1.0,  0.0, 0.0],
                             [ 0.0,  1.0, 0.0],
                             [ 0.0,  0.0, 1.0]]

    IMPEDANCE_STIFFNESS_TRANS = np.array(IMPEDANCE_STIFFNESS_TRANS)
    IMPEDANCE_STIFFNESS_ROT = np.array(IMPEDANCE_STIFFNESS_ROT)

    IMPEDANCE_DAMPING_TRANS = np.array(IMPEDANCE_DAMPING_TRANS)
    IMPEDANCE_DAMPING_ROT = np.array(IMPEDANCE_DAMPING_ROT)

    theta = np.pi/4

    my_rotation_matrix = [[np.cos(theta),-np.sin(theta), 0.0],
                          [np.sin(theta), np.cos(theta), 0.0],
                          [          0.0,           0.0, 1.0]]

    my_rotation_matrix = np.array(my_rotation_matrix)


    IMPEDANCE_STIFFNESS_TRANS = np.dot(np.dot(my_rotation_matrix,IMPEDANCE_STIFFNESS_TRANS),my_rotation_matrix.T)
    IMPEDANCE_STIFFNESS_ROT   = np.dot(np.dot(my_rotation_matrix,IMPEDANCE_STIFFNESS_ROT),my_rotation_matrix.T)
    IMPEDANCE_DAMPING_TRANS   = np.dot(np.dot(my_rotation_matrix,IMPEDANCE_DAMPING_TRANS),my_rotation_matrix.T)
    IMPEDANCE_DAMPING_ROT     = np.dot(np.dot(my_rotation_matrix,IMPEDANCE_DAMPING_ROT),my_rotation_matrix.T)

    my_impedance_mode_helper.set_cart_imepedance_stiffness_matrix(stiffness_trans=IMPEDANCE_STIFFNESS_TRANS, 
                                                                  stiffness_rot=IMPEDANCE_STIFFNESS_ROT, 
                                                                  damping_trans=IMPEDANCE_DAMPING_TRANS,   
                                                                  damping_rot=IMPEDANCE_DAMPING_ROT)

    # wait until messages have been received from all essential ROS topics before proceeding
    rm.wait_for_necessary_data()
    rm.ee_pose_unpack()
    rm.panda_hand_in_base_pose

    time.sleep(.5)

    my_impedance_mode_helper.set_cart_impedance_pose(rm.panda_hand_in_base_pose_list)

    time.sleep(.5)

        




   

 


