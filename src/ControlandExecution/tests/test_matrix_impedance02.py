#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(os.path.dirname(currentdir)))

import rospy
import numpy as np
import time
import tf
import Helpers.impedance_mode_helper as IMH
from Helpers.ros_manager import ros_manager
import Helpers.ros_helper as rh
from Modelling.system_params import SystemParams

if __name__ == '__main__':


    #this starts the ros node
    rospy.init_node("test_impedance_control_with_matrix")

    #wait function (for one second) so that everything is connected before proceeding
    rospy.sleep(1.0)

    rm = ros_manager()
    sys_params = SystemParams()
    controller_params = sys_params.controller_params
    

    TIPI        = controller_params['TRANSLATIONAL_IN_PLANE_IMPEDANCE']
    TOOPI       = controller_params['TRANSLATIONAL_OUT_OF_PLANE_IMPEDANCE']
    RIPI        = controller_params['ROTATIONAL_IN_PLANE_IMPEDANCE']
    ROOPI       = controller_params['ROTATIONAL_OUT_OF_PLANE_IMPEDANCE']

    listener = tf.TransformListener()
    
    rot_mat = rh.lookupTransform_homog('/world_manipulation_frame','base', listener)[0:3,0:3]
    
    rm.subscribe_to_list(['/ee_pose_in_base_from_franka_publisher'])

    my_impedance_mode_helper = IMH.impedance_mode_helper()


    # default [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
    #torque upper
    torque_upper = [40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0]
    # [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
    force_upper = [100.0, 100.0, 100.0, 25.0, 25.0, 25.0]

    my_impedance_mode_helper.initialize_impedance_mode(torque_upper=torque_upper,
                                                       force_upper=force_upper)


    my_impedance_mode_helper.set_matrices_pbal_mode(TIPI,TOOPI,RIPI,ROOPI,rot_mat)


    # wait until messages have been received from all essential ROS topics before proceeding
    rm.wait_for_necessary_data()
    rm.ee_pose_in_base_unpack()

    time.sleep(.5)

    my_impedance_mode_helper.set_cart_impedance_pose(rm.ee_pose_in_base_list)

    time.sleep(.5)

        




   

 


