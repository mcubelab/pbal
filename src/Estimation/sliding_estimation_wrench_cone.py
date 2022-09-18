#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import pdb
import rospy
import time

import Helpers.pbal_msg_helper as pmh
import Helpers.ros_helper as rh
from Helpers.time_logger import time_logger
from Helpers.ros_manager import ros_manager

from Modelling.system_params import SystemParams
import friction_reasoning

if __name__ == '__main__':

    node_name = 'sliding_estimation_wrench_cone'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.estimator_params["RATE"])

    rospy.init_node("sliding_estimation_wrench_cone")
    rospy.sleep(1.0)

    rm = ros_manager()
    rm.subscribe_to_list([  '/end_effector_sensor_in_end_effector_frame',
                            '/end_effector_sensor_in_base_frame',
                            '/friction_parameters'])

    rm.spawn_publisher('/sliding_state')
    
    rm.wait_for_necessary_data()

    #parameters describing how close to the friction 
    #cone boundary you need to be
    #to be considered sliding
    contact_friction_cone_boundary_margin = 3
    external_friction_cone_boundary_margin = 3
    reset_time_length = .25

    dummy, last_slide_time_dict, sliding_state_dict = friction_reasoning.initialize_friction_dictionaries()

    # object for computing loop frequency
    tl = time_logger(node_name)
   
    print("Starting sliding state estimation from wrench cone")
    while not rospy.is_shutdown():
        t0 = time.time()
        tl.reset()
        rm.unpack_all()

        update_and_publish = False

        if rm.end_effector_wrench_has_new:
            update_and_publish = True
            friction_reasoning.compute_sliding_state_contact(
                sliding_state_dict,rm.friction_parameter_dict,last_slide_time_dict,
                t0,rm.measured_contact_wrench,contact_friction_cone_boundary_margin,reset_time_length)
  
        if rm.end_effector_wrench_base_frame_has_new:
            update_and_publish = True
            friction_reasoning.compute_sliding_state_base(
                sliding_state_dict,rm.friction_parameter_dict,last_slide_time_dict,
                t0,rm.measured_base_wrench,external_friction_cone_boundary_margin,reset_time_length)

        if update_and_publish:
            rm.pub_sliding_state(sliding_state_dict)   

        # log timing info       
        tl.log_time()

        rate.sleep()