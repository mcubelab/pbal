#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import time

from Modelling.system_params import SystemParams
import friction_reasoning

from Helpers.ros_manager import ros_manager

import rospy

if __name__ == '__main__':

    node_name = 'sliding_estimation_wrench_cone'
    
    sys_params = SystemParams()

    rm = ros_manager()
    rospy.init_node(node_name)
    rate = rospy.Rate(sys_params.estimator_params["RATE"])
    rm.subscribe_to_list([  '/end_effector_sensor_in_end_effector_frame',
                            '/end_effector_sensor_in_world_manipulation_frame',
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
    rm.init_time_logger(node_name)
   
    print("Starting sliding state estimation from wrench cone")
    while not rospy.is_shutdown():
        t0 = time.time()
        rm.tl_reset()
        rm.unpack_all()

        update_and_publish = False

        if rm.end_effector_wrench_has_new:
            update_and_publish = True
            friction_reasoning.compute_sliding_state_contact(
                sliding_state_dict,rm.friction_parameter_dict,last_slide_time_dict,
                t0,rm.measured_contact_wrench,contact_friction_cone_boundary_margin,reset_time_length)
  
        if rm.end_effector_wrench_world_manipulation_frame_has_new:
            update_and_publish = True
            friction_reasoning.compute_sliding_state_world_manipulation(
                sliding_state_dict,rm.friction_parameter_dict,last_slide_time_dict,
                t0,rm.measured_world_manipulation_wrench,external_friction_cone_boundary_margin,reset_time_length)

        if update_and_publish:
            rm.pub_sliding_state(sliding_state_dict)   

        # log timing info       
        rm.log_time()
        rate.sleep()