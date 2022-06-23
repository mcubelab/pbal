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
import json
import numpy as np
import pdb
import rospy
import time

from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from std_msgs.msg import Float32MultiArray, Float32, Bool, String
from pbal.msg import SlidingStateStamped, FrictionParamsStamped

import Helpers.pbal_msg_helper as pmh
import Helpers.ros_helper as rh
import Helpers.timing_helper as th
from Modelling.system_params import SystemParams
import friction_reasoning

def end_effector_wrench_callback(data):
    global measured_contact_wrench_list
    measured_contact_wrench_6D = rh.wrench_stamped2list(
            data)
    measured_contact_wrench = -np.array([
            measured_contact_wrench_6D[0], 
            measured_contact_wrench_6D[1],
            measured_contact_wrench_6D[-1]])

    measured_contact_wrench_list.append(measured_contact_wrench)
    if len(measured_contact_wrench_list) > 100:
       measured_contact_wrench_list.pop(0)

def end_effector_wrench_base_frame_callback(data):
    global measured_base_wrench_list
    measured_base_wrench_6D = rh.wrench_stamped2list(
            data)
    measured_base_wrench = -np.array([
            measured_base_wrench_6D[0], 
            measured_base_wrench_6D[2],
            measured_base_wrench_6D[-1]])

    measured_base_wrench_list.append(measured_base_wrench)
    if len(measured_base_wrench_list) > 100:
       measured_base_wrench_list.pop(0)

def friction_parameter_callback(data):
    global friction_parameter_list
    friction_parameter_list.append(
        pmh.friction_stamped_to_friction_dict(data))

    if len(friction_parameter_list) > 3:
       friction_parameter_list.pop(0)



if __name__ == '__main__':

    node_name = 'sliding_estimation_wrench_cone'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.estimator_params["RATE"])

    rospy.init_node("sliding_estimation_wrench_cone")
    rospy.sleep(1.0)

    # subscribers
    measured_contact_wrench_list = []
    end_effector_wrench_sub = rospy.Subscriber(
        "/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_callback)

    measured_base_wrench_list = []
    end_effector_wrench_base_frame_sub = rospy.Subscriber(
        "/end_effector_sensor_in_base_frame", 
        WrenchStamped,  end_effector_wrench_base_frame_callback)
    
    friction_parameter_list = []
    friction_parameter_sub = rospy.Subscriber('/friction_parameters', 
        FrictionParamsStamped, friction_parameter_callback)

    # publishers
    sliding_state_pub = rospy.Publisher(
        '/sliding_state', 
        SlidingStateStamped,
        queue_size=10)


    #parameters describing how close to the friction 
    #cone boundary you need to be
    #to be considered sliding
    contact_friction_cone_boundary_margin = 3
    external_friction_cone_boundary_margin = 3
    reset_time_length = .25

    friction_parameter_dict,last_slide_time_dict,sliding_state_dict = friction_reasoning.initialize_friction_dictionaries()

    # queue for computing frequnecy
    time_deque = collections.deque(
        maxlen=sys_params.debug_params['QUEUE_LEN'])
   
    print("Starting sliding state estimation from wrench cone")
    while not rospy.is_shutdown():
        t0 = time.time()

        update_and_publish = False

        if measured_contact_wrench_list:
            update_and_publish = True

            while measured_contact_wrench_list:
                measured_contact_wrench = measured_contact_wrench_list.pop(0)
                friction_reasoning.compute_sliding_state_contact(
                    sliding_state_dict,friction_parameter_dict,last_slide_time_dict,
                    t0,measured_contact_wrench,contact_friction_cone_boundary_margin,reset_time_length)
  
        if measured_base_wrench_list:
            update_and_publish = True

            while measured_base_wrench_list:
                measured_base_wrench = measured_base_wrench_list.pop(0)
                friction_reasoning.compute_sliding_state_base(
                    sliding_state_dict,friction_parameter_dict,last_slide_time_dict,
                    t0,measured_base_wrench,external_friction_cone_boundary_margin,reset_time_length)

        if friction_parameter_list:
            while friction_parameter_list:
                friction_parameter_dict = friction_parameter_list.pop(0)
            friction_reasoning.convert_friction_param_dict_to_array(friction_parameter_dict)

        if update_and_publish:
            sliding_state_pub.publish(
                pmh.sliding_dict_to_sliding_stamped(sliding_dict=sliding_state_dict))

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




