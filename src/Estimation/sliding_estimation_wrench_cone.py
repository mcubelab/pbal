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


def end_effector_wrench_callback(data):
    global measured_contact_wrench_list
    end_effector_wrench = data
    measured_contact_wrench_6D = rh.wrench_stamped2list(
            end_effector_wrench)
    measured_contact_wrench = -np.array([
            measured_contact_wrench_6D[0], 
            measured_contact_wrench_6D[1],
            measured_contact_wrench_6D[-1]])

    measured_contact_wrench_list.append(measured_contact_wrench)
    if len(measured_contact_wrench_list) > 100:
       measured_contact_wrench_list.pop(0)

def end_effector_wrench_base_frame_callback(data):
    global measured_base_wrench_list
    base_wrench = data
    measured_base_wrench_6D = rh.wrench_stamped2list(
            base_wrench)
    measured_base_wrench = -np.array([
            measured_base_wrench_6D[0], 
            measured_base_wrench_6D[2],
            measured_base_wrench_6D[-1]])

    measured_base_wrench_list.append(measured_base_wrench)
    if len(measured_base_wrench_list) > 100:
       measured_base_wrench_list.pop(0)

def friction_parameter_callback(data):
    global friction_parameter_list
    friction_dict = pmh.friction_stamped_to_friction_dict(
        data)
    friction_parameter_list.append(friction_dict)
    # friction_parameter_list.append(json.loads(data.data))
    if len(friction_parameter_list) > 3:
       friction_parameter_list.pop(0)



if __name__ == '__main__':

    node_name = 'sliding_estimation_wrench_cone'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.estimator_params["RATE"])

    theta_min_contact = np.arctan(
        sys_params.controller_params["pivot_params"]["mu_contact"])
    theta_min_external = np.arctan(
        sys_params.controller_params["pivot_params"]["mu_ground"])

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
    # friction_parameter_sub = rospy.Subscriber(
    #     '/friction_parameters', String, friction_parameter_callback)
    friction_parameter_sub = rospy.Subscriber('/friction_parameters', 
        FrictionParamsStamped, friction_parameter_callback)

    # publishers
    sliding_state_pub = rospy.Publisher(
        '/sliding_state', 
        SlidingStateStamped,
        queue_size=10)

    A_contact_right = np.zeros([1,3])
    B_contact_right = 0
    A_contact_left = np.zeros([1,3])
    B_contact_left = 0
    
    A_external_right = np.zeros([0,3])
    B_external_right = np.zeros(0)
    A_external_left = np.zeros([0,3])
    B_external_left = np.zeros(0)
    

    #parameters describing how close to the friction 
    #cone boundary you need to be
    #to be considered sliding
    contact_friction_cone_boundary_margin = 2
    external_friction_cone_boundary_margin = 2

    pivot_sliding_wrench_measured_flag = False 
    pivot_sliding_left_wrench_measured_flag = False 
    pivot_sliding_right_wrench_measured_flag = False

    last_pivot_slide_left_time = -1
    last_pivot_slide_right_time = -1 

    contact_sliding_wrench_measured_flag = False 
    contact_sliding_left_wrench_measured_flag = False 
    contact_sliding_right_wrench_measured_flag = False 

    last_contact_slide_left_time = -1
    last_contact_slide_right_time = -1

    reset_time_length = .25

    sliding_state_dict = {
        "psf": pivot_sliding_wrench_measured_flag,
        "pslf": pivot_sliding_left_wrench_measured_flag,
        "psrf": pivot_sliding_right_wrench_measured_flag,

        "csf": contact_sliding_wrench_measured_flag,
        "cslf": contact_sliding_left_wrench_measured_flag,
        "csrf": contact_sliding_right_wrench_measured_flag
    }

    # queue for computing frequnecy
    time_deque = collections.deque(
        maxlen=sys_params.debug_params['QUEUE_LEN'])
   
    print("Starting sliding state estimation from wrench cone")
    while not rospy.is_shutdown():

        t0 = time.time()

        update_and_publish = False

        if measured_contact_wrench_list:
            update_and_publish = True

            if time.time()-last_contact_slide_left_time>reset_time_length:
                contact_sliding_left_wrench_measured_flag = False 
            if time.time()-last_contact_slide_right_time>reset_time_length:
                contact_sliding_right_wrench_measured_flag = False

            while measured_contact_wrench_list:
                measured_contact_wrench = measured_contact_wrench_list.pop(0)
             
                slide_right_bool = np.dot(A_contact_right,measured_contact_wrench) > B_contact_right - contact_friction_cone_boundary_margin
                slide_left_bool = np.dot(A_contact_left,measured_contact_wrench) > B_contact_left - contact_friction_cone_boundary_margin

                slide_right_bool = slide_right_bool.item()
                slide_left_bool = slide_left_bool.item()
                if slide_left_bool:
                    last_contact_slide_left_time = time.time()

                if slide_right_bool:
                    last_contact_slide_right_time = time.time()


                contact_sliding_right_wrench_measured_flag = contact_sliding_right_wrench_measured_flag or slide_right_bool
                contact_sliding_left_wrench_measured_flag = contact_sliding_left_wrench_measured_flag or slide_left_bool

            contact_sliding_wrench_measured_flag = contact_sliding_right_wrench_measured_flag or contact_sliding_left_wrench_measured_flag

            # if contact_sliding_right_wrench_measured_flag and not contact_sliding_left_wrench_measured_flag:
            #     print 'sliding right'
            # if contact_sliding_left_wrench_measured_flag and not contact_sliding_right_wrench_measured_flag:
            #     print 'sliding left'
            # if contact_sliding_left_wrench_measured_flag and contact_sliding_right_wrench_measured_flag:
            #     print 'not enough normal force'
            # if not (contact_sliding_left_wrench_measured_flag or contact_sliding_right_wrench_measured_flag):
            #     print 'sticking'
                    
        if measured_base_wrench_list:

            update_and_publish = True

            if time.time()-last_pivot_slide_left_time>reset_time_length:
                pivot_sliding_left_wrench_measured_flag = False 
            if time.time()-last_pivot_slide_right_time>reset_time_length:
                pivot_sliding_right_wrench_measured_flag = False

            while measured_base_wrench_list:
                measured_base_wrench = measured_base_wrench_list.pop(0)

                if len(A_external_right)>0:
                    slide_right_bool = any(np.dot(A_external_right,measured_base_wrench) > B_external_right - external_friction_cone_boundary_margin)
                else:
                    slide_right_bool = False

                if len(A_external_left)>0:
                    slide_left_bool = any(np.dot(A_external_left,measured_base_wrench) > B_external_left - external_friction_cone_boundary_margin)
                else:
                    slide_left_bool = False

                if slide_left_bool:
                    last_pivot_slide_left_time = time.time()

                if slide_right_bool:
                    last_pivot_slide_right_time = time.time()

                pivot_sliding_right_wrench_measured_flag = pivot_sliding_right_wrench_measured_flag or slide_right_bool
                pivot_sliding_left_wrench_measured_flag = pivot_sliding_left_wrench_measured_flag or slide_left_bool

            pivot_sliding_wrench_measured_flag = pivot_sliding_right_wrench_measured_flag or pivot_sliding_left_wrench_measured_flag

            # if pivot_sliding_right_wrench_measured_flag and not pivot_sliding_left_wrench_measured_flag:
            #     print 'sliding right'
            # if pivot_sliding_left_wrench_measured_flag and not pivot_sliding_right_wrench_measured_flag:
            #     print 'sliding left'
            # if pivot_sliding_left_wrench_measured_flag and pivot_sliding_right_wrench_measured_flag:
            #     print 'not enough normal force'
            # if not (pivot_sliding_left_wrench_measured_flag or pivot_sliding_right_wrench_measured_flag):
            #     print 'sticking'

        if friction_parameter_list:
            while friction_parameter_list:
                friction_parameter_dict = friction_parameter_list.pop(0)

                A_contact_right = np.array(friction_parameter_dict["acr"])
                B_contact_right = friction_parameter_dict["bcr"]
                A_contact_left = np.array(friction_parameter_dict["acl"])
                B_contact_left = friction_parameter_dict["bcl"]
                
                A_external_right = np.array(friction_parameter_dict["aer"])
                B_external_right = np.array(friction_parameter_dict["ber"])
                A_external_left = np.array(friction_parameter_dict["ael"])
                B_external_left = np.array(friction_parameter_dict["bel"])

        if update_and_publish:

            sliding_state_dict["psf"]= pivot_sliding_wrench_measured_flag
            sliding_state_dict["pslf"]= pivot_sliding_left_wrench_measured_flag
            sliding_state_dict["psrf"]= pivot_sliding_right_wrench_measured_flag

            sliding_state_dict["csf"]= contact_sliding_wrench_measured_flag
            sliding_state_dict["cslf"]= contact_sliding_left_wrench_measured_flag
            sliding_state_dict["csrf"]= contact_sliding_right_wrench_measured_flag

            # sliding_state_msg.data = json.dumps(sliding_state_dict)
            sliding_state_msg = pmh.sliding_dict_to_sliding_stamped(
                sliding_dict=sliding_state_dict)

            sliding_state_pub.publish(sliding_state_msg)

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




