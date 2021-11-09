#!/usr/bin/env python
import rospy
import pdb
import json
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, Bool, String
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from scipy.spatial import ConvexHull, convex_hull_plot_2d

import time
import models.ros_helper as ros_helper

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines
from livestats import livestats
from models.system_params import SystemParams

def end_effector_wrench_callback(data):
    global measured_contact_wrench_list
    end_effector_wrench = data
    measured_contact_wrench_6D = ros_helper.wrench_stamped2list(
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
    measured_base_wrench_6D = ros_helper.wrench_stamped2list(
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
    friction_parameter_list.append(json.loads(data.data))
    if len(friction_parameter_list) > 3:
       friction_parameter_list.pop(0)



if __name__ == '__main__':
    measured_contact_wrench_list = []
    measured_base_wrench_list = []
    friction_parameter_list = []

    sys_params = SystemParams()

    theta_min_contact = np.arctan(sys_params.controller_params["pivot_params"]["mu_contact"])
    theta_min_external = np.arctan(sys_params.controller_params["pivot_params"]["mu_ground"])

    rospy.init_node("sliding_estimation_wrench_cone")
    rospy.sleep(1.0)

    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_callback)
    end_effector_wrench_base_frame_sub = rospy.Subscriber("/end_effector_sensor_in_base_frame", 
        WrenchStamped,  end_effector_wrench_base_frame_callback)
    friction_parameter_sub = rospy.Subscriber('/friction_parameters', String, friction_parameter_callback)

    sliding_state_pub = rospy.Publisher(
        '/sliding_state', 
        String,
        queue_size=10)

    sliding_state_msg = String()



    A_contact_right = np.zeros([1,3])
    B_contact_right = 0
    A_contact_left = np.zeros([1,3])
    B_contact_left = 0
    
    A_external_right = np.zeros([0,3])
    B_external_right = np.zeros(0)
    A_external_left = np.zeros([0,3])
    B_external_left = np.zeros(0)
    

    #parameters describing how close to the friction cone boundary you need to be
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
   
    print("Starting sliding state estimation from wrench cone")

    while not rospy.is_shutdown():

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

            sliding_state_msg.data = json.dumps(sliding_state_dict)

            sliding_state_pub.publish(sliding_state_msg)




