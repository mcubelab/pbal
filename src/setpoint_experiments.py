#!/usr/bin/env python
import rospy
import pdb
import random
import numpy as np
import json
from std_msgs.msg import String
import time

import models.ros_helper as ros_helper


def absolute_rotate(absolute_angle):

    return {
        "name": "absolute_rotate",
        "command_flag" : 0,
        "mode" : -1,
        "theta" : absolute_angle,
        "x_pivot" : 0.0,
        "s" : 0.0,
    }



def relative_slide_hand_neg(absolute_angle, relative_sliding_pos_hand):

    if absolute_angle is not None:
        abs_rotate_dict = absolute_rotate(absolute_angle)
    else:
        abs_rotate_dict = None

    rel_hand_sliding_dict = {
        "name": "delta_slide_hand_neg",
        "command_flag" : 1,
        "mode" : 1,
        "delta_theta" : 0,
        "delta_x_pivot" : 0.00,
        "delta_s" : relative_sliding_pos_hand,
    }

    return abs_rotate_dict, rel_hand_sliding_dict


def relative_slide_hand_pos(absolute_angle, relative_sliding_pos_hand):

    if absolute_angle is not None:
        abs_rotate_dict = absolute_rotate(absolute_angle)
    else:
        abs_rotate_dict = None

    rel_hand_sliding_dict = {
        "name": "delta_slide_hand_pos",
        "command_flag" : 1,
        "mode" : 0,
        "delta_theta" : 0,
        "delta_x_pivot" : 0.00,
        "delta_s" : relative_sliding_pos_hand,
    }

    return abs_rotate_dict, rel_hand_sliding_dict

    
def relative_slide_object_neg(absolute_angle, relative_sliding_pos_obj):

    if absolute_angle is not None:
        abs_rotate_dict = absolute_rotate(absolute_angle)
    else:
        abs_rotate_dict = None

    rel_hand_sliding_dict = {
        "name": "delta_slide_object_neg",
        "command_flag" : 1,
        "mode" : 2,
        "delta_theta" : 0,
        "delta_x_pivot" : relative_sliding_pos_obj,
        "delta_s" : 0.00,
    }

    return abs_rotate_dict, rel_hand_sliding_dict

def relative_slide_object_pos(absolute_angle, relative_sliding_pos_obj):

    if absolute_angle is not None:
        abs_rotate_dict = absolute_rotate(absolute_angle)
    else:
        abs_rotate_dict = None

    rel_hand_sliding_dict = {
        "name": "delta_slide_object_pos",
        "command_flag" : 1,
        "mode" : 3,
        "delta_theta" : 0,
        "delta_x_pivot" : relative_sliding_pos_obj,
        "delta_s" : 0.00,
    }

    return abs_rotate_dict, rel_hand_sliding_dict

def qp_debug_message_callback(data):
    global qp_debug_dict
    if data.data != '':
        qp_debug_dict = json.loads(data.data) 

if __name__ == '__main__':

    rospy.init_node("setpoint_experiments")
    rospy.sleep(1.0)
    rate = rospy.Rate(1) # in yaml

    # number of primitives
    num_primitives = 1

    # choose variables
    delta_targets = {
        'delta_sh_plus': 0.01,
        'delta_sh_minus': -0.01, 
        'delta_sg_plus': 0.03,
        'delta_sg_minus': -0.03 
    }

    absolute_targets =  {
        'theta_plus': np.pi/6,
        'theta_minus': -np.pi/6,
        'theta_before_sh': np.pi/12,
        # 'sh_plus': 0.03,
        # 'sh_minus': -0.03, 
        'theta_before_sg': np.pi/6
        # 'sg_plus': 0.1,
        # 'sg_minus': -0.1, 
    }

    # sequence of primitives
    all_primitives_names = [
        "absolute_rotate_pos", 
        "absolute_rotate_neg", 
        # "relative_slide_object_pos", 
        # "relative_slide_object_neg", 
        "relative_slide_hand_pos",
        "relative_slide_hand_neg"
    ]

    # sequence of primitives
    primitive_name_seq = ["relative_slide_hand_neg", "relative_slide_hand_pos"]
    # primitive_name_seq = random.sample(all_primitives_names, 1)

    # build sequence of primitive dict commands from sequence of primitives
    primitive_dict_seq, target_seq = [], []

    for primitive_name in primitive_name_seq:

        if primitive_name == 'absolute_rotate_pos':
            rotate_dict = absolute_rotate(absolute_targets['theta_plus'])

            primitive_dict_seq.extend([rotate_dict])
            target_seq.extend([rotate_dict['theta']])

        if primitive_name == 'absolute_rotate_neg':
            rotate_dict = absolute_rotate(absolute_targets['theta_minus'])
            
            primitive_dict_seq.extend([rotate_dict])
            target_seq.extend([rotate_dict['theta']])

        if primitive_name == 'relative_slide_object_pos':

            rotate_dict, slide_dict = relative_slide_object_pos(
                absolute_targets['theta_before_sg'], 
                delta_targets['delta_sg_plus'])

            primitive_dict_seq.extend([rotate_dict, slide_dict])
            target_seq.extend([rotate_dict['theta'], slide_dict['delta_x_pivot']])

        if primitive_name == 'relative_slide_object_neg':

            rotate_dict, slide_dict = relative_slide_object_neg(
                absolute_targets['theta_before_sg'],
                delta_targets['delta_sg_minus'])

            primitive_dict_seq.extend([rotate_dict, slide_dict])
            target_seq.extend([rotate_dict['theta'], slide_dict['delta_x_pivot']])

        if primitive_name == 'relative_slide_hand_pos':

            rotate_dict, slide_dict = relative_slide_hand_pos(
                -absolute_targets['theta_before_sh'],
                delta_targets['delta_sh_plus'])

            primitive_dict_seq.extend([rotate_dict, slide_dict])
            target_seq.extend([rotate_dict['theta'], slide_dict['delta_s']])

        if primitive_name == 'relative_slide_hand_neg':

            rotate_dict, slide_dict = relative_slide_hand_neg(
                absolute_targets['theta_before_sh'],
                delta_targets['delta_sh_minus'])

            primitive_dict_seq.extend([rotate_dict, slide_dict])
            target_seq.extend([rotate_dict['theta'], slide_dict['delta_s']])

    print("Primitive sequence is... ")
    for primitive_dict in primitive_dict_seq:
        print(primitive_dict['name'])
    rospy.sleep(1.0)

    # publisher
    control_command_pub = rospy.Publisher(
        '/barrier_func_control_command', 
        String,
        queue_size=10)
    command_msg = String()

    #subscribers
    qp_debug_message_sub = rospy.Subscriber(
       '/qp_debug_message',
        String,
        qp_debug_message_callback)
    qp_debug_dict = None

    # print("Waiting for qp debug message")
    # while qp_debug_dict is None:
    #     rospy.sleep(0.1)



    TOL = 1e-3                          # tolerance
    current_primitive_index = 0         # index of current primitive
    error_list = []                     # current error

    # start rosbag
    rostopic_list = [
        "/camera/color/image_raw/compressed",
        "/ground_truth_message",
        "/gravity_torque", 
        "/pivot_frame", 
        "/generalized_positions", 
        '/barrier_func_control_command', 
        '/qp_debug_message',
        "/end_effector_sensor_in_end_effector_frame", 
        "/end_effector_sensor_in_base_frame", 
        '/friction_parameters', 
        '/sliding_state'
    ]

    print("Starting rosbag recording...")
    # ros_helper.initialize_rosbag(rostopic_list, exp_name="experiment001")

    # start loop
    print("Starting experiment...")
    while not rospy.is_shutdown():

        if current_primitive_index >= len(primitive_dict_seq):
            break

        # publish current command    
        print(primitive_dict_seq[current_primitive_index]['name'
            ] + " to %5.2f" % target_seq[current_primitive_index])
        command_msg_string = json.dumps(primitive_dict_seq[current_primitive_index])
        command_msg.data = command_msg_string
        control_command_pub.publish(command_msg)
        
        # make sure we have end effector wrench
        if qp_debug_dict is None or qp_debug_dict[
            'name'] != primitive_dict_seq[current_primitive_index]['name']:
            # print("Waiting for qp debug dict")
            rate.sleep()
            continue

        # compute error
        error_dict = qp_debug_dict['error_dict']
        print(error_dict.keys())
        primitive_error = 0
        for errori in error_dict.values():
            primitive_error += errori ** 2
        print(np.sqrt(primitive_error))

        # move onto next primitive
        if np.sqrt(primitive_error) < TOL:
            print("moving to next primitive")
            current_primitive_index+=1

        rate.sleep()


    # stop recording
    print("experiment finished")
    ros_helper.terminate_rosbag()
    








    # # primitive motions
    # absolute_rotate_neg = {
    #     "name": "absolute_rotate_neg",
    #     "command_flag" : 0,
    #     "mode" : -1,
    #     "theta" : absolute_targets['theta_minus'],
    #     "x_pivot" : 0.0,
    #     "s" : 0.0,
    # }

    # absolute_rotate_pos = {
    #     "name": "absolute_rotate_pos",
    #     "command_flag" : 0,
    #     "mode" : -1,
    #     "theta" : absolute_targets['theta_minus'], #np.pi/6-np.pi/20,
    #     "x_pivot" : 0.0,
    #     "s" : 0.0,
    # }


    # delta_slide_pivot_line_contact_neg = {
    #     "command_flag" : 1,
    #     "mode" : 7,
    #     "delta_theta" : 0,
    #     "delta_x_pivot" : 0.06,
    #     "delta_s" : 0.00,
    # }

    # delta_slide_pivot_line_contact_pos = {
    #     "command_flag" : 1,
    #     "mode" : 8,
    #     "delta_theta" : 0,
    #     "delta_x_pivot" :- 0.06,
    #     "delta_s" : 0.00,
    # }

    # delta_slide_robot_line_contact_neg = {
    #     "command_flag" : 1,
    #     "mode" : 11,
    #     "delta_theta" : 0,
    #     "delta_x_pivot" : 0.00,
    #     "delta_s" : -0.01,
    # }

    # delta_slide_robot_line_contact_pos = {
    #     "command_flag" : 1,
    #     "mode" : 10,
    #     "delta_theta" : 0,
    #     "delta_x_pivot" : 0.00,
    #     "delta_s" : 0.01,
    # }

    # static_object_flush_stick = {
    #     "command_flag" : 1,
    #     "mode" : 9
    # }





    # published = False
    # while (not published) and (not rospy.is_shutdown()):
        
    #     print(control_command_pub.get_num_connections())
    #     if control_command_pub.get_num_connections() == 1:

    #         command_msg.data = command_msg_string
     
    #         
    #         published = True

    # # time.sleep(3)

    # # while True:
    # #     for message in message_queue:
    # #        #print(message["name"])
    # #        command_msg_dict = message
    # #        command_msg_string = json.dumps(command_msg_dict)
    # #        command_msg.data = command_msg_string
    # #        control_command_pub.publish(command_msg)
    # #        time.sleep(5)

    # # command_msg_dict = absolute_rotate_center
    # # # command_msg_dict_pruned = prune_command_message(command_msg_dict)
    # # command_msg_string = json.dumps(command_msg_dict)
    # # command_msg.data = command_msg_string 
    # # control_command_pub.publish(command_msg)






        
