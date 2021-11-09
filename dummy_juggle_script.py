#!/usr/bin/env python
import rospy
import pdb
import numpy as np
import json
from std_msgs.msg import String
import time



if __name__ == '__main__':

    rospy.init_node("barrier_func_commands")
    rospy.sleep(1.0)

    command_msg = String()

    delta_rotate_left = {
        "command_flag" : 1,
        "mode" : -1,
        "delta_theta" : np.pi/20,
        "delta_x_pivot" : 0.0,
        "delta_s" : 0.0,
    }

    delta_rotate_right = {
        "command_flag" : 1,
        "mode" : -1,
        "delta_theta" : -np.pi/20,
        "delta_x_pivot" : 0.0,
        "delta_s" : 0.0,
    }

    delta_slide_robot_left = {
        "name": "delta_slide_robot_left",
        "command_flag" : 1,
        "mode" : 1,
        "delta_theta" : 0,
        "delta_x_pivot" : 0.00,
        "delta_s" : -0.01,
    }

    delta_slide_robot_right = {
        "name": "delta_slide_robot_right",
        "command_flag" : 1,
        "mode" : 0,
        "delta_theta" : 0,
        "delta_x_pivot" : 0.00,
        "delta_s" : 0.01,
    }

    delta_slide_pivot_left = {
        "name": "delta_slide_pivot_left",
        "command_flag" : 1,
        "mode" : 2,
        "delta_theta" : 0,
        "delta_x_pivot" : 0.01,
        "delta_s" : 0.00,
    }

    delta_slide_pivot_right = {
        "name": "delta_slide_pivot_right",
        "command_flag" : 1,
        "mode" : 3,
        "delta_theta" : 0,
        "delta_x_pivot" : -0.01,
        "delta_s" : 0.00,
    }

    delta_guarded_right = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : 0,
        "delta_xhand" : -0.05,
        "delta_zhand" : .00,
    }

    delta_guarded_left = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : 0,
        "delta_xhand" : 0.05,
        "delta_zhand" : .00,
    }

    delta_guarded_down = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : 0,
        "delta_xhand" : 0.0,
        "delta_zhand" : -.03,
    }

    delta_guarded_up = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : 0,
        "delta_xhand" : 0.0,
        "delta_zhand" : .03,
    }

    delta_guarded_clockwise = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : -np.pi/6,
        "delta_xhand" : 0.0,
        "delta_zhand" : .00,
    }

    delta_guarded_counterclockwise = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : np.pi/6,
        "delta_xhand" : 0.0,
        "delta_zhand" : .00,
    }

    delta_guarded_static = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : 0.0,
        "delta_xhand" : 0.0,
        "delta_zhand" : .00,
    }

    delta_flush_right = {
        "command_flag" : 1,
        "mode" : 5,
        "delta_xhand" : -0.05,
        "delta_zhand" : .00,
    }

    delta_flush_left = {
        "command_flag" : 1,
        "mode" : 5,
        "delta_xhand" : 0.05,
        "delta_zhand" : .00,
    }

    delta_flush_static = {
        "command_flag" : 1,
        "mode" : 5,
        "delta_xhand" : 0.00,
        "delta_zhand" : .00,
    }

    absolute_rotate_center = {
        "name": "absolute_rotate_center",
        "command_flag" : 0,
        "mode" : -1,
        "theta" : 0,
        "x_pivot" : 0.0,
        "s" : 0.0,
    }


    absolute_rotate_left = {
        "name": "absolute_rotate_left",
        "command_flag" : 0,
        "mode" : -1,
        "theta" : np.pi/10, #np.pi/6+np.pi/20,
        "x_pivot" : 0.0,
        "s" : 0.0,
    }

    absolute_rotate_right = {
        "name": "absolute_rotate_right",
        "command_flag" : 0,
        "mode" : -1,
        "theta" : -np.pi/10, #np.pi/6-np.pi/20,
        "x_pivot" : 0.0,
        "s" : 0.0,
    }


    delta_slide_pivot_line_contact_left = {
        "command_flag" : 1,
        "mode" : 7,
        "delta_theta" : 0,
        "delta_x_pivot" : 0.06,
        "delta_s" : 0.00,
    }

    delta_slide_pivot_line_contact_right = {
        "command_flag" : 1,
        "mode" : 8,
        "delta_theta" : 0,
        "delta_x_pivot" :- 0.06,
        "delta_s" : 0.00,
    }

    delta_slide_robot_line_contact_left = {
        "command_flag" : 1,
        "mode" : 11,
        "delta_theta" : 0,
        "delta_x_pivot" : 0.00,
        "delta_s" : -0.01,
    }

    delta_slide_robot_line_contact_right = {
        "command_flag" : 1,
        "mode" : 10,
        "delta_theta" : 0,
        "delta_x_pivot" : 0.00,
        "delta_s" : 0.01,
    }

    static_object_flush_stick = {
        "command_flag" : 1,
        "mode" : 9
    }


    command_msg_dict = { 
        "command_flag" : 1,
        "mode" : -1,
        "theta" : 0,
        "x_pivot" : 0.5,
        "s" : -0.01,
        "delta_theta" : 0,
        "delta_x_pivot" : -0.05,
        "delta_s" : .00,
        "xhand" : 0,
        "zhand" : 0,
        "theta" : 0,
        "delta_xhand" : -0.1,
        "delta_zhand" : .00,
    }

    # command_msg_dict = absolute_rotate_right
    #command_msg_dict = absolute_rotate_left
    command_msg_dict = absolute_rotate_center
    #command_msg_dict = delta_rotate_left
    #command_msg_dict = delta_rotate_right
    #command_msg_dict = delta_slide_robot_left
    #command_msg_dict = delta_slide_robot_right
    #command_msg_dict = delta_slide_pivot_left
    #command_msg_dict = delta_slide_pivot_right

    #command_msg_dict = delta_slide_pivot_line_contact_right
    #command_msg_dict = delta_slide_robot_line_contact_right
    #command_msg_dict = delta_slide_robot_line_contact_left
    #command_msg_dict = static_object_flush_stick

    #command_msg_dict = delta_flush_static
    #command_msg_dict = delta_guarded_static
    #command_msg_dict = delta_guarded_right
    #command_msg_dict = delta_guarded_left
    #command_msg_dict = delta_guarded_down
    #command_msg_dict = delta_guarded_up
    #command_msg_dict = delta_guarded_clockwise
    #command_msg_dict = delta_guarded_counterclockwise
    #command_msg_dict = delta_flush_right
    #command_msg_dict = delta_flush_left

    # message_queue= [
    #    absolute_rotate_center,
    #    delta_slide_robot_left,
    #    absolute_rotate_left,
    #    absolute_rotate_center,
    #    delta_slide_robot_right,
    #    absolute_rotate_right
    # ]


    # message_queue = [
    #    # delta_slide_robot_right,
    #    absolute_rotate_right,
    #    absolute_rotate_left,
    #    # delta_slide_robot_left,
    #    # delta_slide_robot_left,
    #    # absolute_rotate_left,
    #    absolute_rotate_center,
    #    # delta_slide_robot_right,
    # ]


    message_queue= [
       absolute_rotate_left,
       absolute_rotate_right
    ]

    #command_msg_dict = delta_flush_static

    command_msg_string = json.dumps(command_msg_dict)


    control_command_pub = rospy.Publisher(
        '/barrier_func_control_command', 
        String,
        queue_size=10)


    published = False
    while (not published) and (not rospy.is_shutdown()):
        
        print(control_command_pub.get_num_connections())
        if control_command_pub.get_num_connections() == 1:

            command_msg.data = command_msg_string
     
            control_command_pub.publish(command_msg)
            published = True

    time.sleep(3)
    print 'hello!'
    while True:
        for message in message_queue:
           #print(message["name"])
           command_msg_dict = message
           command_msg_string = json.dumps(command_msg_dict)
           command_msg.data = command_msg_string
           control_command_pub.publish(command_msg)
           time.sleep(40)

    # command_msg_dict = absolute_rotate_center
    # # command_msg_dict_pruned = prune_command_message(command_msg_dict)
    # command_msg_string = json.dumps(command_msg_dict)
    # command_msg.data = command_msg_string 
    # control_command_pub.publish(command_msg)






        
