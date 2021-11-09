#!/usr/bin/env python
import rospy
import pdb
import numpy as np
import json
from std_msgs.msg import String
import time
from pynput import keyboard





delta_rotate_left = {
    "name": "delta_rotate_left",
    "command_flag" : 1,
    "mode" : -1,
    "delta_theta" : np.pi/25,
    "delta_x_pivot" : 0.0,
    "delta_s" : 0.0,
}

delta_rotate_right = {
    "name": "delta_rotate_right",
    "command_flag" : 1,
    "mode" : -1,
    "delta_theta" : -np.pi/25,
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


absolute_rotate_center = {
    "name": "absolute_rotate_center",
    "command_flag" : 0,
    "mode" : -1,
    "theta" : 0,
    "x_pivot" : 0.0,
    "s" : 0.0,
}


def on_press(key):

    global control_command_pub, command_msg

    if key == keyboard.Key.esc:
        return False  # stop listener
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys

    command_msg_dict = None


    if k == 'c':  # center
        command_msg_dict = absolute_rotate_center
        print("absolute_rotate_center")
    if k == 'left': # pivot left
        command_msg_dict = delta_rotate_left
        print("delta_rotate_left")
    if k == 'right': # pivot right
        command_msg_dict = delta_rotate_right
        print("delta_rotate_right")
    if k == 'a': # robot left
        command_msg_dict = delta_slide_robot_left
        print("delta_slide_robot_left")
    if k == 'd': # robot right
        command_msg_dict = delta_slide_robot_right
        print("delta_slide_robot_right")
    if k == 'q': # object left
        command_msg_dict = delta_slide_pivot_left
        print("delta_slide_pivot_left")
    if k == 'e': # object right
        command_msg_dict = delta_slide_pivot_right
        print("delta_slide_pivot_right")


    if command_msg_dict is not None:
        command_msg_string = json.dumps(command_msg_dict)
    else: 
        command_msg_string = None


    if command_msg_string is not None:
        command_msg.data = command_msg_string
        control_command_pub.publish(command_msg)
        # published = False
        # while (not published) and (not rospy.is_shutdown()):

            
        #     print "num connections: ", control_command_pub.get_num_connections()

        #     if control_command_pub.get_num_connections() >= 1:
        #         print("command received")
        #         command_msg.data = command_msg_string     
        #         control_command_pub.publish(command_msg)
        #         published = True


if __name__ == '__main__':

    rospy.init_node("barrier_func_commands")
    rospy.sleep(1.0)

    command_msg = String()

    control_command_pub = rospy.Publisher(
        '/barrier_func_control_command', 
        String,
        queue_size=10)


    # published = False
    listener = keyboard.Listener(on_press=on_press)
    listener.start()                # start to listen on a separate thread
    listener.join()

    # while (not published) and (not rospy.is_shutdown()):
        
    #     print(control_command_pub.get_num_connections())
    #     if control_command_pub.get_num_connections() == 1:

    #         command_msg.data = command_msg_string
     
    #         control_command_pub.publish(command_msg)
    #         published = True

    # time.sleep(3)

    # while True:
    #     for message in message_queue:
    #        #print(message["name"])
    #        command_msg_dict = message
    #        command_msg_string = json.dumps(command_msg_dict)
    #        command_msg.data = command_msg_string
    #        control_command_pub.publish(command_msg)
    #        time.sleep(5)

    # command_msg_dict = absolute_rotate_center
    # # command_msg_dict_pruned = prune_command_message(command_msg_dict)
    # command_msg_string = json.dumps(command_msg_dict)
    # command_msg.data = command_msg_string 
    # control_command_pub.publish(command_msg)






        
