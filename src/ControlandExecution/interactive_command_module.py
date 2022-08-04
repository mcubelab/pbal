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

import Helpers.pbal_msg_helper as pmh
import numpy as np
import rospy
import pdb
from pynput import keyboard
import time

from pbal.msg import ControlCommandStamped

command_pause = {
    "name": "pause",
    "command_flag" : 1,
    "mode" : -1,
    "delta_theta" : 0.0,
    "delta_x_pivot" : 0.0,
    "delta_s" : 0.0,
}

delta_rotate_left = {
    "name": "delta_rotate_left",
    "command_flag" : 1,
    "mode" : -1,
    "delta_theta" : np.pi/12,
    "delta_x_pivot" : 0.0,
    "delta_s" : 0.0,
}

delta_rotate_right = {
    "name": "delta_rotate_right",
    "command_flag" : 1,
    "mode" : -1,
    "delta_theta" : -np.pi/12,
    "delta_x_pivot" : 0.0,
    "delta_s" : 0.0,
}

delta_rotate_corner_left = {
    "name": "delta_rotate_left",
    "command_flag" : 1,
    "mode" : 14,
    "delta_theta" : np.pi/12,
    "delta_x_pivot" : 0.0,
    "delta_s" : 0.0,
}

delta_rotate_corner_right = {
    "name": "delta_rotate_right",
    "command_flag" : 1,
    "mode" : 14,
    "delta_theta" : -np.pi/12,
    "delta_x_pivot" : 0.0,
    "delta_s" : 0.0,
}

delta_slide_robot_left = {
    "name": "delta_slide_robot_left",
    "command_flag" : 1,
    "mode" : 1,
    "delta_theta" : 0,
    "delta_x_pivot" : 0.00,
    "delta_s" : -0.015,
}

delta_slide_robot_right = {
    "name": "delta_slide_robot_right",
    "command_flag" : 1,
    "mode" : 0,
    "delta_theta" : 0,
    "delta_x_pivot" : 0.00,
    "delta_s" : 0.015,
}

delta_super_slide_robot_left = {
    "name": "delta_super_slide_robot_left",
    "command_flag" : 1,
    "mode" : 13,
    "delta_theta" : 0,
    "delta_x_pivot" : 0.00,
    "delta_s" : -0.001,
}

delta_super_slide_robot_right = {
    "name": "delta_super_slide_robot_right",
    "command_flag" : 1,
    "mode" : 12,
    "delta_theta" : 0,
    "delta_x_pivot" : 0.00,
    "delta_s" : 0.001,
}

delta_slide_pivot_left = {
    "name": "delta_slide_pivot_left",
    "command_flag" : 1,
    "mode" : 2,
    "delta_theta" : 0,
    "delta_x_pivot" : 0.17,
    "delta_s" : 0.00,
}

delta_slide_pivot_right = {
    "name": "delta_slide_pivot_right",
    "command_flag" : 1,
    "mode" : 3,
    "delta_theta" : 0,
    "delta_x_pivot" : -0.17,
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

absolute_rotate_left = {
    "name": "absolute_rotate_center",
    "command_flag" : 0,
    "mode" : -1,
    "theta" : 1.1*np.pi/6,
    "x_pivot" : 0.0,
    "s" : 0.0,
}

absolute_rotate_right = {
    "name": "absolute_rotate_center",
    "command_flag" : 0,
    "mode" : -1,
    "theta" : -1.1*np.pi/6,
    "x_pivot" : 0.0,
    "s" : 0.0,
}

def on_press(key):

    global control_command_pub, command_msg

    command_msg_dict = None
    
    if key == keyboard.Key.esc:
        return False  # stop listener
    if key == keyboard.Key.space:
        command_msg_dict = command_pause
        print("pausing motion!")
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys

    if k == 'c':  # center
        command_msg_dict = absolute_rotate_center
        print("absolute_rotate_center")
    if k == 'left': # pivot left
        command_msg_dict = delta_rotate_left
        print("delta_rotate_left")
        # command_msg_dict = absolute_rotate_left
        # print("absolute_rotate_left")
    if k == 'right': # pivot right
        command_msg_dict = delta_rotate_right
        print("delta_rotate_right")
        # command_msg_dict = absolute_rotate_right
        # print("absolute_rotate_right")
    if k == 'g': # pivot left
        command_msg_dict = delta_rotate_corner_left
        print("delta_rotate_corner_left")
    if k == 'h': # pivot right
        command_msg_dict = delta_rotate_corner_right
        print("delta_rotate_corner_right")
    if k == 'a': # robot left
        command_msg_dict = delta_slide_robot_left
        print("delta_slide_robot_left")
    if k == 'd': # robot right
        command_msg_dict = delta_slide_robot_right
        print("delta_slide_robot_right")
    if k == '1': # robot left
        command_msg_dict = delta_super_slide_robot_left
        print("delta_super_slide_robot_left")
    if k == '3': # robot right
        command_msg_dict = delta_super_slide_robot_right
        print("delta_super_slide_robot_right")
    if k == 'q': # object left
        command_msg_dict = delta_slide_pivot_left
        print("delta_slide_pivot_left")
    if k == 'e': # object right
        command_msg_dict = delta_slide_pivot_right
        print("delta_slide_pivot_right")




    if command_msg_dict is not None:
        # command_msg_string = json.dumps(command_msg_dict)
        command_msg = pmh.command_dict_to_command_stamped(
            command_msg_dict)
    else: 
        command_msg = None

    # pdb.set_trace()
    if command_msg is not None:
        # command_msg.data = command_msg_string
        # pdb.set_trace()
        # print(command_msg)
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
    print('-----------------------------')
    print('COMMAND SUMMARY')
    print('c: absolute rotate center')
    print('left arrow: rotate left')
    print('right arrow: rotate right')
    print('g: rotate corner left')
    print('h: rotate corner right')
    print('a: slide hand left') 
    print('d: slide hand right')
    print('1: super slide left')
    print('3: super slide right')
    print('q: slide pivot left')
    print('e: slide pivote right')
    print('Space: pause')
    print('-----------------------------')

    rospy.init_node("barrier_func_commands")
    rospy.sleep(1.0)

    command_msg = ControlCommandStamped()

    # control_command_pub = rospy.Publisher(
    #     '/barrier_func_control_command', 
    #     String,
    #     queue_size=10)
    control_command_pub = rospy.Publisher('/barrier_func_control_command', 
        ControlCommandStamped, queue_size=10)


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






        
