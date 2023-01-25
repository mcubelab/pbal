#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

from Helpers.ros_manager import ros_manager
import numpy as np
from pynput import keyboard

import rospy

command_pause = {
    'name': 'pause',
    'command_flag' : 1,
    'mode' : -1,
    'delta_theta' : 0.0,
    'delta_s_pivot' : 0.0,
    'delta_s_hand' : 0.0,
}

delta_rotate_left = {
    'name': 'delta_rotate_left',
    'command_flag' : 1,
    'mode' : -1,
    'delta_theta' : np.pi/12,
    'delta_s_pivot' : 0.0,
    'delta_s_hand' : 0.0,
}

delta_rotate_right = {
    'name': 'delta_rotate_right',
    'command_flag' : 1,
    'mode' : -1,
    'delta_theta' : -np.pi/12,
    'delta_s_pivot' : 0.0,
    'delta_s_hand' : 0.0,
}

delta_rotate_corner_left = {
    'name': 'delta_rotate_left',
    'command_flag' : 1,
    'mode' : 6,
    'delta_theta' : np.pi/12,
    'delta_s_pivot' : 0.0,
    'delta_s_hand' : 0.0,
}

delta_rotate_corner_right = {
    'name': 'delta_rotate_right',
    'command_flag' : 1,
    'mode' : 6,
    'delta_theta' : -np.pi/12,
    'delta_s_pivot' : 0.0,
    'delta_s_hand' : 0.0,
}

delta_slide_robot_left = {
    'name': 'delta_slide_robot_left',
    'command_flag' : 1,
    'mode' : 1,
    'delta_theta' : 0.0,
    'delta_s_pivot' : 0.00,
    'delta_s_hand' : -0.015,
}

delta_slide_robot_right = {
    'name': 'delta_slide_robot_right',
    'command_flag' : 1,
    'mode' : 0,
    'delta_theta' : 0.0,
    'delta_s_pivot' : 0.00,
    'delta_s_hand' : 0.015,
}

delta_super_slide_robot_left = {
    'name': 'delta_super_slide_robot_left',
    'command_flag' : 1,
    'mode' : 5,
    'delta_theta' : 0.0,
    'delta_s_pivot' : 0.00,
    'delta_s_hand' : -0.015,
}

delta_super_slide_robot_right = {
    'name': 'delta_super_slide_robot_right',
    'command_flag' : 1,
    'mode' : 4,
    'delta_theta' : 0.0,
    'delta_s_pivot' : 0.00,
    'delta_s_hand' : 0.015,
}

delta_slide_pivot_left = {
    'name': 'delta_slide_pivot_left',
    'command_flag' : 1,
    'mode' : 2,
    'delta_theta' : 0.0,
    'delta_s_pivot' : 0.03,
    'delta_s_hand' : 0.00,
}

delta_slide_pivot_right = {
    'name': 'delta_slide_pivot_right',
    'command_flag' : 1,
    'mode' : 3,
    'delta_theta' : 0.0,
    'delta_s_pivot' : -0.03,
    'delta_s_hand' : 0.00,
}

absolute_rotate_center = {
    'name': 'absolute_rotate_center',
    'command_flag' : 0,
    'mode' : -1,
    'theta' : 0,
    's_pivot' : 0.0,
    's_hand' : 0.0,
}

absolute_rotate_left = {
    'name': 'absolute_rotate_center',
    'command_flag' : 0,
    'mode' : -1,
    'theta' : 1.1*np.pi/6,
    's_pivot' : 0.0,
    's_hand' : 0.0,
}

absolute_rotate_right = {
    'name': 'absolute_rotate_center',
    'command_flag' : 0,
    'mode' : -1,
    'theta' : -1.1*np.pi/6,
    's_pivot' : 0.0,
    's_hand' : 0.0,
}

external_line_contact = {
    'name': 'external_line_contact',
    'command_flag' : 1,
    'mode' : 7,
    'theta' : 0.0,
    's_pivot' : 0.0,
    's_hand' : 0.0,
}

wall_contact_on = {
    'name': 'wall_contact_on',
    'command_flag' : 2,
    'mode' : 0,
    'theta' : 0,
    's_pivot' : 0.0,
    's_hand' : 0.0,
}

wall_contact_off = {
    'name': 'wall_contact_off',
    'command_flag' : 2,
    'mode' : 1,
    'theta' : 0,
    's_pivot' : 0.0,
    's_hand' : 0.0,
}


def on_press(key):

    global rm

    command_msg_dict = None
    
    if key == keyboard.Key.esc:
        return False  # stop listener
    if key == keyboard.Key.space:
        command_msg_dict = command_pause
        print('pausing motion!')
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys

    if k == 'c':  # center
        command_msg_dict = absolute_rotate_center
        print('absolute_rotate_center')
    elif k == 'left': # pivot left
        command_msg_dict = delta_rotate_left
        print('delta_rotate_left')
    elif k == 'right': # pivot right
        command_msg_dict = delta_rotate_right
        print('delta_rotate_right')
    elif k == 'g': # pivot left
        command_msg_dict = delta_rotate_corner_left
        print('delta_rotate_corner_left')
    elif k == 'h': # pivot right
        command_msg_dict = delta_rotate_corner_right
        print('delta_rotate_corner_right')
    elif k == 'a': # robot left
        command_msg_dict = delta_slide_robot_left
        print('delta_slide_robot_left')
    elif k == 'd': # robot right
        command_msg_dict = delta_slide_robot_right
        print('delta_slide_robot_right')
    elif k == '1': # robot left
        command_msg_dict = delta_super_slide_robot_left
        print('delta_super_slide_robot_left')
    elif k == '3': # robot right
        command_msg_dict = delta_super_slide_robot_right
        print('delta_super_slide_robot_right')
    elif k == 'q': # object left
        command_msg_dict = delta_slide_pivot_left
        print('delta_slide_pivot_left')
    elif k == 'e': # object right
        command_msg_dict = delta_slide_pivot_right
        print('delta_slide_pivot_right')
    elif k == 'i': # object right
        command_msg_dict = external_line_contact
        print('external_line_contact')
    elif k == 'o': #wall_contact_on
        command_msg_dict = wall_contact_on
        print('wall_contact_on')
    elif k == 'p': # wall_contact_off
        command_msg_dict = wall_contact_off
        print('wall_contact_off')

    if command_msg_dict is not None:
        rm.pub_barrier_func_control_command(command_msg_dict)


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
    print('i: external_line_contact')
    print('Space: pause')
    print('o: wall contact on')
    print('p: wall contact off')
    print('-----------------------------')

    rm = ros_manager()
    rospy.init_node('barrier_func_commands')
    rm.spawn_publisher('/barrier_func_control_command')

    listener = keyboard.Listener(on_press=on_press)

    # start to listen on a separate thread
    listener.start()                
    listener.join()







        
