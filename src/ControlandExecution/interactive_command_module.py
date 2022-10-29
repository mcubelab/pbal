#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

from Helpers.ros_manager import ros_manager
import numpy as np
from pynput import keyboard


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
    'delta_s_pivot' : 0.17,
    'delta_s_hand' : 0.00,
}

delta_slide_pivot_right = {
    'name': 'delta_slide_pivot_right',
    'command_flag' : 1,
    'mode' : 3,
    'delta_theta' : 0.0,
    'delta_s_pivot' : -0.17,
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
    if k == 'left': # pivot left
        command_msg_dict = delta_rotate_left
        print('delta_rotate_left')
    if k == 'right': # pivot right
        command_msg_dict = delta_rotate_right
        print('delta_rotate_right')
    if k == 'g': # pivot left
        command_msg_dict = delta_rotate_corner_left
        print('delta_rotate_corner_left')
    if k == 'h': # pivot right
        command_msg_dict = delta_rotate_corner_right
        print('delta_rotate_corner_right')
    if k == 'a': # robot left
        command_msg_dict = delta_slide_robot_left
        print('delta_slide_robot_left')
    if k == 'd': # robot right
        command_msg_dict = delta_slide_robot_right
        print('delta_slide_robot_right')
    if k == '1': # robot left
        command_msg_dict = delta_super_slide_robot_left
        print('delta_super_slide_robot_left')
    if k == '3': # robot right
        command_msg_dict = delta_super_slide_robot_right
        print('delta_super_slide_robot_right')
    if k == 'q': # object left
        command_msg_dict = delta_slide_pivot_left
        print('delta_slide_pivot_left')
    if k == 'e': # object right
        command_msg_dict = delta_slide_pivot_right
        print('delta_slide_pivot_right')

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
    print('Space: pause')
    print('-----------------------------')

    rm = ros_manager()
    rm.init_node('barrier_func_commands')
    rm.spawn_publisher('/barrier_func_control_command')

    listener = keyboard.Listener(on_press=on_press)

    # start to listen on a separate thread
    listener.start()                
    listener.join()







        
