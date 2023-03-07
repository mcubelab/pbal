#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

from Helpers.ros_manager import ros_manager
import numpy as np
from pynput import keyboard
import PySimpleGUI27 as sg

import rospy


keys_held_down = set()

command_pause = {
    'name': 'pause',
    'command_flag' : 3,
    'delta_theta' : 0.0,
    'delta_x': 0.0,
    'delta_y': 0.0,
    'delta_normal': 0.0,
    'delta_tangential': 0.0,
}

delta_rotate_left = {
    'name': 'delta_rotate_left',
    'command_flag' : 1,
    'mode' : -1,
    'delta_theta' : np.pi/50, #np.pi/12, 
    'delta_s_pivot' : 0.0,
    'delta_s_hand' : 0.0,
}

delta_rotate_right = {
    'name': 'delta_rotate_right',
    'command_flag' : 1,
    'mode' : -1,
    'delta_theta' : -np.pi/50, #-np.pi/12, 
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


delta_slide_robot_left_external_line_contact = {
    'name': 'delta_slide_robot_left_external_line_contact',
    'command_flag' : 1,
    'mode' : 9,
    'delta_theta' : 0.0,
    'delta_s_pivot' : 0.00,
    'delta_s_hand' : -0.015,
}

delta_slide_robot_right_external_line_contact = {
    'name': 'delta_slide_robot_right_external_line_contact',
    'command_flag' : 1,
    'mode' : 8,
    'delta_theta' : 0.0,
    'delta_s_pivot' : 0.00,
    'delta_s_hand' : 0.015,
}

delta_slide_object_left_external_line_contact = {
    'name': 'delta_slide_object_left_external_line_contact',
    'command_flag' : 1,
    'mode' : 10,
    'delta_theta' : 0.0,
    'delta_s_pivot' : 0.03,
    'delta_s_hand' : 0.0,
}

delta_slide_object_right_external_line_contact = {
    'name': 'delta_slide_object_right_external_line_contact',
    'command_flag' : 1,
    'mode' : 11,
    'delta_theta' : 0.0,
    'delta_s_pivot' : -0.03,
    'delta_s_hand' : 0.00,
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


jog_rotate_left = {
    'name': 'jog_rotate_left',
    'command_flag' : 3,
    'delta_theta' : np.pi/50,
    'delta_x': 0.0,
    'delta_y': 0.0,
    'delta_normal': 0.0,
    'delta_tangential': 0.0,
}

jog_rotate_right = {
    'name': 'jog_rotate_right',
    'command_flag' : 3,
    'delta_theta' : -np.pi/50,
    'delta_x': 0.0,
    'delta_y': 0.0,
    'delta_normal': 0.0,
    'delta_tangential': 0.0,
}

jog_move_left = {
    'name': 'jog_move_left',
    'command_flag' : 3,
    'delta_theta' : 0.0,
    'delta_x': -0.005,
    'delta_y': 0.0,
    'delta_normal': 0.0,
    'delta_tangential': 0.0,
}

jog_move_right = {
    'name': 'jog_move_right',
    'command_flag' : 3,
    'delta_theta' : 0.0,
    'delta_x': 0.005,
    'delta_y': 0.0,
    'delta_normal': 0.0,
    'delta_tangential': 0.0,
}

jog_move_up = {
    'name': 'jog_move_up',
    'command_flag' : 3,
    'delta_theta' : 0.0,
    'delta_x': 0.0,
    'delta_y': 0.005,
    'delta_normal': 0.0,
    'delta_tangential': 0.0,
}

jog_move_down = {
    'name': 'jog_move_down',
    'command_flag' : 3,
    'delta_theta' : 0.0,
    'delta_x': 0.0,
    'delta_y': -0.005,
    'delta_normal': 0.0,
    'delta_tangential': 0.0,
}

jog_move_tangential_left = {
    'name': 'jog_move_tangential_left',
    'command_flag' : 3,
    'delta_theta' : 0.0,
    'delta_x': 0.0,
    'delta_y': 0.0,
    'delta_normal': 0.0,
    'delta_tangential': -0.005,
}

jog_move_tangential_right = {
    'name': 'jog_move_tangential_right',
    'command_flag' : 3,
    'delta_theta' : 0.0,
    'delta_x': 0.0,
    'delta_y': 0.0,
    'delta_normal': 0.0,
    'delta_tangential': 0.005,
}

jog_move_normal_up = {
    'name': 'jog_move_normal_left',
    'command_flag' : 3,
    'delta_theta' : 0.0,
    'delta_x': 0.0,
    'delta_y': 0.0,
    'delta_normal': 0.005,
    'delta_tangential': 0.0,
}

jog_move_normal_down = {
    'name': 'jog_move_normal_right',
    'command_flag' : 3,
    'delta_theta' : 0.0,
    'delta_x': 0.0,
    'delta_y': 0.0,
    'delta_normal': -0.005,
    'delta_tangential': 0.0,
}

delta_rotate_hand_left_object_corner = {
    'name': 'delta_rotate_hand_left_object_corner',
    'command_flag' : 1,
    'mode' : 12,
    'delta_theta_relative' : np.pi/10, #np.pi/12,
    'delta_theta_object' : 0.0,
}

delta_rotate_hand_right_object_corner = {
    'name': 'delta_rotate_hand_right_object_corner',
    'command_flag' : 1,
    'mode' : 12,
    'delta_theta_relative' : -np.pi/10, #-np.pi/12,
    'delta_theta_object' : 0.0,
}

delta_rotate_object_left_object_corner = {
    'name': 'delta_rotate_object_left_object_corner',
    'command_flag' : 1,
    'mode' : 12,
    'delta_theta_relative' : 0.0,
    'delta_theta_object' : np.pi/30, #np.pi/12, 
}

delta_rotate_object_right_object_corner = {
    'name': 'delta_rotate_object_right_object_corner',
    'command_flag' : 1,
    'mode' : 12,
    'delta_theta_relative' : 0.0,
    'delta_theta_object' : -np.pi/30, #-np.pi/12, 
}


delta_slide_robot_left_corner_contact = {
    'name': 'delta_slide_robot_left_corner_contact',
    'command_flag' : 1,
    'mode' : 14,
    'delta_theta' : 0.0,
    'delta_s_pivot' : 0.00,
    'delta_s_hand' : -0.015,
}

delta_slide_robot_right_corner_contact = {
    'name': 'delta_slide_robot_right_corner_contact',
    'command_flag' : 1,
    'mode' : 13,
    'delta_theta' : 0.0,
    'delta_s_pivot' : 0.00,
    'delta_s_hand' : 0.015,
}


def on_release(key):
    # try:
    #     keys_held_down.remove(key)
    # except KeyError:
    #     pass
    pass

def on_press(key):

    # keys_held_down.add(key)


    global rm

    global shift_on
    global ctrl_on
    global tab_on

    command_msg_dict = None

    not_a_modifier_key = key!=keyboard.Key.shift and key!=keyboard.Key.ctrl and key!=keyboard.Key.tab
    # shift_on = keyboard.Key.shift in keys_held_down
    # ctrl_on = keyboard.Key.ctrl in keys_held_down
    # tab_on = keyboard.Key.tab in keys_held_down

    if key==keyboard.Key.shift:
        ctrl_on = False
        tab_on = False

        shift_on = not shift_on

        if shift_on:
            print('shift on')
        else:
            print('shift off')

    if key==keyboard.Key.ctrl:
        shift_on = False
        tab_on = False

        ctrl_on = not ctrl_on

        if ctrl_on:
            print('ctrl on')
        else:
            print('ctrl off')

    if key==keyboard.Key.tab:
        shift_on = False
        ctrl_on = False

        tab_on = not tab_on

        if tab_on:
            print('tab on')
        else:
            print('tab off')

    modifier_state = shift_on + 2*ctrl_on + 4*tab_on

    # if not_a_modifier_key:
    #     print('modifier_state: ',modifier_state)
    
    if key == keyboard.Key.esc:
        print('terminating!')
        return False  # stop listener
    if key == keyboard.Key.space:
        command_msg_dict = command_pause
        print('pausing motion!')
    try:
        k = key.char  # single-char keys
        k = k.lower()
    except:
        k = key.name  # other keys



    if k == 'o': #wall_contact_on
        command_msg_dict = wall_contact_on
        print('wall_contact_on')
    elif k == 'p': # wall_contact_off
        command_msg_dict = wall_contact_off
        print('wall_contact_off')

    if modifier_state==0:
        if k == 'c':  # center
            command_msg_dict = absolute_rotate_center
            print('absolute_rotate_center')

        elif k == 'left': # pivot left
            command_msg_dict = delta_rotate_left
            print('delta_rotate_left')
        elif k == 'right': # pivot right
            command_msg_dict = delta_rotate_right
            print('delta_rotate_right')
        
        elif k == 'a': # robot left
            command_msg_dict = delta_slide_robot_left
            print('delta_slide_robot_left')
        elif k == 'd': # robot right
            command_msg_dict = delta_slide_robot_right
            print('delta_slide_robot_right')
       
        elif k == 'q': # object left
            command_msg_dict = delta_slide_pivot_left
            print('delta_slide_pivot_left')
        elif k == 'e': # object right
            command_msg_dict = delta_slide_pivot_right
            print('delta_slide_pivot_right')

        elif k == '1': # robot left
            command_msg_dict = delta_super_slide_robot_left
            print('delta_super_slide_robot_left')
        elif k == '3': # robot right
            command_msg_dict = delta_super_slide_robot_right
            print('delta_super_slide_robot_right')

    if modifier_state==1:
        if k == 'c': # external_line_contact
            command_msg_dict = external_line_contact
            print('external_line_contact')

        elif k == 'left': # pivot left
            command_msg_dict = delta_rotate_corner_left
            print('delta_rotate_corner_left')
        elif k == 'right': # pivot right
            command_msg_dict = delta_rotate_corner_right
            print('delta_rotate_corner_right')

        elif k == 'a': # delta_slide_robot_left_external_line_contact
            command_msg_dict = delta_slide_robot_left_external_line_contact
            print('delta_slide_robot_left_external_line_contact')
        elif k == 'd': # delta_slide_robot_left_external_line_contact
            command_msg_dict = delta_slide_robot_right_external_line_contact
            print('delta_slide_robot_right_external_line_contact')

        elif k == 'q': # delta_slide_object_left_external_line_contact
            command_msg_dict = delta_slide_object_left_external_line_contact
            print('delta_slide_object_left_external_line_contact')
        elif k == 'e': # delta_slide_object_left_external_line_contact
            command_msg_dict = delta_slide_object_right_external_line_contact
            print('delta_slide_object_right_external_line_contact')

        elif k == '1':
            command_msg_dict = delta_slide_robot_left_corner_contact
            print('delta_slide_robot_left_corner_contact')

        elif k == '3':
            command_msg_dict = delta_slide_robot_right_corner_contact
            print('delta_slide_robot_right_corner_contact')

    if modifier_state==2:
        if k == 'left': # jog_rotate_left
            command_msg_dict = jog_rotate_left
            print('jog_rotate_left')

        if k == 'right': # jog_rotate_right
            command_msg_dict = jog_rotate_right
            print('jog_rotate_right')

        if k == 'up': # jog_move_normal_up
            command_msg_dict = jog_move_normal_up
            print('jog_move_normal_up')

        if k == 'down': # jog_move_normal_down
            command_msg_dict = jog_move_normal_down
            print('jog_move_normal_down')

        if k == 'q': # jog_move_tangential_left
            command_msg_dict = jog_move_tangential_left
            print('jog_move_tangential_left')

        if k == 'e': # jog_move_tangential_right
            command_msg_dict = jog_move_tangential_right
            print('jog_move_tangential_right')

        if k == 'w': # jog_move_up
            command_msg_dict = jog_move_up
            print('jog_move_up')

        if k == 's': # jog_move_down
            command_msg_dict = jog_move_down
            print('jog_move_down')

        if k == 'a': # jog_move_left
            command_msg_dict = jog_move_left
            print('jog_move_left')

        if k == 'd': # jog_move_right
            command_msg_dict = jog_move_right
            print('jog_move_right')

    if modifier_state==4:
        if k == 'left':
            command_msg_dict = delta_rotate_hand_left_object_corner
            print('delta_rotate_hand_left_object_corner')

        if k == 'right':
            command_msg_dict = delta_rotate_hand_right_object_corner
            print('delta_rotate_hand_right_object_corner')

        if k == 'a':
            command_msg_dict = delta_rotate_object_left_object_corner
            print('delta_rotate_object_left_object_corner')

        if k == 'd':
            command_msg_dict = delta_rotate_object_right_object_corner
            print('delta_rotate_object_right_object_corner')

    if command_msg_dict is not None:
        rm.pub_barrier_func_control_command(command_msg_dict)


if __name__ == '__main__':
    print('-----------------------------')
    print('COMMAND SUMMARY')

    print('')
    print('Non-Modified Commands!')

    print('c: absolute rotate center')
    print('left arrow: rotate left')
    print('right arrow: rotate right')
    print('a: slide hand left') 
    print('d: slide hand right')
    print('q: slide pivot left')
    print('e: slide pivote right')
    print('1: super slide left')
    print('3: super slide right')

    print('')
    print('Shift (only) Commands!')

    print('shift + c: external_line_contact')
    print('shift + left arrow: rotate corner left')
    print('shift + right arrow: rotate corner right')
    print('shift + a: slide robot left external_line_contact')
    print('shift + d: slide robot right external_line_contact')
    print('shift + q: slide object left external_line_contact')
    print('shift + e: slide object right external_line_contact')


    print('')
    print('Modifier Independent Commands!')

    print('Space: pause')
    print('o: wall contact on')
    print('p: wall contact off')
    print('-----------------------------')

    shift_on = False
    ctrl_on = False
    tab_on = False

    rm = ros_manager()
    rospy.init_node('barrier_func_commands')
    rm.spawn_publisher('/barrier_func_control_command')

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)

    # start to listen on a separate thread
    listener.start()

    my_dummy_window = sg.Window(title='Dummy Window', layout=[[]],margins=(50,50), location=(3500,300))

    my_dummy_window.read()

    listener.join()


    
    







        
