#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(os.path.dirname(currentdir)))

import json
import matplotlib.pyplot as plt
import numpy as np
import time

from Helpers.ros_manager import ros_manager

import rospy

#function that defines a triangle wave that oscillates between  +/- amplitude, has a full period of period_in
#t is the time that you are evaluating the triangle wave function at
def triangle_wave(t, amplitude_in, period_in):
    t_transform = (t/period_in) % 1.0

    return (amplitude_in*4.0*t_transform*(t_transform <= .25) +
            amplitude_in*(2.0-4.0*t_transform)*(t_transform > .25)*(t_transform <= .75) +
            amplitude_in*(4.0*t_transform-4.0)*(t_transform > .75))

#function that defines a sine wave that oscillates between  +/- amplitude, has a full period of period_in
#t is the time that you are evaluating the sine wave function at
def sine_wave(t, amplitude_in, period_in):
    return amplitude_in*np.sin(2*np.pi*t/period_in)

#function that defines a cosine wave that oscillates between  +/- amplitude, has a full period of period_in
#t is the time that you are evaluating the cosine wave function at
def cos_wave(t, amplitude_in, period_in):
    return amplitude_in*(np.cos(2*np.pi*t/period_in)-1.0)

#this function evaluates to x if x is on the interval [x_min,x_max], otherwise it projects onto the interval
def limit_range(x, x_min, x_max):
    return x_min*(x <= x_min)+x*(x > x_min)*(x < x_max)+x_max*(x >= x_max)

if __name__ == '__main__':

    #path where the recorded data is stored
    save_path = os.path.join(currentdir, '..', '..', '..', 'data', 'BlockedData')

    record_on = False
    plot_on = True

    #this starts the ros node
    rm = ros_manager()
    rospy.init_node('test_impedance_control')

    rm.subscribe_to_list(['/end_effector_sensor_in_end_effector_frame',
                          '/end_effector_sensor_in_world_manipulation_frame',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/ee_pose_in_base_from_franka_publisher'])

    rm.impedance_mode_helper()

    rm.wait_for_necessary_data()
    rm.ee_pose_in_base_unpack()


    #the stiffness values used for the cartesian impedance Fx,Fy,Fz,Tx,Ty,Tz
    #the stiffness matrix is diagonal, we are only defining the diagonal entries
    IMPEDANCE_STIFFNESS_LIST = [3000, 3000, 3000, 100, 100, 100]

    #the damping values for the cartesian impedance, also a diagonal matrix
    IMPEDANCE_DAMPING_LIST = [0.5 * np.sqrt(k) for k in IMPEDANCE_STIFFNESS_LIST] 

    rm.initialize_impedance_mode()
    rm.set_cart_impedance_stiffness(IMPEDANCE_STIFFNESS_LIST,IMPEDANCE_DAMPING_LIST)

    #at the end of a ros loop, we call rate.sleep so that the node sleeps a while and isn't constantly running
    #larger numbers means it sleeps for less time (the value is in Hz)
    rate = rospy.Rate(100.)

    #makes a copy of the current pose
    adjusted_current_pose = list(rm.ee_pose_in_base_list)

    #pulls out the x and z values of the end effector pose (used for centroid of path for robot)
    base_horizontal_pose = adjusted_current_pose[0]
    base_vertical_pose = adjusted_current_pose[2]

    #defines the filename to store recorded data

    # fname_in = 'RobotDelayData34.txt'
    # move_type = 'BlockedData'

    fname_in = 'FreeMoveData13.txt'
    move_type = 'FreeMoveData'


    # motion schedule
    amplitude = 0.01
    period =2.0
    num_periods = 10.0
    tmax = period*num_periods

    #extra buffer time at the end so the data of the time range we care about doesn't suck
    tmax_margin = 3.0

    #lists used for recording measured data
    tlist = []
    measured_horizontal_pose_list = []
    measured_vertical_pose_list = []

    impedance_target_horizontal_pose_list = []
    impedance_target_vertical_pose_list = []

    measured_horiztonal_force_list = []
    measured_vertical_force_list = []

    print('starting control loop')

    # start loop
    start_time = time.time()+3.0

    #number of iterations of the loop
    ct = 0.
    while not rospy.is_shutdown():
        rm.unpack_all()

        ct+= 1.

        # current time
        t = time.time() - start_time

        # end if t > tmax
        if t > tmax+tmax_margin:
            break

        #defines the horizontal and vertical locations of the impedance target
        #puts the location on a circle, with radius and speed defined by the motion schedule
        horizontal_impedance_target = base_horizontal_pose + sine_wave(limit_range(t, 0.0, tmax), amplitude, period)
        vertical_impedance_target = base_vertical_pose + cos_wave(limit_range(t, 0.0, tmax), amplitude, period)

        # move to next position on schedule
        adjusted_current_pose[0] = horizontal_impedance_target
        adjusted_current_pose[2] = vertical_impedance_target 

        rm.set_cart_impedance_pose(adjusted_current_pose)

        #record all measurements
        tlist.append(t)
        impedance_target_horizontal_pose_list.append(horizontal_impedance_target)
        impedance_target_vertical_pose_list.append(vertical_impedance_target)

        measured_horizontal_pose_list.append(rm.ee_pose_in_world_manipulation_list[0])
        measured_vertical_pose_list.append(rm.ee_pose_in_world_manipulation_list[1])

        measured_horiztonal_force_list.append(rm.measured_world_manipulation_wrench[0])
        measured_vertical_force_list.append(rm.measured_world_manipulation_wrench[1])

        rate.sleep()

    print("Average runtime: ", (tmax + tmax_margin)/ct)

    print('control loop completed')

    #put all measurements in one massive dictionary so it can be sent to a json file
    data_dict = {
        "impedance_target_horizontal_pose_list": impedance_target_horizontal_pose_list,
        "impedance_target_vertical_pose_list": impedance_target_vertical_pose_list,
        "measured_horizontal_pose_list": measured_horizontal_pose_list,
        "measured_vertical_pose_list": measured_vertical_pose_list,
        "measured_horiztonal_force_list": measured_horiztonal_force_list,
        "measured_vertical_force_list": measured_vertical_force_list,
        "tlist": tlist,
        "IMPEDANCE_STIFFNESS_LIST": IMPEDANCE_STIFFNESS_LIST,
        "IMPEDANCE_DAMPING_LIST": IMPEDANCE_DAMPING_LIST,
        "amplitude": amplitude,
        "period": period,
        "num_periods": num_periods,
        "tmax": tmax,
        "tmax_margin": tmax_margin,
        "base_horizontal_pose": base_horizontal_pose,
        "base_vertical_pose": base_vertical_pose,
    }

    if record_on:
        ###converts measurement dictionary into json
        data_str = json.dumps(data_dict)

        ###saves measurements in file
        dir_path = os.path.dirname(os.path.abspath(__file__))
        save_path = os.path.join(dir_path, '..', '..', '..', 'data', move_type)

        with open(os.path.join(save_path, fname_in), 'w') as f:
            f.write(data_str)

    if plot_on:
        ###plots some stuff
        fig, axs = plt.subplots(1, 3)
        axs[0].plot(tlist, impedance_target_horizontal_pose_list,
                    marker='o', color='b', label='target')
        axs[0].plot(tlist, measured_horizontal_pose_list,
                    marker='o', color='r', label='measured')

        axs[1].plot(tlist, impedance_target_vertical_pose_list,
                    marker='o', color='b', label='target')
        axs[1].plot(tlist, measured_vertical_pose_list,
                    marker='o', color='r', label='measured')

        axs[0].set_xlabel('time (s)')
        axs[1].set_xlabel('time (s)')

        axs[0].set_ylabel('position (m)')
        axs[1].set_ylabel('position (m)')

        axs[0].legend()
        axs[1].legend()

        axs[0].set_title('Horizontal')
        axs[1].set_title('Vertical')

        axs[2].plot(tlist, measured_horiztonal_force_list, color='b')
        axs[2].plot(tlist, measured_vertical_force_list, color='r')

        plt.show()

