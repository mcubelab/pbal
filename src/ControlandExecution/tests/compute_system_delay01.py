#!/usr/bin/env python
import matplotlib.pyplot as plt
import json
import os
import sys
import numpy as np
import sys
import inspect

currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, gparentdir)


def sine_wave(t, amplitude_in, period_in):
    return amplitude_in*np.sin(2*np.pi*t/period_in)


def cos_wave(t, amplitude_in, period_in):
    return amplitude_in*np.cos(2*np.pi*t/period_in)


if __name__ == '__main__':

    with open('RobotDelayData15.txt', 'r') as f:
        data_dict = json.loads(f.read())

    impedance_target_horizontal_pose_list = data_dict["impedance_target_horizontal_pose_list"]
    impedance_target_vertical_pose_list = data_dict["impedance_target_vertical_pose_list"]
    measured_horizontal_pose_list = data_dict["measured_horizontal_pose_list"]
    measured_vertical_pose_list = data_dict["measured_vertical_pose_list"]
    measured_horiztonal_force_list = data_dict["measured_horiztonal_force_list"]
    measured_vertical_force_list = data_dict["measured_vertical_force_list"]
    tlist = data_dict["tlist"]
    IMPEDANCE_STIFFNESS_LIST = data_dict["IMPEDANCE_STIFFNESS_LIST"]
    amplitude = data_dict["amplitude"]
    period = data_dict["period"]
    num_periods = data_dict["num_periods"]
    tmax = data_dict["tmax"]
    tmax_margin = data_dict["tmax_margin"]
    base_horizontal_pose = data_dict["base_horizontal_pose"]
    base_vertical_pose = data_dict["base_vertical_pose"]

    min_index = 0
    while tlist[min_index] < period:
        min_index += 1

    max_index = len(tlist)-1
    while tlist[max_index] > tmax:
        max_index -= 1

    tlist = np.array(tlist[min_index:max_index])
    impedance_target_horizontal_pose_list = np.array(
        impedance_target_horizontal_pose_list[min_index:max_index])
    measured_horizontal_pose_list = np.array(
        measured_horizontal_pose_list[min_index:max_index])
    impedance_target_vertical_pose_list = np.array(
        impedance_target_vertical_pose_list[min_index:max_index])
    measured_vertical_pose_list = np.array(
        measured_vertical_pose_list[min_index:max_index])
    measured_horiztonal_force_list = np.array(
        measured_horiztonal_force_list[min_index:max_index])
    measured_vertical_force_list = np.array(
        measured_vertical_force_list[min_index:max_index])

    sine_t = sine_wave(tlist, 1, period)
    cos_t = cos_wave(tlist, 1, period)
    const_t = np.array([1]*len(tlist))

    regression_matrix = np.vstack([cos_t, sine_t, const_t])

    Coeff_Vec_horz = np.linalg.lstsq(np.transpose(
        regression_matrix), measured_horiztonal_force_list)[0]
    Coeff_Vec_vert = np.linalg.lstsq(np.transpose(
        regression_matrix), measured_vertical_force_list)[0]

    delay_horz = np.arctan2(-Coeff_Vec_horz[0],
                            Coeff_Vec_horz[1])/(2*np.pi/period)
    amplitude_horz = np.sqrt(Coeff_Vec_horz[0]**2+Coeff_Vec_horz[1]**2)
    phase_horz = np.arctan2(-Coeff_Vec_horz[0], Coeff_Vec_horz[1])*180.0/np.pi

    delay_vert = np.arctan2(
        Coeff_Vec_vert[1], Coeff_Vec_vert[0])/(2*np.pi/period)
    amplitude_vert = np.sqrt(Coeff_Vec_vert[0]**2+Coeff_Vec_vert[1]**2)
    phase_vert = np.arctan2(Coeff_Vec_vert[1], Coeff_Vec_vert[0])*180.0/np.pi

    fig, axs = plt.subplots(1, 4)
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
    axs[2].plot(tlist, Coeff_Vec_horz[2] + sine_wave(tlist -
                delay_horz, amplitude_horz, period), color='r')

    axs[3].plot(tlist, measured_vertical_force_list, color='b')
    axs[3].plot(tlist, Coeff_Vec_vert[2] + cos_wave(tlist -
                delay_vert, amplitude_vert, period), color='r')

    print("horizontal delay (s) = ", delay_horz)
    print("vertical delay (s) = ", delay_vert)
    print("phase horz (deg) = ", phase_horz)
    print("phase vert (deg) = ", phase_vert)

    plt.show()
