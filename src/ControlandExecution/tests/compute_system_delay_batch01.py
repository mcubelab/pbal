#!/usr/bin/env python
import matplotlib.pyplot as plt
import json
import os
import sys
import pdb
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


def analyze_data_file(filename, plot_data=True):

    with open(filename, 'r') as f:
        data_dict = json.loads(f.read())

    impedance_target_horizontal_pose_list = data_dict["impedance_target_horizontal_pose_list"]
    impedance_target_vertical_pose_list = data_dict["impedance_target_vertical_pose_list"]
    measured_horizontal_pose_list = data_dict["measured_horizontal_pose_list"]
    measured_vertical_pose_list = data_dict["measured_vertical_pose_list"]
    measured_horiztonal_force_list = data_dict["measured_horiztonal_force_list"]
    measured_vertical_force_list = data_dict["measured_vertical_force_list"]
    filter_params_ = data_dict["filter_params_"]
    tlist = data_dict["tlist"]
    period = data_dict["period"]
    tmax = data_dict["tmax"]

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

    if plot_data:

        fig, axs = plt.subplots(4, 1, figsize=(20, 10))
        axs[0].plot(tlist, impedance_target_horizontal_pose_list,
                    marker='o', color='b', markersize=1, label='target')
        axs[0].plot(tlist, measured_horizontal_pose_list,
                    marker='o', color='r', markersize=1, label='measured')

        axs[1].plot(tlist, impedance_target_vertical_pose_list,
                    marker='o', color='b', markersize=1, label='target')
        axs[1].plot(tlist, measured_vertical_pose_list,
                    marker='o', color='r', markersize=1, label='measured')

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

    return 1./period, delay_horz, delay_vert, phase_horz, phase_vert, filter_params_


if __name__ == '__main__':

    (freq_list, delay_horz_list, delay_vert_list,
     phase_horz_list, phase_vert_list) = [], [], [], [], []
    for filename in os.listdir():
        if filename.lower().endswith('.txt'):

            print("==========="+filename+"===============")

            (freq, delay_horz, delay_vert, phase_horz, phase_vert,
                filter_params_) = analyze_data_file(filename, plot_data=False)

            print("Freq (hz) = ", freq)
            # print("horizontal delay (s) = ", delay_horz)
            # print("vertical delay (s) = ", delay_vert)
            # print("phase horz (deg) = ", phase_horz)
            # print("phase vert (deg) = ", phase_vert)
            print("filter params = ", filter_params_)

            if filter_params_ == 0.005:

                freq_list.append(freq)
                delay_horz_list.append(delay_horz)
                delay_vert_list.append(delay_vert)
                phase_horz_list.append(phase_horz)
                phase_vert_list.append(phase_vert)

    fig, axs = plt.subplots(2, 1, figsize=(20, 10))
    axs[0].scatter(freq_list, delay_horz_list, color='blue',
                   marker='o', label='horizontal')
    axs[0].scatter(freq_list, delay_vert_list, color='red',
                   marker='o', label='vertical')
    axs[0].set_title('Filter Params = 0.005')
    axs[0].set_xlabel('freq (hz)')
    axs[0].set_ylabel('latency (s)')
    axs[0].legend()

    axs[1].scatter(freq_list, phase_horz_list, color='blue',
                   marker='o', label='horizontal')
    axs[1].scatter(freq_list, phase_vert_list, color='red',
                   marker='o', label='vertical')
    axs[1].set_xlabel('freq (hz)')
    axs[1].set_ylabel('phase (deg)')
    axs[1].legend()

    plt.show()
