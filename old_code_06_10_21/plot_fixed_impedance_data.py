import pickle
import os
import pdb

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

from pbal_impedance_forward_model import PbalImpedanceForwardModel

def plot_all_data(directory, axes):

    for root, dirs, files, in os.walk(directory):
        lines = np.linspace(0, 1, len(files))
        colors = [cm.jet(x) for x in lines]
        for file, color in zip(files, colors):
            print(file)
            if file.endswith(".pickle"):
                with open(os.path.join(directory, file), 'rb') as handle:
                    data = pickle.load(handle)
                    state = data['estimated']
                    param_dict = data['param_dict']

                    s_list, theta_list = [], []
                    for statei in state:
                        s_list.append(statei[1])
                        theta_list.append(statei[2])

                    axes.plot(s_list, theta_list, color=color, marker='o')
             

    return param_dict

def plot_model(param_dict, d, ax):

	# create obj
    pbal_impedance_fwd = PbalImpedanceForwardModel(param_dict)

    percent_d = 1.0
    smin = percent_d * d
    smax = -percent_d * d
    sarray = np.linspace(smin, smax, 100)

    mu_contact_range = np.linspace(0.5*param_dict['obj_params']['mu_contact'], 
        1.5*param_dict['obj_params']['mu_contact'], 5)

    for mu_contact in mu_contact_range:

        pbal_impedance_fwd.pbal_helper.mu_contact = mu_contact

        theta_boundary_array, s_left_array = pbal_impedance_fwd.find_all_sticking_patch_values_theta_sweep(d, 0.,0)
        _, s_right_array = pbal_impedance_fwd.find_all_sticking_patch_values_theta_sweep(d, 0.,1)

        # theta_boundary_array, s_left_array = pbal_impedance_fwd.find_all_sticking_patch_values_theta_sweep(d, 0.,0)
        # _, s_right_array = pbal_impedance_fwd.find_all_sticking_patch_values_theta_sweep(d, 0.,1)


        # theta_boundary_array, s_left_array = pbal_impedance_fwd.find_all_sticking_patch_values_theta_sweep(d, 0.,0)
        # _, s_right_array = pbal_impedance_fwd.find_all_sticking_patch_values_theta_sweep(d, 0.,1)


        s_boundary_array, theta_left_torque_array = pbal_impedance_fwd.find_all_sticking_patch_values_s_sweep(d, 0.,2)
        _, theta_right_torque_array = pbal_impedance_fwd.find_all_sticking_patch_values_s_sweep(d, 0.,3)

        s_list = []
        theta_list = []
        mask_list = []
        for s in sarray:
            eq_angle_list = pbal_impedance_fwd.find_all_equilibrium_angles(d, s)
            for theta_eq in eq_angle_list:

                is_in_cone, violation = pbal_impedance_fwd.wrench_cone_check_with_imepedance_contact(
                    np.array([d, s, theta_eq]))

                s_list.append(s)
                theta_list.append(theta_eq)
                mask_list.append(is_in_cone)

        is_sticking_mask = np.array(mask_list, dtype=bool)
        ax.scatter(np.array(s_list)[is_sticking_mask],
                   np.array(theta_list)[is_sticking_mask],
                   color='g')
        ax.scatter(np.array(s_list)[~is_sticking_mask],
                   np.array(theta_list)[~is_sticking_mask],
                   color='r')
        ax.set_xlabel('Sliding position')
        ax.set_ylabel('Object Angle')

        frictionless_equilbrium_pose = pbal_impedance_fwd.find_frictionless_equilibrium(
            d, 0., 0.)
        ax.plot(frictionless_equilbrium_pose[1],
                frictionless_equilbrium_pose[2],
                'k*',
                markersize=15)

        ax.plot(s_left_array,
            theta_boundary_array,
           color='c')
        ax.plot(s_right_array,
           theta_boundary_array,
           color='m')
        ax.plot(s_boundary_array,
           theta_left_torque_array,
           color='b')
        ax.plot(s_boundary_array,
           theta_right_torque_array,
           color='g')

        # find boundaries
        s_guess, theta_guess = 0, 0
        for i in range(4):
            contact_pose_boundary = pbal_impedance_fwd.find_sticking_srange(
                d, i, percent_d=percent_d)
            if not contact_pose_boundary:
                print("no boundary found")
                continue
            if i < 2:
                ax.plot(contact_pose_boundary[0][1],
                        contact_pose_boundary[0][2],
                        'ks',
                        markersize=15)
            else:  # # find boundaries
                ax.plot(contact_pose_boundary[0][1],
                        contact_pose_boundary[0][2],
                        'kd',
                        markersize=15)

    plt.xlim([-0.04, 0.04])
    plt.ylim([-np.pi/6, np.pi/6])




if __name__ == "__main__":

    # # object parameters
    # obj_params = dict()
    # obj_params['pivot'] = np.array([0., 0.])
    # obj_params['mgl'] = 0.75
    # obj_params['theta0'] = 0.
    # obj_params['mu_contact'] = 0.2
    # obj_params['mu_ground'] = 0.7
    # obj_params['l_contact'] = 0.065

    # # impedance parameters
    # param_dict=dict()
    # param_dict['obj_params'] = obj_params
    # param_dict['impedance_target'] = np.array([-0.00, 0.08, 0.])
    # param_dict['impedance_stiffness'] = np.array([1000, 1000, 30.0])
    fig, ax = plt.subplots(1,1)
    param_dict = plot_all_data('./Fixed_Impedance_Data', ax)
    print(param_dict)
    plot_model(param_dict, -0.11, ax)
    plt.show()



   