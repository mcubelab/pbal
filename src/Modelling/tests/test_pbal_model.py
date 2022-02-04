import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pdb

from pbal_impedance_forward_model import PbalImpedanceForwardModel
from pbal_impedance_inverse_model import PbalImpedanceInverseModel


if __name__ == "__main__":

    # pose target 
    contact_pose_target = np.array([-0.1, 0.0, np.pi / 4])

    # object parameters
    obj_params = dict()
    obj_params['pivot'] = np.array([0., 0.])
    obj_params['mgl'] = .6
    obj_params['theta0'] = np.pi/12
    obj_params['mu_contact'] = 0.15
    obj_params['mu_ground'] = 1.0
    obj_params['l_contact'] = 0.065

    # impedance parameters
    param_dict = dict()
    param_dict['obj_params'] = obj_params
    param_dict['contact_pose_target'] = contact_pose_target
    
    # find force
    pbal_impedance_inv = PbalImpedanceInverseModel(param_dict)
    x=pbal_impedance_inv.solve_linear_program_aux(20)
    robot_wrench = x[:3]
    
    fig, ax = plt.subplots(1, 1)
    pbal_impedance_inv.plot_state(robot_wrench, ax, 20)
    ax.axis('equal')

    print(pbal_impedance_inv.pbal_helper.forward_kin(contact_pose_target))
    print(robot_wrench)
    
    # compute impedance target for fixed stiffness
    impedance_stiffness=np.array([100, 100, 100])
    impedance_target_delta= robot_wrench / impedance_stiffness
    impedance_target=pbal_impedance_inv.pbal_helper.forward_kin(
        contact_pose_target) + impedance_target_delta

    print(impedance_target)
    # forward model
    param_dict['impedance_stiffness'] = impedance_stiffness
    param_dict['impedance_target'] = impedance_target

    pbal_impedance_fwd=PbalImpedanceForwardModel(param_dict)
    
    fig1, ax1=plt.subplots(1, 1)
    pbal_impedance_fwd.plot_state(pbal_impedance_inv.pbal_helper.forward_kin(
        contact_pose_target), ax1)
    ax1.axis('equal')

    # find eq angle for fixed d & s
    d = contact_pose_target[0]
    percent_d = 1.0
    smin = percent_d * d
    smax = -percent_d * d
    sarray = np.linspace(smin, smax, 100)

    s_list = []
    theta_list = []
    mask_list = []
    fig2, ax2 = plt.subplots(1, 1)
    for s in sarray:
        eq_angle_list = pbal_impedance_fwd.find_all_equilibrium_angles(d, s)
        for theta_eq in eq_angle_list:

            is_in_cone, violation = pbal_impedance_fwd.wrench_cone_check_with_imepedance_contact(
                np.array([d, s, theta_eq]))

            s_list.append(s)
            theta_list.append(theta_eq)
            mask_list.append(is_in_cone)

    is_sticking_mask = np.array(mask_list, dtype=bool)
    ax2.scatter(np.array(s_list)[is_sticking_mask],
               np.array(theta_list)[is_sticking_mask],
               color='g')
    ax2.scatter(np.array(s_list)[~is_sticking_mask],
               np.array(theta_list)[~is_sticking_mask],
               color='r')
    ax2.set_xlabel('Sliding position')
    ax2.set_ylabel('Object Angle')


    frictionless_equilbrium_pose = pbal_impedance_fwd.find_frictionless_equilibrium(
        d, 0., 0.)
    ax2.plot(frictionless_equilbrium_pose[1],
            frictionless_equilbrium_pose[2],
            'k*',
            markersize=15)

    # find boundaries
    s_guess, theta_guess = 0, 0
    for i in range(4):
        contact_pose_boundary = pbal_impedance_fwd.find_sticking_srange(
            d, i, percent_d=percent_d)
        if not contact_pose_boundary:
            print("no boundary found")
            continue
        if i < 2:
            ax2.plot(contact_pose_boundary[0][1],
                    contact_pose_boundary[0][2],
                    'ks',
                    markersize=15)
        else:  # # find boundaries
            ax2.plot(contact_pose_boundary[0][1],
                    contact_pose_boundary[0][2],
                    'kd',
                    markersize=15)

    ax2.plot(contact_pose_target[1], contact_pose_target[2],
        'co', markersize=15)
    plt.show()