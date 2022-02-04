import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pdb

from pbal_impedance_inverse_model import PbalImpedanceInverseModel

if __name__ == "__main__":

    # object parameters
    obj_params = dict()
    obj_params['pivot'] = np.array([0., 0.])
    obj_params['mgl'] = 0.1
    obj_params['theta0'] = 0
    obj_params['mu_contact'] = 0.2
    obj_params['mu_ground'] = 1.0
    obj_params['l_contact'] = 0.065

    # position control parameters
    param_dict = dict()
    param_dict['obj_params'] = obj_params
    param_dict['contact_pose_target'] = np.array([-0.1, 0.00, -np.pi / 6])

    # inverse model
    pbal_impedance_inv = PbalImpedanceInverseModel(param_dict)

    Nmax = 40  # maximum force
    dnom = -0.11  # nominal value of d

    NPTS = 30
    theta_vec, s_vec = np.linspace(-np.pi / 3, np.pi / 3,
                                   NPTS), np.linspace(-0.04, 0.04, NPTS)
    theta_grid, s_grid = np.meshgrid(theta_vec, s_vec)
    is_feasible = np.zeros([NPTS, NPTS, 5])

    for i in range(NPTS):
        for j in range(NPTS):
            for k in range(5):
                # print(s_grid[i, j], (180/np.pi) * theta_grid[i,j])
                # update current pose
                current_pose = np.array([dnom, s_grid[i, j], theta_grid[i, j]])
                pbal_impedance_inv.contact_pose_target = current_pose

                # solve for wrench
                robot_wrench = pbal_impedance_inv.solve_linear_program_mode(
                    Nmax, mode=k - 1)

                if robot_wrench is not None:
                    is_feasible[i, j, k] = 1

    fig, axs = plt.subplots(1, 5)
    titles = [
        'Sticking', 'Robot sliding right', 'Robot sliding left',
        'Pivot sliding left', 'Pivot sliding right'
    ]
    is_feasible_sticking = is_feasible[:, :, 0]

    for mode, ax in enumerate(axs):
        is_feasible_mode = is_feasible[:, :, mode]
        mask_feasible = (is_feasible_mode == 1) * (is_feasible_sticking == 1)
        ax.scatter(s_grid[is_feasible_mode == 1],
                   (180 / np.pi) * theta_grid[is_feasible_mode == 1],
                   color='y')
        ax.scatter(s_grid[mask_feasible],
                   (180 / np.pi) * theta_grid[mask_feasible],
                   color='g')
        ax.scatter(s_grid[is_feasible_mode == 0],
                   (180 / np.pi) * theta_grid[is_feasible_mode == 0],
                   color='r')

        if mode == 0:

            # one boundary
            theta_boundary1 = theta_vec[theta_vec > obj_params['theta0']]
            s_pivot_boundary1 = -dnom * np.tan(theta_boundary1 - np.arctan(
                obj_params['mu_ground'])) - obj_params['l_contact'] / 2
            ax.plot(s_pivot_boundary1, (180 / np.pi) * theta_boundary1, 'k')

            # other boundary
            theta_boundary2 = theta_vec[theta_vec < obj_params['theta0']]
            s_pivot_boundary2 = -dnom * np.tan(theta_boundary2 + np.arctan(
                obj_params['mu_ground'])) + obj_params['l_contact'] / 2
            ax.plot(s_pivot_boundary2, (180 / np.pi) * theta_boundary2, 'k')

            ax.vlines(
                -obj_params['mu_contact'] * dnom + obj_params['l_contact'] / 2,
                -(180 / np.pi) * np.pi / 3, (180 / np.pi) * np.pi / 3)
            ax.vlines(
                obj_params['mu_contact'] * dnom - obj_params['l_contact'] / 2,
                -(180 / np.pi) * np.pi / 3, (180 / np.pi) * np.pi / 3)

            ax.hlines((180 / np.pi) * (np.arctan(obj_params['mu_contact']) +
                                       np.arctan(obj_params['mu_ground'])),
                      -0.04, 0.04)

            ax.hlines(
                -(180 / np.pi) * (np.arctan(obj_params['mu_contact']) +
                                  np.arctan(obj_params['mu_ground'])), -0.04,
                0.04)

        if mode == 1:

            ax.hlines(
                -(180 / np.pi) * (np.arctan(obj_params['mu_contact']) -
                                  np.arctan(obj_params['mu_ground'])), -0.04,
                0.04)

            ax.vlines(
                obj_params['mu_contact'] * dnom + obj_params['l_contact'] / 2,
                -(180 / np.pi) * np.pi / 3,
                (180 / np.pi) * obj_params['theta0'])

            # other boundary
            theta_boundary2 = theta_vec[theta_vec < obj_params['theta0']]
            s_pivot_boundary2 = -dnom * np.tan(theta_boundary2 + np.arctan(
                obj_params['mu_ground'])) + obj_params['l_contact'] / 2
            ax.plot(s_pivot_boundary2, (180 / np.pi) * theta_boundary2, 'k')

        if mode == 2:

            ax.hlines((180 / np.pi) * (np.arctan(obj_params['mu_contact']) -
                                       np.arctan(obj_params['mu_ground'])),
                      -0.04, 0.04)

            ax.vlines(
                -obj_params['mu_contact'] * dnom - obj_params['l_contact'] / 2,
                (180 / np.pi) * obj_params['theta0'],
                (180 / np.pi) * np.pi / 3)

            # one boundary
            theta_boundary1 = theta_vec[theta_vec > obj_params['theta0']]
            s_pivot_boundary1 = -dnom * np.tan(theta_boundary1 - np.arctan(
                obj_params['mu_ground'])) - obj_params['l_contact'] / 2
            ax.plot(s_pivot_boundary1, (180 / np.pi) * theta_boundary1, 'k')

        if mode == 3:

            ax.hlines((180 / np.pi) * (np.arctan(obj_params['mu_contact']) -
                                       np.arctan(obj_params['mu_ground'])),
                      -0.04, 0.04)
         
            # one boundary
            theta_boundary1 = theta_vec[theta_vec > obj_params['theta0']]
            s_pivot_boundary1 = -dnom * np.tan(theta_boundary1 + np.arctan(
                obj_params['mu_ground'])) - obj_params['l_contact'] / 2
            ax.plot(s_pivot_boundary1, (180 / np.pi) * theta_boundary1, 'k')


        if mode == 4:

            ax.hlines(
                -(180 / np.pi) * (np.arctan(obj_params['mu_contact']) -
                                  np.arctan(obj_params['mu_ground'])), -0.04,
                0.04)
   
            # other boundary
            theta_boundary2 = theta_vec[theta_vec < obj_params['theta0']]
            s_pivot_boundary2 = -dnom * np.tan(theta_boundary2 - np.arctan(
                obj_params['mu_ground'])) + obj_params['l_contact'] / 2
            ax.plot(s_pivot_boundary2, (180 / np.pi) * theta_boundary2, 'k')

        ax.hlines((180 / np.pi) * obj_params['theta0'], -0.04, 0.04)
        ax.set_xlabel('S [m]')
        ax.set_ylabel('Theta [deg]')
        ax.set_xlim([-0.04, 0.04])
        ax.set_title(titles[mode])

    plt.show()
