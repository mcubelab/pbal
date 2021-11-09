import copy
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d

from mpl_toolkits.mplot3d import Axes3D
import pdb

from pbal_impedance_inverse_model import PbalImpedanceInverseModel
np.set_printoptions(precision=4, suppress=True)

if __name__ == "__main__":

    # pose target 
    contact_pose_target = np.array([-0.1, 0.0, np.pi/12])


    l_contact, mu_contact = 0.065, 0.15

    # object parameters
    obj_params = dict()
    obj_params['pivot'] = np.array([0., 0.])
    obj_params['mgl'] = 0.6
    obj_params['theta0'] = 0
    obj_params['mu_contact'] = mu_contact
    obj_params['mu_ground'] = 1.0
    obj_params['l_contact'] = l_contact

    # impedance parameters
    param_dict = dict()
    param_dict['obj_params'] = obj_params
    param_dict['contact_pose_target'] = contact_pose_target

    # pbal_model
    pbal_inv_model = PbalImpedanceInverseModel(param_dict)  
    
    # # boundary values
    # npts = 100

    # mu_max = np.vstack([mu_contact * np.ones(npts), 
    #     0.5 * l_contact * np.linspace(-1, 1, npts)])

    # l_max = np.vstack([mu_contact * np.linspace(1, -1, npts), 
    #     0.5 * l_contact * np.ones(npts)])

    # mu_min = np.vstack([-mu_contact * np.ones(npts), 
    #     0.5 * l_contact * np.linspace(1, -1, npts)])

    # l_min = np.vstack([mu_contact * np.linspace(-1, 1, npts), 
    #     -0.5 * l_contact * np.ones(npts)])

    # boundary_pts = np.hstack([mu_max, l_max, mu_min, l_min])


    # plot generators
    scale = 30
    generator_list = [[scale, scale*mu_contact, scale*0.5*l_contact], 
        [scale, -scale*mu_contact, 0.5*scale*l_contact], 
        [scale, -scale*mu_contact, -0.5*scale*l_contact], 
        [scale, scale*mu_contact, -0.5*scale*l_contact]]

    generator_list_wrap = generator_list + [generator_list[0]]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i, generator in enumerate(generator_list):

        fn = [0, generator[0], generator_list_wrap[i+1][0]]
        # negative_fn = [0, -generator[0], -generator_list_wrap[i+1][0]]
        ff = [0, generator[1], generator_list_wrap[i+1][1]]
        tau = [0, generator[2], generator_list_wrap[i+1][2]]

        tri = mpl_toolkits.mplot3d.art3d.Poly3DCollection([list(zip(ff, tau, fn))])
        # tri2 = mpl_toolkits.mplot3d.art3d.Poly3DCollection([list(zip(ff, tau, negative_fn))])

        ax.add_collection3d(tri)
        # ax.add_collection3d(tri2)

    ax.hold(True)


    # # plot se plane
    # Ase, bse = pbal_inv_model.static_equilbrium_contact()
    # ff, tt =  np.meshgrid([np.linspace(-mu_contact, mu_contact, 50), 
    #     0.5 * np.linspace(-l_contact, l_contact, 50)])

    # nn = bse - ase[2] * tt - ase[1] * ff



    # fig0, ax0 = plt.subplots(1,1)

    # cost_vec = [[0., mu_contact, 0.5*l_contact],
    #     [0., -mu_contact, 0.5*l_contact], 
    #     [0., -mu_contact, -0.5*l_contact], 
    #     [0., mu_contact, -0.5*l_contact]]

    # feed forward wrench
    ff_robot_wrench = pbal_inv_model.solve_linear_program_aux(scale)
    ff_contact_wrench = np.dot(pbal_inv_model.pbal_helper.contact2robot(contact_pose_target), 
        ff_robot_wrench[:3])

    color_list = ['r', 'g', 'y', 'c']
    null_space_corners = pbal_inv_model.solve_linear_program_control_nullspace(
        scale)

    for color, null_space_corner in zip(color_list, null_space_corners):
        ax.plot([null_space_corner[1]], [null_space_corner[2]],
            [null_space_corner[0]], color + '*', markersize=20)

    ax.plot([ff_contact_wrench[1]], [ff_contact_wrench[2]],
            [ff_contact_wrench[0]], 'kd', markersize=20)
        
        # boundary_wrench_robot.append(robot_wrench)


    # boundary_wrench_robot_array = np.array(boundary_wrench_robot)

    # fig1, ax1 = plt.subplots(1,1)
    # # ax1.plot(boundary_wrench_robot_array[:, 1], boundary_wrench_robot_array[:, 2])

    # ax1.scatter(boundary_wrench_robot_array[:npts, 1], 
    #     boundary_wrench_robot_array[:npts, 2], c='r', edgecolor='r')

    # ax1.scatter(boundary_wrench_robot_array[npts:2*npts, 1], 
    #     boundary_wrench_robot_array[npts:2*npts, 2], c='g', edgecolor='g')

    # ax1.scatter(boundary_wrench_robot_array[2*npts:3*npts, 1], 
    #     boundary_wrench_robot_array[2*npts:3*npts, 2], c='b', edgecolor='b')

    # ax1.scatter(boundary_wrench_robot_array[3*npts:4*npts, 1], 
    #     boundary_wrench_robot_array[3*npts:4*npts, 2], c='k', edgecolor='k')

    # fig2 = plt.figure()
    # ax2 = fig2.add_subplot(111, projection='3d')
    # ax2.plot(boundary_wrench_robot_array[:, 0], boundary_wrench_robot_array[:, 1], 
    #     boundary_wrench_robot_array[:, 2])
    # ax.scatter(boundary_wrench_robot_array[:npts, 1], boundary_wrench_robot_array[:npts, 2], 
    #     boundary_wrench_robot_array[:npts, 0], c='r', edgecolor='r')
    # ax.scatter(boundary_wrench_robot_array[npts:2*npts, 1], boundary_wrench_robot_array[npts:2*npts, 2], 
    #     boundary_wrench_robot_array[npts:2*npts, 0], c='g', edgecolor='g')

    # ax.scatter(boundary_wrench_robot_array[2*npts:3*npts, 1], boundary_wrench_robot_array[2*npts:3*npts, 2], 
    #     boundary_wrench_robot_array[2*npts:3*npts, 0], c='b', edgecolor='b')

    # ax.scatter(boundary_wrench_robot_array[3*npts:4*npts, 1], boundary_wrench_robot_array[3*npts:4*npts, 2], 
    #     boundary_wrench_robot_array[3*npts:4*npts, 0], c='k', edgecolor='k')

    # ax.set_xlim([-scale, scale])
    # ax.set_ylim([-scale, scale])
    # ax.set_zlim([0, scale])

    plt.show()