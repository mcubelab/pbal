import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pdb

from pbal_helper import PbalHelper


class PbalImpedanceForwardModel(object):
    def __init__(self, param_dict):

        # object parameters
        self.pbal_helper = PbalHelper(param_dict['obj_params'])

        # impedance parameters
        self.impedance_target = param_dict['impedance_target']
        self.impedance_stiffness = param_dict['impedance_stiffness']

    def impedance_model_robot(self, robot_pose):

        # current minus target
        delta_pose = robot_pose - self.impedance_target
        return -self.impedance_stiffness * delta_pose

    def impedance_model_contact(self, contact_pose):

        # robot pose
        robot_pose = self.pbal_helper.forward_kin(contact_pose)

        # wrench in robot frame
        impedance_wrench_robot = self.impedance_model_robot(robot_pose)

        # rotation from robot to contact
        robot2contact = self.pbal_helper.contact2robot(contact_pose).T

        # wrench in contact frame
        return np.dot(robot2contact, impedance_wrench_robot)

    def torque_balance(self, contact_pose, contact_wrench):

        d, s, theta = contact_pose[0], contact_pose[1], contact_pose[2]
        Fd, Fs, tau = contact_wrench[0], contact_wrench[1], contact_wrench[2]

        return (self.pbal_helper.mgl * np.sin(theta - self.pbal_helper.theta0) + Fs * d - Fd * s + tau)

    def equilibrium_check(self, contact_pose):

        # contact wrench based on controller
        contact_wrench = self.impedance_model_contact(contact_pose)

        # net torque
        return self.torque_balance(contact_pose, contact_wrench)

    def frictionless_equilbrium_check(self, contact_pose):

        contact_wrench = self.impedance_model_contact(contact_pose)
        net_torque = self.torque_balance(contact_pose, contact_wrench)

        return np.array([net_torque, contact_wrench[1]])

    def wrench_cone_check_contact(self, contact_wrench):

        Fd, Fs, tau = contact_wrench[0], contact_wrench[1], contact_wrench[2]

        # constraints statisfied if cvec > 0
        cvec = np.array([
            Fs + self.pbal_helper.mu_contact * Fd,
            -Fs + self.pbal_helper.mu_contact * Fd,
            tau + 0.5 * self.pbal_helper.l_contact * Fd,
            -tau + 0.5 * self.pbal_helper.l_contact * Fd
        ])

        return np.all(cvec > 0), cvec

    def wrench_cone_check_with_imepedance_contact(self, contact_pose):

        contact_wrench = self.impedance_model_contact(contact_pose)
        return self.wrench_cone_check_contact(contact_wrench)

    def wrench_cone_check_with_imepedance_robot(self, robot_pose):

        contact_pose = self.pbal_helper.inverse_kin(robot_pose)
        return self.wrench_cone_check_with_imepedance_contact(contact_pose)

    def find_equilibrium_angle(self, d, s, theta_guess):

        # params
        TOL, DTHETA = 1e-6, 1e-4

        # pack
        guess_pose = np.array([d, s, theta_guess])

        # check if at equilbrium
        net_torque = self.equilibrium_check(guess_pose)

        while np.abs(net_torque) > TOL:

            # gradient
            ntp = self.equilibrium_check(guess_pose + np.array([0, 0, DTHETA]))
            ntm = self.equilibrium_check(guess_pose +
                                         np.array([0, 0, -DTHETA]))
            dnt_dtheta = (ntp - ntm) / (2 * DTHETA)

            # current value
            net_torque = self.equilibrium_check(guess_pose)

            # new guess
            theta_guess = theta_guess - net_torque / dnt_dtheta
            guess_pose[-1] = theta_guess

        return theta_guess

    def find_frictionless_equilibrium(self, d, s_guess, theta_guess):

        # params
        TOL, DELTA = 1e-6, 1e-4

        # pack
        guess_pose = np.array([d, s_guess, theta_guess])

        # check if at equilbrium
        equilibrium_violation = self.frictionless_equilbrium_check(guess_pose)
        objective = np.sqrt(equilibrium_violation[0]**2 +
                            (self.pbal_helper.l_contact * equilibrium_violation[1])**2)

        while objective > TOL:

            # gradient
            equilibrium_violation_grad = np.zeros([2, 2])
            delta_list = [
                DELTA * np.array([0, 1, 0]), DELTA * np.array([0, 0, 1])
            ]
            for i, delta in enumerate(delta_list):
                # print(i, delta)
                equilibrium_violationp = self.frictionless_equilbrium_check(
                    guess_pose + delta)
                equilibrium_violationm = self.frictionless_equilbrium_check(
                    guess_pose - delta)
                equilibrium_violation_grad[:, i] = (
                    equilibrium_violationp - equilibrium_violationm) / (2 *
                                                                        DELTA)

            # new guess
            newton_step = -np.linalg.solve(equilibrium_violation_grad,
                                           equilibrium_violation)
            guess_pose[1:] += newton_step

            # current value
            equilibrium_violation = self.frictionless_equilbrium_check(
                guess_pose)
            objective = np.sqrt(equilibrium_violation[0]**2 +
                                (self.pbal_helper.l_contact * equilibrium_violation[1])**2)

        return guess_pose

    def find_all_equilibrium_angles(self, d, s):

        theta_guess_array = np.linspace(-np.pi / 2, np.pi / 2, 100)
        net_torque_array = np.zeros_like(theta_guess_array)

        for i, theta_guess in enumerate(theta_guess_array):
            guess_pose = np.array([d, s, theta_guess])
            net_torque_array[i] = self.equilibrium_check(guess_pose)

        zero_crossing_array = np.zeros_like(net_torque_array, dtype=bool)
        for i in range(net_torque_array.shape[0] - 2):
            if (net_torque_array[i] * net_torque_array[i + 1]) < 0:
                zero_crossing_array[i] = True
        theta_zero_crossing = theta_guess_array[zero_crossing_array]

        eq_theta_list = np.zeros_like(theta_zero_crossing)
        for i, theta0 in enumerate(theta_zero_crossing):
            eq_theta_list[i] = self.find_equilibrium_angle(d, s, theta0)

        if len(eq_theta_list) > 1:
            print("WARNING: More than one theta equilibrium")
        return eq_theta_list

    def find_sticking_patch_boundary_contact_fixed_theta(self, d, s_guess, theta, const_index):
        # params
        TOL, DS, max_iter = 1e-6, 1e-4, 20
        sp, sm = s_guess + DS, s_guess - DS

        # initial violation
        _, violation = self.wrench_cone_check_with_imepedance_contact(
            np.array([d, s_guess, theta]))

        count = 0
        while np.abs(violation[const_index]) > TOL and count<max_iter:
            count+=1
            # find violation
            _, violation = self.wrench_cone_check_with_imepedance_contact(
                np.array([d, s_guess, theta]))

            # find gradient of violation[const_index] w.r.t s
            _, violationp = self.wrench_cone_check_with_imepedance_contact(
                np.array([d, sp, theta]))
            _, violationm = self.wrench_cone_check_with_imepedance_contact(
                np.array([d, sm, theta]))
            dviolation_ds = (violationp - violationm) / (2 * DS)
            # newton step
            s_guess -= violation[const_index] / dviolation_ds[const_index]


        if np.abs(violation[const_index]) > TOL:
            print('Newtons method iteration limit reached')
            s_guess = -0.01
        return np.array([d, s_guess, theta])


    def find_all_sticking_patch_values_theta_sweep(self,d,s_guess,const_index):
        theta_array = np.linspace(-np.pi / 2, np.pi / 2, 100)
        s_boundary_array = np.zeros_like(theta_array)
        for i, theta in enumerate(theta_array):
            boundary_pose = self.find_sticking_patch_boundary_contact_fixed_theta(d, s_guess, theta, const_index)
            s_boundary_array[i] = boundary_pose[1]
        return theta_array,s_boundary_array


    def find_sticking_patch_boundary_contact_fixed_s(self, d, s, theta_guess, const_index):
        # params
        TOL, DTHT, max_iter = 1e-6, 1e-4, 20
        thtp, thtm = theta_guess + DTHT, theta_guess - DTHT

        # initial violation
        _, violation = self.wrench_cone_check_with_imepedance_contact(
            np.array([d, s, theta_guess]))

        count = 0
        while np.abs(violation[const_index]) > TOL and count<max_iter:
            count+=1
            # find violation
            _, violation = self.wrench_cone_check_with_imepedance_contact(
                np.array([d, s, theta_guess]))

            # find gradient of violation[const_index] w.r.t s
            _, violationp = self.wrench_cone_check_with_imepedance_contact(
                np.array([d, s, thtp]))
            _, violationm = self.wrench_cone_check_with_imepedance_contact(
                np.array([d, s, thtm]))
            dviolation_dtht = (violationp - violationm) / (2 * DTHT)
            # newton step
            theta_guess -= violation[const_index] / dviolation_dtht[const_index]


        if np.abs(violation[const_index]) > TOL:
            print('Newtons method iteration limit reached')
            theta_guess = -0.01

        return np.array([d, s, theta_guess])

    def find_all_sticking_patch_values_s_sweep(self, d, theta_guess, const_index, percent_d=1.):
        s_array = np.linspace(-percent_d * d, percent_d * d, 100)
        theta_boundary_array = np.zeros_like(s_array)

        for i, s in enumerate(s_array):
            boundary_pose = self.find_sticking_patch_boundary_contact_fixed_s(d, s, theta_guess, const_index)
            theta_boundary_array[i] = boundary_pose[2]

        return s_array, theta_boundary_array

    def find_sticking_patch_boundary_contact(self, d, s_guess, theta_guess,
                                             const_index):

        # params
        TOL, DS = 1e-6, 1e-4
        sp, sm = s_guess + DS, s_guess - DS

        # initial violation
        theta_guess = self.find_equilibrium_angle(d, s_guess, theta_guess)
        _, violation = self.wrench_cone_check_with_imepedance_contact(
            np.array([d, s_guess, theta_guess]))

        while np.abs(violation[const_index]) > TOL:

            # find theta that results in eq.
            theta_guess = self.find_equilibrium_angle(d, s_guess, theta_guess)
            thetap = self.find_equilibrium_angle(d, sp, theta_guess)
            thetam = self.find_equilibrium_angle(d, sm, theta_guess)

            # find violation
            _, violation = self.wrench_cone_check_with_imepedance_contact(
                np.array([d, s_guess, theta_guess]))

            # find gradient of violation[const_index] w.r.t s
            _, violationp = self.wrench_cone_check_with_imepedance_contact(
                np.array([d, sp, thetap]))
            _, violationm = self.wrench_cone_check_with_imepedance_contact(
                np.array([d, sm, thetam]))
            dviolation_ds = (violationp - violationm) / (2 * DS)

            # newton step
            s_guess -= violation[const_index] / dviolation_ds[const_index]

        return np.array([d, s_guess, theta_guess])

    def find_sticking_srange(self,
                             d,
                             const_index,
                             theta_guess=0,
                             percent_d=1.0):

        s_range = np.linspace(-percent_d * d, percent_d * d, 100)
        contact_violation_array = np.zeros_like(s_range)

        for i, s_guess in enumerate(s_range):

            # find eq angle 
            theta_eq = self.find_equilibrium_angle(d, s_guess, theta_guess)

            # find contact constraint violation
            _, contact_violation = self.wrench_cone_check_with_imepedance_contact(
                np.array([d, s_guess, theta_eq]))
            contact_violation_array[i] = contact_violation[const_index]

        # find zero crossing in contact violation
        zero_crossing_array = np.zeros_like(contact_violation_array,
                                            dtype=bool)
        for i in range(contact_violation_array.shape[0] - 2):
            if (contact_violation_array[i] *
                    contact_violation_array[i + 1]) < 0:
                zero_crossing_array[i] = True

        # finding value of s at zero crossing
        s_zero_crossing = s_range[zero_crossing_array]
        pose_boundary_list = []
        for i, s0 in enumerate(s_zero_crossing):
            pose_boundary_list.append(
                self.find_sticking_patch_boundary_contact(
                    d, s0, theta_guess, const_index))

        return pose_boundary_list

    # def wrench_cone_check_with_impedance_contact_all_s(self, d, theta, percent_d=1.0):
        
    #     s_range = np.linspace(-percent_d * d, percent_d * d, 100)
    #     contact_violation_list = []

    #     for s in s_range:
    #         _, cvec = self.wrench_cone_check_with_imepedance_contact(np.array([d, s, theta]))
    #         contact_violation_list.append(cvec[2:])

    #     return s_range, np.array(contact_violation_list)

    # def wrench_cone_check_with_impedance_contact_all_theta(self, d, s):
        
    #     theta_range = np.linspace(-np.pi/2, np.pi/2, 100)
    #     contact_violation_list = []

    #     for theta in theta_range:
    #         _, cvec = self.wrench_cone_check_with_imepedance_contact(np.array([d, s, theta]))
    #         contact_violation_list.append(cvec[2:])

    #     return theta_range, np.array(contact_violation_list)

    def plot_state(self, robot_pose, ax):

        AXIS_LENGTH = 0.02
        FORCE_SCALE = 0.002
        TORQUE_SCALE = 0.005

        # contact pose
        contact_pose = self.pbal_helper.inverse_kin(robot_pose)
        contact2robot = self.pbal_helper.contact2robot(robot_pose)

        # impedance target
        xt, yt = self.impedance_target[0], self.impedance_target[1]
        target2robot = self.pbal_helper.contact2robot(self.impedance_target)

        # robot x-axis
        ax.plot(self.pbal_helper.pivot[0] + np.array([0, AXIS_LENGTH]),
                self.pbal_helper.pivot[1] + np.array([0, 0]),
                'r',
                linewidth=3)
        # robot z-axis
        ax.plot(self.pbal_helper.pivot[0] + np.array([0, 0]),
                self.pbal_helper.pivot[1] + np.array([0, AXIS_LENGTH]),
                'b',
                linewidth=3)
        # plot pivot
        ax.plot(self.pbal_helper.pivot[0], self.pbal_helper.pivot[1], 'k.', markersize=10)

        # contact x-axis
        ax.plot(
            robot_pose[0] + np.array([0, AXIS_LENGTH * contact2robot[0, 0]]),
            robot_pose[1] + np.array([0, AXIS_LENGTH * contact2robot[0, 1]]),
            'r',
            linewidth=3)
        # contact y-axis
        ax.plot(
            robot_pose[0] + np.array([0, AXIS_LENGTH * contact2robot[1, 0]]),
            robot_pose[1] + np.array([0, AXIS_LENGTH * contact2robot[1, 1]]),
            'g',
            linewidth=3)
        # contact point
        ax.plot(robot_pose[0], robot_pose[1], 'k.', markersize=10)

        # pivot to project point
        projection_point = self.pbal_helper.pivot + contact_pose[0] * contact2robot[:2, 0]
        ax.plot(np.array([self.pbal_helper.pivot[0], projection_point[0]]),
                np.array([self.pbal_helper.pivot[1], projection_point[1]]),
                'k',
                linewidth=1)
        # projection to contact
        contact_point = projection_point + contact_pose[1] * contact2robot[:2,
                                                                           1]
        ax.plot(np.array([projection_point[0], contact_point[0]]),
                np.array([projection_point[1], contact_point[1]]),
                'k',
                linewidth=1)

        # impedance target x-axis
        ax.plot(xt + np.array([0, AXIS_LENGTH * target2robot[0, 0]]),
                yt + np.array([0, AXIS_LENGTH * target2robot[0, 1]]),
                'm',
                linewidth=3)
        # impedance target y-axis
        ax.plot(xt + np.array([0, AXIS_LENGTH * target2robot[1, 0]]),
                yt + np.array([0, AXIS_LENGTH * target2robot[1, 1]]),
                'c',
                linewidth=3)
        # impedance target coordinates
        ax.plot(xt, yt, 'k.', markersize=10)

        # plot wrench
        impedance_wrench = self.impedance_model_robot(robot_pose)

        # torque
        if impedance_wrench[2] > 0:
            tcolor = 'y'
        else:
            tcolor = 'b'
        cc = plt.Circle((robot_pose[0], robot_pose[1]),
                        TORQUE_SCALE * np.abs(impedance_wrench[2]),
                        color=tcolor)
        ax.add_artist(cc)

        # force
        ax.plot(
            robot_pose[0] + FORCE_SCALE * np.array([0, impedance_wrench[0]]),
            robot_pose[1] + FORCE_SCALE * np.array([0, impedance_wrench[1]]),
            'k',
            linewidth=2)

        robotpose_2 = self.pbal_helper.forward_kin(contact_pose)
        print(robot_pose - robotpose_2)


if __name__ == "__main__":


    # object parameters
    obj_params = dict()
    obj_params['pivot'] = np.array([0., 0.])
    obj_params['mgl'] = 0.75
    obj_params['theta0'] = 0.
    obj_params['mu_contact'] = 0.2
    obj_params['mu_ground'] = 0.7
    obj_params['l_contact'] = 0.065

    # impedance parameters
    param_dict=dict()
    param_dict['obj_params'] = obj_params
    param_dict['impedance_target'] = np.array([-0.00, 0.08, 0.])
    param_dict['impedance_stiffness'] = np.array([1000, 1000, 30.0])

    # create obj
    pbal_impedance_fwd = PbalImpedanceForwardModel(param_dict)

    # # find eq angle for fixed d & s
    d = -0.1 # meters
    percent_d = 1.0
    smin = percent_d * d
    smax = -percent_d * d
    sarray = np.linspace(smin, smax, 100)

    theta_boundary_array, s_left_array = pbal_impedance_fwd.find_all_sticking_patch_values_theta_sweep(d, 0.,0)
    _, s_right_array = pbal_impedance_fwd.find_all_sticking_patch_values_theta_sweep(d, 0.,1)
    s_boundary_array, theta_left_torque_array = pbal_impedance_fwd.find_all_sticking_patch_values_s_sweep(d, 0.,2)
    _, theta_right_torque_array = pbal_impedance_fwd.find_all_sticking_patch_values_s_sweep(d, 0.,3)

    s_list = []
    theta_list = []
    mask_list = []
    fig, ax = plt.subplots(1, 1)
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

    plt.xlim([-0.5*d, 0.5*d])
    plt.ylim([-np.pi/2, np.pi/2])

    # s_range, contact_violation_array = pbal_impedance_fwd.wrench_cone_check_with_impedance_contact_all_theta(
    #     d, 0.)

    # fig2, ax2 = plt.subplots(1,1)
    # ax2.plot(s_range, contact_violation_array[:,0])
    # ax2.plot(s_range, contact_violation_array[:,1])

    # s_guess, theta_guess = 0, 0
    # for i in range(4):
    #     contact_pose_boundary = pbal_impedance_fwd.find_sticking_srange(d, i, percent_d=percent_d)
    #     if not contact_pose_boundary:
    #         print("no boundary found")
    #         continue
    #     if i < 2:
    #         ax.plot(contact_pose_boundary[0][1], contact_pose_boundary[0][2], 'k*', markersize=15)
    #     else:
    #         ax.plot(contact_pose_boundary[0][1], contact_pose_boundary[0][2], 'kd', markersize=15)

    # test cone constraints
    # npts = 20
    # Fdvec=np.linspace(-1.0, 0.1, npts)
    # Fsvec=np.linspace(-2 * pbal_impedance_fwd.mu_contact, 2 * pbal_impedance_fwd.mu_contact, npts)
    # tauvec=np.linspace(-pbal_impedance_fwd.l_contact, pbal_impedance_fwd.l_contact, npts)

    # [fdplot, fsplot, tauplot] = np.meshgrid(Fdvec, Fsvec, tauvec, indexing='ij')

    # is_in_cone=np.zeros([Fdvec.shape[0], Fsvec.shape[0], tauvec.shape[0]], dtype=bool)

    # for i, Fd in enumerate(Fdvec):
    #     for j, Fs in enumerate(Fsvec):
    #         for k, tau in enumerate(tauvec):
    #             is_in_cone[i, j, k], _ = pbal_impedance_fwd.wrench_cone_check_contact(
    #                 np.array([Fd, Fs, tau]))

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(fdplot[is_in_cone], fsplot[is_in_cone], tauplot[is_in_cone] , c='g', marker='o')
    # ax.scatter(fdplot[~_boundary = pbal_impedance_fwd.find_sticking_srange(d, i, percent_d=percent_d)
    #     if not contact_pose_boundary:
    #         print("no boundary found")
    #         continue
    #     if i < 2:
    #         ax.plot(contact_pose_boundary[0][1], contact_pose_boundary[0][2], 'k*', markersize=15)
    #     else:
    #         ax.plot(contact_pose_boundary[0][1], contact_pose_boundary[0][2], 'kd', markersize=15)

    # test cone constraints
    # npts = 20
    # Fdvec=np.linspace(-1.0, 0.1, npts)
    # Fsvec=np.linspace(-2 * pbal_impedance_fwd.mu_contact, 2 * pbal_impedance_fwd.mu_contact, npts)
    # tauvec=np.linspace(-pbal_impedance_fwd.l_contact, pbal_impedance_fwd.l_contact, npts)

    # [fdplot, fsplot, tauplot] = np.meshgrid(Fdvec, Fsvec, tauvec, indexing='ij')

    # is_in_cone=np.zeros([Fdvec.shape[0], Fsvec.shape[0], tauvec.shape[0]], dtype=bool)

    # for i, Fd in enumerate(Fdvec):
    #     for j, Fs in enumerate(Fsvec):
    #         for k, tau in enumerate(tauvec):
    #             is_in_cone[i, j, k], _ = pbal_impedance_fwd.wrench_cone_check_contact(
    #                 np.array([Fd, Fs, tau]))

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(fdplot[is_in_cone], fsplot[is_in_cone], tauplot[is_in_cone] , c='g', marker='o')
    # ax.scatter(fdplot[~is_in_cone], fsplot[~is_in_cone], tauplot[~is_in_cone], c='r', marker='o', alpha=0.1)
    # ax.set_xlabel('Fd')
    # ax.set_ylabel('Fs')
    # ax.set_zlabel('tau')

    plt.show()

    # # build eq pose
    # eq_contact_pose=copy.deepcopy(contact_pose)
    # eq_contact_pose[-1] = theta_eq_set[0]

    # # plot eq pose
    # fig, ax = plt.subplots(1,1)
    # ax.invert_xaxis()
    # pbal_impedance_fwd.plot_state(pbal_impedance_fwd.forward_kin(eq_contact_pose), ax)
    # ax.axis('equal')
    # plt.show()