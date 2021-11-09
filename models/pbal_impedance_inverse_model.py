import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
import pdb

from pbal_helper import PbalHelper


class PbalImpedanceInverseModel(object):
    def __init__(self, param_dict):

        # object parameters
        self.pbal_helper = PbalHelper(param_dict['obj_params'])

        # pose target parameters
        self.contact_pose_target = param_dict['contact_pose_target']

    def static_equilbrium_contact(self):

        d, s, theta = self.contact_pose_target[0], self.contact_pose_target[
            1], self.contact_pose_target[2]
        beq = -self.pbal_helper.mgl * np.sin(theta - self.pbal_helper.theta0)
        Aeq = np.array([-s, d, 1.])

        return np.expand_dims(Aeq, axis=0), np.array([beq])

    def static_equilbrium_robot(self):

        Aeq_contact, beq_contact = self.static_equilbrium_contact()
        robot2contact = self.pbal_helper.contact2robot(
            self.contact_pose_target)

        return np.dot(Aeq_contact, robot2contact), beq_contact

    def static_equilbrium_robot_aux(self):

        Aeq_robot, beq_robot = self.static_equilbrium_robot()

        return np.hstack([Aeq_robot, np.array([[0]])]), beq_robot

    def wrench_cone_constraints_line_contact(self):
        '''
        sliding plus (right)
        sliding minus (left)
        torque plus (ccw)
        torque minus (cw)
        '''

        mu, lc = self.pbal_helper.mu_contact, self.pbal_helper.l_contact

        # written in the form Aiq x <= biq
        biq = np.zeros(4)
        Aiq = np.array([[-mu, 1., 0.], [-mu, -1., 0.], [-lc / 2., 0., 1.],
                        [-lc / 2., 0., -1.]])

        return Aiq, biq

    def wrench_cone_constraints_line_robot(self):

        Aiq_contact, biq_contact = self.wrench_cone_constraints_line_contact()
        robot2contact = self.pbal_helper.contact2robot(
            self.contact_pose_target)
        return np.dot(Aiq_contact, robot2contact), biq_contact

    def wrench_cone_constraints_line_robot_aux(self):

        Aiq_contact, biq_contact = self.wrench_cone_constraints_line_contact()
        robot2contact = self.pbal_helper.contact2robot(
            self.contact_pose_target)

        temp1 = np.dot(Aiq_contact, robot2contact)
        temp2 = np.dot(Aiq_contact, np.array([-1, 0, 0]))

        return np.hstack([temp1, np.expand_dims(temp2, axis=1)]), biq_contact

    def wrench_cone_constraint_pivot_robot(self):

        mu = self.pbal_helper.mu_ground
        biq = np.zeros(2)
        Aiq = np.array([[1., mu, 0.], [-1., mu, 0]])
        return Aiq, biq

    def wrench_cone_constraint_pivot_robot_aux(self):

        Aiq, biq = self.wrench_cone_constraint_pivot_robot()

        temp2 = np.expand_dims(np.array([-1., 0., 0.]), axis=1)
        return np.hstack([Aiq, np.dot(Aiq, temp2)]), biq

    def normal_force_constraint_contact(self, Nmax):

        Aiq_contact = np.array([1., 0., 0.])
        biq_contact = Nmax
        return np.expand_dims(Aiq_contact, axis=0), np.array([biq_contact])

    def normal_force_constraint_robot(self, Nmax):

        Aiq_contact, biq_contact = self.normal_force_constraint_contact(Nmax)
        robot2contact = self.pbal_helper.contact2robot(
            self.contact_pose_target)
        return np.dot(Aiq_contact, robot2contact), biq_contact

    def normal_force_constraint_robot_aux(self, Nmax):

        Aiq_robot, biq_robot = self.normal_force_constraint_robot(Nmax)
        return np.hstack([Aiq_robot, np.array([[0]])]), biq_robot

    def solve_linear_program(self, Nmax, cost=np.array([0., 0., 0.])):

        Aeq_r, beq_r = self.static_equilbrium_robot()

        Aline_r, bline_r = self.wrench_cone_constraints_line_robot()
        Apivot_r, bpivot_r = self.wrench_cone_constraint_pivot_robot()
        Anormal_r, bnormal_r = self.normal_force_constraint_robot(Nmax)

        Aiq = np.vstack([Aline_r, Apivot_r, Anormal_r])
        biq = np.concatenate([bline_r, bpivot_r, bnormal_r])

        result2 = self.solve_lp_cvxopt(cost, Aeq_r, beq_r, Aiq, biq)

        robot_wrench = np.squeeze(np.array(result2['x']))

        return robot_wrench

    def solve_linear_program_control_nullspace(self, Nmax):

        null_space_corners = []
        index_list = [[0, 1, 3], [0, 1, 4], [0, 2, 3], [0, 2, 4], [0, 1, 5],
                      [0, 2, 5], [0, 3, 5], [0, 4, 5]]

        for index in index_list:

            Aeq_c, beq_c = self.static_equilbrium_contact()
            Aline_c, bline_c = self.wrench_cone_constraints_line_contact()
            Anormal_c, bnormal_c = self.normal_force_constraint_contact(Nmax)

            Aiq = np.vstack([Aline_c, Anormal_c])
            biq = np.concatenate([bline_c, bnormal_c])

            Aall = np.vstack([Aeq_c, Aiq])
            ball = np.concatenate([beq_c, biq])

            candidate_wrench = np.linalg.solve(Aall[index, :], ball[index])

            print
            if np.all(np.dot(Aiq, candidate_wrench) - biq <= 0):
                null_space_corners.append(candidate_wrench)
                print(candidate_wrench)

        return null_space_corners

    def solve_linear_program_aux(self, Nmax):

        Aeq_aux, beq_aux = self.static_equilbrium_robot_aux()

        Aline_aux, bline_aux = self.wrench_cone_constraints_line_robot_aux()
        Apivot_aux, bpivot_aux = self.wrench_cone_constraint_pivot_robot_aux()
        Anormal_aux, bnormal_aux = self.normal_force_constraint_robot_aux(Nmax)

        # positive aux constraints
        Aiq_aux = np.expand_dims(np.array([0., 0., 0., -1]), axis=0)
        biq_aux = np.array([0.])

        Aiq = np.vstack([Aline_aux, Apivot_aux, Anormal_aux, Aiq_aux])
        biq = np.concatenate([bline_aux, bpivot_aux, bnormal_aux, biq_aux])

        beta = -1.
        cost = np.array([0., 0., 0., beta])

        result2 = self.solve_lp_cvxopt(cost, Aeq_aux, beq_aux, Aiq, biq)

        robot_wrench = np.squeeze(np.array(result2['x']))

        return robot_wrench

    def solve_linear_program_aux_feedback(self, Nmax, Es, Etheta):

        # add Etheta to static eq constraint to shift target
        Aeq_aux, beq_aux = self.static_equilbrium_robot_aux()
        beq_aux += Etheta

        # keep only torque constraints
        Aline_aux, bline_aux = self.wrench_cone_constraints_line_robot_aux()
        bline_aux[0] += Es
        bline_aux[1] -= Es
        # Aline_aux, bline_aux = Aline_aux[2:, :], bline_aux[2:]

        Apivot_aux, bpivot_aux = self.wrench_cone_constraint_pivot_robot_aux()
        Anormal_aux, bnormal_aux = self.normal_force_constraint_robot_aux(Nmax)

        # positive aux constraints
        Aiq_aux = np.expand_dims(np.array([0., 0., 0., -1]), axis=0)
        biq_aux = np.array([0.])

        Aiq = np.vstack([Aline_aux, Apivot_aux, Anormal_aux, Aiq_aux])
        biq = np.concatenate([bline_aux, bpivot_aux, bnormal_aux, biq_aux])

        beta = -1.
        cost = np.array([0., 0., 0., beta])

        result2 = self.solve_lp_cvxopt(cost, Aeq_aux, beq_aux, Aiq, biq)

        robot_wrench = np.squeeze(np.array(result2['x']))

        try:
            slack = np.dot(Aiq, robot_wrench) - biq
        except TypeError:
            slack = None

        return robot_wrench, slack

    def solve_linear_program_mode(self, Nmax, mode=-1):
        '''
        mode 0: sticking pivot, robot slide right 
        mode 1: sticking pivot, robot slide left
        mode 2: pivot sliding left, robot sticking
        mode 3: pivot sliding right, robot_sticking
        mode not in [0, 3]: sticking, sticking
        '''
        Aeq_r, beq_r = self.static_equilbrium_robot()

        Aline_r, bline_r = self.wrench_cone_constraints_line_robot()
        Apivot_r, bpivot_r = self.wrench_cone_constraint_pivot_robot()
        Anormal_r, bnormal_r = self.normal_force_constraint_robot(Nmax)

        if mode == -1:
            pass
        elif mode == 0:  # robot slide right
            Aline_r[0, :] *= -1
            bline_r[0] *= -1
        elif mode == 1:  # robot slide left
            Aline_r[1, :] *= -1
            bline_r[1] *= -1
        elif mode == 2:  # pivot slide left
            Apivot_r[0, :] *= -1
            bpivot_r[0] *= -1
        elif mode == 3:  # pivot slide right
            Apivot_r[1, :] *= -1
            bpivot_r[1] *= -1
        else:
            raise RuntimeError("mode not implemented yet")

        Aiq = np.vstack([Aline_r, Apivot_r, Anormal_r])
        biq = np.concatenate([bline_r, bpivot_r, bnormal_r])

        # pdb.set_trace
        cost = np.zeros(3)

        result = self.solve_lp_cvxopt(cost, Aeq_r, beq_r, Aiq, biq)

        if result['status'] == 'optimal':
            print("solution found")
            robot_wrench = np.squeeze(np.array(result['x']))
        elif result['status'] == 'unknown':

            robot_wrench = np.squeeze(np.array(result['x']))

            if np.abs(np.dot(Aeq_r, robot_wrench) - beq_r) <= 1e-6 and np.all(
                    np.dot(Aiq, robot_wrench) - biq <= 1e-6):
                print("solution found")
            else:
                print("no solution found")
                robot_wrench = None

        else:
            print("no solution found")
            robot_wrench = None

        return robot_wrench

    def solve_linear_program_mode_aux(self, Nmax, mode=-1):
        '''
        mode 0: sticking pivot, robot slide right 
        mode 1: sticking pivot, robot slide left
        mode 2: pivot sliding left, robot sticking
        mode 3: pivot sliding right, robot_sticking
        mode not in [0, 3]: sticking, sticking
        '''

        # with auxilliary variable
        Aeq_aux, beq_aux = self.static_equilbrium_robot_aux()
        Aline_aux, bline_aux = self.wrench_cone_constraints_line_robot_aux()
        Apivot_aux, bpivot_aux = self.wrench_cone_constraint_pivot_robot_aux()
        Anormal_aux, bnormal_aux = self.normal_force_constraint_robot_aux(Nmax)
        Aiq_aux, biq_aux = np.expand_dims(np.array([0., 0., 0., -1]),
                                          axis=0), np.array([0.])

        # without auxilliary variable
        Aline, bline = self.wrench_cone_constraints_line_robot()
        Apivot, bpivot = self.wrench_cone_constraint_pivot_robot()

        # pad
        Aline = np.hstack([Aline, np.zeros([Aline.shape[0], 1])])
        Apivot = np.hstack([Apivot, np.zeros([Apivot.shape[0], 1])])

        #pre-stack matrices
        Atemp_aux = np.vstack([Aline_aux[:2, :], Apivot_aux])
        btemp_aux = np.concatenate([bline_aux[:2], bpivot_aux])

        Atemp_torque, btemp_torque = Aline_aux[2:, :], bline_aux[2:]

        Atemp = np.vstack([Aline[:2, :], Apivot])
        btemp = np.concatenate([bline[:2], bpivot])

        # build matrices
        mode_mask = np.array(range(4)) == mode

        Aiq = np.vstack(
            [Atemp_aux[~mode_mask, :], Atemp_torque, Anormal_aux, Aiq_aux])
        biq = np.concatenate(
            [btemp_aux[~mode_mask], btemp_torque, bnormal_aux, biq_aux])

        Aeq = np.vstack([Aeq_aux, Atemp[mode_mask, :]])
        beq = np.concatenate([beq_aux, btemp[mode_mask]])

        beta = -1
        cost = np.array([0., 0., 0., beta])

        # print(np.append(Aeq[0, :], beq[0]))
        # print(np.vstack([Aeq, Aiq[:2, :]]))
        # print(np.concatenate([beq, biq[:2]]))
        # print(np.linalg.det(np.vstack([Aeq, Aiq[:2, :]])))

        Aactive_const = np.vstack([Aeq, Aiq[:2, :]])
        bactive_const = np.concatenate([beq, biq[:2]])
        # Asol_inv = np.linalg.inv(Aactive_const)
        # print(Asol_inv)
        # sol2 = np.dot(Asol_inv, np.concatenate([beq, biq[:2]]))
        # print(sol2)
        result2 = self.solve_lp_cvxopt(cost, Aeq, beq, Aiq, biq)

        robot_wrench = np.squeeze(np.array(result2['x']))
        # print(np.dot(Aactive_const, robot_wrench) - bactive_const)
        # print(robot_wrench)
        # print(sol2 - robot_wrench)

        # if np.all(robot_wrench != None):

        #     TOL = 0.001
        #     iq_const_slack = np.dot(Aiq, robot_wrench) - biq
        #     const_name_list = ['Other friction', 'Pivot 1', 'Pivot 2',
        #         'Torque CCW', 'Torque CW', 'Normal force cap', 'Aux variable']
        #     # for i, iq_slacki in enumerate(iq_const_slack):
        #     #         # if iq_slacki < TOL and ((i <= 4 and robot_wrench[-1] < TOL
        #     #         #     ) or (i >= 5)):
        #     #         if iq_slacki > -TOL:
        #     #             # pass
        #     #             # print(iq_slacki)
        #     #             print(const_name_list[i])

        # print("Iq Const: ", np.dot(Aiq, robot_wrench) - biq)

        return robot_wrench

    # def solve_linear_program_fixed_mu_and_lcontact(self, mu, l_contact):

    #     # save original mu and l
    #     mu0, lcontact0 = self.pbal_helper.mu_contact, self.pbal_helper.l_contact

    #     # update mu and l
    #     self.pbal_helper.mu_contact, self.pbal_helper.l_contact = mu, l_contact

    #     # get contsraints
    #     Ase, bse = self.static_equilbrium_contact()
    #     # Ase = np.array([[1., 0., 0.]])
    #     # bse = np.array([1])
    #     Aline, bline = self.wrench_cone_constraints_line_contact()

    #     Aeq = np.vstack([Ase[0,:], Aline[0, :], Aline[2, :]])
    #     beq = np.array([bse[0], bline[0], bline[2]])

    #     print(Aeq[0, :])

    #     robot_wrench = np.linalg.solve(Aeq, beq)
    #     # print("Eq Const: ", np.dot(Aeq, robot_wrench) - beq)

    #     # reset mu and l
    #     self.pbal_helper.mu_contact, self.pbal_helper.l_contact = mu0, lcontact0

    #     return robot_wrench

    def solve_lp_cvxopt(self, c, Aeq, beq, Aiq, biq):

        c_cvxopt = matrix(c)

        Aeq_cvxopt = matrix(Aeq)
        beq_cvxopt = matrix(beq)

        Aiq_cvxopt = matrix(Aiq)
        biq_cvxopt = matrix(biq)

        result = solvers.lp(c_cvxopt, Aiq_cvxopt, biq_cvxopt, Aeq_cvxopt,
                            beq_cvxopt)
        return result

    def plot_state(self, robot_wrench, ax, Nmax):

        AXIS_LENGTH = 0.02
        FORCE_SCALE = 0.002
        TORQUE_SCALE = 0.05

        contact_pose = self.contact_pose_target
        robot_pose = self.pbal_helper.forward_kin(contact_pose)
        contact2robot = self.pbal_helper.contact2robot(robot_pose)

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
        ax.plot(self.pbal_helper.pivot[0],
                self.pbal_helper.pivot[1],
                'k.',
                markersize=10)

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
        projection_point = self.pbal_helper.pivot + contact_pose[
            0] * contact2robot[:2, 0]
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

        # torque
        if robot_wrench[2] > 0:
            tcolor = 'y'
        else:
            tcolor = 'b'
        cc = plt.Circle((robot_pose[0], robot_pose[1]),
                        TORQUE_SCALE * np.abs(robot_wrench[2]),
                        color=tcolor)
        cmax = plt.Circle((robot_pose[0], robot_pose[1]),
                          TORQUE_SCALE * Nmax * self.pbal_helper.l_contact / 2,
                          edgecolor='k',
                          facecolor='w')
        ax.add_artist(cmax)
        ax.add_artist(cc)

        # generators in contact frame
        fp_contact = np.array([Nmax, Nmax * self.pbal_helper.mu_contact, 0])
        fm_contact = np.array([Nmax, -Nmax * self.pbal_helper.mu_contact, 0])

        # generators in world frame
        fp_world = np.dot(contact2robot, fp_contact)
        fm_world = np.dot(contact2robot, fm_contact)

        ax.plot(robot_pose[0] + FORCE_SCALE * np.array([0, fp_world[0]]),
                robot_pose[1] + FORCE_SCALE * np.array([0, fp_world[1]]),
                'k--',
                linewidth=2)
        ax.plot(robot_pose[0] + FORCE_SCALE * np.array([0, fm_world[0]]),
                robot_pose[1] + FORCE_SCALE * np.array([0, fm_world[1]]),
                'k--',
                linewidth=2)

        # force
        ax.plot(robot_pose[0] + FORCE_SCALE * np.array([0, robot_wrench[0]]),
                robot_pose[1] + FORCE_SCALE * np.array([0, robot_wrench[1]]),
                'k',
                linewidth=2)

        ax.plot(self.pbal_helper.pivot[0] +
                FORCE_SCALE * np.array([0, Nmax * self.pbal_helper.mu_ground]),
                self.pbal_helper.pivot[1] + FORCE_SCALE * np.array([0, Nmax]),
                'k--',
                linewidth=2)
        ax.plot(
            self.pbal_helper.pivot[0] +
            FORCE_SCALE * np.array([0, -Nmax * self.pbal_helper.mu_ground]),
            self.pbal_helper.pivot[1] + FORCE_SCALE * np.array([0, Nmax]),
            'k--',
            linewidth=2)
        # pivot_force
        ax.plot(self.pbal_helper.pivot[0] +
                FORCE_SCALE * np.array([0, -robot_wrench[0]]),
                self.pbal_helper.pivot[1] +
                FORCE_SCALE * np.array([0, -robot_wrench[1]]),
                'k',
                linewidth=2)


if __name__ == "__main__":

    # object parameters
    obj_params = dict()
    obj_params['pivot'] = np.array([0., 0.])
    obj_params['mgl'] = .6
    obj_params['theta0'] = np.pi / 12
    obj_params['mu_contact'] = 0.3
    obj_params['mu_ground'] = 0.3
    obj_params['l_contact'] = 0.065

    # impedance parameters
    param_dict = dict()
    param_dict['obj_params'] = obj_params
    param_dict['contact_pose_target'] = np.array([-0.1, 0.00, -np.pi / 6])

    # find force
    Nmax = 20
    pbal_impedance_inv = PbalImpedanceInverseModel(param_dict)
    robot_wrench = pbal_impedance_inv.solve_linear_program_mode_aux(Nmax,
                                                                    mode=2)
    print(robot_wrench)

    # plot
    fig, ax = plt.subplots(1, 1)
    ax.invert_xaxis()
    pbal_impedance_inv.plot_state(robot_wrench, ax, Nmax)
    ax.axis('equal')
    plt.show()
