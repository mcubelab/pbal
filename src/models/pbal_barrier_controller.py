import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
solvers.options['reltol'] = 1e-6
solvers.options['abstol'] = 1e-6
solvers.options['feastol'] = 1e-6
import pdb

from pbal_helper import PbalHelper


class PbalBarrierController(object):

    def __init__(self, param_dict):

        # object parameters
        self.pbal_helper = PbalHelper(param_dict['obj_params'])

        # objective function parameters
        self.K_theta = param_dict['K_theta']
        self.K_s = param_dict['K_s']
        self.K_x_pivot = param_dict['K_x_pivot']
        self.concavity_sliding = param_dict['concavity_sliding']
        self.concavity_rotating = param_dict['concavity_rotating']
        self.concavity_x_sliding = param_dict['concavity_x_sliding']
        self.regularization_constant = param_dict['regularization_constant']
        self.s_scale = param_dict['s_scale']
        self.x_piv_scale = param_dict['x_piv_scale']

        # barrier function parameters
        self.trust_region = param_dict['trust_region']
        self.torque_margin = param_dict['torque_margin']

    def compute_slack_values(self, contact_pose, 
        measured_wrench, Nmax):
        ''' computes slacks and and gradients '''

        # extract parameters
        mu_c, mu_g, lc = self.pbal_helper.mu_contact, \
            self.pbal_helper.mu_ground, self.pbal_helper.l_contact

        # compute pivot constraints
        contact2robot = self.pbal_helper.contact2robot(
            contact_pose)
        Aiq_pivot_robot = np.array([[1., mu_g, 0.], # friction 1
            [-1., mu_g, 0], # friction 2
            [0., 1., 0.]]) # normal force pivot positive
        Aiq_pivot_contact = np.dot(Aiq_pivot_robot, contact2robot)

        # measured wrench is in contact frame
        biq = np.array([0., 0., -self.torque_margin, -self.torque_margin,
            Nmax, 0., 0., 0.])
        Aiq = np.array([[-mu_c, 1., 0.]/np.sqrt(1 + mu_c ** 2), # friction robot 1
            [-mu_c, -1., 0.]/np.sqrt(1 + mu_c ** 2), # friction robot 2
            [-lc / 2., 0., 1.], # torque 1
            [-lc / 2., 0., -1.], # torque 2
            [1., 0., 0.]]) # normal force robot

        Aiq = np.vstack([Aiq, Aiq_pivot_contact]) # pivot friction 

        return np.dot(Aiq, measured_wrench) - biq, Aiq, biq

    def barrier_function_values(self, slacks):
        ''' evaluates barrier function f = exp(k*slacks) '''

        k = self.trust_region
        return -k*slacks

    def qp_cost_values(self, contact_pose, delta_contact_pose,
        delta_x_piv, measured_wrench, Nmax, mode):
        ''' computes cost function for the QP '''

        # unpack
        d, s, theta = contact_pose[0], contact_pose[1], contact_pose[2]
        delta_s, delta_theta = delta_contact_pose[1], delta_contact_pose[2]

        delta_s = 2 * np.arctan(delta_s/self.s_scale)/np.pi
        delta_N = 2 * np.arctan((0.5 * Nmax - measured_wrench[0])/
            (0.2*Nmax))/np.pi

        if delta_x_piv is not None:
            delta_x_piv = 2 * np.arctan(delta_x_piv/self.x_piv_scale)/np.pi

        contact2robot = self.pbal_helper.contact2robot(
            contact_pose)

        # theta error cost
        a1 = self.concavity_rotating * np.array([-s, d, 1.])
        b1 = self.K_theta * delta_theta
        P = np.outer(a1, a1)
        q = -2 * a1 * b1
        const = b1 ** 2

        if mode == -1:   # sticking, sticking

            a3 = self.concavity_sliding * np.array(
               [1, 0., 0.])
            b3 = delta_N
            P  += np.outer(a3, a3)
            q  -= 2 * a3 * b3
            const += b3 ** 2
            
        elif mode == 0:    # sticking pivot, robot slide right

            # s error cost slide right
            a2 = self.concavity_sliding * np.array(
               [-self.pbal_helper.mu_contact, 1., 0.])
            # a2 = self.concavity_sliding * np.array(
               # [0., 1., 0.])
            b2 = self.K_s * delta_s
            P  += np.outer(a2, a2)
            q  -= 2 * a2 * b2
            const += b2 ** 2

        elif mode == 1:    # sticking pivot, robot slide left

            # s error slide left
            a2 = self.concavity_sliding * np.array(
               [-self.pbal_helper.mu_contact, -1., 0.])
            # a2 = self.concavity_sliding * np.array(
            #    [0., -1., 0.])
            b2 = -self.K_s * delta_s
            P += np.outer(a2, a2)
            q -= 2 * a2 * b2
            const += b2 ** 2

        elif mode == 2: # slide left pivot, robot stick
            # s error cost slide left pivot
            a2 = self.concavity_x_sliding * np.dot(np.array(
               [1., -self.pbal_helper.mu_ground, 0.]), contact2robot)
            b2 = self.K_x_pivot * delta_x_piv
            P  += np.outer(a2, a2)
            q  -= 2 * a2 * b2
            const += b2 ** 2

        elif mode == 3: # slide right pivot, robot stick
            # s error cost slide right pivot
            a2 = self.concavity_x_sliding * np.dot(np.array(
               [-1., -self.pbal_helper.mu_ground, 0.]), contact2robot)
            b2 = -self.K_x_pivot * delta_x_piv
            P  += np.outer(a2, a2)
            q  -= 2 * a2 * b2
            const += b2 ** 2

        else:
            raise RuntimeError("Invalid mode: must be -1, 0, 1, 2, and 3")

        P += self.regularization_constant * np.identity(3)

        return P, q, const
        

    def solve_qp(self, measured_wrench, contact_pose, 
        delta_contact_pose, delta_x_piv, mode, Nmax):
        '''
        mode -1: sticking, sticking
        mode 0: sticking pivot, robot slide right 
        mode 1: sticking pivot, robot slide left
        mode not in [0, 1]: 
        ''' 

        # unpack        
        delta_s = delta_contact_pose[1]

        if mode == 0 and delta_s < 0:
            mode = -1
        if mode == 1 and delta_s > 0:
            mode = -1
        if mode == 2 and delta_x_piv < 0:
            mode = -1
        if mode == 3 and delta_x_piv > 0:
            mode = -1
        
        # compute cost
        P, q, _ = self.qp_cost_values(contact_pose, 
            delta_contact_pose, delta_x_piv, measured_wrench, Nmax, mode)

        # compute slacks
        slacks, normals, _ = self.compute_slack_values(contact_pose, 
            measured_wrench, Nmax)

        # compute valube of barrier function
        fbarrier = self.barrier_function_values(slacks)
  
        # inequality constraints
        Aiq = normals
        biq = fbarrier

        if mode == 0:  # slide right
            Aiq, biq = np.delete(Aiq, 0, axis=0), np.delete(biq, 0, axis=0)

        elif mode == 1: # slide left
            Aiq, biq = np.delete(Aiq, 1, axis=0), np.delete(biq, 1, axis=0)

        elif mode == 2: # slide left pivot
            Aiq, biq = np.delete(Aiq, 5, axis=0), np.delete(biq, 5, axis=0)

        elif mode == 3: # slide right pivot
            Aiq, biq = np.delete(Aiq, 6, axis=0), np.delete(biq, 6, axis=0)

        # solve qp
        result = self.solve_qp_cvxopt(P, q, Aiq, biq)

        return np.squeeze(np.array(result['x']))


    def solve_qp_cvxopt(self, P, q, Aiq, biq):

        P_cvxopt = matrix(2 * P)
        q_cvxopt = matrix(q)
        
        Aiq_cvxopt = matrix(Aiq)
        biq_cvxopt = matrix(biq)

        result = solvers.qp(P_cvxopt, q_cvxopt, 
            Aiq_cvxopt, biq_cvxopt)
        return result


    def plot_boundaries(self, axs, contact_pose, measured_wrench, Nmax):

        slacks, Aiq, biq = self.compute_slack_values(contact_pose, 
            measured_wrench, Nmax)

        slack_indicies = [[0, 1, 4], [2, 3, 4], [5, 6]]
        projection_indicies = [[1, 0], [2, 0], [0, 1]]
        plot_titles = ['Friction Robot', 'Torque Robot', 'Friction Pivot']

        for ax, sind, pind, title  in zip(
            axs, slack_indicies, projection_indicies, plot_titles):

            if title == 'Friction Pivot':

                scale = Nmax
                generator = np.array([[-self.pbal_helper.mu_ground, 1.], # friction 1
                    [self.pbal_helper.mu_ground, 1.]]) # friction 2

                ax.plot([0, scale * generator[0, 0]], [0, scale * generator[0, 1]],
                    color = 'k')
                ax.plot([0, scale * generator[1, 0]], [0, scale * generator[1, 1]], 
                    color = 'k')

            else:

                Aiq_proj = Aiq[np.ix_(sind, pind)]
                biq_proj = biq[sind]

                solve_indices = [[0, 1], [1, 2], [0, 2]]

                corners = []
                for solve_ind in solve_indices:

                    corners += [np.linalg.solve(Aiq_proj[solve_ind, :], 
                        biq_proj[solve_ind]).tolist()]

                corners += [corners[0]]
                corner_array = np.array(corners)
                ax.plot(corner_array[:, 0], corner_array[:, 1], color='k')

            ax.set_title(title)

        return axs


    def plot_projections(self, axs, contact_pose, delta_wrench, 
        measured_wrench, Nmax, color, plot_slack_flag):
        # note that ax needs to be a (3,) array

        slacks, normals, biq = self.compute_slack_values(contact_pose, 
            measured_wrench, Nmax)

        slack_indicies = [[0, 1, 4], [2, 3, 4], [5, 6]]
        projection_indicies = [[1, 0], [2, 0], [0, 1]]

        ct = 0
        for ax, sind, pind  in zip(axs, slack_indicies, projection_indicies):
            
            if ct == 2:

                contact2robot = self.pbal_helper.contact2robot(
                    contact_pose)
                measured_wrench_robot = -np.dot(contact2robot, measured_wrench)
                delta_wrench_robot = -np.dot(contact2robot, delta_wrench)

                current_normals =  np.array([[1., self.pbal_helper.mu_ground, 0.],
                    [-1., self.pbal_helper.mu_ground, 0.]])
                current_slacks = slacks[sind]
                ax.arrow(measured_wrench_robot[pind[0]], measured_wrench_robot[pind[1]], 
                    delta_wrench_robot[pind[0]], delta_wrench_robot[pind[1]],
                    edgecolor=color, facecolor=color)

            else:

                current_normals = normals[sind, :]
                current_slacks = slacks[sind]
                ax.arrow(measured_wrench[pind[0]], measured_wrench[pind[1]], 
                        delta_wrench[pind[0]], delta_wrench[pind[1]],
                        edgecolor=color, facecolor=color)

            if plot_slack_flag:

                for slack, normal in zip(current_slacks, current_normals):
                    print(normal)

                    if ct == 2:

                        point_on_boundary = slack * normal/(np.linalg.norm(normal, 
                        ord=2) ** 2) + measured_wrench_robot
                        ax.plot(
                            [measured_wrench_robot[pind[0]], point_on_boundary[pind[0]]], 
                            [measured_wrench_robot[pind[1]], point_on_boundary[pind[1]]], 
                            color = 'r', marker='.')
                    else:
                        point_on_boundary = -slack * normal/(np.linalg.norm(normal, 
                        ord=2) ** 2) + measured_wrench                
                        ax.plot(
                            [measured_wrench[pind[0]], point_on_boundary[pind[0]]], 
                            [measured_wrench[pind[1]], point_on_boundary[pind[1]]], 
                            color = 'r', marker='.')      
            ct += 1 

        return axs


    def plot_cost_function_projection(self, axs, contact_pose, 
        delta_contact_pose, measured_wrench, mode, color):

        # unpack
        d, s, theta = contact_pose[0], contact_pose[1], contact_pose[2]
        delta_s, delta_theta = delta_contact_pose[1], delta_contact_pose[2]

        delta_s = 2 * np.arctan(delta_s/self.s_scale)/np.pi


        a1 = self.concavity_rotating * np.array([-s, d, 1.])
        a1 = a1 / np.linalg.norm(a1) ** 2
        b1 = self.K_theta * delta_theta
        displacement1 = a1 * b1

        # theta error cost
        if mode == -1:
            pass
        elif mode == 0:
            a2 = self.concavity_sliding * np.array(
                [-self.pbal_helper.mu_contact, 1., 0.])
            a2 = a2 / np.linalg.norm(a2) ** 2
            b2 = self.K_s * delta_s
            displacement2 = a2 * b2
        elif mode == 1:
            a2 = self.concavity_sliding * np.array(
                [-self.pbal_helper.mu_contact, -1., 0.])
            a2 = a2 / np.linalg.norm(a2) ** 2
            b2 = -self.K_s * delta_s
            displacement2 = a2 * b2
        else:
            raise RuntimeError("Invalid mode: must be -1, 0, or 1")

        projection_indicies = [[1, 0], [2, 0], [0, 1]]

        ct = 0

        for ax, pind in zip(axs, projection_indicies):
            
            if ct == 2:

                contact2robot = self.pbal_helper.contact2robot(contact_pose)
                measured_wrench_robot = -np.dot(contact2robot, measured_wrench)
                displacement1_robot = -np.dot(contact2robot, displacement1)
                ax.arrow(measured_wrench_robot[pind[0]], measured_wrench_robot[pind[1]], 
                        displacement1_robot[pind[0]], displacement1_robot[pind[1]],
                        edgecolor=color, facecolor=color)

                if mode == 0 or mode == 1:

                    displacement2_robot = -np.dot(contact2robot, displacement2)
                    ax.arrow(measured_wrench_robot[pind[0]], measured_wrench_robot[pind[1]], 
                        displacement2_robot[pind[0]], displacement2_robot[pind[1]],
                        edgecolor=color, facecolor=color)
            else:

                ax.arrow(measured_wrench[pind[0]], measured_wrench[pind[1]], 
                        displacement1[pind[0]], displacement1[pind[1]],
                        edgecolor=color, facecolor=color)

                if mode == 0 or mode == 1:

                    ax.arrow(measured_wrench[pind[0]], measured_wrench[pind[1]], 
                        displacement2[pind[0]], displacement2[pind[1]],
                        edgecolor=color, facecolor=color)

            ct += 1 

        return axs



if __name__ == "__main__":

    # object parameters
    obj_params = dict()
    obj_params['pivot'] = np.array([0., 0.])
    obj_params['mgl'] = .6
    obj_params['theta0'] = np.pi/12
    obj_params['mu_contact'] = 0.3
    obj_params['mu_ground'] = 0.75
    obj_params['l_contact'] = 0.065

    # position control parameters
    param_dict = dict()
    param_dict['obj_params'] = obj_params
    param_dict['K_theta'] = 0.5
    param_dict['K_s'] = 10.
    param_dict['trust_region'] = np.array([1., 1., 5., 5., 1., 1., 1.])
    param_dict['concavity_rotating'] = 1.
    param_dict['concavity_sliding'] = 1.
    param_dict['regularization_constant'] = 0.0001
    param_dict['torque_margin'] = 0.


    pbc = PbalBarrierController(param_dict)

    measured_wrench = np.array([10., 1., .1])
    contact_pose = np.array([-0.1, 0.01, 0.1])
    delta_contact_pose = np.array([0, 0.03, np.pi/12])
    mode = 0

    delta_wrench = pbc.solve_qp(measured_wrench, contact_pose, 
        delta_contact_pose, mode, 20.)

    fig, axs = plt.subplots(1, 3)
    axs = pbc.plot_boundaries(axs, contact_pose, measured_wrench, 20.)
    axs = pbc.plot_projections(axs, contact_pose, delta_wrench, 
        measured_wrench, 20.)
    axs = pbc.plot_cost_function_projection(axs, contact_pose, 
        delta_contact_pose, measured_wrench, mode, 'b')


    P, q, const= pbc.qp_cost_values(contact_pose, delta_contact_pose, mode)

    print(np.dot(delta_wrench, np.dot(P, delta_wrench))
        + np.dot(q, delta_wrench) + const)


    plt.show()