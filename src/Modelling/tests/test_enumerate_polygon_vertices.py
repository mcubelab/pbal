#!/usr/bin/env python
import rospy
import pdb
import json
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, Bool, String
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from scipy.spatial import ConvexHull, convex_hull_plot_2d

import time
import models.ros_helper as ros_helper

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines
from livestats import livestats
from models.system_params import SystemParams
from convex_hull_estimator import ConvexHullEstimator


from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
solvers.options['reltol'] = 1e-6
solvers.options['abstol'] = 1e-6
solvers.options['feastol'] = 1e-6




if __name__ == '__main__':

    bad_data = np.array(
        [[ 0.00000000e+00,  5.44672097e-02],
        [ 1.25663706e-01,  7.61400933e-02],
        [ 2.51327412e-01,  9.66122023e-02],
        [ 3.76991118e-01,  1.15560679e-01],
        [ 5.02654825e-01,  1.32686695e-01],
        [ 6.28318531e-01,  1.47720162e-01],
        [ 7.53982237e-01,  1.60423994e-01],
        [ 8.79645943e-01,  1.70597844e-01],
        [ 1.00530965e+00,  1.78081264e-01],
        [ 1.13097336e+00,  1.82756237e-01],
        [ 1.25663706e+00,  1.84549034e-01],
        [ 1.38230077e+00,  1.83431382e-01],
        [ 1.50796447e+00,  1.79420908e-01],
        [ 1.63362818e+00,  1.72599903e-01],
        [ 1.75929189e+00,  1.63075937e-01],
        [ 1.88495559e+00,  1.50980166e-01],
        [ 2.01061930e+00,  1.36503348e-01],
        [ 2.13628300e+00,  1.19873790e-01],
        [ 2.26194671e+00,  1.01353752e-01],
        [ 2.38761042e+00,  8.12353035e-02],
        [ 2.51327412e+00,  5.95597629e-02],
        [ 2.63893783e+00,  3.72849079e-02],
        [ 2.76460154e+00,  3.01535014e-02],
        [ 2.89026524e+00,  3.14120373e-02],
        [ 3.01592895e+00,  3.21751867e-02],
        [ 3.14159265e+00,  3.24309141e-02],
        [ 3.26725636e+00,  3.21751867e-02],
        [ 3.39292007e+00,  3.14120373e-02],
        [ 3.51858377e+00,  3.01535014e-02],
        [ 3.64424748e+00,  2.84194267e-02],
        [ 3.76991118e+00,  2.62371607e-02],
        [ 3.89557489e+00,  2.36411189e-02],
        [ 4.02123860e+00,  2.06722427e-02],
        [ 4.14690230e+00,  1.73773528e-02],
        [ 4.27256601e+00,  1.38084116e-02],
        [ 4.39822972e+00,  1.00217036e-02],
        [ 4.52389342e+00,  6.07694732e-03],
        [ 4.64955713e+00,  2.03635395e-03],
        [ 4.77522083e+00, -1.72760880e-03],
        [ 4.90088454e+00, -5.15558097e-03],
        [ 5.02654825e+00, -8.50224655e-03],
        [ 5.15221195e+00, -1.17148266e-02],
        [ 5.27787566e+00, -1.47426569e-02],
        [ 5.40353936e+00, -1.75379866e-02],
        [ 5.52920307e+00, -2.00567319e-02],
        [ 5.65486678e+00, -2.22591705e-02],
        [ 5.78053048e+00, -2.41105686e-02],
        [ 5.90619419e+00, -1.42760174e-02],
        [ 6.03185789e+00,  8.89984200e-03],
        [ 6.15752160e+00,  3.19353456e-02]])
# ('length of vertex list:', 3)
# ('vertex_index_sequence:', [47, 20, 21, 22])
# ('optimization solution:', array([0.0544219 , 0.17623442]))
# [[-0.62833288 -0.05956226]
#  [-0.50265482 -0.03728491]
#  [ 2.63835417  0.03731674]]


    theta_range = []
    b_list = []
    for i in range(len(bad_data)):
        theta_range.append(bad_data[i][0])
        b_list.append(bad_data[i][1])

    num_constraints = len(theta_range)
    A = np.zeros([num_constraints,2])
    B = np.array(b_list)

 
    #build constraint matrix for the polygon
    for i in range(num_constraints):
        A[i][0] = np.cos(theta_range[i])
        A[i][1] = np.sin(theta_range[i])


    hull_estimator = ConvexHullEstimator(theta_range=theta_range, quantile_value=.99, distance_threshold=.5, closed = True)

    vertex_x_list, vertex_y_list = hull_estimator.enumerate_vertices_of_constraint_polygon(theta_range,b_list,closed=True)

    fig, ax = plt.subplots(1,1)

    line_list = hull_estimator.initialize_line_plot_list(ax, num_constraints, 'b')
    hull_estimator.update_constraint_line_plots(line_list,A,B)

    vertex_x_list = vertex_x_list.tolist()
    vertex_y_list = vertex_y_list.tolist()

    vertex_x_list.append(vertex_x_list[0])
    vertex_y_list.append(vertex_y_list[0])

    ax.plot(vertex_x_list,vertex_y_list,'k',linewidth=2)
    ax.set_xlim([-.15,.15])
    ax.set_ylim([-.05, .25])
    plt.show()

