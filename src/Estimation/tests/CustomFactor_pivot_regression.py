"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

CustomFactor demo that simulates a 1-D sensor fusion task.
Author: Fan Jiang, Frank Dellaert
"""

from functools import partial
from typing import List, Optional

import gtsam
import numpy as np

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines


I = np.eye(1)
l_triangle_true = .12
h_ground = .01
l_contact = .5

x_pivot_true = .15


def error_contact_hand(measurement: np.ndarray, this: gtsam.CustomFactor,
              values: gtsam.Values,
              jacobians: Optional[List[np.ndarray]]) -> float:
    """
    :param measurement: [x_hand,y_hand,theta_hand],  to be filled with `partial`
    :param this: gtsam.CustomFactor handle
    :param values: gtsam.Values
    :param jacobians: Optional list of Jacobians
    :return: the unwhitened error
    """
    key1 = this.keys()[0]
    key2 = this.keys()[1]
    key3 = this.keys()[1]

    x_pivot, y_pivot = values.atVector(key1), values.atVector(key2)
    d = values.atVector(key3)

    x_hand = measurement[0]
    y_hand = measurement[1]
    theta_hand = measurement[2]

    error = np.sin(theta_hand)*(x_pivot-x_hand)-np.cos(theta_hand)*(y_pivot-y_hand)-d

    # error = estimate - measurement
    if jacobians is not None:
        jacobians[0] = np.array([np.sin(theta_hand)])
        jacobians[1] = np.array([-np.cos(theta_hand)])
        jacobians[2] = np.array([-1])

    print (error)
    return error

def error_contact_ground(measurement: np.ndarray, this: gtsam.CustomFactor,
              values: gtsam.Values,
              jacobians: Optional[List[np.ndarray]]) -> float:
    """
    :param measurement: [x_hand,y_hand,theta_hand],  to be filled with `partial`
    :param this: gtsam.CustomFactor handle
    :param values: gtsam.Values
    :param jacobians: Optional list of Jacobians
    :return: the unwhitened error
    """
    key1 = this.keys()[0]
    key2 = this.keys()[1]
    x_pivot, y_pivot = values.atVector(key1), values.atVector(key2)

    x_hand = measurement[0]
    y_hand = measurement[1]
    theta_hand = measurement[2]

    error = y_pivot-h_ground

    # error = estimate - measurement
    if jacobians is not None:
        jacobians[0] = np.array([0.0])
        jacobians[1] = np.array([1.0])

    return error



def main():
    # Create an empty nonlinear factor graph
    my_graph = gtsam.NonlinearFactorGraph()

    num_pts = 3

    unknown_x = [gtsam.symbol('x', k) for k in range(num_pts)]
    unknown_y = [gtsam.symbol('y', k) for k in range(num_pts)]
    unknown_d = [gtsam.symbol('d',0)]

    # New Values container
    v = gtsam.Values()

    # Add initial estimates to the Values container
    for i in range(num_pts):
        v.insert(unknown_x[i], np.array([0.0]))
        v.insert(unknown_y[i], np.array([0.0]))


    sigma_pos = .1
    my_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)


    theta_hand_list = [np.pi/10,-np.pi/20,0,np.pi/10,np.pi/20]

    x_hand_list = []
    y_hand_list = []

    for i in range(len(theta_hand_list)):
        theta_hand = theta_hand_list[i]
        x_hand = x_pivot_true - l_triangle_true*np.sin(theta_hand) + np.random.normal(scale = .001)
        y_hand = h_ground + l_triangle_true*np.cos(theta_hand) + np.random.normal(scale = .001)

        x_hand_list.append(x_hand)
        y_hand_list.append(y_hand)

        my_graph.add(gtsam.CustomFactor(my_model, [unknown_x[0],unknown_y[0],unknown_d[0]],
               partial(error_contact_hand, np.array([x_hand,y_hand,theta_hand]))))






    # my_graph.add(gtsam.CustomFactor(my_model, [unknown_x[0],unknown_y[0]],
    #            partial(error_contact_ground, np.array([x_hand,y_hand,theta_hand]))))


    # Initialize optimizer
    params = gtsam.GaussNewtonParams()
    print(my_graph)
    print(v)
    print(params)
    optimizer = gtsam.GaussNewtonOptimizer(my_graph, v, params)

    # Optimize the factor graph
    result = optimizer.optimize()

    # print(dir(result))
    x_pivot = result.atVector(unknown_x[0])
    y_pivot = result.atVector(unknown_y[0])
    fig, ax = plt.subplots(1,1)

    ax.set_xlim([-.2,.2])
    ax.set_ylim([-.05,.3])
    ax.set_aspect('equal', adjustable='box')

    ax.plot([-.5,.5],[h_ground,h_ground],'g')

    hand_plot,          = ax.plot([0],[0],'r')
    hand_plot.set_xdata([x_hand+.5*l_contact*np.cos(theta_hand),x_hand-.5*l_contact*np.cos(theta_hand)])
    hand_plot.set_ydata([y_hand+.5*l_contact*np.sin(theta_hand),y_hand-.5*l_contact*np.sin(theta_hand)])


    triangle_plot,       = ax.plot([x_pivot,x_pivot-l_triangle_true*np.sin(theta_hand)],[y_pivot,y_pivot+l_triangle_true*np.cos(theta_hand)],'b')
    pivot_plot,          = ax.plot([x_pivot],[y_pivot],'y.')
    pivot_plot,          = ax.plot([x_pivot_true],[h_ground],'r.')

    plt.show()


if __name__ == "__main__":
    main()
