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

I = np.eye(1)


class gtsam_pivot_estimator(object):
    def __init__(self):
        self.my_graph = gtsam.NonlinearFactorGraph()
        self.num_data_points = 0
        self.x_sym_list = []
        self.s_sym_list = []
        self.mgl_sin_phi_sym = gtsam.symbol('mgl_sin_phi', 0)
        self.mgl_cos_phi_sym = gtsam.symbol('mgl_cos_phi', 0) 
        self.h_sym = gtsam.symbol('h', 0)
        self.d_sym = gtsam.symbol('d',0)

        # New Values container
        self.v = gtsam.Values()

    #hand_pose[x_hand,y_hand,theta_hand]
    #
    def add_data_point(self,hand_pose,measured_base_wrench,sliding_state_dict):
        self.num_data_points+=1
        self.x_sym_list.append(gtsam.symbol('x', self.num_data_points))
        self.s_sym_list.append(gtsam.symbol('s', self.num_data_points))

        
    def kinematic_error_d(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:
        """
        :param measurement: [x_hand,y_hand,theta_hand],  to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """
        x_key = this.keys()[0]
        h_key = this.keys()[1]
        s_key = this.keys()[2]
        d_key = this.keys()[3]

        x_pivot = values.atVector(x_key)
        h_pivot = values.atVector(h_key)
        s_pivot = values.atVector(s_key)
        d_pivot = values.atVector(d_key)

        x_hand = measurement[0]
        y_hand = measurement[1]
        theta_hand = measurement[2]


        error_s = np.sin(theta_hand)*(x_pivot-x_hand)-np.cos(theta_hand)*(y_pivot-y_hand)-s
        error_d = np.sin(theta_hand)*(x_pivot-x_hand)-np.cos(theta_hand)*(y_pivot-y_hand)-d

        # error = estimate - measurement
        if jacobians is not None:
            jacobians[0] = np.array([np.sin(theta_hand)])
            jacobians[1] = np.array([-np.cos(theta_hand)])
            jacobians[2] = np.array([-1])

        return error


    def error_contact_hand(self,measurement: np.ndarray, this: gtsam.CustomFactor,
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

        return error

    def error_contact_ground(self,measurement: np.ndarray, this: gtsam.CustomFactor,
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
