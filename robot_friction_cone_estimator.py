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
import copy

from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
solvers.options['reltol'] = 1e-6
solvers.options['abstol'] = 1e-6
solvers.options['feastol'] = 1e-6

class RobotFrictionConeEstimator(object):

    def __init__(self, quantile_value, vertical_offset, theta_threshold):   
        self.stats = livestats.LiveStats([quantile_value])   
        self.vertical_offset =  vertical_offset
        self.theta_threshold = theta_threshold

    def add_data_point(self,measured_contact_wrench):
        f_tangential = np.abs(measured_contact_wrench[1])
        f_normal     =        measured_contact_wrench[0]

        f_tangential_diff = f_tangential
        f_normal_diff     = f_normal+self.vertical_offset

        self.stats.add(min(np.arctan2(f_tangential_diff,f_normal_diff),np.pi/2))

    def update_estimator(self):    
        self.theta_friction_contact=self.stats.quantiles()[0][1]
        self.A_right = np.array([-np.sin(self.theta_friction_contact), np.cos(self.theta_friction_contact),0])
        self.A_left  = np.array([-np.sin(self.theta_friction_contact),-np.cos(self.theta_friction_contact),0])
        self.B_right = 0
        self.B_left = 0

    def return_left_right_friction_dictionary(self):
        return {
            "acr": self.A_right.tolist(),
            "bcr": self.B_right,
            "acl": self.A_left.tolist(),
            "bcl": self.B_left,
            "cu": (self.theta_friction_contact >  self.theta_threshold).item()
            }

    def update_constraint_line_plots(self,line_in,A,B):
        P0 =[A[0]*B,A[1]*B]
        line_in.set_ydata([P0[0]-100*A[1],P0[0]+100*A[1]])
        line_in.set_xdata([P0[1]+100*A[0],P0[1]-100*A[0]])

    def initialize_left_right_plot(self,axis_in):
        self.constraint_plot_left, = axis_in.plot([0], [0], 'g')
        self.constraint_plot_right, = axis_in.plot([0], [0], 'b')

    def update_left_right_plot(self):
        self.update_constraint_line_plots(self.constraint_plot_left, self.A_left, self.B_left)
        self.update_constraint_line_plots(self.constraint_plot_right, self.A_right, self.B_right)
