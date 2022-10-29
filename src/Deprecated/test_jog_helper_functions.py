#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0,parentdir) 
sys.path.insert(0,gparentdir)

import rospy
import pdb
import json
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, Bool, String
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from scipy.spatial import ConvexHull, convex_hull_plot_2d

import time
import Modelling.ros_helper as ros_helper

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines
from livestats import livestats
from Modelling.system_params import SystemParams
import copy

from cvxopt import matrix, solvers

def triangle_wave(t,amplitude_in,period_in):
    t_transform = (t/period_in) % 1.0



    return (amplitude_in*4.0*t_transform*(t_transform<=.25)+
            amplitude_in*(2.0-4.0*t_transform)*(t_transform>.25)*(t_transform<=.75)+
            amplitude_in*(4.0*t_transform-4.0)*(t_transform>.75))

def sine_wave(t,amplitude_in,period_in):
    return amplitude_in*np.sin(2*np.pi*t/period_in)

def limit_range(x,x_min,x_max):
    return x_min*(x<=x_min)+x*(x>x_min)*(x<x_max)+x_max*(x>=x_max)



if __name__ == '__main__':
    fig, axs = plt.subplots(1,1)


    amplitude = 10.0
    period = 4.0
    num_periods = 2
    t_max = period*num_periods

    tlist = np.linspace(0,t_max,100)

    axs.set_xlim([0.0, t_max])
    axs.set_ylim([-1.5*amplitude, 1.5*amplitude])



    #axs.plot(tlist,limit_range(tlist,3.0,7.0),marker='o')

    axs.plot(tlist,sine_wave(tlist,amplitude,period),color='r')

    #axs.plot(tlist,triangle_wave(tlist,amplitude,period),marker='o')
        
    plt.show()