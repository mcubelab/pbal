import os,sys,inspect

currentdir = os.path.dirname(os.path.abspath(
inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, gparentdir)

sys.path.append('/home/taylorott/Documents/gpis-touch-public/mex')

import json
import pickle
import numpy as np
import scipy

from scipy.spatial.transform import Rotation as Rotation_class
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines
import copy
import friction_reasoning

from functools import partial
from typing import List, Optional

import gtsam

from gp_wrapper import gp_wrapper



I = np.eye(1)


if __name__ == '__main__':
    print('hello!')

    dll_fname='/home/taylorott/Documents/gpis-touch-public/mex/build/libgp_python.so'

    my_gp = gp_wrapper(dll_fname)

    varNoise = [1e-5, 1e-5, 1e-5] # measurement noise
    priorNoise = [1e-3, 1e-3, 1e-3] # circular prior noise
    #GP parameters
    testLim = 1.5
    testRes =  0.01
    isLocal = True
    priorRad = 0.03


    fname = "/home/taylorott/Documents/gpis-touch-public/data/contacts/contacts-rect1-20200810-1811.txt"
    read_file = open(fname,'r')
    Lines = read_file.readlines()

    fig, axs = plt.subplots(1,1)
    my_gp.init_gp(varNoise, priorNoise, testLim, testRes, isLocal, priorRad)

    for line in Lines:
        num_list = line.split(',')
        pos_vec = [float(num_list[0]),float(num_list[1])]
        normal_vec = [float(num_list[2]),float(num_list[3])]

        current_val = [-.03,-0.04]
        # print('hi!')
        for i in range(10):
            func_val = my_gp.evalAtPoint(current_val[0],current_val[1])
            alpha = func_val[0]/(func_val[1]**2+func_val[2]**2)
            current_val = [current_val[0]-func_val[1]*alpha,current_val[1]-func_val[2]*alpha]
            # print(current_val)
        # print (func_val)
        # print('hello!')
        my_gp.update_gp(pos_vec,normal_vec)
        # print('boogah boogah!')
        contour_x,contour_y = my_gp.eval_contour()

        axs.cla()
        axs.plot(contour_x,contour_y)
        axs.plot(current_val[0],current_val[1],'ro')
        axs.axis("equal")

        axs.set_ylim([-0.09, 0.09])
        axs.set_xlim([-0.09, 0.09])

        plt.pause(0.01)




    print ('hello!')
    