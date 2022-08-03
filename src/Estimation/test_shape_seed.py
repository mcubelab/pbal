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

from Estimation.basic_gpis_shape_seed import basic_gpis_shape_seed

from gp_wrapper import gp_wrapper

if __name__ == '__main__':
    fig, axs = plt.subplots(1,1)

    # varNoise = [1e-5, 1e-5, 1e-5] # measurement noise
    # priorNoise = [1e-3, 1e-3, 1e-3] # circular prior noise

    # #GP parameters
    # # testLim = 1.5
    # # testRes =  0.01
    # isLocal = True
    # # priorRad = 0.03

    # varNoise = [1e-5, 1e-5, 1e-5] # measurement noise
    # priorNoise = [1e-4, 1e-4, 1e-4] # circular prior noise
    # priorRad = 0.01
    # testRes =  0.005
    # testLim = 1.5

    # dll_fname='/home/taylorott/Documents/gpis-touch-public/mex/build/libgp_python.so'

    # my_gp = gp_wrapper(dll_fname)
    # my_gp.init_gp(varNoise, priorNoise, testLim, testRes, isLocal, priorRad)

 

    # shape_name = 'big_triangle'
    shape_name = 'big_rectangle'
    # shape_name = 'rectangle_bump_in'
    # shape_name = 'rectangle_bump_out'
    # shape_name = 'big_square'

    my_basic_gpis_shape_seed = basic_gpis_shape_seed()
    # my_gpis, object_vertex_array = my_basic_gpis_shape_seed.init_gp(shape_name)
    # my_pivot_estimator = gtsam_pivot_estimator()
    # my_shape_contact_estimator = gtsam_shape_contact_estimator(my_gpis)

    # vertex_array, apriltag_id, apriltag_pos = self.load_shape_data('rectangle')
    # vertex_array, apriltag_id, apriltag_pos = self.load_shape_data('rectangle_bump_out')
    vertex_array, apriltag_id, apriltag_pos = my_basic_gpis_shape_seed.load_shape_data(shape_name)

    # print(vertex_array)
    # print(self.get_perimeter(vertex_array))
    va_rescaled = vertex_array
    va_rescaled = my_basic_gpis_shape_seed.rescale_resolution(va_rescaled,500)
    my_perimeter = my_basic_gpis_shape_seed.get_perimeter(va_rescaled)
    va_rescaled = my_basic_gpis_shape_seed.prune_by_length(va_rescaled,my_perimeter/500,10)
    va_rescaled = my_basic_gpis_shape_seed.prune_by_length(va_rescaled,my_perimeter/200,2)

    va_rescaled = my_basic_gpis_shape_seed.smooth_perimeter(va_rescaled)

    

    # my_basic_gpis_shape_seed.update_gp(my_gp,va_rescaled)


    # for i in range(len(va_rescaled[0])):

        # print(my_gp.evalAtPoint(va_rescaled[0][i],va_rescaled[1][i]))
        # print(my_gp.evalAtPoint(0,-.1))
        # print(my_gp.evalAtPoint(-.1,-.2))
    # contour_x,contour_y = my_gp.eval_contour()
    # axs.plot(contour_x,contour_y,'b')
    # axs.plot(vertex_array[0],vertex_array[1],'ro')
    axs.plot(va_rescaled[0],va_rescaled[1],'ro',markersize = 1)
    axs.axis('equal')
    plt.show()

    
    # fname = "/home/taylorott/Documents/gpis-touch-public/data/contacts/contacts-rect1-20200810-1811.txt"
    # read_file = open(fname,'r')
    # Lines = read_file.readlines()


    # for line in Lines:
    #     num_list = line.split(',')
    #     pos_vec = [float(num_list[0]),float(num_list[1])]
    #     normal_vec = [float(num_list[2]),float(num_list[3])]

    #     current_val = [-.03,-0.04]
    #     # print('hi!')
    #     for i in range(10):
    #         func_val = my_gp.evalAtPoint(current_val[0],current_val[1])
    #         alpha = func_val[0]/(func_val[1]**2+func_val[2]**2)
    #         current_val = [current_val[0]-func_val[1]*alpha,current_val[1]-func_val[2]*alpha]
     
    #     my_gp.self.update_gp(pos_vec,normal_vec)
    #     # print('boogah boogah!')
    #     contour_x,contour_y = my_gp.eval_contour()

    #     axs.cla()
    #     axs.plot(contour_x,contour_y)
    #     axs.plot(current_val[0],current_val[1],'ro')
    #     axs.axis("equal")

    #     axs.set_ylim([-0.09, 0.09])
    #     axs.set_xlim([-0.09, 0.09])

    #     plt.pause(0.01)