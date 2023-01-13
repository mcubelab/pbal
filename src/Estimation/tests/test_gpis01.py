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

def load_shape_data(name_in):
    curr_dir = os.path.dirname(
        os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(curr_dir)
    gparentdir = os.path.dirname(parentdir)
    print('parentdir', parentdir)
    fname = os.path.join(gparentdir, 'Modelling', 'shape_description',
                         name_in + ".json")
    f = open(fname)
    # f = open("/home/oneills/Documents/panda/base_ws/src/franka_ros_interface/franka_ros_controllers/scripts/models/shape_description/"+name_in+".json")
    shape_data = json.load(f)

    vertex_array = np.array([shape_data["x_vert"], shape_data["y_vert"]])
    apriltag_id = shape_data["apriltag_id"]
    apriltag_pos = shape_data["centroid_to_apriltag"]

    return vertex_array, apriltag_id, apriltag_pos


def get_perimeter(vertex_array):
    perimeter = 0

    n_points = len(vertex_array[0])
    for i in range(len(vertex_array[0])):
        dx = vertex_array[0][(i-1)%n_points]-vertex_array[0][i]
        dy = vertex_array[1][(i-1)%n_points]-vertex_array[1][i]
        perimeter+= np.sqrt(dx**2+dy**2)
    return perimeter

#this only increases the resolution, does not decrease it
def rescale_resolution(vertex_array,np_target):
    perimeter = get_perimeter(vertex_array)
    l_target = perimeter/np_target

    n_points = len(vertex_array[0])

    x_out = []
    y_out = []

    count=0
    while count<n_points:
        x0 = vertex_array[0][count%n_points]
        y0 = vertex_array[1][count%n_points]
        x1 = vertex_array[0][(count+1)%n_points]
        y1 = vertex_array[1][(count+1)%n_points]

        side_length = np.sqrt((x0-x1)**2+(y0-y1)**2)

        if side_length>=1.5*l_target:
            num_divisions = 1.0* np.ceil(side_length/l_target)
            for i in range(int(num_divisions)):
                x_out.append(x0+(x1-x0)*(i/num_divisions))
                y_out.append(y0+(y1-y0)*(i/num_divisions))

        else:
            x_out.append(x0)
            y_out.append(y0)

        count+=1

    return np.array([x_out,y_out])

def prune_by_length(vertex_array,l_min,num_iter = 1):

    for j in range(num_iter):
        x_out = []
        y_out = []

        n_points = len(vertex_array[0])
        if n_points%2==0:
            for i in range(0,n_points,2):
     
                x0 = vertex_array[0][i]
                y0 = vertex_array[1][i]
     
                x1 = vertex_array[0][i+1]
                y1 = vertex_array[1][i+1]
     
                x2 = vertex_array[0][(i+2)%n_points]
                y2 = vertex_array[1][(i+2)%n_points]
     
                d0 = np.sqrt((x0-x1)**2+(y0-y1)**2)
                d1 = np.sqrt((x1-x2)**2+(y1-y2)**2)

                x_out.append(x0)
                y_out.append(y0)

                if d0>l_min and d1>l_min:
                    x_out.append(x1)
                    y_out.append(y1)
        else:
            for i in range(0,n_points-1,2):
     
                x0 = vertex_array[0][i]
                y0 = vertex_array[1][i]
     
                x1 = vertex_array[0][i+1]
                y1 = vertex_array[1][i+1]
     
                x2 = vertex_array[0][(i+2)%n_points]
                y2 = vertex_array[1][(i+2)%n_points]
     
                d0 = np.sqrt((x0-x1)**2+(y0-y1)**2)
                d1 = np.sqrt((x1-x2)**2+(y1-y2)**2)

                x_out.append(x0)
                y_out.append(y0)

                if d0>l_min and d1>l_min:
                    x_out.append(x1)
                    y_out.append(y1)

            x0 = x_out[-1]
            y0 = x_out[-1]

            x1 = vertex_array[0][-1]
            y1 = vertex_array[1][-1]

            x2 = vertex_array[0][0]
            y2 = vertex_array[1][0]

            d0 = np.sqrt((x0-x1)**2+(y0-y1)**2)
            d1 = np.sqrt((x1-x2)**2+(y1-y2)**2)

            if d0>l_min and d1>l_min:
                x_out.append(x1)
                y_out.append(y1)

        vertex_array = np.array([x_out,y_out])
    return vertex_array

def update_gp(my_gp,vertex_array):
    n_points = len(vertex_array[0])

    winding_angle = 0
    for i in range(n_points):
        x0 = vertex_array[0][i]
        y0 = vertex_array[1][i]

        x1 = vertex_array[0][(i+1)%n_points]
        y1 = vertex_array[1][(i+1)%n_points]

        x2 = vertex_array[0][(i+2)%n_points]
        y2 = vertex_array[1][(i+2)%n_points]

        v0_x = x1-x0
        v0_y = y1-y0

        v1_x = x2-x1
        v1_y = y2-y1

        d0 = np.sqrt(v0_x**2+v0_y**2)
        d1 = np.sqrt(v1_x**2+v1_y**2)



        my_cross_prod = v0_x*v1_y-v0_y*v1_x
        winding_angle += np.arcsin(my_cross_prod/(d0*d1))

    
    for i in range(n_points):
        x0 = vertex_array[0][i]
        y0 = vertex_array[1][i]

        x1 = vertex_array[0][(i+1)%n_points]
        y1 = vertex_array[1][(i+1)%n_points]

        v0_x = x1-x0
        v0_y = y1-y0

        d0 = np.sqrt(v0_x**2+v0_y**2)

        x_mid = (x1+x0)/2
        y_mid = (y1+y0)/2
        v0_x/=d0
        v0_y/=d0

        pos_vec = [x_mid,y_mid]
        if winding_angle>0:
            normal_vec = [ v0_y,-v0_x]
        else:
            normal_vec = [-v0_y, v0_x]

        my_gp.update_gp(pos_vec,normal_vec)

if __name__ == '__main__':
    fig, axs = plt.subplots(1,1)

    # varNoise = [1e-5, 1e-5, 1e-5] # measurement noise
    # priorNoise = [1e-3, 1e-3, 1e-3] # circular prior noise

    #GP parameters
    # testLim = 1.5
    # testRes =  0.01
    isLocal = True
    # priorRad = 0.03

    varNoise = [1e-5, 1e-5, 1e-5] # measurement noise
    priorNoise = [1e-4, 1e-4, 1e-4] # circular prior noise
    priorRad = 0.01
    testRes =  0.005
    # testLim = 1.5
    testLim = .3

    dll_fname='/home/taylorott/Documents/gpis-touch-public/mex/build/libgp_python.so'

    my_gp = gp_wrapper(dll_fname)
    my_gp.init_gp(varNoise, priorNoise, testLim, testRes, isLocal, priorRad)

 

    # vertex_array, apriltag_id, apriltag_pos = load_shape_data('rectangle')
    # vertex_array, apriltag_id, apriltag_pos = load_shape_data('rectangle_bump_out')
    vertex_array, apriltag_id, apriltag_pos = load_shape_data('rectangle_bump_in')

    # print(vertex_array)
    # print(get_perimeter(vertex_array))
    va_rescaled = vertex_array
    va_rescaled = rescale_resolution(va_rescaled,500)
    my_perimeter = get_perimeter(va_rescaled)
    va_rescaled = prune_by_length(va_rescaled,my_perimeter/500,10)
    va_rescaled = prune_by_length(va_rescaled,my_perimeter/200,2)


    

    update_gp(my_gp,va_rescaled)


    # for i in range(len(va_rescaled[0])):

        # print(my_gp.evalAtPoint(va_rescaled[0][i],va_rescaled[1][i]))
        # print(my_gp.evalAtPoint(0,-.1))
        # print(my_gp.evalAtPoint(-.1,-.2))
    contour_x,contour_y = my_gp.eval_contour()
    axs.plot(contour_x,contour_y,'b')
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
     
    #     my_gp.update_gp(pos_vec,normal_vec)
    #     # print('boogah boogah!')
    #     contour_x,contour_y = my_gp.eval_contour()

    #     axs.cla()
    #     axs.plot(contour_x,contour_y)
    #     axs.plot(current_val[0],current_val[1],'ro')
    #     axs.axis("equal")

    #     axs.set_ylim([-0.09, 0.09])
    #     axs.set_xlim([-0.09, 0.09])

    #     plt.pause(0.01)




    