#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(os.path.dirname(currentdir)))

import numpy as np


import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines
import random

from Estimation.contact_mode_reasoning import contact_mode_reasoning


def plot_closed_loop(axis_in,shape_in,color):
    l0 = shape_in[0].tolist()
    l1 = shape_in[1].tolist()

    l0.append(l0[0])
    l1.append(l1[0])

    axis_in.plot(l0,l1,color)

def rotate_shape_indices(shape_in,delta_i):
    shape_out = 0.0*shape_in
    num_vertices = len(shape_in[0])

    for i in range(num_vertices):
        for j in range(2):
            shape_out[j,(i+delta_i)%num_vertices] = shape_in[j,i]

    return shape_out

def transform_shape(shape_in,position,theta):
    rot_mat = np.array([[np.cos(theta),-np.sin(theta)],
                        [np.sin(theta), np.cos(theta)]])

    shape_out = np.dot(rot_mat,shape_in)

    for i in range(2):
        shape_out[i] = shape_out[i]+position[i]

    return shape_out

def generate_random_pose(r=.2):
    theta_out = random.uniform(0,2*np.pi)

    position_out = np.array([random.uniform(-r,r),random.uniform(-r,r)])

    return position_out, theta_out

def test_vision_reasoning():
    # test_shape = np.array([[0.0, .1, .06, 0.0],
    #                        [0.0, 0.0, .2, .2]])

    test_shape = np.array([[0.0, .1, .1, 0.0],
                           [0.0, 0.0, .2, .2]])

    num_vertices = len(test_shape[0])

    position0, theta0 =  generate_random_pose(r=.2)

    transformed_test_shape = transform_shape(test_shape,position0,theta0)

    adjusted_shape = test_shape+0.0

    for i in range(2):
        for j in range(num_vertices):
            adjusted_shape[i,j]+=random.uniform(-.01,.01)

    position1, theta1 =  generate_random_pose(r=.2)
    delta_index = random.randint(0,num_vertices-1)

    adjusted_shape = rotate_shape_indices(transform_shape(adjusted_shape,position1,theta1),delta_index)




    fig, axs = plt.subplots(2,2,figsize=(10, 8))

    plot_closed_loop(axs[0][0],transformed_test_shape,'r')
    plot_closed_loop(axs[0][0],adjusted_shape,'b')

  

    my_cmr_reasoning = contact_mode_reasoning(.1)

    my_cmr_reasoning.update_vision(adjusted_shape)
    my_cmr_reasoning.vertices_obj_frame = test_shape
    my_cmr_reasoning.vertices_wm_estimated = transformed_test_shape
    my_cmr_reasoning.num_vertices = num_vertices

    output_dict = my_cmr_reasoning.compute_hypothesis_object_poses_from_vision()

    
    
    plot_closed_loop(axs[0][1],adjusted_shape,'b')

    color_list = ['g','m','c','k']
    for i in range(output_dict['num_hypothesis']):
        corrected_shape_from_object_frame = transform_shape(test_shape,output_dict['hypothesis_object_position_list'][i],output_dict['hypothesis_theta_list'][i])
        plot_closed_loop(axs[0][1],corrected_shape_from_object_frame,color_list[i])
        


    axs[0][0].axis('equal')
    axs[0][0].set_xlim([-.5, .5])
    axs[0][0].set_ylim([-.5, .5])

    axs[0][1].axis('equal')
    axs[0][1].set_xlim([-.5, .5])
    axs[0][1].set_ylim([-.5, .5])
    plt.show()


if __name__ == '__main__':
    test_vision_reasoning()