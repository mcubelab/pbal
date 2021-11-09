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
from polygon_representation import PolygonRepresentation

if __name__ == '__main__':
    fig, axs = plt.subplots(1,1)
    axs.set_xlim([-3, 3])
    axs.set_ylim([-3, 3])

    triangle_vertices_CCW = np.array([[1,0,-1],[0,2,0]])
    triangle_vertices_CW = np.array([[-1,0,1],[0,2,0]])

    collision_margin = .1
    position = np.array([0,0])
    theta = 0

    triangle_object_CCW = PolygonRepresentation(vertex_array = triangle_vertices_CCW, position = position, theta = theta, is_closed = True, collision_margin = collision_margin)
    triangle_object_CW = PolygonRepresentation(vertex_array = triangle_vertices_CW, position = position, theta = theta, is_closed = True, collision_margin = collision_margin)


    triangle_object_CW.initialize_polygon_plot(axs,'g','g')
    triangle_object_CW.initialize_normal_plot(axs,'b')
    triangle_object_CW.update_pose(position=np.array([.5,-.3]),theta=np.pi/16)
    triangle_object_CW.update_all_plots()

    num_points=5000
    random_points = np.random.rand(2,num_points)
    random_points = 6*(random_points-.5)

   

    collision_list = triangle_object_CW.compute_collision_faces_world(random_points)
    pts_list = [np.array([2,0]),np.array([2,0]),np.array([2,0])]
    color_list = ['k','y','c']
    for i in range(num_points):
        if len(collision_list[i])>0:
            axs.plot(random_points[0,i],random_points[1,i],marker='o',color=color_list[collision_list[i][0]])
        else:
            axs.plot(random_points[0,i],random_points[1,i],'ro')



    # triangle_object_CCW.initialize_polygon_plot(axs,'r','r')
    # triangle_object_CCW.initialize_normal_plot(axs,'b')
    # triangle_object_CCW.update_pose(position=np.array([0,-3]),theta=-np.pi/16)
    # triangle_object_CCW.update_all_plots()

    plt.show()