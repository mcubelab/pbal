#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
from collections import deque
from Modelling.convex_hull_estimator import ConvexHullEstimator
import PlottingandVisualization.image_overlay_helper as ioh

def find_blob(start_coord,cv_image,visited_dict):
    l0 = len(cv_image)
    l1 = len(cv_image[0])
    direction_list = [np.array([0,1]),np.array([0,-1]),np.array([1,0]),np.array([-1,0])]

    average_color = np.array(cv_image[start_coord[0]][start_coord[1]]) 
    num_points = 0


    to_visit = deque()
    to_visit.append(start_coord)

    visited_dict[start_coord] = None

    while len(to_visit)>0:
        
        current_coord = np.array(list(to_visit.popleft()))

        for direction in direction_list:
            next_coord = current_coord+direction
            next_coord_tuple = tuple(next_coord.tolist())

            if 0<=next_coord[0] and next_coord[0]<l0 and 0<=next_coord[1] and next_coord[1]<l1:
                color_diff = np.array(cv_image[next_coord[0]][next_coord[1]])-average_color
                my_dist = np.linalg.norm(color_diff)

                if my_dist<20 and next_coord_tuple not in visited_dict:
                    visited_dict[next_coord_tuple] = None
                    to_visit.append(next_coord_tuple)

                    average_color = (average_color*num_points + np.array(cv_image[next_coord[0]][next_coord[1]]))/(num_points+1)

                    num_points+=1



def compute_vertices(coord_dict,l0,l1):
    num_divisions = 64
    theta_range = 2*np.pi*(1.0*np.array(range(num_divisions)))/num_divisions
    shape_hull_estimator = ConvexHullEstimator(theta_range=theta_range, quantile_value=.99, 
                                                distance_threshold=.5,   closed = True)

    for coord in coord_dict.keys():
        shape_hull_estimator.add_data_point(np.array([1.0*coord[0],1.0*coord[1]]))

    shape_hull_estimator.generate_convex_hull_closed_polygon()

    x_list = []
    y_list = []

    for item in shape_hull_estimator.final_polygon_vertex_x_list:
        x_list.append(min(max(int(item),0),l0-1))

    for item in shape_hull_estimator.final_polygon_vertex_y_list:
        y_list.append(min(max(int(item),0),l1-1))

    return x_list,y_list

def compute_polygon_coords(cv_image,ee_pose_homog,camera_transformation_matrix):
    hand_normal = np.array([1.0, 0.0, 0.0, 0.0])
    hand_front_center = np.array([0.0, 0.0, .041, 1.0])

    hand_front_center_world = np.dot(ee_pose_homog,hand_front_center)
    hand_normal_world = np.dot(ee_pose_homog,hand_normal)

    object_center_world = hand_front_center_world+.04*hand_normal_world

    l0 = len(cv_image)
    l1 = len(cv_image[0])


    x_coord, y_coord = ioh.get_pix_easier(object_center_world.T, camera_transformation_matrix)

    coord_dict = {}

    for i in range(len(cv_image)):
        for j in range(len(cv_image[0])):
            my_dist = (x_coord-j)**2+(y_coord-i)**2

            if my_dist<5**2 and (i,j) not in coord_dict:
                find_blob((i,j),cv_image,coord_dict)


    x_list,y_list = compute_vertices(coord_dict,l0,l1)

    return x_list,y_list

def transform_pix_to_world_frame(x,y,ee_pose_homog,camera_transformation_matrix):
    hand_z = np.array([0.0, 0.0, 1.0, 0.0])
    hand_front_center = np.array([0.0, 0.0, .041, 1.0])

    hand_front_center_world = np.dot(ee_pose_homog,hand_front_center)
    hand_z_world = np.dot(ee_pose_homog,hand_z)

    M00 = camera_transformation_matrix[0:3,0:3]
    M01 = -np.array([[y],[x],[1.0]])
    M10 = np.array([np.dot(hand_z_world,ee_pose_homog)[0:3]])
    M11 = np.array([[0.0]])

    M = np.vstack([np.hstack([M00,M01]),np.hstack([M10,M11])])

    B0 = -camera_transformation_matrix[0:3,3]
    B1 = np.array([np.dot(hand_front_center_world,hand_z_world)])

    B = np.hstack([B0,B1])

    A = np.linalg.solve(M,B)

    v_out = np.hstack([A[0:3],np.array([1.0])]) 

    check_val0 = np.dot(v_out,hand_z_world)-np.dot(hand_front_center_world,hand_z_world)

    check_val_temp = np.dot(camera_transformation_matrix,v_out)
    x_test = check_val_temp[0]/check_val_temp[2]
    y_test = check_val_temp[1]/check_val_temp[2]

    return v_out

def find_the_object(cv_image,ee_pose_homog,camera_transformation_matrix):

    x_list,y_list = compute_polygon_coords(cv_image,ee_pose_homog,camera_transformation_matrix)

    vertex_list = []

    for i in range(len(x_list)):
        vertex_out = transform_pix_to_world_frame(x_list[i],y_list[i],ee_pose_homog,camera_transformation_matrix)
        vertex_list.append(vertex_out)

    return vertex_list
