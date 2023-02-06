#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
from collections import deque
from Modelling.convex_hull_estimator import ConvexHullEstimator
import cv2
import PlottingandVisualization.image_overlay_helper as ioh
import random
import time



def fast_polygon_estimate(cv_image,ee_pose_homog,camera_transformation_matrix,visited_array,is_fine = False,seed_point = None,color_dist = 25):

    num_divisions =16
    theta_range = 2*np.pi*(1.0*np.array(range(num_divisions)))/num_divisions


    shape_hull_estimator = ConvexHullEstimator(theta_range=theta_range,  boundary_update_rate=.9,
                                               boundary_max_update_step_ratio=10,   closed = True)
    shape_hull_estimator.curvature_threshold = -.5

    if is_fine:
        num_divisions = 60
        theta_range = 2*np.pi*(1.0*np.array(range(num_divisions)))/num_divisions

        shape_hull_estimator = ConvexHullEstimator(theta_range=theta_range,  boundary_update_rate=.05,
                                               boundary_max_update_step_ratio=100,   closed = True)

    hand_normal = np.array([1.0, 0.0, 0.0, 0.0])
    hand_front_center = np.array([0.0, 0.0, .041, 1.0])

    hand_front_center_world = np.dot(ee_pose_homog,hand_front_center)
    hand_normal_world = np.dot(ee_pose_homog,hand_normal)

    object_center_world = None
    
    if seed_point is None:
        object_center_world = hand_front_center_world+.03*hand_normal_world
    else: 
        object_center_world = seed_point

    l0 = len(cv_image)
    l1 = len(cv_image[0])


    x_coord, y_coord = ioh.get_pix_easier(object_center_world.T, camera_transformation_matrix)

    r = 16

    delta_coord = 8

    if is_fine:
        delta_coord = 2

    min_i = max(0,y_coord-r)
    max_i = min(l0,y_coord+r+1)

    min_j = max(0,x_coord-r)
    max_j = min(l1,x_coord+r+1)

    i_list = []
    j_list = []
    i_boundary = []
    j_boundary = []

    average_color = np.array([0.0,0.0,0.0])
    centroid = np.array([0.0,0.0])


    for i in range(min_i,max_i,delta_coord):
        for j in range(min_j,max_j,delta_coord):

            my_dist = (x_coord-j)**2+(y_coord-i)**2

            if my_dist<r**2 and not visited_array[i][j]:
                n_prev = len(i_list)
                average_color_temp, centroid_temp = find_blob_v2(i,j,l0,l1,delta_coord,cv_image,visited_array,i_list,j_list,i_boundary,j_boundary,color_dist)
                n_current = len(i_list)
                dn = n_current-n_prev

                average_color = average_color*(n_prev/n_current)+average_color_temp*(dn/n_current)
                centroid = centroid*(n_prev/n_current)+centroid_temp*(dn/n_current)
    
    for n in range(100):
        count = random.randint(0,len(i_boundary)-1)
        v = np.array([i_boundary[count],j_boundary[count]])-centroid
        v = v/np.linalg.norm(v)
        v_perp = np.array([-v[1],v[0]])

        v_out = random.randint(0,delta_coord)*v+random.randint(-delta_coord//2,delta_coord//2)*v_perp

        i = round(v_out[0])+i_boundary[count]
        j = round(v_out[1])+j_boundary[count]

        i = max(min(i,l0-1),0)
        j = max(min(j,l1-1),0)

  
        if not visited_array[i][j] and np.linalg.norm(average_color-cv_image[i][j])<color_dist:
            visited_array[i][j]=1

            shape_hull_estimator.add_data_point(np.array([1.0*i,1.0*j]))

    num_random_points = 6

    if is_fine:
        num_random_points = 2*delta_coord*delta_coord

    for count in range(len(i_boundary)):
        # count = random.randint(0,len(i_boundary)-1)
        v = np.array([i_boundary[count],j_boundary[count]])-centroid
        v = v/np.linalg.norm(v)
        v_perp = np.array([-v[1],v[0]])

        for dummy in range(num_random_points):

            

            v_out = random.randint(0,delta_coord)*v+random.randint(-delta_coord//2,delta_coord//2)*v_perp

            i = round(v_out[0])+i_boundary[count]
            j = round(v_out[1])+j_boundary[count]

            i = max(min(i,l0-1),0)
            j = max(min(j,l1-1),0)

      
            if not visited_array[i][j] and np.linalg.norm(average_color-cv_image[i][j])<color_dist:
                visited_array[i][j]=1

                shape_hull_estimator.add_data_point(np.array([1.0*i,1.0*j]))




    shape_hull_estimator.generate_convex_hull_closed_polygon()

    x_list = []
    y_list = []

    for item in shape_hull_estimator.final_polygon_vertex_x_list:
        x_list.append(min(max(int(item),0),l0-1))

    for item in shape_hull_estimator.final_polygon_vertex_y_list:
        y_list.append(min(max(int(item),0),l1-1))


    vertex_list = []

    for i in range(len(x_list)):
        vertex_out = transform_pix_to_world_frame(x_list[i],y_list[i],ee_pose_homog,camera_transformation_matrix)
        vertex_list.append(vertex_out)

    

    return vertex_list




def find_blob_v2(i,j,l0,l1,delta_coord,cv_image,visited_array,i_list,j_list,i_boundary,j_boundary,color_dist = 23):

    t0 = time.time()
    visited_array[i][j] = 1
    t1 = time.time()


    average_color = 1.0*cv_image[i][j]
    centroid = np.array([0.0,0.0])
    num_pts = 0.0

    delta_i_list = [-delta_coord,delta_coord,0,0]
    delta_j_list = [0,0,-delta_coord,delta_coord]



    i_list.append(i)
    j_list.append(j)

    count = len(i_list)-1

    while count<len(i_list):

        i = i_list[count]
        j = j_list[count]


        is_boundary = False


        for k in range(4):
            i_next = i+delta_i_list[k]
            j_next = j+delta_j_list[k]

            if (0<=i_next and i_next<l0 and 0<=j_next and j_next<l1 and 
                not visited_array[i_next][j_next] and 
                np.linalg.norm(average_color-cv_image[i_next][j_next])<color_dist):


                average_color = average_color*(num_pts/(num_pts+1))+cv_image[i_next][j_next]/(num_pts+1)
                centroid[0] = centroid[0]*(num_pts/(num_pts+1)) + i_next/(num_pts+1)
                centroid[1] = centroid[1]*(num_pts/(num_pts+1)) + j_next/(num_pts+1)

                num_pts+=1

                visited_array[i_next][j_next] = 1
                i_list.append(i_next)
                j_list.append(j_next)

            elif (0<=i_next and i_next<l0 and 0<=j_next and j_next<l1) and not visited_array[i_next][j_next]:
                is_boundary = True

            if not (0<=i_next and i_next<l0 and 0<=j_next and j_next<l1):
                is_boundary = False

        if is_boundary:
            i_boundary.append(i)
            j_boundary.append(j)

        count+=1

    return average_color, centroid




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

def find_blob_boundaries(dict_in):
    dict_out = {}

    direction_list = [np.array([0,1]),np.array([0,-1]),np.array([1,0]),np.array([-1,0])]

    for coord in dict_in:
        coord_array = np.array(list(coord))

        is_boundary = False

        for direction in direction_list:
            neighbor_coord_array = coord_array+direction
            is_boundary = is_boundary or (tuple(neighbor_coord_array.tolist()) not in dict_in)

        if is_boundary:
            dict_out[coord]=None

    return dict_out



def compute_vertices(coord_dict,l0,l1):
    num_divisions = 16
    theta_range = 2*np.pi*(1.0*np.array(range(num_divisions)))/num_divisions
    # shape_hull_estimator = ConvexHullEstimator(theta_range=theta_range, quantile_value=.99, 
    #                                             distance_threshold=.5,   closed = True)
    shape_hull_estimator = ConvexHullEstimator(theta_range=theta_range,  boundary_update_rate=.05,
                                               boundary_max_update_step_ratio=100,   closed = True)

    t0 = time.time()
    for coord in coord_dict.keys():
        shape_hull_estimator.add_data_point(np.array([1.0*coord[0],1.0*coord[1]]))

    t1 = time.time()

    print('adding data points time: ',t1-t0)

    shape_hull_estimator.generate_convex_hull_closed_polygon()

    t2 = time.time()

    print('computing convex hull time: ',t2-t1)

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

    t0 = time.time()
    for i in range(len(cv_image)):
        for j in range(len(cv_image[0])):
            my_dist = (x_coord-j)**2+(y_coord-i)**2

            if my_dist<5**2 and (i,j) not in coord_dict:
                find_blob((i,j),cv_image,coord_dict)

    t1 = time.time()

    print('blob detection time: ', t1-t0)



    # coord_dict = find_blob_boundaries(coord_dict)


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
