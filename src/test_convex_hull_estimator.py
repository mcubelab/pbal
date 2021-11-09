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
from convex_hull_estimator import ConvexHullEstimator


from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
solvers.options['reltol'] = 1e-6
solvers.options['abstol'] = 1e-6
solvers.options['feastol'] = 1e-6

def generate_randomized_convex_data(hull_vertices, num_hull_vertices, num_data_points,variance):
    num_triangles = num_hull_vertices-2
    my_cov = np.array([[variance,0],[0,variance]])

    triangle_list = []
    area_list = []

    total_area = 0.0

    for i in range(num_triangles):
        my_triangle = hull_vertices[[0,i+1,i+2],:]
        l1 = np.linalg.norm(my_triangle[0,:]-my_triangle[1,:])
        l2 = np.linalg.norm(my_triangle[1,:]-my_triangle[2,:])
        l3 = np.linalg.norm(my_triangle[2,:]-my_triangle[0,:])
        s= (l1+l2+l3)/2

        my_area = np.sqrt(s*(s-l1)*(s-l2)*(s-l3))

        triangle_list.append(my_triangle)
        area_list.append(my_area)
        total_area+=my_area

    area_threshold_list = [0.0]
    for i in range(num_triangles):
        area_threshold_list.append(area_threshold_list[i]+area_list[i]/total_area)
    area_threshold_list[-1] = 1.0

    randomized_vertices = []

    for i in range(num_data_points):
        my_random = np.random.rand(1)[0]
        for j in range(num_triangles):
            if area_threshold_list[j]<= my_random and my_random<area_threshold_list[j+1]:
                #randomized_vertices.append(generate_randomized_triangle_point(triangle_list[j]))
                my_mean = generate_randomized_triangle_point(triangle_list[j])
                randomized_vertices.append(np.random.multivariate_normal(my_mean, my_cov))


    # print area_threshold_list
    # print generate_randomized_triangle_point(triangle_list[0])

    return np.array(randomized_vertices)
def generate_randomized_triangle_point(my_triangle):
    v1 = my_triangle[1,:]-my_triangle[0,:]
    v2 = my_triangle[2,:]-my_triangle[0,:]
    v3 = my_triangle[2,:]-my_triangle[1,:]
    v3 = v3/np.linalg.norm(v3)
    v3 = np.array([-v3[1],v3[0]])

    bound1_temp = np.dot(v3,my_triangle[0,:])
    bound2_temp = np.dot(v3,my_triangle[1,:])
    bound1=np.min([bound1_temp,bound2_temp])
    bound2=np.max([bound1_temp,bound2_temp])


    while True:
        np.random.rand(num_hull_vertices)
        a = np.random.rand(1)[0]
        b = np.random.rand(1)[0]

        candidate_point = my_triangle[0,:] + a*v1+b*v2
        q=np.dot(candidate_point,v3)
        if bound1<=q and q<=bound2:
            return candidate_point





if __name__ == '__main__':

    num_divisions = 1
  
    theta_range = 2*np.pi*(1.0*np.array(range(num_divisions)))/num_divisions


    hull_estimator = ConvexHullEstimator(theta_range=theta_range, quantile_value=.99, distance_threshold=.5, closed = True)
    hull_estimator.curvature_threshold = -0.0


    #fig, axs = plt.subplots(2,2)
    fig2, axs2 = plt.subplots(1,1)
    fig3, axs3 = plt.subplots(1,1)
    fig4, axs4 = plt.subplots(1,1)
    fig5, axs5 = plt.subplots(1,1)
    fig6, axs6 = plt.subplots(1,1)
    fig7, axs7 = plt.subplots(1,1)
    fig8, axs8 = plt.subplots(1,1)

    polygon_radius = 25
    #num_polygon_vertices = np.random.randint(low=3,high=10)
    num_polygon_vertices = 3
    theta_range_polygon = 2*np.pi*(1.0*np.array(range(num_polygon_vertices)))/num_polygon_vertices

    polygon_radii = polygon_radius*(np.random.rand(num_polygon_vertices)+.3)
    polygon_vertices = np.zeros([num_polygon_vertices,2])

    for i in range(num_polygon_vertices):
        polygon_vertices[i][0] = polygon_radii[i] * np.cos(theta_range_polygon[i])
        polygon_vertices[i][1] = polygon_radii[i] * np.sin(theta_range_polygon[i])

    # print polygon_vertices
    polygon_hull = ConvexHull(polygon_vertices)
    hull_vertices = polygon_vertices[polygon_hull.vertices]

    num_hull_vertices = len(polygon_hull.vertices)




    offset_radius = 15
    offset_coord = offset_radius*(np.random.rand(2)-.5)

    for i in range(num_hull_vertices):
        hull_vertices[i][0]+=offset_coord[0]
        hull_vertices[i][1]+=offset_coord[1]
        
        #axs8.plot(hull_vertices[i][0], hull_vertices[i][1], 'g.')

    max_x = np.max(np.transpose(hull_vertices)[0])
    min_x = np.min(np.transpose(hull_vertices)[0])
    delta_x =  max_x-min_x

    mean_x = (max_x+min_x)/2

    max_y = np.max(np.transpose(hull_vertices)[1])
    min_y = np.min(np.transpose(hull_vertices)[1])
    delta_y =  max_y-min_y

    mean_y = (max_y+min_y)/2


    range_max = .675 * np.max([delta_x,delta_y])

    # max_x+=delta_x/4
    # min_x-=delta_x/4

    # max_y+=delta_y/4
    # min_y-=delta_y/4

    max_x = mean_x + range_max
    min_x = mean_x - range_max

    max_y = mean_y + range_max
    min_y = mean_y - range_max

    num_data_points = 2000
    noise_radius = 2.0

    randomized_vertices = generate_randomized_convex_data(hull_vertices, num_hull_vertices, num_data_points,noise_radius)


    #randomized_vertices = randomized_vertices - noise_radius*(np.random.rand(num_data_points,2)-.5)

    # print np.array(randomized_vertices)
    naive_polygon_hull = ConvexHull(randomized_vertices)
    naive_hull_vertices = randomized_vertices[naive_polygon_hull.vertices]

    naive_num_hull_vertices = len(naive_polygon_hull.vertices)

    for i in range(num_data_points):
        # convex_combo = np.random.rand(num_hull_vertices)

        # cumulative_sum = convex_combo[0]
        # for j in range(1,num_hull_vertices-1):
        #     convex_combo[j]=(1-cumulative_sum)*convex_combo[j]
        #     cumulative_sum+=convex_combo[j]
        # convex_combo[-1] = 1-cumulative_sum

        # data_point = np.dot(np.random.permutation(convex_combo),hull_vertices)
        data_point = randomized_vertices[i,:]

        #data_point = data_point - noise_radius*(np.random.rand(2)-.5)

        hull_estimator.add_data_point(data_point)

        axs5.plot(data_point[0], data_point[1], 'r.')
        axs6.plot(data_point[0], data_point[1], 'r.')
        axs7.plot(data_point[0], data_point[1], 'r.')
        axs8.plot(data_point[0], data_point[1], 'r.')

        axs3.plot(data_point[0], data_point[1], 'r.')
        axs4.plot(data_point[0], data_point[1], 'r.')

    for i in range(num_hull_vertices):      
        axs3.plot([hull_vertices[i][0],hull_vertices[np.mod(i+1,num_hull_vertices)][0]], [hull_vertices[i][1],hull_vertices[np.mod(i+1,num_hull_vertices)][1]], 'g')
        axs4.plot([hull_vertices[i][0],hull_vertices[np.mod(i+1,num_hull_vertices)][0]], [hull_vertices[i][1],hull_vertices[np.mod(i+1,num_hull_vertices)][1]], 'g')
        axs8.plot([hull_vertices[i][0],hull_vertices[np.mod(i+1,num_hull_vertices)][0]], [hull_vertices[i][1],hull_vertices[np.mod(i+1,num_hull_vertices)][1]], 'g')

    for i in range(naive_num_hull_vertices):
        axs4.plot([naive_hull_vertices[i][0],naive_hull_vertices[np.mod(i+1,naive_num_hull_vertices)][0]], [naive_hull_vertices[i][1],naive_hull_vertices[np.mod(i+1,naive_num_hull_vertices)][1]], 'k')        

    last_update_time = time.time() 

    hull_estimator.generate_convex_hull_closed_polygon()

    print time.time() - last_update_time

    hull_estimator.initialize_quantile_boundary_plot(axs5)
    hull_estimator.initialize_quantile_polygon_plot(axs6)
    hull_estimator.initialize_polygon_star_plot(axs7)
    
    
    

    hull_estimator.update_quantile_boundary_plot()
    hull_estimator.update_quantile_polygon_plot()
    hull_estimator.update_polygon_star_plot()

    hull_estimator.initialize_final_constraint_plot(axs8)
    hull_estimator.update_final_constraint_plot()

    hull_estimator.initialize_side_detection_plot(axs2)
    hull_estimator.update_side_detection_plot()
    
    # hull_estimator.initialize_final_constraint_plot_left_right(axs8)
    # hull_estimator.update_final_constraint_plot_left_right()

    axs2.set_xlim([0, 4*np.pi])
    axs2.set_ylim([-2, 20])
    axs2.set_title('Inverse (Discrete) Curvature')

    axs5.set_xlim([min_x, max_x])
    axs5.set_ylim([min_y, max_y])
    axs5.set_title('Quantile Boundaries')

    axs6.set_xlim([min_x, max_x])
    axs6.set_ylim([min_y, max_y])
    axs6.set_title('Interior Polygon')

    axs7.set_xlim([min_x, max_x])
    axs7.set_ylim([min_y, max_y])
    axs7.set_title('Star Operation')

    axs8.set_xlim([min_x, max_x])
    axs8.set_ylim([min_y, max_y])
    axs8.set_title('Final Estimate')

    axs3.set_xlim([min_x, max_x])
    axs3.set_ylim([min_y, max_y])
    axs3.set_title('Initial Data')

    axs4.set_xlim([min_x, max_x])
    axs4.set_ylim([min_y, max_y])
    axs4.set_title('Naive Convex Hull')

    # axs5.set_xlim([-50, 50])
    # axs5.set_ylim([-50, 50])
    # axs5.set_title('Quantile Boundaries')

    # axs6.set_xlim([-50, 50])
    # axs6.set_ylim([-50, 50])
    # axs6.set_title('Interior Polygon')

    # axs7.set_xlim([-50, 50])
    # axs7.set_ylim([-50, 50])
    # axs7.set_title('Star Operation')

    # axs8.set_xlim([-50, 50])
    # axs8.set_ylim([-50, 50])
    # axs8.set_title('Final Estimate')
    plt.show()
