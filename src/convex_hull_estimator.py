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

class ConvexHullEstimator(object):
    def __init__(self, theta_range,quantile_value,distance_threshold=.5,closed=False):
        self.curvature_threshold = 0.0
        self.closed=closed
        self.theta_range=theta_range
        self.num_external_params = len(self.theta_range)
        self.A_external_stats_intermediate = np.zeros([self.num_external_params,2])
        self.B_external_stats_intermediate = np.zeros(self.num_external_params)
        self.stats_external_list = []
        self.distance_threshold = distance_threshold

        for i in range(self.num_external_params):
           self.stats_external_list.append(livestats.LiveStats([quantile_value]))
           self.A_external_stats_intermediate[i][0] = np.cos(self.theta_range[i])
           self.A_external_stats_intermediate[i][1] = np.sin(self.theta_range[i])

    def add_data_point(self,data_point):
        b_temp = np.dot(self.A_external_stats_intermediate,data_point)
        for i in range(self.num_external_params):
            self.stats_external_list[i].add(b_temp[i])

    def update_quantile_list(self):
        for i in range(self.num_external_params):
            self.B_external_stats_intermediate[i] = self.stats_external_list[i].quantiles()[0][1]

    def enumerate_vertices_of_constraint_polygon(self,theta_list,b_list,closed=True):
        if len(theta_list)<3:
            return np.array([]),np.array([])

        num_constraints = len(theta_list)
        A = np.zeros([num_constraints,2])
        B = np.array(b_list) + (1e-7)

        vertex_x_list = []
        vertex_y_list = []

        #build constraint matrix for the polygon
        for i in range(num_constraints):
            A[i][0] = np.cos(theta_list[i])
            A[i][1] = np.sin(theta_list[i])

        theta_A=theta_list[-1]
        theta_B=theta_list[0]
        while theta_A-theta_B>np.pi:
            theta_A=theta_A-2*np.pi
        while theta_A-theta_B<-np.pi:
            theta_A=theta_A+2*np.pi

        theta_start = (theta_A+theta_B)/2
        cost_start = np.array([-np.cos(theta_start),-np.sin(theta_start)])

        sol = solvers.lp(matrix(cost_start),matrix(A),matrix(B))

        # print np.transpose(np.array([theta_list,b_list]))
        if sol['x'] is None:
            return np.array([]),np.array([])

        #find the first vertex
        x_sol = np.squeeze(sol['x'])
        vertex_x_list.append(x_sol[0])
        vertex_y_list.append(x_sol[1])

        #active constraints for the first vertex
        dual_list = np.squeeze(sol['z'])
        dual_value_index_list = np.argsort(dual_list)
        # print np.sort(np.squeeze(sol['z'])) 

        if len(dual_value_index_list)<3:
            return np.array([]),np.array([])
        
        #find the active constraint for the first vertex
        # index_CW  = np.max([dual_value_index_list[-1],dual_value_index_list[-2]])
        # index_CCW = np.min([dual_value_index_list[-1],dual_value_index_list[-2]])
        
        theta_copy = copy.deepcopy(theta_list)
        CW_index_list = []
        CW_dual_list = []
        CCW_index_list = []
        CCW_dual_list = []

        for i in range(len(theta_copy)):
            while theta_copy[i]-theta_start>np.pi:
                theta_copy[i]-=2*np.pi
            while theta_copy[i]-theta_start<-np.pi:
                theta_copy[i]+=2*np.pi

            if theta_copy[i]>theta_start:
                CCW_index_list.append(i)
                CCW_dual_list.append(dual_list[i])
            if theta_copy[i]<theta_start:
                CW_index_list.append(i)
                CW_dual_list.append(dual_list[i])
        CW_dual_sorted_args = np.argsort(CW_dual_list)  
        CCW_dual_sorted_args = np.argsort(CCW_dual_list)  

        index_CW = CW_index_list[CW_dual_sorted_args[-1]]
        index_CCW = CCW_index_list[CCW_dual_sorted_args[-1]]


        vertex_index = [index_CW,index_CCW]

        # print [index_CW, index_CCW,len(A)]
        current_index_0 = index_CCW
        current_index_1 = current_index_0+1

        stopping_index = index_CW

        while current_index_1<=stopping_index:
            test_vertex = np.linalg.solve(A[[current_index_0,current_index_1]],B[[current_index_0,current_index_1]])

            interior_check = np.dot(A,test_vertex) < B
            interior_check[[current_index_0,current_index_1]] = True

            if all(interior_check):
                vertex_x_list.append(test_vertex[0])
                vertex_y_list.append(test_vertex[1])
                current_index_0 = current_index_1
                vertex_index.append(current_index_1)

            current_index_1 = current_index_1+1

        # print('length of vertex list:', len(vertex_x_list))
        # print('vertex_index_sequence:', vertex_index)
        # print('optimization solution:', np.squeeze(sol['x']))

        return np.array(vertex_x_list),np.array(vertex_y_list)

    def prune_vetices_by_curvature(self,vertex_x_list_in,vertex_y_list_in):

        theta_list, A, B = self.vertices_to_theta_and_offset(
                vertex_x_list_in,vertex_y_list_in,closed=True)

        theta_double_list=np.hstack([theta_list,theta_list])
        for i in range(1,len(theta_double_list)):
            while theta_double_list[i]-theta_double_list[i-1]>np.pi:
                theta_double_list[i]-=2*np.pi
            while theta_double_list[i]-theta_double_list[i-1]<-np.pi:
                theta_double_list[i]+=2*np.pi

        distance_list=np.zeros(len(vertex_x_list_in))

        for i in range(len(vertex_x_list_in)-1):
            distance_list[i]=np.sqrt(
                (vertex_x_list_in[i+1]-vertex_x_list_in[i])**2+
                (vertex_y_list_in[i+1]-vertex_y_list_in[i])**2)

        distance_list[-1]=np.sqrt(
            (vertex_x_list_in[-1]-vertex_x_list_in[0])**2+
            (vertex_y_list_in[-1]-vertex_y_list_in[0])**2)

        distance_list = distance_list/np.sum(distance_list)
        distance_double_list = np.hstack([distance_list,distance_list])

        slope_list = np.zeros(len(theta_double_list)-2)
        theta_middle_list = np.zeros(len(theta_double_list)-2)
        for i in range(1,len(theta_double_list)-1):
            slope_list[i-1]=2*np.pi*(distance_double_list[i])/(theta_double_list[i+1]-theta_double_list[i-1])-1
            theta_middle_list[i-1] = theta_double_list[i]

        
        theta_select_list = []
        slope_select_list = []
        index_select_list = []
        for i in range(1,len(slope_list)-1):
            if slope_list[i]>slope_list[i-1] and slope_list[i]>slope_list[i+1] and (
                theta_middle_list[i]>=np.pi and theta_middle_list[i]<3*np.pi and slope_list[i]>self.curvature_threshold):
                theta_select_list.append(theta_middle_list[i])
                slope_select_list.append(slope_list[i])
                index_select_list.append(np.mod(i+1,len(theta_list)))
        
        self.theta_middle_list = theta_middle_list
        self.slope_list = slope_list
        self.theta_select_list = copy.deepcopy(theta_select_list)
        self.slope_select_list = copy.deepcopy(slope_select_list)


        A_final = A[index_select_list]
        B_final = B[index_select_list]



        vertex_x_final_list,vertex_y_final_list= self.enumerate_vertices_of_constraint_polygon(theta_select_list,B_final,closed=True)


        return theta_select_list, A_final, B_final, vertex_x_final_list, vertex_y_final_list


    def vertices_to_theta_and_offset(self,vertex_x_list,vertex_y_list,closed=False):
        if closed:
            theta_list=np.zeros(len(vertex_x_list))
            b_list=np.zeros(len(vertex_x_list))
            A = np.zeros([len(vertex_x_list),2])
        else:
            theta_list=np.zeros(len(vertex_x_list)-1)
            b_list=np.zeros(len(vertex_x_list)-1)
            A = np.zeros([len(vertex_x_list)-1,2])

        for i in range(0,len(vertex_x_list)-1):
            theta_list[i] = np.arctan2(vertex_y_list[i+1]-vertex_y_list[i],vertex_x_list[i+1]-vertex_x_list[i])-np.pi/2
            b_list[i] = np.cos(theta_list[i])*vertex_x_list[i]+np.sin(theta_list[i])*vertex_y_list[i]
            A[i][0]=np.cos(theta_list[i])
            A[i][1]=np.sin(theta_list[i])
            while theta_list[i]>3*np.pi/2:
                theta_list[i]=theta_list[i]-2*np.pi
            while theta_list[i]<-np.pi/2:
                theta_list[i]=theta_list[i]+2*np.pi

        if closed:
            theta_list[-1] = np.arctan2(vertex_y_list[0]-vertex_y_list[-1],vertex_x_list[0]-vertex_x_list[-1])-np.pi/2
            b_list[-1] = np.cos(theta_list[-1])*vertex_x_list[-1]+np.sin(theta_list[-1])*vertex_y_list[-1]
            A[-1][0]=np.cos(theta_list[-1])
            A[-1][1]=np.sin(theta_list[-1])
            while theta_list[-1]>3*np.pi/2:
                theta_list[-1]=theta_list[-1]-2*np.pi
            while theta_list[-1]<-np.pi/2:
                theta_list[-1]=theta_list[-1]+2*np.pi

        return theta_list,A,b_list

    def merge_polygons_by_theta(self,vertex_x1_list,vertex_y1_list,vertex_x2_list,vertex_y2_list):
        theta_list_1, A1, B1 = self.vertices_to_theta_and_offset(vertex_x1_list,vertex_y1_list,closed=False)
        theta_list_2, A2, B2 = self.vertices_to_theta_and_offset(vertex_x2_list,vertex_y2_list,closed=False)

        theta_list_out = np.hstack([theta_list_1,theta_list_2])

        A_out = np.vstack([A1,A2])
        B_out = np.hstack([B1,B2])

        arg_list = np.argsort(theta_list_out)
        theta_list_out = theta_list_out[arg_list]
        A_out = A_out[arg_list]
        B_out = B_out[arg_list]

        return theta_list_out, A_out, B_out

    def split_constraints_left_right(self,theta_list,A,B):
        A1 = np.zeros([0,2])
        B1 = np.zeros(0)

        A2 = np.zeros([0,2])
        B2 = np.zeros(0)
        for i in range(len(theta_list)):
            while theta_list[i]>3*np.pi/2:
                theta_list[i]=theta_list[i]-2*np.pi
            while theta_list[i]<-np.pi/2:
                theta_list[i]=theta_list[i]+2*np.pi

            if theta_list[i]>=np.pi/2 and theta_list[i]<np.pi:
            # if theta_list[i]>=np.pi/2 and theta_list[i]<=3*np.pi/2:
                A1 = np.vstack([A1,A[i]])
                B1 = np.hstack([B1,B[i]])
            if theta_list[i]<np.pi/2 and theta_list[i]>0:
            # if theta_list[i]<np.pi/2 and theta_list[i]>=-np.pi/2:
                A2 = np.vstack([A2,A[i]])
                B2 = np.hstack([B2,B[i]])

        return A1,B1,A2,B2


    def initialize_line_plot_list(self,axis_in, num_lines, color_in):
        line_list=[]
        for i in range(num_lines):
            line_plot, = axis_in.plot([0], [0], color_in)
            line_list.append(line_plot)
        return line_list

    def update_constraint_line_plots(self,line_list,A,B):
        for i in range(len(line_list)):
            if i<len(B):
                P0 =[A[i][0]*B[i],A[i][1]*B[i]]
                line_list[i].set_xdata([P0[0]-100*A[i][1],P0[0]+100*A[i][1]])
                line_list[i].set_ydata([P0[1]+100*A[i][0],P0[1]-100*A[i][0]])
            else:
                line_list[i].set_xdata(0)
                line_list[i].set_ydata(0)

    #quantile boundaries only    
    def initialize_quantile_boundary_plot(self,axis_in):
        self.quantile_constraint_plots_A = self.initialize_line_plot_list(axis_in, self.num_external_params, 'b')

    def update_quantile_boundary_plot(self):
        self.update_constraint_line_plots(self.quantile_constraint_plots_A, self.A_external_stats_intermediate, self.B_external_stats_intermediate)

    #quantile boundaries and interior polygon of quantile boundaries
    def initialize_quantile_polygon_plot(self,axis_in):
        self.quantile_constraint_plots_B = self.initialize_line_plot_list(axis_in, self.num_external_params, 'b')
        self.exterior_polygon_plot_A, = axis_in.plot([0], [0], 'k',linewidth=3)

    def update_quantile_polygon_plot(self):
        self.update_constraint_line_plots(self.quantile_constraint_plots_B, self.A_external_stats_intermediate, self.B_external_stats_intermediate)
        if len(self.exterior_polygon_vertex_x_list)>0:
            if self.closed:
                self.exterior_polygon_plot_A.set_xdata(np.hstack([self.exterior_polygon_vertex_x_list,
                    self.exterior_polygon_vertex_x_list[0]]))
                self.exterior_polygon_plot_A.set_ydata(np.hstack([self.exterior_polygon_vertex_y_list,
                    self.exterior_polygon_vertex_y_list[0]]))
            else:
                self.exterior_polygon_plot_A.set_xdata(self.exterior_polygon_vertex_x_list)
                self.exterior_polygon_plot_A.set_ydata(self.exterior_polygon_vertex_y_list)

    #polygon of quantile boundaries
    def initialize_polygon_plot(self,axis_in):
        self.exterior_polygon_plot_B, = axis_in.plot([0], [0], 'k',linewidth=3)

    def update_polygon_plot(self):
        if len(self.exterior_polygon_vertex_x_list)>0:
            if self.closed:
                self.exterior_polygon_plot_B.set_xdata(np.hstack([self.exterior_polygon_vertex_x_list,
                    self.exterior_polygon_vertex_x_list[0]]))
                self.exterior_polygon_plot_B.set_ydata(np.hstack([self.exterior_polygon_vertex_y_list,
                    self.exterior_polygon_vertex_y_list[0]]))
            else:
                self.exterior_polygon_plot_B.set_xdata(self.exterior_polygon_vertex_x_list)
                self.exterior_polygon_plot_B.set_ydata(self.exterior_polygon_vertex_y_list)

    #exterior polygon, the star, and the final polygon
    def initialize_polygon_star_plot(self,axis_in):
        self.exterior_polygon_plot_C, = axis_in.plot([0], [0], 'k',linewidth=3)
        self.star_plot_A, = axis_in.plot([0], [0], 'b',linewidth=3)
        self.star_plot_B, = axis_in.plot([0], [0], 'b',linewidth=3)
        #self.final_polygon_plot_A, = axis_in.plot([0], [0], 'g',linewidth=3)

    def update_polygon_star_plot(self):
        if len(self.final_polygon_vertex_x_list)>0:
            if self.closed:
                self.exterior_polygon_plot_C.set_xdata(np.hstack([self.exterior_polygon_vertex_x_list,
                    self.exterior_polygon_vertex_x_list[0]]))
                self.exterior_polygon_plot_C.set_ydata(np.hstack([self.exterior_polygon_vertex_y_list,
                    self.exterior_polygon_vertex_y_list[0]]))

                # self.star_plot_A.set_xdata(np.hstack([self.star_vertex_x_list_A,
                #     self.star_vertex_x_list_A[0]]))
                # self.star_plot_A.set_ydata(np.hstack([self.star_vertex_y_list_A,
                #     self.star_vertex_y_list_A[0]]))

                self.star_plot_B.set_xdata(np.hstack([self.star_vertex_x_list_B,
                    self.star_vertex_x_list_B[0]]))
                self.star_plot_B.set_ydata(np.hstack([self.star_vertex_y_list_B,
                    self.star_vertex_y_list_B[0]]))

                # self.final_polygon_plot_A.set_xdata(np.hstack([self.final_polygon_vertex_x_list,
                #     self.final_polygon_vertex_x_list[0]]))
                # self.final_polygon_plot_A.set_ydata(np.hstack([self.final_polygon_vertex_y_list,
                #     self.final_polygon_vertex_y_list[0]]))
            else:
                self.exterior_polygon_plot_C.set_xdata(self.exterior_polygon_vertex_x_list)
                self.exterior_polygon_plot_C.set_ydata(self.exterior_polygon_vertex_y_list)

                # self.star_plot_A.set_xdata(self.star_vertex_x_list_A)
                # self.star_plot_A.set_ydata(self.star_vertex_y_list_A)

                self.star_plot_B.set_xdata(self.star_vertex_x_list_B)
                self.star_plot_B.set_ydata(self.star_vertex_y_list_B)

                # self.final_polygon_plot_A.set_xdata(self.final_polygon_vertex_x_list)
                # self.final_polygon_plot_A.set_ydata(self.final_polygon_vertex_y_list)

    #final polygon and corresponding constraints
    def initialize_final_constraint_plot(self,axis_in):
        self.final_constraint_plots = self.initialize_line_plot_list(axis_in, self.num_external_params, 'b')
        self.final_polygon_plot_B, = axis_in.plot([0], [0], 'k',linewidth=3)

    def update_final_constraint_plot(self):
        if len(self.B_final)>0:
            self.update_constraint_line_plots(self.final_constraint_plots, self.A_final, self.B_final)

        if len(self.final_polygon_vertex_x_list)>0:
            if self.closed:
                self.final_polygon_plot_B.set_xdata(np.hstack([self.final_polygon_vertex_x_list,
                    self.final_polygon_vertex_x_list[0]]))
                self.final_polygon_plot_B.set_ydata(np.hstack([self.final_polygon_vertex_y_list,
                    self.final_polygon_vertex_y_list[0]]))
            else:
                self.final_polygon_plot_B.set_xdata(self.final_polygon_vertex_x_list)
                self.final_polygon_plot_B.set_ydata(self.final_polygon_vertex_y_list)

    def initialize_final_constraint_plot_left_right(self,axis_in):
        self.final_constraint_plots_left = self.initialize_line_plot_list(axis_in, self.num_external_params, 'g')
        self.final_constraint_plots_right = self.initialize_line_plot_list(axis_in, self.num_external_params, 'b')
        

    def update_final_constraint_plot_left_right(self):
        if len(self.B_left)>0:
            self.update_constraint_line_plots(self.final_constraint_plots_left, self.A_left, self.B_left)
        if len(self.B_right)>0:
            self.update_constraint_line_plots(self.final_constraint_plots_right, self.A_right, self.B_right)


    def initialize_side_detection_plot(self,axis_in):
        self.side_detection_plot_A, = axis_in.plot([0],[0],'b')
        self.side_detection_plot_B, = axis_in.plot([0],[0],'ro')

    def update_side_detection_plot(self):
        if len(self.B_final)>0:
            self.side_detection_plot_A.set_xdata(self.theta_middle_list)
            self.side_detection_plot_A.set_ydata(self.slope_list)

            self.side_detection_plot_B.set_xdata(self.theta_select_list)
            self.side_detection_plot_B.set_ydata(self.slope_select_list)

    def generate_convex_hull_closed_polygon(self):

        self.final_polygon_vertex_x_list = np.zeros(0)
        self.final_polygon_vertex_y_list = np.zeros(0)
        self.A_final = np.zeros([2,0])
        self.B_final = np.zeros(0)
        self.A_right = np.zeros([2,0])
        self.B_right = np.zeros(0)
        self.A_left = np.zeros([2,0])
        self.B_left = np.zeros(0)

        self.update_quantile_list()

        #find interior polygon of the boundaries from the quantile estimates
        self.exterior_polygon_vertex_x_list, self.exterior_polygon_vertex_y_list = self.enumerate_vertices_of_constraint_polygon(self.theta_range,self.B_external_stats_intermediate,closed=True)

        if len(self.exterior_polygon_vertex_x_list)>=3:
            #repeat the first two vertices purpose of making sure the star catches all edges
            vertex_x_list= np.hstack([self.exterior_polygon_vertex_x_list,self.exterior_polygon_vertex_x_list[0],self.exterior_polygon_vertex_x_list[1]])
            vertex_y_list= np.hstack([self.exterior_polygon_vertex_y_list,self.exterior_polygon_vertex_y_list[0],self.exterior_polygon_vertex_y_list[1]])

            #form the star
            self.star_vertex_x_list_A = vertex_x_list[range(0,len(vertex_x_list),2)]
            self.star_vertex_y_list_A = vertex_y_list[range(0,len(vertex_y_list),2)]

            self.star_vertex_x_list_B = vertex_x_list[range(1,len(vertex_x_list),2)]
            self.star_vertex_y_list_B = vertex_y_list[range(1,len(vertex_y_list),2)]

            #merge the two star boundaries
            theta_final_with_redundancies, A_final_with_redundancies, B_final_with_redundancies = self.merge_polygons_by_theta(
                self.star_vertex_x_list_A, self.star_vertex_y_list_A,
                self.star_vertex_x_list_B, self.star_vertex_y_list_B)

            #find the interior polygon of the merged set of star boundaries
            vertex_x_list, vertex_y_list = self.enumerate_vertices_of_constraint_polygon(
                theta_final_with_redundancies,B_final_with_redundancies,closed=True)

            #if a polygon was actually generated then...
            if len(vertex_x_list)>0:
                #select the best faces of the interior polygon of the merged boundaries
                self.theta_final, self.A_final, self.B_final, self.final_polygon_vertex_x_list, self.final_polygon_vertex_y_list = self.prune_vetices_by_curvature(
                    vertex_x_list,
                    vertex_y_list)

                self.A_right,self.B_right,self.A_left,self.B_left = self.split_constraints_left_right(self.theta_final,self.A_final,self.B_final)


    def return_left_right_friction_dictionary(self):
        return {
            "aer": np.hstack([self.A_right,np.zeros([len(self.A_right),1])]).tolist(),
            "ber": self.B_right.tolist(),
            "ael": np.hstack([self.A_left,np.zeros([len(self.A_left),1])]).tolist(),
            "bel": self.B_left.tolist(),
            "elu": (len(self.B_left)>0),
            "eru": (len(self.B_right)>0)
            }
    def return_polygon_representation_dictionary(self):
        return {
            "xl": self.final_polygon_vertex_x_list.tolist(),
            "yl": self.final_polygon_vertex_y_list.tolist(),
            "thl": self.theta_final,
            "A": self.A_final.tolist(),
            "B": self.B_final.tolist()
            }

