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

class PolygonRepresentation(object):
    #internal variables:
    #num_faces               #number of faces of the polygon
    #num_vertices            #number of vertices of the polygon
    #vertex_array_base       #array of vertices in the base frame  [coordinate,vertex_index]
    #vertex_array_world      #array of vertices in the world frame [coordinate,vertex_index]
    #tangential_list_base    #list of unit tangent vectors for each face  [coordinate,face_index]
    #normal_list_base        #list of outward facing normals in the base frame  [coordinate,face_index]
    #normal_list_world       #list of outward facing normals in the world frame [coordinate,face_index]
    #position                #position of the polygon in the world frame
    #theta                   #orientation of the polygon in the world frame
    #rotation_matrix         #rotation matrix corresponding to theta
    #is_closed               #true if the polygon is closed (first and last vertices form an edge). Zero otherwise
    #polygon_plot            #plot of the boundary of the polygon, in the world frame
    #polygon_plot_exists     #true if polygon_plot has been initialized, false otherwise
    #normal_plot_list        #list of plots of the outward facing surface normals of the polygon, in the world frame
    #normal_plot_exists      #true if normal_plot has been initialized, false otherwise
    #winding_number          #value is 1 if the order of vertices is counterclockwise, -1 if clockwise, 0 if fewer than 3 vertices
                             #or if the polygon is not closed
    #collision_margin        #maximum distance between a point and the face of a polygon to be considered in collision
    #A1                      #Collision check matrices used to determine if a point is in collision with a face of polygon
    #B1_min                  #the condition for collision between face i and point X is given by:
    #B1_max                  #(B1_min)_i <= (A1*X)_i <= (B1_max)_i AND (B2_min)_i <= (A2*X)_i <= (B2_max)_i
    #A2                      #note that this computation is done the BASE FRAME!!!
    #B2_min
    #B2_max
    #normal_plot_axis        #axis used for plotting the normal plots
    #normal_color            #color used for the normal plots
    #normal_plot_length      #length of arrors used for normal plots (will be some fraction of the perimeter)
    #perimeter_length        #length of perimeter of the object

    def __init__(self,vertex_array = None, position = None, theta = None, is_closed = True, collision_margin = 0):
        self.rot_mat_90_CCW = np.array([[0,-1],[1,0]])
        self.rot_mat_90_CW = np.array([[0,1],[-1,0]])

        self.is_closed = is_closed
        self.polygon_plot_exists = False 
        self.normal_plot_exists = False
        self.collision_margin = collision_margin

        self.rotation_matrix = None
        self.update_vertex_array_base(vertex_array = vertex_array)
        self.update_pose(position = position, theta = theta)

        

    def generate_base_parameters(self):
        self.generate_tangentials_base()
        self.update_winding_number()
        self.generate_normals_base()
        self.update_perimeter()
        self.update_collision_check_matrices()

    #update the winding_number of the polygon given the list of vertices in the base frame
    def update_winding_number(self):
        if self.vertex_array_base is None:
            self.winding_number =  None
            return

        if (not self.is_closed) or self.num_vertices<3:
            self.winding_number=0
        else:
            theta = 0.0 #counts the total exterior angle
            for i in range(self.num_vertices):
                v1 = self.tangential_list_base[:,i]
                v2 = self.tangential_list_base[:,np.mod(i+1,self.num_vertices)]
                delta_theta = np.arccos(np.dot(v1,v2))
                while delta_theta>np.pi:
                    delta_theta=delta_theta-2*np.pi
                while delta_theta<-np.pi:
                    delta_theta=delta_theta+2*np.pivot
                if np.cross(v1,v2)>=0:
                    theta=theta+delta_theta
                else:
                    theta=theta-delta_theta
            self.winding_number=np.round(theta/(2*np.pi))

    def update_rotation_matrix(self):
        if self.theta is None:
            self.rotation_matrix is None
        else:
            self.rotation_matrix = np.array([[np.cos(self.theta),-np.sin(self.theta)],[np.sin(self.theta),np.cos(self.theta)]])

    #generates a list of tangential vectors in the base frame using the list of vertices in the base frame
    def generate_tangentials_base(self):
        if self.vertex_array_base is None:
            self.tangential_list_base =  None
            return
        self.tangential_list_base = np.zeros([2,self.num_faces])
        for i in range(self.num_faces):
            v = self.vertex_array_base[:,np.mod(i+1,self.num_vertices)]-self.vertex_array_base[:,i]
            v = v/np.linalg.norm(v)
            self.tangential_list_base[:,i] = v

    #generates a list of outward facing surface normals in the base frame using the list of vertices in the base frame
    def generate_normals_base(self):
        if self.vertex_array_base is None:
            self.normal_list_base =  None
            return
        
        if self.winding_number>=0:
            self.normal_list_base = np.dot(self.rot_mat_90_CW,self.tangential_list_base)
        else:
            self.normal_list_base = np.dot(self.rot_mat_90_CCW,self.tangential_list_base)


    #updates the list of outward facing surface normals in the world frame
    def update_normals_world(self):
        if (self.rotation_matrix is not None) and (self.normal_list_base is not None):
            self.normal_list_world = np.dot(self.rotation_matrix,self.normal_list_base)
        else:
            self.normal_list_world = None

    #updates the list of vertices in the world frame
    def update_vertices_world(self):
        if (self.rotation_matrix is not None) and (self.normal_list_base is not None) and (self.position is not None):
            self.vertex_array_world = np.dot(self.rotation_matrix,self.vertex_array_base)
            self.vertex_array_world[0]+=self.position[0]
            self.vertex_array_world[1]+=self.position[1]
        else:
            self.vertex_array_world = None

    #updates the polygon geometry in the world frame
    def update_polygon_world(self):
        self.update_normals_world()
        self.update_vertices_world()

    def update_collision_margin(self, collision_margin=0):
        self.collision_margin = collision_margin
        self.update_collision_check_matrices()

    #computes the collision check matrices in the base frame
    def update_collision_check_matrices(self):
        self.A1 = np.transpose(self.normal_list_base)
        self.B1_min = np.zeros(self.num_faces)
        self.B1_max = np.zeros(self.num_faces)

        self.A2 = np.transpose(self.tangential_list_base)
        self.B2_min = np.zeros(self.num_faces)
        self.B2_max = np.zeros(self.num_faces)

        for i in range(self.num_faces):
            val_1 = np.dot(self.A1[i,:],self.vertex_array_base[:,i])
            val_2 = [np.dot(self.A2[i,:],self.vertex_array_base[:,i]),np.dot(self.A2[i,:],self.vertex_array_base[:,np.mod(i+1,self.num_vertices)])]
            self.B1_min[i] = val_1 - self.collision_margin
            self.B1_max[i] = val_1 + self.collision_margin    
            self.B2_min[i] = np.min(val_2)
            self.B2_max[i] = np.max(val_2)    

    #given some input point X_base in the base frame, computes the faces that are in collision with X_base
    def compute_collision_faces_base(self,X_base_list):
        check_matrix_1 = np.dot(self.A1,X_base_list)
        check_matrix_2 = np.dot(self.A2,X_base_list)

        face_list = []
        for i in range(len(X_base_list[0])):
            collision_list = np.argwhere(
                (check_matrix_1[:,i]>=self.B1_min)*
                (check_matrix_1[:,i]<=self.B1_max)*
                (check_matrix_2[:,i]>=self.B2_min)*
                (check_matrix_2[:,i]<=self.B2_max)
                )[:,0]
            if len(collision_list)>0:
                face_list.append([i,collision_list])
        return face_list

    #given some input point X_world in the world frame, computes the faces that in collision with X_world
    def compute_collision_faces_world(self,X_world_list):
        X_base_list = self.transform_world_point_to_base_point(X_world_list)
        return self.compute_collision_faces_base(X_base_list)

    #converts a point X_world given in the world frame to X_base, which is the corresponding point in the base frame
    def transform_world_point_to_base_point(self,X_world_list):
        X_base_list = X_world_list + 0
        X_base_list[0]-=self.position[0]
        X_base_list[1]-=self.position[1]
        X_base_list = np.dot(np.transpose(self.rotation_matrix),X_base_list)
        return X_base_list

    def compute_s_contact_world(self,X_world,face_index,pivot_index=None):
        return self.compute_s_contact_base(X_base = self.transform_world_point_to_base_point(X_world),
                                            face_index=face_index, pivot_index= pivot_index)

    #computes the corresponding s value given a point X_base in the base frame, a face of the polygon with index face_index,
    #and a pivot of the polygon with index pivot_index. If pivot_index = None, the s=0 corresponds to the centroid of the face
    def compute_s_contact_base(self,X_base,face_index,pivot_index=None):
        if pivot_index is None:
            return (np.dot(self.tangential_list_base[:,face_index],X_base)
                -.5*np.dot(self.tangential_list_base[:,face_index],self.vertex_array_base[:,face_index])
                -.5*np.dot(self.tangential_list_base[:,face_index],self.vertex_array_base[:,np.mod(face_index+1,self.num_vertices)]))
        else:
            return (np.dot(self.tangential_list_base[:,face_index],X_base)
                -np.dot(self.tangential_list_base[:,face_index],self.vertex_array_base[:,pivot_index]))

    #initializes the plot of the polygon in the world frame
    def initialize_polygon_plot(self, axis_in, edge_color, vertex_color):
        self.polygon_plot_exists = True

        polygon_plot, = axis_in.plot([0], [0], edge_color, linewidth=3)
        self.polygon_plot = polygon_plot

        self.update_polygon_plot()

    #updates the plot of the polygon in the world frame
    def update_polygon_plot(self):
        if self.is_closed:
            self.polygon_plot.set_xdata(np.hstack([self.vertex_array_world[0,:],self.vertex_array_world[0,:]]))
            self.polygon_plot.set_ydata(np.hstack([self.vertex_array_world[1,:],self.vertex_array_world[1,:]]))
        else:
            self.polygon_plot.set_xdata(self.vertex_array_world[0,:])
            self.polygon_plot.set_ydata(self.vertex_array_world[1,:])


    #initializes the plots of the surface normals in the world frame
    def initialize_normal_plot(self, axis_in, normal_color):
        self.normal_plot_exists = True
        self.normal_plot_list = []

        for i in range(self.num_faces):
            normal_plot, = axis_in.plot([0], [0], normal_color, linewidth=3)
            self.normal_plot_list.append(normal_plot)

        self.update_normal_plot()

    #updates the plots of the surface normals in the world frame
    def update_normal_plot(self):
        if len(self.normal_plot_list)<self.num_faces:
            self.add_more_normal_plots()

        for i in range(len(self.normal_plot_list)):
            if i<self.num_faces:
                v = .5*self.vertex_array_world[:,np.mod(i+1,self.num_vertices)]+.5*self.vertex_array_world[:,i]
                self.normal_plot_list[i].set_xdata([v[0],v[0]+.1*self.perimeter_length*self.normal_list_world[0,i]])
                self.normal_plot_list[i].set_ydata([v[1],v[1]+.1*self.perimeter_length*self.normal_list_world[1,i]])
            else:
                self.normal_plot_list[i].set_xdata([0])
                self.normal_plot_list[i].set_ydata([0])

    #if there are few plot objects of the surface normals than are necessary, will add more
    def add_more_normal_plots(self):
        while len(self.normal_plot_list)<self.num_faces:
            normal_plot, = axis_in.plot([0], [0], normal_color, linewidth=3)
            self.normal_plot_list.append(normal_plot)

    #updates the length of the object perimeter
    def update_perimeter(self):
        if self.vertex_array_base is None:
            self.perimeter_length=None
            return
        self.perimeter_length = 0
        for i in range(self.num_faces):
            v = self.vertex_array_base[:,np.mod(i+1,self.num_vertices)]-self.vertex_array_base[:,np.mod(i,self.num_vertices)]
            self.perimeter_length = self.perimeter_length+ np.linalg.norm(v)

    #updates all existing plots of the system
    def update_all_plots(self):
        if self.polygon_plot_exists:
            self.update_polygon_plot()
        if self.normal_plot_exists:
            self.update_normal_plot()

    #updates the position and orientation of the polygon in the world frame
    def update_pose(self,position = None, theta = None):
        self.position = position
        self.theta = theta
        self.update_rotation_matrix()
        self.update_polygon_world()

    #updates the list of vertices in the base frame
    def update_vertex_array_base(self,vertex_array = None):
        self.vertex_array_base = vertex_array

        if self.vertex_array_base is not None:
            self.num_vertices = len(self.vertex_array_base[0])
            if self.is_closed:
                self.num_faces = self.num_vertices
            else:
                self.num_faces = np.max(self.num_vertices - 1,0)
        else:
            self.num_vertices = 0
            self.num_faces = 0

        self.generate_base_parameters()
        self.update_polygon_world()

    def min_dot_product_vertex(self,normal_in,contact_margin):
        if self.vertex_array_world is not None:
            dot_product_arg_list = np.argsort(np.dot(normal_in,self.vertex_array_world))
            if np.dot(normal_in,self.vertex_array_world[:,dot_product_arg_list[1]]-
                self.vertex_array_world[:,dot_product_arg_list[0]])<contact_margin:
                return [dot_product_arg_list[0],dot_product_arg_list[1]]
            else:
                return [dot_product_arg_list[0]]
        else:
            return None

    def min_dot_product_face(self,normal_in,face_dot_margin):
        if self.normal_list_world is not None:
            dot_product_arg_list = np.argsort(np.dot(normal_in,self.normal_list_world))
            if np.dot(normal_in,self.normal_list_world[:,dot_product_arg_list[0]])<face_dot_margin:
                return dot_product_arg_list[0]
            else:
                return None
        else:
            return None







