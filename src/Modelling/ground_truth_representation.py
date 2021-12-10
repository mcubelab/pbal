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

from polygon_representation import PolygonRepresentation
from cvxopt import matrix, solvers

class GroundTruthRepresentation(object):
    def __init__(self):
        self.hand_polygon = None                #polygon object representing the hand
        self.object_polygon = None              #polygon object representing the object
        self.ground_normal = None               #the upward facing surface normal of the ground (world frame)
        self.ground_offset = None
        self.ground_theta = None                #the angle of the ground relative to the horizontal
        self.s_hand_on_object = None            #the contact parameter corresponding to where on hand vertex of object is touching
                                                #or how far the face of the object has slid along the hand
        self.s_object_on_hand = None            #the contact parameter corresponding to where on the object the hand vertex is touching
                                                #or how far the face of the hand has slid along the object
        self.contact_mode_hand_object =None     #this describes which contact mode we are currently in between the hand and object. values can be
                                                #0: no contact
                                                #1: line (hand) - line (object)
                                                #2: point(hand) - line (object)
                                                #3: line(hand) - point (object)
                                                #4: point(hand) - point (object) (very unlikely, should never happen)
        self.s_object_on_ground = None          #the contact parameter corresponding to where on the ground the vertex of the object is touching
                                                #or how far the face of the object has slid along the ground
        self.contact_mode_ground_object = None  #this describes which contact mode we are currently in between the ground and object. values can be
                                                #0: no contact
                                                #1: line (ground) - line (object)
                                                #2: line (ground) - point (object)

        self.object_on_ground_face = None       #this is the face of the object in contact with the ground
        self.object_on_ground_vertices = []     #this is a list of the vertices of the object in contact with the ground
        

        self.object_on_hand_face = None        #this is the face of the object in contact with the hand
        self.object_on_hand_vertices = []      #this is a list of vertices of the object in contact with the hand

        self.hand_on_object_face = None       #this is the face of the hand in contact with the object
        self.hand_on_object_vertices = []     #this is a  list of vertices of the hand in contact with the object



        self.object_on_ground_face_plot = None       #plot of the face of the object in contact with the ground
        self.object_on_ground_vertices_plot = None     #plot of the vertices of the object in contact with the ground
        

        self.object_on_hand_face_plot = None        #plot of the face of the object in contact with the hand
        self.object_on_hand_vertices_plot = None      #plot of the vertices of the object in contact with the hand

        self.hand_on_object_face_plot = None       #plot of the face of the hand in contact with the object
        self.hand_on_object_vertices_plot = None     #plot of the vertices of the hand in contact with the object

        self.ground_contact_margin = 0.0
        self.ground_plot_exists = False
        self.ground_plot = None

        self.face_dot_margin = -1.0


    def update_ground(self,theta_ground=0.0, ground_offset=0.0):
        self.ground_normal=np.array([-np.sin(theta_ground),np.cos(theta_ground)])
        self.ground_offset = ground_offset

    def initialize_hand(self,L_contact=None,position=None,theta=None,collision_margin = 0):
        vertex_array = np.array([[-L_contact/2,L_contact/2],[0,0]])
        self.hand_polygon = PolygonRepresentation(
            vertex_array = vertex_array, position = position, theta = theta, is_closed = False, collision_margin = collision_margin)

    def initialize_object(self,vertex_array = None, position = None, theta = None, collision_margin = 0):
        self.object_polygon = PolygonRepresentation(
            vertex_array = vertex_array, position = position, theta = theta, is_closed = True, collision_margin = collision_margin)

    def set_collision_margins(self,hand_margin=0,object_margin=0):
        self.hand_polygon.update_collision_margin(collision_margin=hand_margin)
        self.object_polygon.update_collision_margin(collision_margin=object_margin)

    def update_hand_pose(self,position = None, theta = None):
        self.hand_polygon.update_pose(position = position, theta = theta)

    def update_object_pose(self,position = None, theta = None):
        self.object_polygon.update_pose(position = position, theta = theta)

    def update_ground_contacts(self):
        self.object_on_ground_vertices = self.find_ground_contact_vertices()
        if len(self.object_on_ground_vertices)==2:
            if np.abs(self.object_on_ground_vertices[0]-self.object_on_ground_vertices[1])==1:
                self.object_on_ground_face = np.min([self.object_on_ground_vertices[0],self.object_on_ground_vertices[1]])
            else:
                self.object_on_ground_face = 0
        else:
            self.object_on_ground_face = None

        #print 'vertices: ', self.object_on_ground_vertices, '  face: ', self.object_on_ground_face

    def update_hand_object_contacts(self):

        self.object_on_hand_face = None        #this is the face of the object in contact with the hand
        self.object_on_hand_vertices = []      #this is a list of vertices of the object in contact with the hand

        self.hand_on_object_face = None       #this is the face of the hand in contact with the object
        self.hand_on_object_vertices = []     #this is a  list of vertices of the hand in contact with the object


        #list of hand vertices that are touching object
        collision_list_hand = self.object_polygon.compute_collision_faces_world(self.hand_polygon.vertex_array_world)

        #list of object vertices that are touching hand
        collision_list_object = self.hand_polygon.compute_collision_faces_world(self.object_polygon.vertex_array_world)

        num_object_vertices_contact = len(collision_list_object)
        num_hand_vertices_contact = len(collision_list_hand)


        if num_object_vertices_contact == 0 and num_hand_vertices_contact == 0:
            #print 'No Contact'
            pass

        if num_object_vertices_contact == 2 and num_hand_vertices_contact < 2:
            #print 'Line-Line Contact'

            #this is a list of vertices of the object in contact with the hand
            self.object_on_hand_vertices = [collision_list_object[0][0],collision_list_object[1][0]]      

            if np.abs(self.object_on_hand_vertices[0]-self.object_on_hand_vertices[1])==1:
                #this is the face of the object in contact with the hand
                self.object_on_hand_face = np.min([self.object_on_hand_vertices[0],self.object_on_hand_vertices[1]])
            else:
                #this is the face of the object in contact with the hand
                self.object_on_hand_face = 0

            shared_faces = list(set(collision_list_object[0][1]) & set(collision_list_object[1][1]))
            if len(shared_faces)==1:
                #this is the face of the hand in contact with the object
                self.hand_on_object_face = shared_faces[0]

                #this is a  list of vertices of the hand in contact with the object      
                self.hand_on_object_vertices = [self.hand_on_object_face, np.mod(self.hand_on_object_face+1,self.hand_polygon.num_vertices)]     
            else: 
                print "either 0 or more than 1 shared faces of hand contact, not sure how to resolve"

        if  num_hand_vertices_contact == 2:
            #print 'Line-Line Contact'

            #this is a list of vertices of the hand in contact with the object
            self.hand_on_object_vertices = [collision_list_hand[0][0],collision_list_hand[1][0]]

            if np.abs(self.hand_on_object_vertices[0]-self.hand_on_object_vertices[1])==1:
                #this is the face of the hand in contact with the object
                self.hand_on_object_face = np.min([self.hand_on_object_vertices[0],self.hand_on_object_vertices[1]])
            else:
                #this is the face of the hand in contact with the object
                self.hand_on_object_face = 0

            shared_faces = list(set(collision_list_hand[0][1]) & set(collision_list_hand[1][1]))
            if len(shared_faces) == 1:

                #this is the face of the object in contact with the hand
                self.object_on_hand_face = shared_faces[0]

                #this is a  list of vertices of the object in contact with the hand      
                self.object_on_hand_vertices = [self.object_on_hand_face, np.mod(self.object_on_hand_face+1,self.object_polygon.num_vertices)]     

            else:
                if len(shared_faces) == 0:

                    candidate_object_face = self.object_polygon.min_dot_product_face(
                        self.hand_polygon.normal_list_world[:,self.hand_on_object_face],
                        self.face_dot_margin)

                    if candidate_object_face is not None:
                        test_normal = self.object_polygon.normal_list_world[:,candidate_object_face]
                        vertex_dot1 = np.dot(self.hand_polygon.vertex_array_world[:,self.hand_on_object_vertices[0]],
                                             test_normal)
                        vertex_dot2 = np.dot(self.hand_polygon.vertex_array_world[:,self.hand_on_object_vertices[1]],
                                             test_normal)
                        vertex_dot3 = np.dot(self.object_polygon.vertex_array_world[:,candidate_object_face],
                                             test_normal)

                        if (np.abs(vertex_dot1-vertex_dot3)<=self.object_polygon.collision_margin and
                            np.abs(vertex_dot2-vertex_dot3)<=self.object_polygon.collision_margin):
                            self.object_on_hand_face = candidate_object_face
                            self.object_on_hand_vertices = [self.object_on_hand_face, np.mod(self.object_on_hand_face+1,self.object_polygon.num_vertices)]

                        else:
                            self.hand_on_object_vertices = []
                            self.hand_on_object_face = None
                    else:
                        self.hand_on_object_vertices = []
                        self.hand_on_object_face = None

        if num_object_vertices_contact == 0 and num_hand_vertices_contact == 1:
            #print 'Hand (Point) - Object (Line)'

            #this is the face of the bloberta in contact with the hand
            self.hand_on_object_face = None

            #this is a list of vertices of the hand in contact with the object        
            self.hand_on_object_vertices = [collision_list_hand[0][0]]      

            #this is the face of the object in contact with the hand
            if len(collision_list_hand[0][1] == 1):
                self.object_on_hand_face = collision_list_hand[0][1][0]

                #this is a  list of vertices of the object in contact with the hand       
                self.object_on_hand_vertices = [self.object_on_hand_face, np.mod(self.object_on_hand_face+1,self.object_polygon.num_vertices)]


                test_normal = self.object_polygon.normal_list_world[:,self.object_on_hand_face]

                min_hand_vertex = self.hand_polygon.min_dot_product_vertex(test_normal, -1.0)[0]
                vertex_dot1 = np.dot(self.hand_polygon.vertex_array_world[:,min_hand_vertex],test_normal)
                vertex_dot2 = np.dot(self.object_polygon.vertex_array_world[:,self.object_on_hand_face],test_normal)

                if vertex_dot1 < vertex_dot2 - self.object_polygon.collision_margin:
                    self.hand_on_object_face = None
                    self.hand_on_object_vertices = []
                    self.object_on_hand_face = None
                    self.object_on_hand_vertices = []

            else:

                self.hand_on_object_vertices = []
                print 'hand vertex is touching 2+ faces of object, not sure how to resolve'

        if num_object_vertices_contact == 1 and num_hand_vertices_contact == 0:
            #print 'Hand (Line) - Object (Point)'

            #this is the face of the object in contact with the hand
            self.object_on_hand_face = None

            #this is a list of vertices of the object in contact with the hand        
            self.object_on_hand_vertices = [collision_list_object[0][0]]      

            #this is the face of the hand in contact with the object
            if len(collision_list_object[0][1] == 1):
                self.hand_on_object_face = collision_list_object[0][1][0]

                #this is a  list of vertices of the hand in contact with the object       
                self.hand_on_object_vertices = [self.hand_on_object_face, np.mod(self.hand_on_object_face+1,self.hand_polygon.num_vertices)]

                test_normal = self.hand_polygon.normal_list_world[:,self.hand_on_object_face]

                min_object_vertex = self.object_polygon.min_dot_product_vertex(test_normal, -1.0)[0]
                vertex_dot1 = np.dot(self.object_polygon.vertex_array_world[:,min_object_vertex],test_normal)
                vertex_dot2 = np.dot(self.hand_polygon.vertex_array_world[:,self.hand_on_object_face],test_normal)

                if vertex_dot1 < vertex_dot2 - self.hand_polygon.collision_margin:
                    self.hand_on_object_face = None
                    self.hand_on_object_vertices = []
                    self.object_on_hand_face = None
                    self.object_on_hand_vertices = []

            else:

                self.object_on_hand_vertices = []

                print 'object vertex is touching 2+ faces of hand, not sure how to resolve'

        if num_object_vertices_contact == 1 and num_hand_vertices_contact == 1:
            candidate_object_face = self.object_polygon.min_dot_product_face(
                self.hand_polygon.normal_list_world[:,0],
                1000.0)

            test_normal = self.object_polygon.normal_list_world[:,candidate_object_face]
            vertex_dot1 = np.dot(self.hand_polygon.vertex_array_world[:,0],
                                 test_normal)
            vertex_dot2 = np.dot(self.hand_polygon.vertex_array_world[:,1],
                                 test_normal)
            vertex_dot3 = np.dot(self.object_polygon.vertex_array_world[:,candidate_object_face],
                                 test_normal)


            if np.dot(self.object_polygon.normal_list_world[:,candidate_object_face],
                self.hand_polygon.normal_list_world[:,0])<=-np.cos(np.pi/6):
 
                if (np.abs(vertex_dot1-vertex_dot3)<=self.object_polygon.collision_margin and
                    np.abs(vertex_dot2-vertex_dot3)<=self.object_polygon.collision_margin):
                    self.hand_on_object_face = 0
                    self.hand_on_object_vertices = [0,1]
                    self.object_on_hand_face = candidate_object_face
                    self.object_on_hand_vertices = [self.object_on_hand_face, np.mod(self.object_on_hand_face+1,self.object_polygon.num_vertices)]
            else:
                if (np.abs(vertex_dot1-vertex_dot3)<=self.object_polygon.collision_margin and
                    np.abs(vertex_dot2-vertex_dot3)<=self.object_polygon.collision_margin):

                    self.hand_on_object_face = None
                    self.hand_on_object_vertices = [collision_list_hand[0][0]]

                    self.object_on_hand_face = candidate_object_face
                    self.object_on_hand_vertices = [self.object_on_hand_face, np.mod(self.object_on_hand_face+1,self.object_polygon.num_vertices)]
                else:
                    self.hand_on_object_face = 0
                    self.hand_on_object_vertices = [0,1]
                    self.object_on_hand_face = None
                    self.object_on_hand_vertices = [collision_list_object[0][0]]


        # print 'collision_list_hand: ', collision_list_hand
        # print 'collision_list_object: ', collision_list_object 

    def update_contacts(self):
        self.update_ground_contacts()
        self.update_hand_object_contacts()


    #initializes the plot of the polygon in the world frame
    def initialize_ground_plot(self, axis_in):
        if (self.ground_offset is not None) and (self.ground_normal is not None):
            ground_tangent=np.array([self.ground_normal[1],-self.ground_normal[0]])
            pt1 = ground_tangent*100 + self.ground_normal*self.ground_offset
            pt2 =-ground_tangent*100 + self.ground_normal*self.ground_offset
            ground_plot, = axis_in.plot([pt1[0],pt2[0]], [pt1[1],pt2[1]], 'k', linewidth=3)
            self.ground_plot_exists = True
            self.ground_plot = ground_plot

    def initialize_contacts_plot(self,axis_in):
        self.object_on_ground_face_plot, = axis_in.plot([0],[0], 'y', linewidth=3)
        self.object_on_hand_face_plot, = axis_in.plot([0],[0], 'y', linewidth=3)
        self.hand_on_object_face_plot, = axis_in.plot([0],[0], 'y', linewidth=3)

        self.object_on_ground_vertices_plot, = axis_in.plot([0],[0], 'mo', markersize = 10)
        self.object_on_hand_vertices_plot, = axis_in.plot([0],[0], 'mo', markersize = 10)
        self.hand_on_object_vertices_plot, = axis_in.plot([0],[0], 'mo', markersize = 10)

    def update_contacts_plot(self):
        if self.object_on_ground_face == None:
            self.object_on_ground_face_plot.set_xdata(0)
            self.object_on_ground_face_plot.set_ydata(0)
        else:
            self.object_on_ground_face_plot.set_xdata(self.object_polygon.vertex_array_world[0,self.object_on_ground_vertices])
            self.object_on_ground_face_plot.set_ydata(self.object_polygon.vertex_array_world[1,self.object_on_ground_vertices])

        if self.object_on_hand_face == None:
            self.object_on_hand_face_plot.set_xdata(0)
            self.object_on_hand_face_plot.set_ydata(0)
        else:
            self.object_on_hand_face_plot.set_xdata(self.object_polygon.vertex_array_world[0,self.object_on_hand_vertices])
            self.object_on_hand_face_plot.set_ydata(self.object_polygon.vertex_array_world[1,self.object_on_hand_vertices])

        if self.hand_on_object_face == None:
            self.hand_on_object_face_plot.set_xdata(0)
            self.hand_on_object_face_plot.set_ydata(0)
        else:
            self.hand_on_object_face_plot.set_xdata(self.hand_polygon.vertex_array_world[0,self.hand_on_object_vertices])
            self.hand_on_object_face_plot.set_ydata(self.hand_polygon.vertex_array_world[1,self.hand_on_object_vertices])


        if len(self.object_on_ground_vertices) == 0:
            self.object_on_ground_vertices_plot.set_xdata(0)
            self.object_on_ground_vertices_plot.set_ydata(0)
        else:
            self.object_on_ground_vertices_plot.set_xdata(self.object_polygon.vertex_array_world[0,self.object_on_ground_vertices])
            self.object_on_ground_vertices_plot.set_ydata(self.object_polygon.vertex_array_world[1,self.object_on_ground_vertices])

        if len(self.object_on_hand_vertices) == 0:
            self.object_on_hand_vertices_plot.set_xdata(0)
            self.object_on_hand_vertices_plot.set_ydata(0)
        else:
            self.object_on_hand_vertices_plot.set_xdata(self.object_polygon.vertex_array_world[0,self.object_on_hand_vertices])
            self.object_on_hand_vertices_plot.set_ydata(self.object_polygon.vertex_array_world[1,self.object_on_hand_vertices])

        if len(self.hand_on_object_vertices) == 0:
            self.hand_on_object_vertices_plot.set_xdata(0)
            self.hand_on_object_vertices_plot.set_ydata(0)
        else:
            self.hand_on_object_vertices_plot.set_xdata(self.hand_polygon.vertex_array_world[0,self.hand_on_object_vertices])
            self.hand_on_object_vertices_plot.set_ydata(self.hand_polygon.vertex_array_world[1,self.hand_on_object_vertices])

    def initialize_world_plot(self,axis_in):
        self.hand_polygon.initialize_polygon_plot(axis_in, 'b', 'b')
        self.hand_polygon.initialize_normal_plot(axis_in,'r')

        self.object_polygon.initialize_polygon_plot(axis_in, 'b', 'b')
        self.object_polygon.initialize_normal_plot(axis_in,'r')
        self.initialize_ground_plot(axis_in)
        self.initialize_contacts_plot(axis_in)

    def update_world_plot(self):
        self.hand_polygon.update_all_plots()
        self.object_polygon.update_all_plots()
        self.update_contacts_plot()

    def find_ground_contact_vertices(self):
        return self.object_polygon.min_dot_product_vertex(self.ground_normal, self.ground_contact_margin)

    def compute_ground_s(self):
        ground_tangent=np.array([self.ground_normal[1],-self.ground_normal[0]])
        return np.dot(ground_tangent,  
            self.object_polygon.vertex_array_world[:,self.object_on_ground_vertices])

    def compute_object_s(self):
        if self.object_on_hand_face is not None:
            hand_center = .5*self.hand_polygon.vertex_array_world[:,0] + .5*self.hand_polygon.vertex_array_world[:,1]

            s_list = []
            for pivot_index in self.object_on_ground_vertices:
                s_list.append(-self.object_polygon.compute_s_contact_world(X_world=hand_center,
                face_index=self.object_on_hand_face,
                pivot_index=pivot_index))
            return np.array(s_list)
        else:
            return np.array([])

    def compute_pivot_positions(self):
        return self.object_polygon.vertex_array_world[:,self.object_on_ground_vertices]

    def compute_object_theta(self):
        if self.object_on_hand_face is not None:
            return np.arctan2(
                self.object_polygon.normal_list_world[0,self.object_on_hand_face],
                self.object_polygon.normal_list_world[1,self.object_on_hand_face])
        else:
            # temp_face =  self.object_polygon.min_dot_product_face(
            #             np.array([0,-1.0]),
            #             1000.0)
            # return np.arctan2(
            #     self.object_polygon.normal_list_world[0,temp_face],
            #     self.object_polygon.normal_list_world[1,temp_face])
            return None

    def set_ground_contact_margin(self, ground_contact_margin=0):
        self.ground_contact_margin = ground_contact_margin

    def set_face_dot_margin(self, angle_threshold=0.0):
        self.face_dot_margin = -np.cos(angle_threshold)






