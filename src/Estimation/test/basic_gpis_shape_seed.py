import os,sys,inspect

currentdir = os.path.dirname(os.path.abspath(
inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, gparentdir)

# sys.path.append('/home/taylorott/Documents/gpis-touch-public/mex')
sys.path.append('/home/thecube/Documents/gpis-touch-public/mex')

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


class basic_gpis_shape_seed(object):
    def __init__(self):
        pass

    def load_shape_data(self,name_in):
        curr_dir = os.path.dirname(
            os.path.abspath(inspect.getfile(inspect.currentframe())))
        parentdir = os.path.dirname(curr_dir)
        gparentdir = os.path.dirname(parentdir)
        print('parentdir', parentdir)
        fname = os.path.join(parentdir, 'Modelling', 'shape_description',
                             name_in + ".json")
        f = open(fname)
        # f = open("/home/oneills/Documents/panda/base_ws/src/franka_ros_interface/franka_ros_controllers/scripts/models/shape_description/"+name_in+".json")
        shape_data = json.load(f)

        vertex_array = np.array([shape_data["x_vert"], shape_data["y_vert"]])
        hull = scipy.spatial.ConvexHull(np.array(vertex_array).transpose())
        vertex_array = [vertex_array[0][hull.vertices],vertex_array[1][hull.vertices]]
        print('hello!')
        print(vertex_array)
        apriltag_id = shape_data["apriltag_id"]
        apriltag_pos = shape_data["centroid_to_apriltag"]

        return vertex_array, apriltag_id, apriltag_pos


    def get_perimeter(self,vertex_array):
        perimeter = 0

        n_points = len(vertex_array[0])
        for i in range(len(vertex_array[0])):
            dx = vertex_array[0][(i-1)%n_points]-vertex_array[0][i]
            dy = vertex_array[1][(i-1)%n_points]-vertex_array[1][i]
            perimeter+= np.sqrt(dx**2+dy**2)
        return perimeter

    #this only increases the resolution, does not decrease it
    def rescale_resolution(self,vertex_array,np_target):
        perimeter = self.get_perimeter(vertex_array)
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

    def prune_by_length(self,vertex_array,l_min,num_iter = 1):

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

    def smooth_perimeter(self,vertex_array,num_iter = 40):
        for i in range(num_iter):
            list_out = []
            for i in range(len(vertex_array)):
                q = list(vertex_array[i])
                q+=[q[0]]

                list_out.append(np.convolve(np.array(q),[.5,.5],mode='valid'))
            vertex_array = np.array(list_out)
        return vertex_array
            
    def update_gp(self,my_gp,vertex_array):
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

    def init_gp(self, shape_name=None):
        varNoise = [4e-5, 4e-5, 4e-5] # measurement noise #variance terms, length units
        priorNoise = [1e-2, 1e-2, 1e-2] # circular prior noise #variance terms, length units
        priorRad = 0.03 #radius of circle used in initialization
        testRes =  0.01 #gridpoint spacing 
        testLim = 1.5 #scaling * ground truth size of object

        # isLocal = True
        isLocal = False


        # dll_fname='/home/taylorott/Documents/gpis-touch-public/mex/build/libgp_python.so'
        dll_fname='/home/thecube/Documents/gpis-touch-public/mex/build/libgp_python.so'
        my_gp = gp_wrapper(dll_fname)
        my_gp.init_gp(varNoise, priorNoise, testLim, testRes, isLocal, priorRad)

        if shape_name is not None:
            vertex_array, apriltag_id, apriltag_pos = self.load_shape_data(shape_name)
            # if sum(vertex_array[1])>0:
            #     vertex_array[1] = -vertex_array[1]

            va_rescaled = vertex_array
            va_rescaled = self.rescale_resolution(va_rescaled,500)
            my_perimeter = self.get_perimeter(va_rescaled)
            va_rescaled = self.prune_by_length(va_rescaled,my_perimeter/500,10)
            va_rescaled = self.prune_by_length(va_rescaled,my_perimeter/200,2)
            va_rescaled = self.smooth_perimeter(va_rescaled)

            self.update_gp(my_gp,va_rescaled)

            return my_gp,va_rescaled

        return my_gp,[]



    