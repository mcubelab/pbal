#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np

class BoundaryEstimator(object):
    def __init__(self, update_rate = .05):
        self.max_val = None
        self.min_val = None
        self.boundary_val = None
        self.update_rate = update_rate
        self.average_val = None
        self.num_data_points = 0

    def add_data_point(self,val,can_increase = False, can_decrease = False):
        if self.num_data_points==0:
            self.max_val = val
            self.min_val = val
            self.average_val = val
        else:
            self.max_val = max(self.max_val,val)
            self.min_val = min(self.min_val,val)
            self.average_val = self.average_val*(self.num_data_points/(self.num_data_points+1.0))+val*(1.0/(self.num_data_points+1.0))

        self.num_data_points+=1

        if self.num_data_points<30:
            self.boundary_val = min(0.0,self.average_val)


        max_update_step = (self.max_val-self.min_val)/min(self.num_data_points,100)

        if can_increase and val>self.boundary_val:
            update_step = self.update_rate*(val-self.boundary_val)
            update_step = min(update_step,max_update_step)
            self.boundary_val+=update_step

        elif can_decrease and val<self.boundary_val:
            update_step = self.update_rate*(val-self.boundary_val)
            update_step = max(update_step,-max_update_step)
            self.boundary_val+=update_step

    def get_boundary_val(self):
        return self.boundary_val

        


