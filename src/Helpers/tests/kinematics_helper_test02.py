#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(os.path.dirname(currentdir)))

import numpy as np
import Helpers.kinematics_helper as kh
import random

if __name__ == '__main__':
    print('hello!')

    v_in = []
    n_pts = 10

    for i in range(n_pts):
        v_in.append([random.uniform(-.5,.5),random.uniform(-.5,.5)])

    v_in = np.array(v_in).T


    theta = np.random.uniform(-np.pi,np.pi)

    a = random.uniform(-.5,.5)
    b = random.uniform(-.5,.5)

    rot_mat = np.array([[np.cos(theta),-np.sin(theta)],
                        [np.sin(theta), np.cos(theta)]])


    v_out = np.dot(rot_mat,v_in)

    v_out[0]+=a
    v_out[1]+=b

    r_out,theta_out = kh.regress_2D_transform(v_in,v_out)

    print(theta-theta_out,theta)


    
    