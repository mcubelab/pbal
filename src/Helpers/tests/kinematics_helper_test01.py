#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(os.path.dirname(currentdir)))

import numpy as np
import Helpers.kinematics_helper as kh
import random

if __name__ == '__main__':
    print('hello!')

    quat = []
    for i in range(4):
        quat.append(random.uniform(-.5,.5))

    quat = np.array(quat)
    quat/=np.linalg.norm(quat)

    trans = []
    for i in range(3):
        trans.append(random.uniform(-10,10))

    trans = np.array(trans)

    pose_homog = kh.matrix_from_trans_and_quat(trans=trans,quat=quat)

    # print(pose_homog)
    
    homog_inv = kh.invert_transform_homog(pose_homog)

    ans_check = np.dot(homog_inv,pose_homog)
    for i in range(4):
        for j in range(4):
            if abs(ans_check[i][j])<.1**14:
                ans_check[i][j]=0.0
    print(ans_check)

    ans_check = np.dot(pose_homog,homog_inv)
    for i in range(4):
        for j in range(4):
            if abs(ans_check[i][j])<.1**14:
                ans_check[i][j]=0.0
    print(ans_check)
    