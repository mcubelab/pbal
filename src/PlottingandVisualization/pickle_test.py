#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import pickle
import numpy as np

if __name__ == "__main__":

    path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data_fall_2022'
    fname = '/test_data-experiment0001.pickle'

    with open(path+fname, 'rb') as handle:
        b = pickle.load(handle)
        # print(b.keys())

        # print(b['/netft/netft_data']['time_list'])
        # print(b['/torque_cone_boundary_test']['time_list'])
        # print(b['/torque_cone_boundary_test']['msg_list'])
        # print(b['/tag_detections']['msg_list'][0])

        for key in b.keys():
            dt = b[key]['time_list'][-1]-b[key]['time_list'][0]
            num_items = len(b[key]['time_list'])

            if num_items==1:
                print(key+': only one message recieved')
            elif num_items==0:
                print(key+': no messages recieved')
            elif dt==0.0:
                print(key+': dt=0')
            else:
                avg_rate = num_items/dt
                print(key+': '+str(avg_rate))


        # num_frames = len(b['/near_cam/color/image_raw']['msg_list'])
        # num_expected_frames = 60*30
        # print('num_frames: ',num_frames)
        # print('num_expected_frames: ',num_expected_frames)
        

