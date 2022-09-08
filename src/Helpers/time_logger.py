#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import time
import rospy
import collections
from Modelling.system_params import SystemParams
def compute_std_dev(my_deque, mean_val):
    ''' compute std deviation of a deque'''

    spread = 0

    for val in my_deque:
        spread += (val - mean_val) ** 2

    return (spread/len(my_deque)) ** (0.5)

class time_logger(object):
    def __init__(self,node_name):
        self.sys_params = SystemParams()
        self.node_name = node_name
        self.time_deque = collections.deque(maxlen=self.sys_params.debug_params['QUEUE_LEN'])
        

    def reset(self):
        self.t0 = time.time()

    def log_time(self):
        # update time deque
        self.time_deque.append(1000 * (time.time() - self.t0))   

        # log timing info
        if len(self.time_deque) == self.sys_params.debug_params['QUEUE_LEN']:
            rospy.loginfo_throttle(self.sys_params.debug_params["LOG_TIME"], 
                (self.node_name + " runtime: {mean:.3f} +/- {std:.3f} [ms]")
                .format(mean=sum(self.time_deque)/len(self.time_deque), 
                std=compute_std_dev(my_deque=self.time_deque, 
                    mean_val=sum(self.time_deque)/len(self.time_deque))))