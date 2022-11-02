#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import rospy
import time
from sensor_msgs.msg import CameraInfo, Image


def image_message_callback(data):
	global time_list
	time_list.append(time.time())

if __name__ == "__main__":
	time_list = []

	rospy.init_node("realsense_speed_test")
	rate = rospy.Rate(60)
	rospy.sleep(.5)


	image_message_sub                   = rospy.Subscriber(
		'/near_cam/color/image_raw', 
		Image,
		image_message_callback)

	while not rospy.is_shutdown():
		if len(time_list)>=2:
			my_freq = len(time_list)/(time_list[-1]-time_list[0])
			print(my_freq)
		rate.sleep()