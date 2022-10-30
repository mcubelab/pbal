#!/usr/bin/env python
import os, sys, inspect

currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, gparentdir)

import copy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from cvxopt import matrix, solvers
import json
import numpy as np
import pdb
import rospy
import time
import tf
import tf.transformations as tfm

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from pbal.msg import FrictionParamsStamped, QPDebugStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray

from Modelling.system_params import SystemParams
import Helpers.ros_helper as ros_helper
import Helpers.pbal_msg_helper as pmh

def apriltag_message_callback(apriltag_array):
    global apriltag_id

    global object_detected
    global obj_pose_homog

    obj_apriltag_list = [
        detection for detection in apriltag_array.detections
        if detection.id == (apriltag_id, )
    ]

    if obj_apriltag_list:
        object_detected = True

        obj_pose_homog = ros_helper.matrix_from_pose(obj_apriltag_list[0].pose.pose)

    else:
        object_detected = False

def image_message_callback(data):
    global image_list

    image_list.append(data)
    if len(image_list) > 3:
        image_list.pop(0)

def camera_info_callback(data):
    global camera_info

    camera_info = data

def get_pix(xyz, transformation_matrix):
    # xyz should be n x 3

    # Project trajectories into pixel space
    vector = np.hstack([xyz, np.ones([xyz.shape[0], 1])])
    pixels = np.dot(transformation_matrix, vector.T)
    pix_x = np.divide(pixels[0], pixels[2])
    pix_y = np.divide(pixels[1], pixels[2])

    return pix_x, pix_y


def get_pix_easier(xyz, transformation_matrix):
    pixels = np.dot(transformation_matrix, xyz)
    pix_x = np.divide(pixels[0], pixels[2])
    pix_y = np.divide(pixels[1], pixels[2])

    return np.round(pix_x).astype(int), np.round(pix_y).astype(int)



if __name__ == '__main__':

    apriltag_id = 10
    object_detected = False
    obj_pose_homog = None

    camera_info = None

    image_list = []

    l_tag = .042
    x_tag_boundary = np.array([-l_tag/2,l_tag/2,l_tag/2,-l_tag/2])
    y_tag_boundary = np.array([l_tag/2,l_tag/2,-l_tag/2,-l_tag/2])
    z_tag_boundary = np.array([0.0]*4)
    one_tag_boundary = np.array([1.0]*4)

    tag_boundary_pts = np.vstack([x_tag_boundary,y_tag_boundary,z_tag_boundary,one_tag_boundary])
  

    
    


    rospy.init_node("realsense_live_calibration_test")
    rate = rospy.Rate(10)
    rospy.sleep(1.0)

    listener = tf.TransformListener()
 
    bridge = CvBridge()

    image_message_sub = rospy.Subscriber('/far_cam/color/image_raw', Image,
                                         image_message_callback)

    camera_info_sub = rospy.Subscriber('/far_cam/color/camera_info', CameraInfo,
                                       camera_info_callback)


    apriltag_message_sub = rospy.Subscriber('/tag_detections',
                                            AprilTagDetectionArray,
                                            apriltag_message_callback)

    print("Waiting for image message")
    while len(image_list) == 0:
        rospy.sleep(0.1)

    print("Waiting for camera info")
    while camera_info is None:
        rospy.sleep(.1)

    # (3, 4) camera matrix
    camera_matrix = np.reshape(camera_info.P, (3, 4))
    camera_info_sub.unregister()

    M1 = np.zeros([3,3])
    M2 = np.zeros(3)
    forgetting_factor = .01

    trans_diff = np.zeros(3)
    camera_pose_mat = np.zeros([4,4])
    camera_pose_mat[3,3]=1.0

    while not rospy.is_shutdown():
       
        if image_list:
            while image_list:
                current_image = image_list.pop(0)

        cv_image = bridge.imgmsg_to_cv2(current_image, "bgr8")

        if object_detected:
            
            x_coord, y_coord = get_pix_easier(np.dot(obj_pose_homog,tag_boundary_pts),camera_matrix)
            obj_pose_homog


            (cam_in_base_trans, cam_in_base_rot) = ros_helper.lookupTransform(
                '/panda_april_tag', 'base', listener)

            robot_apriltag_pose_matrix = ros_helper.matrix_from_trans_and_quat(cam_in_base_trans,cam_in_base_rot)
            
            robot_apriltag_rot = robot_apriltag_pose_matrix[0:3,0:3]
            robot_apriltag_trans = robot_apriltag_pose_matrix[0:3,3]

            camera_apriltag_rot = obj_pose_homog[0:3,0:3]
            camera_apriltag_trans = obj_pose_homog[0:3,3]

            M1 = (1-forgetting_factor)*M1 + forgetting_factor*np.dot(robot_apriltag_rot,np.transpose(robot_apriltag_rot))
            M2 = (1-forgetting_factor)*M2 + forgetting_factor*np.dot(camera_apriltag_rot,np.transpose(robot_apriltag_rot))


            rot_mat_out = np.linalg.solve(M1,M2)
            trans_diff = (1-forgetting_factor)*trans_diff + forgetting_factor*(camera_apriltag_trans-np.dot(rot_mat_out,robot_apriltag_trans))

            camera_pose_mat[0:3,0:3]=rot_mat_out
            camera_pose_mat[0:3,3]=trans_diff
            # print np.dot(rot_mat_out,robot_apriltag_rot)-camera_apriltag_rot
            # print trans_diff - (np.dot(rot_mat_out,robot_apriltag_trans)-camera_apriltag_trans)

            cv2.polylines(cv_image, [np.vstack([x_coord, y_coord]).T],
                True, (0, 255, 0),thickness=2)

            x_coord, y_coord = get_pix_easier(np.dot(camera_pose_mat,np.dot(robot_apriltag_pose_matrix,tag_boundary_pts)),camera_matrix)
            cv2.polylines(cv_image, [np.vstack([x_coord, y_coord]).T],
                True, (255, 0, 0),thickness=2)

        
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        rate.sleep()
