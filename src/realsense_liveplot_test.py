#!/usr/bin/env python
import tf
import tf.transformations as tfm
from ros_helper import lookupTransform
import rospy
import pdb
import json
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped

import matplotlib.pyplot as plt
from matplotlib import cm
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
import roslib
from franka_interface import ArmInterface 
from apriltag_ros.msg import AprilTagDetectionArray
import models.ros_helper as ros_helper
import franka_helper
from models.system_params import SystemParams


def qp_debug_message_callback(data):
    global qp_debug_dict_list
    if data.data != '':
        qp_debug_dict = json.loads(data.data)
        qp_debug_dict_list.append(qp_debug_dict)

    if len(qp_debug_dict_list)>10:
        qp_debug_dict_list.pop(0)


def image_message_callback(data):
    global image_list

    image_list.append(data)
    if len(image_list) > 3:
        image_list.pop(0)
     
def camera_info_callback(data):
    global camera_info

    camera_info = data

def load_shape_data(name_in):
    f = open("/home/oneills/Documents/panda/base_ws/src/franka_ros_interface/franka_ros_controllers/scripts/models/shape_description/"+name_in+".json")
    shape_data = json.load(f)

    vertex_array = np.array([shape_data["x_vert"],shape_data["y_vert"]])
    apriltag_id = shape_data["apriltag_id"]
    apriltag_pos = shape_data["centroid_to_apriltag"]

    return vertex_array, apriltag_id ,apriltag_pos


def get_tranf_matrix():

    # Make listener to get camera transform
    listener = tf.TransformListener()
    rospy.sleep(.5)

    # panda hand pose in base frame WHEN TARING
    (translate, quaternion) = \
        lookupTransform('/camera_color_optical_frame', 'base', listener)


    # Matrix for extrinsics
    extrinsics_matrix = np.linalg.inv(
        np.dot(tfm.compose_matrix(translate=translate),
               tfm.quaternion_matrix(quaternion)))

    # (3, 4) camera matrix
    camera_matrix = np.reshape(camera_info.P, (3,4))

    # Transformation matrix
    return np.dot(camera_matrix, extrinsics_matrix)


def apriltag_message_callback(apriltag_array):
    global apriltag_id
    global obj_apriltag_in_camera_pose
    global obj_apriltag_in_world_pose
    global base_in_base_pose
    global cam_in_base_pose
    global marker_pose_apriltag_frame
    global obj_orientation_matrix
    global object_detected

    obj_apriltag_list = [detection for detection in apriltag_array.detections if detection.id == (apriltag_id,)]
 
    if obj_apriltag_list:
        object_detected = True

        obj_apriltag_in_camera_pose = obj_apriltag_list[0].pose.pose

        obj_apriltag_in_world_pose = ros_helper.convert_reference_frame(obj_apriltag_in_camera_pose, base_in_base_pose,
                                                      cam_in_base_pose, frame_id = "base")

        marker_apriltag_in_world_pose = ros_helper.convert_reference_frame(marker_pose_apriltag_frame, base_in_base_pose,
                                                      obj_apriltag_in_world_pose, frame_id = "base")

        obj_orientation_matrix = ros_helper.matrix_from_pose(marker_apriltag_in_world_pose)

    else:
        object_detected = False

def end_effector_wrench_callback(data):
    global measured_contact_wrench_list
    global measured_contact_wrench_6D_list

    end_effector_wrench = data
    measured_contact_wrench_6D = -np.array(ros_helper.wrench_stamped2list(
            end_effector_wrench))
    measured_contact_wrench = np.array([
            measured_contact_wrench_6D[0], 
            measured_contact_wrench_6D[1],
            measured_contact_wrench_6D[-1]])

    measured_contact_wrench_list.append(measured_contact_wrench)
    measured_contact_wrench_6D_list.append(measured_contact_wrench_6D)

    if len(measured_contact_wrench_list) > 100:
       measured_contact_wrench_list.pop(0)
    if len(measured_contact_wrench_6D_list) > 100:
        measured_contact_wrench_6D_list.pop(0)

def end_effector_wrench_base_frame_callback(data):
    global measured_base_wrench_list
    global measured_base_wrench_6D_list

    base_wrench = data
    measured_base_wrench_6D = -np.array(ros_helper.wrench_stamped2list(
            base_wrench))
    measured_base_wrench = np.array([
            measured_base_wrench_6D[0], 
            measured_base_wrench_6D[2],
            measured_base_wrench_6D[-1]])

    measured_base_wrench_list.append(measured_base_wrench)
    measured_base_wrench_6D_list.append(measured_base_wrench_6D)

    if len(measured_base_wrench_list) > 100:
       measured_base_wrench_list.pop(0)
    if len(measured_base_wrench_6D_list) > 100:
        measured_base_wrench_6D_list.pop(0)

def friction_parameter_callback(data):
    global friction_parameter_list

    friction_parameter_list.append(json.loads(data.data))
    if len(friction_parameter_list) > 10:
       friction_parameter_list.pop(0)


def get_pix(xyz,transformation_matrix):
    # xyz should be n x 3

    # Project trajectories into pixel space
    vector = np.hstack([xyz , np.ones([xyz.shape[0], 1])])
    pixels = np.dot(transformation_matrix, vector.T)
    pix_x = np.divide(pixels[0], pixels[2])
    pix_y = np.divide(pixels[1], pixels[2])

    return pix_x, pix_y

def get_pix_easier(xyz,transformation_matrix):
    pixels = np.dot(transformation_matrix, xyz)
    pix_x = np.divide(pixels[0], pixels[2])
    pix_y = np.divide(pixels[1], pixels[2])

    return np.round(pix_x).astype(int), np.round(pix_y).astype(int)


    



if __name__ == '__main__':
    image_list = []
    camera_info = None


    obj_apriltag_in_camera_pose = None
    obj_apriltag_in_world_pose = None
    base_in_base_pose = None
    cam_in_base_pose = None
    marker_pose_apriltag_frame = None
    obj_orientation_matrix = None
    object_detected = False

    measured_contact_wrench_list = []
    measured_base_wrench_list = []
    friction_parameter_list = []
    measured_contact_wrench_6D_list = []
    measured_base_wrench_6D_list = []

    measured_contact_wrench_6D = None
    measured_base_wrench_6D = None

    current_image = None

    qp_debug_dict_list = []
    qp_debug_dict = None


    hand_normal_offset = -.01
    hand_points = np.array([[hand_normal_offset]*2,
                            [.0505,-.0505],
                            [0.0375,0.0375],
                            [1.0,1.0]])

    force_scale = .001



    rospy.init_node("realsense_liveplot_test")
    rospy.sleep(1.0)

    arm = ArmInterface()
    bridge = CvBridge()

    sys_params = SystemParams()
    l_contact = sys_params.object_params["L_CONTACT_MAX"]


    object_vertex_array, apriltag_id ,apriltag_pos = load_shape_data('triangle')

    object_vertex_array = np.vstack([object_vertex_array,
                                np.zeros(len(object_vertex_array[0])),
                                np.ones(len(object_vertex_array[0]))])


    listener = tf.TransformListener()   
    (cam_in_base_trans, cam_in_base_rot) = ros_helper.lookupTransform('/camera_color_optical_frame', 'base', listener)
    cam_in_base_pose = ros_helper.list2pose_stamped(cam_in_base_trans + cam_in_base_rot, frame_id="base")
    base_in_base_pose = ros_helper.unit_pose()
    marker_quat_apriltag_frame = tf.transformations.quaternion_from_euler(0, 0, 0)
    marker_pose_apriltag_frame = ros_helper.list2pose_stamped([-apriltag_pos[0], -apriltag_pos[1], 0] + marker_quat_apriltag_frame.tolist())






    end_effector_wrench_sub = rospy.Subscriber(
        "/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  
        end_effector_wrench_callback)

    end_effector_wrench_base_frame_sub = rospy.Subscriber(
        "/end_effector_sensor_in_base_frame", 
        WrenchStamped,  
        end_effector_wrench_base_frame_callback)

    friction_parameter_sub = rospy.Subscriber(
        '/friction_parameters', 
        String, 
        friction_parameter_callback)

    image_message_sub = rospy.Subscriber(
       '/camera/color/image_raw',
        Image,
        image_message_callback)

    camera_info_sub = rospy.Subscriber(
        '/camera/color/camera_info',
        CameraInfo,
        camera_info_callback
        )

    apriltag_message_sub = rospy.Subscriber(
        '/tag_detections',
        AprilTagDetectionArray,
        apriltag_message_callback
        )

    qp_debug_message_sub = rospy.Subscriber(
       '/qp_debug_message',
        String,
        qp_debug_message_callback)



    



    print("Waiting for image message")
    while len(image_list)==0:
        rospy.sleep(0.1)

    print("Waiting for camera info")
    while camera_info is None:
        rospy.sleep(.1)



    transformation_matrix = get_tranf_matrix()
    camera_info_sub.unregister()




    while not rospy.is_shutdown():
        if qp_debug_dict_list:
            while qp_debug_dict_list:
                qp_debug_dict = qp_debug_dict_list.pop(0)

        if image_list:
            while image_list:
                current_image = image_list.pop(0)

        if measured_contact_wrench_list:
            while measured_contact_wrench_list:
                measured_contact_wrench = measured_contact_wrench_list.pop(0)

        if measured_base_wrench_list:
            while measured_base_wrench_list:
                measured_base_wrench = measured_base_wrench_list.pop(0)

        if measured_contact_wrench_6D_list:
            while measured_contact_wrench_6D_list:
                measured_contact_wrench_6D = measured_contact_wrench_6D_list.pop(0)

        if measured_base_wrench_6D_list:
            while measured_base_wrench_6D_list:
                measured_base_wrench_6D = measured_base_wrench_6D_list.pop(0)

        if friction_parameter_list:
            while friction_parameter_list:
                friction_parameter_dict = friction_parameter_list.pop(0)

                A_contact_right = np.array(friction_parameter_dict["acr"])
                B_contact_right = friction_parameter_dict["bcr"]
                A_contact_left = np.array(friction_parameter_dict["acl"])
                B_contact_left = friction_parameter_dict["bcl"]
                
                A_external_right = np.array(friction_parameter_dict["aer"])
                B_external_right = np.array(friction_parameter_dict["ber"])
                A_external_left = np.array(friction_parameter_dict["ael"])
                B_external_left = np.array(friction_parameter_dict["bel"])

                P0_L =[A_contact_left[0]*B_contact_left,A_contact_left[1]*B_contact_left]
                P0_R =[A_contact_right[0]*B_contact_right,A_contact_right[1]*B_contact_right]


        if current_image is not None:

            cv_image = bridge.imgmsg_to_cv2(current_image, "bgr8")

            # face_center franka pose
            endpoint_pose_franka = arm.endpoint_pose()

            # face_center list
            endpoint_pose_list = franka_helper.franka_pose2list(endpoint_pose_franka)

            contact_pose_stamped = ros_helper.list2pose_stamped(endpoint_pose_list)
            contact_pose_homog = ros_helper.matrix_from_pose(contact_pose_stamped)
                


            if object_detected:
                xyz=np.array([[obj_apriltag_in_world_pose.pose.position.x,
                               obj_apriltag_in_world_pose.pose.position.y,
                               obj_apriltag_in_world_pose.pose.position.z]])
                pix_x, pix_y = get_pix(xyz,transformation_matrix)
                x_coord= int(np.round(pix_x[0]))
                y_coord= int(np.round(pix_y[0]))
               
                #cv2.circle(cv_image, (x_coord,y_coord), 5, 255, -1)

                
                current_dot_positions = np.dot(obj_orientation_matrix,object_vertex_array)
                height_indices = np.argsort(current_dot_positions[2])
                
                h0 = current_dot_positions[2][height_indices[0]]
                h1 = current_dot_positions[2][height_indices[1]]

                
                desired_dot_positions = current_dot_positions

                if qp_debug_dict is not None and 'err_theta' in qp_debug_dict['error_dict']:
                    d_theta = qp_debug_dict['error_dict']['err_theta']

                    temp_vertex_array = object_vertex_array+0.0
                    min_vertex = object_vertex_array[0:3,height_indices[0]]
                    for i in range(0,len(temp_vertex_array[0])):
                        temp_vertex_array[0:3,i]-=min_vertex
                    
                    my_rotation = np.array([[np.cos(d_theta),-np.sin(d_theta),0.,min_vertex[0]],
                                  [np.sin(d_theta), np.cos(d_theta),0.,min_vertex[1]],
                                  [0.,0.,0.,min_vertex[2]],
                                  [0.,0.,0.,1.],])
                    desired_dot_positions = np.dot(obj_orientation_matrix,np.dot(my_rotation,temp_vertex_array))



                if qp_debug_dict is not None and 'err_x_pivot' in qp_debug_dict['error_dict']:
                    dx_pivot = qp_debug_dict['error_dict']['err_x_pivot']
                    desired_dot_positions[0,:]+=dx_pivot

                x_coord, y_coord = get_pix_easier(desired_dot_positions,transformation_matrix)
                cv2.polylines(cv_image, [np.vstack([x_coord,y_coord]).T], True, (51,51,255), thickness = 2)


                count = 0
                height_threshold = .003
                while count<len(height_indices) and (current_dot_positions[2][height_indices[count]]
                    -current_dot_positions[2][height_indices[0]])<height_threshold:

                    x_coord, y_coord = get_pix_easier(current_dot_positions[:,height_indices[count]],transformation_matrix)
                    cv2.circle(cv_image, (x_coord,y_coord), 5, (0,0,153), -1)
                    count+=1


                if measured_contact_wrench_6D is not None and measured_base_wrench_6D is not None:

                    P0 = None

                    if current_dot_positions[2][height_indices[1]]-current_dot_positions[2][height_indices[0]]<height_threshold:
                        P_a = current_dot_positions[:,height_indices[0]]
                        P_b = current_dot_positions[:,height_indices[1]]
                        P_e = contact_pose_homog[:,3]
                        
                        Tau_a = np.dot(np.cross(P_a[0:3]-P_e[0:3],measured_base_wrench_6D[0:3]),obj_orientation_matrix[0:3,2])
                        Tau_b = np.dot(np.cross(P_b[0:3]-P_e[0:3],measured_base_wrench_6D[0:3]),obj_orientation_matrix[0:3,2])
                        Tau_net = np.dot(measured_base_wrench_6D[3:6],obj_orientation_matrix[0:3,2])

                        epsilon1 = (Tau_net-Tau_b)/(Tau_a-Tau_b)
                        epsilon1 = np.max([np.min([epsilon1,1]),0])

                        epsilon2 = 1-epsilon1
                        #epsilon2 = (Tau_net-Tau_a)/(Tau_b-Tau_a)

                        P0 = epsilon1*P_a + epsilon2*P_b


                    else:
                        P0 = current_dot_positions[:,height_indices[0]]

                    if P0 is not None:
                        force_tip = np.hstack([P0[0:3] - force_scale*measured_base_wrench_6D[0:3], np.array([1])])

                        x_coord, y_coord = get_pix_easier(np.vstack([P0,force_tip]).T,transformation_matrix)

                        #cv2.polylines(cv_image, [np.vstack([x_coord,y_coord]).T], True, (0,25,51), thickness = 2)
                        cv2.arrowedLine(cv_image, (x_coord[0],y_coord[0]), (x_coord[1],y_coord[1]), (0,25,51), thickness = 2)

                        x_coord, y_coord = get_pix_easier(P0,transformation_matrix)

                        cv2.circle(cv_image, (x_coord,y_coord), 5, (153,0,0), -1)

            if qp_debug_dict is not None and 'err_s' in qp_debug_dict['error_dict']:
                    ds_hand = qp_debug_dict['error_dict']['err_s']
                    desired_hand_points = hand_points+0.0
                    desired_hand_points[1,:]+=ds_hand

                    x_coord, y_coord = get_pix_easier(np.dot(contact_pose_homog,desired_hand_points),transformation_matrix)
                    cv2.polylines(cv_image, [np.vstack([x_coord,y_coord]).T], True, (0,255,0), thickness = 2)

            x_coord, y_coord = get_pix_easier(np.dot(contact_pose_homog,hand_points),transformation_matrix)
            cv2.polylines(cv_image, [np.vstack([x_coord,y_coord]).T], True, (0,128,255), thickness = 2)







        if measured_contact_wrench_6D is not None and measured_base_wrench_6D is not None:
            if np.abs(measured_contact_wrench_6D[0])>.1:
                contact_point = measured_contact_wrench_6D[-1]/measured_contact_wrench_6D[0]
                contact_point = np.max([np.min([contact_point,l_contact/2]),-l_contact/2])
                alpha_2 = (contact_point+l_contact/2)/(l_contact)
                alpha_1 = 1-alpha_2
                mid_point = np.dot(np.dot(contact_pose_homog,hand_points),np.array([alpha_1,alpha_2]))

                

                force_tip = np.hstack([mid_point[0:3] + force_scale*measured_base_wrench_6D[0:3], np.array([1])])

                x_coord, y_coord = get_pix_easier(np.vstack([mid_point,force_tip]).T,transformation_matrix)

                #cv2.polylines(cv_image, [np.vstack([x_coord,y_coord]).T], True, (0,25,51), thickness = 2)
                cv2.arrowedLine(cv_image, (x_coord[0],y_coord[0]), (x_coord[1],y_coord[1]), (0,25,51), thickness = 2)

                x_coord, y_coord = get_pix_easier(mid_point,transformation_matrix)

                cv2.circle(cv_image, (x_coord,y_coord), 5, (0,0,153), -1)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)



   





        
