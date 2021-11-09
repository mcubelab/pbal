#!/usr/bin/env python

import numpy as np
import tf
import tf.transformations as tfm
import rospy
import pdb

import ros_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import PoseStamped, WrenchStamped, Quaternion
from visualization_msgs.msg import Marker


# def face_contact_center_pose_callback(data):
#     global face_contact_center_pose
#     face_contact_center_pose = data

def initialize_marker():
    marker_message = Marker()
    marker_message.header.frame_id = "base"
    marker_message.header.stamp = rospy.get_rostime()
    marker_message.type = Marker.SPHERE
    marker_message.scale.x=.02
    marker_message.scale.y=.02
    marker_message.scale.z=.02
    marker_message.color.a=1.0
    marker_message.color.r=0
    marker_message.color.g=1
    marker_message.color.b=0
    marker_message.lifetime.secs=1.0
    return marker_message

def update_marker_pose(marker_pose, marker_message):
    marker_message.header.stamp = marker_pose.header.stamp
    marker_message.pose = marker_pose.pose

def get_2D_normal(random_pose_array):

    nx_list = []
    nz_list = []

    for pose_array in random_pose_array:
        pose_stamped = ros_helper.list2pose_stamped(pose_array.tolist())
        pose_homog = ros_helper.matrix_from_pose(pose_stamped)
        
        # 2-D unit contact normal in world frame
        e_n = pose_homog[:3, 0]
        n2D = e_n[[0,2]]
        e_n2D = n2D/np.sqrt(np.sum(n2D ** 2))

        nx_list.append(e_n2D[0])
        nz_list.append(e_n2D[1])

    return np.array(nx_list), np.array(nz_list)

def update_center_of_rotation_estimate(x0,z0,pose_list):

    #select three random tooltip positions out of the tooltip history
    #these three points form a triangle with a circumsbriing circle
    #note that we are projecting this triangle onto the x-z plane
    #ignoring the y coordinate

    # random_index=np.random.randint(0,len(pose_list),3)
    # random_points = np.array([pose_list[random_index[0]],
    #                         pose_list[random_index[1]],
    #                         pose_list[random_index[2]]])

    pose_array = np.array(pose_list) # TODO: this should always be an array


    # pull out the x and z coordinates of this triangle 
    random_x = pose_array[:, 0]
    random_z = pose_array[:, 2]

    # pull out 2D unit contact normal in world frame
    nx_array, nz_array = get_2D_normal(pose_array)

    # build Ax = b 
    b = nx_array * random_x + nz_array * random_z
    A = np.vstack([nx_array, nz_array, np.ones_like(nx_array)]).T
    
    #update the estimate for the center of rotation. 
    # input_weight = np.linalg.det(A)

    # if input_weight > 0.0001: # check determinant number

    # solve Ax = b to find COR (x = [x0, y0, C])
    Coeff_Vec = np.linalg.lstsq(A,b)[0]

    # extract the center location for the three points chosen
    x0_new=Coeff_Vec[0]
    z0_new=Coeff_Vec[1]

    # c1 = input_weight/(input_weight+weight_aggregate)
    # c2 = weight_aggregate/(input_weight+weight_aggregate)

    # x0 = c1*x0_new + c2*x0
    # z0 = c1*z0_new + c2*z0

    # weight_aggregate = .95*(weight_aggregate)+input_weight

    return x0_new,z0_new

def franka_pose2list(arm_pose):
    '''
    arm pose is a frank pose
    output is a list
    '''


    position_list = arm_pose['position'].tolist()
    orientation_list = [arm_pose['orientation'].x,
                               arm_pose['orientation'].y,
                               arm_pose['orientation'].z,
                               arm_pose['orientation'].w]
    return position_list + orientation_list

def face_contact_center_pose(arm_pose):
    '''
    arm_pose is pose stamped
    face_contact_center_pose_in_base_frame is pose stamped
    '''


    # face contact center in hand frame
    face_contact_quat_in_hand_frame = tf.transformations.quaternion_from_euler(0, 0, 0)
    face_contact_center_pose_in_hand_frame = ros_helper.list2pose_stamped([0.012, 0, -0.0375] 
        + face_contact_quat_in_hand_frame.tolist())

    # face contact center in base frame
    face_contact_center_pose_in_base_frame =  ros_helper.convert_reference_frame(
        face_contact_center_pose_in_hand_frame, ros_helper.unit_pose(), arm_pose, frame_id = "base")

    return face_contact_center_pose_in_base_frame

if __name__ == '__main__':

    rospy.init_node("test_face_contact_pivot_estimation01")
    arm = ArmInterface()
    rospy.sleep(0.5)

    rate = rospy.Rate(100)

    # initialize globals
    # face_contact_center_pose = None

    # #setting up subscribers
    # face_contact_center_pose_sub = rospy.Subscriber("/face_contact_center_pose_in_world_frame_publisher", 
    #     PoseStamped, face_contact_center_pose_callback)

    # # make sure subscribers are receiving commands
    # while face_contact_center_pose is None:
    #     print("Waiting for face contact pose")

    # set up center of rotation marker publisher
    marker_message = initialize_marker()
    center_of_rotation_visualization_publisher = rospy.Publisher(
        '/center_of_rotation_visualization_in_world_face_contact_publisher', Marker, queue_size=10)      

    # initialize_list 
    face_contact_center_pose_all = []

    # intialize estimator
    current_pose = arm.endpoint_pose()      # original pose of robot
    # weight_aggregate = 0
    x0 = current_pose['position'][0]
    z0 = current_pose['position'][2]-.5

    Nbatch = 250

    print('starting estimation loop')
    while not rospy.is_shutdown():

        # end_effector franka pose
        endpoint_franka_pose = arm.endpoint_pose()

        # end effector pose_stamped
        endpoint_pose_stamped = ros_helper.list2pose_stamped(franka_pose2list(endpoint_franka_pose))

        # face center pose stamped
        face_center_pose_stamped = face_contact_center_pose(endpoint_pose_stamped)

        # face center pose list
        face_contact_center_pose_list = ros_helper.pose_stamped2list(face_center_pose_stamped)

        if not face_contact_center_pose_all:
            face_contact_center_pose_all.append(face_contact_center_pose_list)

        # current_quaternion
        face_center_quat = ros_helper.quat2list(face_center_pose_stamped.pose.orientation)

        # last quaternion in list
        face_center_quat_old = face_contact_center_pose_all[-1][3:]

        # difference quaternion
        diff_quat = tfm.quaternion_multiply(face_center_quat, 
                tfm.quaternion_inverse(face_center_quat_old))

        # diff angle
        diff_angle = 2 * np.arccos(diff_quat[-1])
        # print(diff_angle)

        # end effector velocity        
        # end_effector_velocity = arm.endpoint_velocity()
        # print(np.linalg.norm(end_effector_velocity['angular'], ord=2))

        # if measured pose is new
        if np.abs(diff_angle) > 0.001:

            print("updating: " + str(len(face_contact_center_pose_all)))
            
            # append to list
            face_contact_center_pose_all.append(face_contact_center_pose_list)
            if len(face_contact_center_pose_all) > Nbatch:
                face_contact_center_pose_all.pop(0)
                print("LIST IS FULL")

            if len(face_contact_center_pose_all) > 3:

                # update center of rotation estimate
                x0, z0 = update_center_of_rotation_estimate(
                    x0,z0,face_contact_center_pose_all)

                # updare maker for center of rotation
                marker_center_of_rotation_in_world_pose=ros_helper.list2pose_stamped(
                    [x0,face_contact_center_pose_all[-1][1] ,z0,0,0,0,1])
                update_marker_pose(marker_center_of_rotation_in_world_pose,marker_message)


        # publish maker
        center_of_rotation_visualization_publisher.publish(marker_message)
        rate.sleep()    

    