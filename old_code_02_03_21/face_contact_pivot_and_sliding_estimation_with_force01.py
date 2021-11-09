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
import matplotlib.pyplot as plt

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

def moving_average(x, w):
    	return np.convolve(x, np.ones(w), mode='same') / w

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

def update_sliding_velocity(x0, z0, contact_pose, contact_vel):

    contact_pose_stamped = ros_helper.list2pose_stamped(contact_pose)
    contact_pose_homog = ros_helper.matrix_from_pose(contact_pose_stamped)
    
    # 2-D unit contact normal in world frame
    e_n = contact_pose_homog[:3, 0]
    n2D = e_n[[0,2]]
    e_n2D = n2D/np.sqrt(np.sum(n2D ** 2))

    # 2-D unit contact tangent in world frame
    e_t = contact_pose_homog[:3, 1]
    t2D = e_t[[0,2]]
    e_t2D = t2D/np.sqrt(np.sum(t2D ** 2))

    # 2-D angular velocity
    theta_dot = contact_vel[4]

    # xc and zc
    xc, zc = contact_pose[0] - x0, contact_pose[2] - z0

    # compute velocity jacobian between world frame (x, z, tht) and contact frame (n, t, tht)
    velocity_jacobian = np.vstack([np.vstack([e_n2D, e_t2D, np.array([zc, -xc])]).T,
         	np.array([0., 0., 1.])])

    # compute end effector velocity in contact frame
    ee_vel_contact_frame = np.linalg.solve(velocity_jacobian, 
    	np.array([contact_vel[0], contact_vel[2], theta_dot]))

    # find nomral and tangential displacment
    s = e_t2D[0]*xc + e_t2D[1]*zc 
    d = e_n2D[0]*xc + e_n2D[1]*zc 

    return ee_vel_contact_frame, np.array([d, s])
        

def update_center_of_rotation_estimate(x0,z0,pose_list):

    pose_array = np.array(pose_list) # TODO: this should always be an array


    # pull out the x and z coordinates of this triangle 
    random_x = pose_array[:, 0]
    random_z = pose_array[:, 2]

    # pull out 2D unit contact normal in world frame
    nx_array, nz_array = get_2D_normal(pose_array)

    # build Ax = b 
    b = nx_array * random_x + nz_array * random_z
    A = np.vstack([nx_array, nz_array, np.ones_like(nx_array)]).T

    # solve Ax = b to find COR (x = [x0, y0, C])
    Coeff_Vec = np.linalg.lstsq(A,b)[0]

    # extract the center location for the three points chosen
    x0_new=Coeff_Vec[0]
    z0_new=Coeff_Vec[1]

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

def end_effector_wrench_in_end_effector_frame_callback(data):
    global end_effector_wrench_in_end_effector_frame
    end_effector_wrench_in_end_effector_frame = data


def get_xy_wrench(wrench_list):
    return [wrench_list[0], wrench_list[1], wrench_list[-1]]


if __name__ == '__main__':

    rospy.init_node("face_contact_pivot_and_sliding_estimation01")
    arm = ArmInterface()
    rospy.sleep(0.5)

    rate = rospy.Rate(100)

    # initialize globals
    end_effector_wrench_in_end_effector_frame = None

    #setting up subscribers
    end_effector_wrench_in_end_effector_frame_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_in_end_effector_frame_callback)

    # make sure subscribers are receiving commands
    while end_effector_wrench_in_end_effector_frame is None:
        print("Waiting for end effector wrench")

    # initialize lists
    t_list = []
    s_list = []
    vs_list = []
    vs_fd_list = []
    x0_list = []
    z0_list = []
    vx_panda_list = []
    vz_panda_list = []
    vtheta_panda_list = []
    face_contact_center_pose_all = []
    end_effector_2D_wrench_list = []

    # set up center of rotation marker publisher
    marker_message = initialize_marker()
    center_of_rotation_visualization_publisher = rospy.Publisher(
        '/center_of_rotation_visualization_in_world_face_contact_publisher', Marker, queue_size=10)      

    # intialize estimator
    current_pose = arm.endpoint_pose()      # original pose of robot
    x0 = current_pose['position'][0]
    z0 = current_pose['position'][2]-.5

    Nbatch = 250

    print('starting estimation loop')
    start_time = rospy.Time.now().to_sec()
    tmax = 30
    while not rospy.is_shutdown():

        # end loop
        t = rospy.Time.now().to_sec() - start_time
        if t > tmax:
        	break

        # face_center franka pose
        face_center_franka_pose = arm.endpoint_pose()

        # face_center pose_stamped
        face_center_pose_stamped = ros_helper.list2pose_stamped(franka_pose2list(face_center_franka_pose))

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

        # end effector velocity        
        end_effector_velocity = arm.endpoint_velocity()
        end_effector_velocity_list = end_effector_velocity['linear'].tolist(
        	) + end_effector_velocity['angular'].tolist() 

        # end effector 2D wrench
        end_effector_2D_wrench = get_xy_wrench(ros_helper.wrench_stamped2list(
            end_effector_wrench_in_end_effector_frame))   

        # if measured pose is new
        if np.abs(diff_angle) > 0.001:
            
            # append to list
            face_contact_center_pose_all.append(face_contact_center_pose_list)

            # make list a FIFO buffer of length Nbatch
            if len(face_contact_center_pose_all) > Nbatch:
                face_contact_center_pose_all.pop(0)

            if len(face_contact_center_pose_all) > 3:

                # update center of rotation estimate
                x0, z0 = update_center_of_rotation_estimate(
                    x0,z0,face_contact_center_pose_all)

                # update maker for center of rotation
                marker_center_of_rotation_in_world_pose=ros_helper.list2pose_stamped(
                    [x0,face_contact_center_pose_all[-1][1],z0,0,0,0,1])
                update_marker_pose(marker_center_of_rotation_in_world_pose,marker_message)

        # add to list after estimate has settled
        if len(face_contact_center_pose_all) > 50:

            ee_vel_contact_frame, ee_pos_contact_frame = update_sliding_velocity(x0, z0, 
                face_contact_center_pose_list, end_effector_velocity_list)

            t_list.append(t)
            s_list.append(ee_pos_contact_frame[1])
            vs_list.append(ee_vel_contact_frame[1])
            x0_list.append(x0)
            z0_list.append(z0)
            vx_panda_list.append(end_effector_velocity_list[0])
            vz_panda_list.append(end_effector_velocity_list[2])
            vtheta_panda_list.append(end_effector_velocity_list[4])
            end_effector_2D_wrench_list.append(end_effector_2D_wrench)

            # compute via finite differencing
            if len(s_list) > 1:
    			vs_fd_list.append((s_list[-1] - s_list[-2])/(t_list[-1] - t_list[-2]))

        # publish maker
        center_of_rotation_visualization_publisher.publish(marker_message)
        rate.sleep()    

    # moving average filter
    vs_list_filtered = [vs_list[0]]
    alpha=.4
    for ind, vs in enumerate(vs_list[1:]):
    	vs_list_filtered.append((1-alpha)*vs_list_filtered[ind-1] + alpha*vs)   	
   	vs_moving_average = moving_average(np.array(vs_list), 5)


    # convert to numpy array
    end_effector_2D_wrench_array = np.array(
        end_effector_2D_wrench_list)


    fig, ax = plt.subplots(2,1)
    ax[0].plot(np.array(t_list), np.array(s_list))
    # ax[0].set_ylim([-0.02, 0.02])
    ax[1].plot(np.array(t_list), np.array(vs_list))
    ax[1].plot(np.array(t_list), np.array(vs_list_filtered))
    ax[1].plot(np.array(t_list), vs_moving_average)
    # ax[1].set_ylim([-0.01, 0.01])

    fig1, ax1 = plt.subplots(3,1)
    ax1[0].plot(np.array(t_list), np.array(vx_panda_list))    
    ax1[1].plot(np.array(t_list), np.array(vz_panda_list))
    ax1[2].plot(np.array(t_list), np.array(vtheta_panda_list))

    fig2, ax2 = plt.subplots(2,1)
    ax2[0].plot(np.array(t_list), np.array(x0_list))    
    ax2[1].plot(np.array(t_list), np.array(z0_list))   


    fig3, ax3 = plt.subplots(1,2)
    ax3[0].scatter(end_effector_2D_wrench_array[:, 1], end_effector_2D_wrench_array[:,0],
     c=np.abs(np.array(vs_list)), cmap='inferno', edgecolors=None)
    ax3[1].scatter(end_effector_2D_wrench_array[:, 2], end_effector_2D_wrench_array[:,0], 
        edgecolors=None)
    plt.show()


    