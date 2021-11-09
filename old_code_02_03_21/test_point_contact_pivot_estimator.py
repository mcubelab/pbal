#!/usr/bin/env python

# this is to find out the transform between the webcam frame and robot frame
import numpy as np
import tf.transformations as tfm
import rospy
import copy
import pdb
import matplotlib.pyplot as plt


import ros_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import PoseStamped, WrenchStamped
from franka_tools import CollisionBehaviourInterface
from visualization_msgs.msg import Marker


def ee_in_world_pose_callback(data):
    global ee_in_world_pose    
    ee_in_world_pose = data

def obj_apriltag_in_world_pose_callback(data):
    global obj_apriltag_in_world_pose    
    obj_apriltag_in_world_pose = data

def ft_sensor_in_base_frame_callback(data):
    global ft_sensor_in_base_frame   
    ft_sensor_in_base_frame = data

def franka_pose2list(arm_pose):
    position_list = arm_pose['position'].tolist()
    orientation_list = [arm_pose['orientation'].x,
                               arm_pose['orientation'].y,
                               arm_pose['orientation'].z,
                               arm_pose['orientation'].w]
    return position_list + orientation_list

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

def update_center_of_rotation_estimate(x0,z0,pose_list,weight_aggregate):
    random_index=np.random.randint(0,len(pose_list),3)

    #select three random tooltip positions out of the tooltip history
    #these three points form a triangle with a circumsbriing circle
    #note that we are projecting this triangle onto the x-z plane
    #ignoring the y coordinate
    random_points = np.array([pose_list[random_index[0]],
                            pose_list[random_index[1]],
                            pose_list[random_index[2]]])

    #pull out the x and z coordinates of this triangle 
    random_x = random_points[:, 0]
    random_z = random_points[:, 2]

    #compute the lengths of the edges of this triangle
    l1=np.sqrt(np.power(random_x[0]-random_x[1],2)+np.power(random_z[0]-random_z[1],2))
    l2=np.sqrt(np.power(random_x[1]-random_x[2],2)+np.power(random_z[1]-random_z[2],2))
    l3=np.sqrt(np.power(random_x[2]-random_x[0],2)+np.power(random_z[2]-random_z[0],2))

    #use heron's formula to compute the area of the random triangle selected
    triangle_area= np.sqrt((l1+l2+l3)*(-l1+l2+l3)*(l1-l2+l3)*(l1+l2-l3))

    #if the triangle's area is 0, then the circumscribing circle is ill-defined
    #so just skip. otherwise, update the estimate for the center of rotation
    if triangle_area > .00001:

        #the center of the circumscribing circle is the solution to the following
        #set of linear equations
        triangle_mat = np.array([
        [-2*random_x[0], -2*random_z[0], 1],
        [-2*random_x[1], -2*random_z[1], 1],
        [-2*random_x[2], -2*random_z[2], 1]])

        triangle_vec = np.array([
        -np.power(random_x[0],2)-np.power(random_z[0],2),
        -np.power(random_x[1],2)-np.power(random_z[1],2),
        -np.power(random_x[2],2)-np.power(random_z[2],2),])

        Coeff_Vec= np.linalg.solve(triangle_mat,triangle_vec)

        #extract the center location for the three points chosen
        x0_new=Coeff_Vec[0]
        z0_new=Coeff_Vec[1]

        #update the estimate for the center of rotation. Triangles with larger area are given more weight for updates
        c1 = triangle_area/(triangle_area+weight_aggregate)
        c2 = weight_aggregate/(triangle_area+weight_aggregate)

        x0 = c1*x0_new + c2*x0
        z0 = c1*z0_new + c2*z0

        weight_aggregate = .95*(weight_aggregate)+triangle_area

    return x0,z0,weight_aggregate

if __name__ == '__main__':

    rospy.init_node("test_impedance_control01")
    arm = ArmInterface()
    rospy.sleep(0.5)

    print("Setting collision behaviour")
    collision = CollisionBehaviourInterface()
    rospy.sleep(0.5)
    torque_upper = [40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0] # default [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
    force_upper = [100.0, 100.0, 100.0, 25.0, 25.0, 25.0] # [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
    collision.set_ft_contact_collision_behaviour(torque_upper=torque_upper, 
        force_upper=force_upper)
    rospy.sleep(1.0)

    rate = rospy.Rate(30.)

    # set up globals
    ee_in_world_pose, obj_apriltag_in_world_pose, ft_sensor_in_base_frame = None, None, None

    # initialize lists
    ee_pose_proprioception_list = []
    # ee_in_world_pose_list = []
    obj_apriltag_in_world_pose_list = []
    ft_sensor_in_base_frame_list = []
    adjusted_current_pose_list = []
    x0_list= []
    z0_list= []
    t_list=[]
    
    #setting up subscribers
    # ee_pose_in_world_from_camera_sub = rospy.Subscriber("/line_ee_pose_in_world_from_camera_publisher", 
    #     PoseStamped, ee_in_world_pose_callback)    
    obj_apriltag_pose_in_world_from_camera_sub = rospy.Subscriber("/obj_apriltag_pose_in_world_from_camera_publisher", 
        PoseStamped, obj_apriltag_in_world_pose_callback)
    ft_sensor_in_base_frame_sub = rospy.Subscriber("/ft_sensor_in_base_frame",
        WrenchStamped,ft_sensor_in_base_frame_callback)

    # make sure subscribers are receiving commands
    while obj_apriltag_in_world_pose is None:
        print("Waiting for obj pose")

    # while ee_in_world_pose is None:
    #     print("Waiting for ee pose")

    while ft_sensor_in_base_frame is None:
        print("Waiting for ft sensor")


    # set up rosbag
    rostopic_list = ["/ee_pose_in_world_from_franka_publisher",
                     # "/line_ee_pose_in_world_from_camera_publisher",
                     "/obj_apriltag_pose_in_world_from_camera_publisher",
                     "/ft_sensor_in_base_frame"
                     "/camera/color/image_raw"
                     "/center_of_rotation_visualization_in_world_from_camera_publisher"]
    ros_helper.initialize_rosbag(rostopic_list, exp_name="impedance_control01_experiment_data")

    # set up center of rotation marker publisher
    marker_message = initialize_marker()
    center_of_rotation_visualization_publisher = rospy.Publisher(
        '/center_of_rotation_visualization_in_world_from_camera_publisher', Marker, queue_size=10)

    # original pose of robot
    current_pose = arm.endpoint_pose()
    adjusted_current_pose = copy.deepcopy(current_pose)
    adjusted_current_pose['position'][2] -= 0.2
    base_horizontal_pose = adjusted_current_pose['position'][0]

    # motion schedule
    range_amplitude = 0.04
    horizontal_pose_schedule =  np.concatenate((np.linspace(0,range_amplitude,5), 
                                np.linspace(range_amplitude,-range_amplitude,10), 
                                np.linspace(-range_amplitude,0,5)))
    schedule_length = horizontal_pose_schedule.shape[0]

    # print('first impedance commmand')
    # arm.set_cart_impedance_pose(adjusted_current_pose, 
    #                 stiffness=[800, 800, 800, 10, 10, 10])        

    # intialize estimator
    weight_aggregate = 0
    x0 = current_pose['position'][0]
    z0 = current_pose['position'][2]-.5

    # start loop
    start_time = rospy.Time.now().to_sec()
    tmax=15.0

    print('starting control loop')
    while not rospy.is_shutdown():

        # current time
        t = rospy.Time.now().to_sec() - start_time

        # update lists
        t_list.append(t)
        ee_pose_proprioception=arm.endpoint_pose()
        ee_pose_proprioception_list.append(franka_pose2list(ee_pose_proprioception))
        # ee_in_world_pose_list.append(ros_helper.pose_stamped2list(ee_in_world_pose))
        obj_apriltag_in_world_pose_list.append(ros_helper.pose_stamped2list(obj_apriltag_in_world_pose))
        ft_sensor_in_base_frame_list.append(ros_helper.wrench_stamped2list(ft_sensor_in_base_frame))
        adjusted_current_pose_list.append(franka_pose2list(adjusted_current_pose))
        x0_list.append(x0)
        z0_list.append(z0)
        
        #update the center of rotation estimate
        x0,z0,weight_aggregate = update_center_of_rotation_estimate(x0,z0,ee_pose_proprioception_list,weight_aggregate)

        # create marker for center of rotation
        marker_center_of_rotation_in_world_pose=ros_helper.list2pose_stamped([x0,ee_pose_proprioception_list[-1][1] ,z0,0,0,0,1])
        update_marker_pose(marker_center_of_rotation_in_world_pose,marker_message)

        # publish maker
        center_of_rotation_visualization_publisher.publish(marker_message)

        # end if t > tmax
        if t > tmax:
            break

        # move to next position on schedule
        adjusted_current_pose['position'][0] = base_horizontal_pose + \
            horizontal_pose_schedule[int(np.floor(schedule_length*t/tmax))]
        arm.set_cart_impedance_pose(adjusted_current_pose, 
            stiffness=[1000, 600, 200, 50, 0, 50]) 

        rate.sleep()    

    print('control loop completed')

    # terminate rosbags
    ros_helper.terminate_rosbag()

    # unsubscribe from topics
    # ee_pose_in_world_from_camera_sub.unregister()
    obj_apriltag_pose_in_world_from_camera_sub.unregister()
    ft_sensor_in_base_frame_sub.unregister()

    # convert to numpy arrays
    t_np = np.array(t_list)
    ee_pose_proprioception_np= np.array(ee_pose_proprioception_list)
    # ee_in_world_pose_np= np.array(ee_in_world_pose_list)
    obj_apriltag_in_world_pose_np= np.array(obj_apriltag_in_world_pose_list)
    ft_sensor_in_base_frame_np= np.array(ft_sensor_in_base_frame_list)
    adjusted_current_pose_np= np.array(adjusted_current_pose_list)

    # plotting end effector position
    fig1, ax1 = plt.subplots(3, 1, figsize=(8,5))
    ax1 = np.ravel(ax1, order='F')

    ax1[0].plot(t_np, ee_pose_proprioception_np[:, 0], 'r')
    # ax1[0].plot(t_np, ee_in_world_pose_np[:, 0], 'b')
    ax1[0].plot(t_np, adjusted_current_pose_np[:, 0], 'g')
    ax1[0].set_ylabel('End effector X-Position [m]')
    
    ax1[1].plot(t_np, ee_pose_proprioception_np[:, 1], 'r')
    # ax1[1].plot(t_np, ee_in_world_pose_np[:, 1], 'b')
    ax1[1].plot(t_np, adjusted_current_pose_np[:, 1], 'g')
    ax1[1].set_ylabel('End effector Y-Position [m]')
    
    ax1[2].plot(t_np, ee_pose_proprioception_np[:, 2], 'r')
    # ax1[2].plot(t_np, ee_in_world_pose_np[:, 2], 'b')
    ax1[2].plot(t_np, adjusted_current_pose_np[:, 2], 'g')
    ax1[2].set_ylabel('End effector Z-Position [m]')

    # plotting end effector position
    fig2, ax2 = plt.subplots(3, 1, figsize=(8,5))
    ax2 = np.ravel(ax2, order='F')

    ax2[0].plot(t_np, obj_apriltag_in_world_pose_np[:, 0], 'b')
    ax2[0].set_ylabel('Obj X-Position [m]')
    
    ax2[1].plot(t_np, obj_apriltag_in_world_pose_np[:, 1], 'b')
    ax2[1].set_ylabel('Obj Y-Position [m]')
    
    ax2[2].plot(t_np, obj_apriltag_in_world_pose_np[:, 2], 'b')
    ax2[2].set_ylabel('Obj Z-Position [m]')

    # plotting forces
    fig3, ax3 = plt.subplots(3, 1, figsize=(8,5))
    ax3 = np.ravel(ax3, order='F')

    ax3[0].plot(t_np, ft_sensor_in_base_frame_np[:, 0], 'k')
    ax3[0].set_ylabel('X-Force [N]')
    
    ax3[1].plot(t_np, ft_sensor_in_base_frame_np[:, 1], 'k')
    ax3[1].set_ylabel('Y-Force [N]')
    
    ax3[2].plot(t_np, ft_sensor_in_base_frame_np[:, 2], 'k')
    ax3[2].set_ylabel('Z-Force [N]')

    # plotting position
    fig4, ax4 = plt.subplots(1, 1)
    ax4.plot(obj_apriltag_in_world_pose_np[:, 0], obj_apriltag_in_world_pose_np[:, 2], 'b')
    ax4.plot(ee_pose_proprioception_np[:, 0], ee_pose_proprioception_np[:, 2], 'r')
    ax4.plot(np.array(x0_list[3:]), np.array(z0_list[3:]), marker='o', markersize=5, color="red")
    ax4.set_xlabel('X [m]')
    ax4.set_ylabel('Z [m]')
    ax4.set_xlim(0.4,0.6 )
    ax4.set_ylim(-.05, .15)

    plt.show()


