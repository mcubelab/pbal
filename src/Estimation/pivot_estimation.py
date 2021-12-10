 #!/usr/bin/env python
# estimates pivot location and publishes pivot location as Marker and Point
# subscribes to end_effector pose 

import numpy as np
import tf.transformations as tfm
import tf2_ros
import rospy
import pdb
import copy
import json

import ros_helper
import franka_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool, String
from models.system_params import SystemParams


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

def get_hand_orientation_in_base(contact_pose_homog):
    # current orientation
    hand_normal_x = contact_pose_homog[0,0]
    hand_normal_z = contact_pose_homog[2,0]
    return -np.arctan2(hand_normal_x, -hand_normal_z)

def initialize_frame():
    frame_message = TransformStamped()
    frame_message.header.frame_id = "base"
    frame_message.header.stamp = rospy.Time.now()
    frame_message.child_frame_id = "pivot"
    frame_message.transform.translation.x = 0.0
    frame_message.transform.translation.y = 0.0
    frame_message.transform.translation.z = 0.0

    frame_message.transform.rotation.x = 0.0
    frame_message.transform.rotation.y = 0.0
    frame_message.transform.rotation.z = 0.0
    frame_message.transform.rotation.w = 1.0
    return frame_message

def update_frame_translation(frame_origin, frame_message):
    frame_message.header.stamp = rospy.Time.now()
    frame_message.transform.translation.x = frame_origin[0]
    frame_message.transform.translation.y = frame_origin[1]
    frame_message.transform.translation.z = frame_origin[2]

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
       

def update_center_of_rotation_estimate(pose_list):

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
    x0=Coeff_Vec[0]
    z0=Coeff_Vec[1]
    d=Coeff_Vec[2]

    return x0, z0, d


def generate_update_matrices(new_pose):
    x = new_pose[0]
    z = new_pose[2]

    pose_stamped = ros_helper.list2pose_stamped(new_pose)
    pose_homog = ros_helper.matrix_from_pose(pose_stamped)
    
    # 2-D unit contact normal in world frame
    e_n = pose_homog[:3, 0]
    n2D = e_n[[0,2]]
    e_n2D = n2D/np.sqrt(np.sum(n2D ** 2))

    nx = e_n2D[0]
    nz = e_n2D[1]

    Y_i = x * nx + z * nz
    X_i = np.array([nx,nz,1])

    M1_update = np.outer(X_i,X_i)
    M2_update = Y_i * X_i

    return M1_update , M2_update

def update_center_of_rotation_estimate_sliding(current_pose_list,
    pivot_xyz, d):

        pose_stamped = ros_helper.list2pose_stamped(current_pose_list)
        pose_homog = ros_helper.matrix_from_pose(pose_stamped)
        
        # 2-D unit contact normal in world frame
        e_n = pose_homog[:3, 0]
        n2D = e_n[[0,2]]
        e_n2D = n2D/np.sqrt(np.sum(n2D ** 2))

        # 2-D unit contact tangent in world frame
        e_t = contact_pose_homog[:3, 1]
        t2D = e_t[[0,2]]
        e_t2D = t2D/np.sqrt(np.sum(t2D ** 2))

        # new x position of the pivot
        # can't disambiguate sliding at contact versus ground if hand is horizontal
        if np.abs(e_t2D[1]) > 0.2:  

            xp = current_pose_list[0] - d*e_n2D[0] - (
                e_t2D[0]/e_t2D[1]) * (current_pose_list[2] 
                    - pivot_xyz[2] - d*e_n2D[1])
            pivot_xyz[0] = xp        

        return pivot_xyz

def torque_cone_boundary_test_callback(data):
    global torque_boundary_boolean
    torque_boundary_boolean = data.data

def pivot_sliding_commanded_flag_callback(data):
    global pivot_sliding_commanded_boolean
    pivot_sliding_commanded_boolean = data.data

def sliding_state_callback(data):
    global sliding_measured_boolean
    if data.data != '':
        sliding_state_dict = json.loads(data.data) 
        sliding_measured_boolean = sliding_state_dict['psf']

if __name__ == '__main__':

    rospy.init_node("pivot_estimator")
    arm = ArmInterface()
    rospy.sleep(0.5)

    sys_params = SystemParams()
    RATE = sys_params.estimator_params["RATE"]
    rate = rospy.Rate(RATE)

    #endpoint_pose_all = []
    #hand_angle_all = []

    torque_boundary_boolean, pivot_sliding_commanded_boolean = None, None
    sliding_measured_boolean = None

    # set up torque cone boundary subscriber
    torque_cone_boundary_test_sub = rospy.Subscriber("/torque_cone_boundary_test", 
        Bool,  torque_cone_boundary_test_callback)

    # set up sliding state subscriber
    sliding_state_sub = rospy.Subscriber("/sliding_state", String, 
        sliding_state_callback)

    # set up pivot sliding flag
    pivot_sliding_commanded_flag_sub = rospy.Subscriber("/pivot_sliding_commanded_flag", 
        Bool,  pivot_sliding_commanded_flag_callback)

    # set up pivot Point publisher
    frame_message = initialize_frame()
    pivot_xyz_pub = rospy.Publisher('/pivot_frame', TransformStamped, queue_size=10)      

    # set up pivot marker publisher
    marker_message = initialize_marker()
    pivot_marker_pub = rospy.Publisher('/pivot_marker', Marker, queue_size=10)

    # set up transform broadcaster
    pivot_frame_broadcaster = tf2_ros.TransformBroadcaster()   

    # intialize pivot estimate
    current_pose = arm.endpoint_pose()      # original pose of robot
    x0=None
    z0=None
    pivot_xyz=None
    pivot_xyz_sticking = None
    pivot_pose=None
    d=None
    hand_angle_at_recording=None
    num_data_points = 0

    M1 = np.zeros([3,3])
    M2 = np.zeros([1,3])

    # hyper parameters
    NBATCH = sys_params.estimator_params["NBATCH_PIVOT"]             # in yaml
    UPDATE_LENGTH = sys_params.estimator_params["UPDATE_LENGTH_PIVOT"]     # in yaml
    ANG_DIFF_THRESH = sys_params.estimator_params["ANGLE_DIFF_THRESH_PIVOT"]   # in yaml

    # flags
    previous_loop_sliding = False

    # make sure subscribers are receiving commands
    # print("Waiting for torque boundary check")
    # while torque_boundary_boolean is None:
    #     pass

    # # make sure subscribers are receiving commands
    # print("Waiting for pivot sliding check")
    # while pivot_sliding_commanded_boolean is None:
    #     pass

    print('starting pivot estimation loop')
    while not rospy.is_shutdown():

        if torque_boundary_boolean is not None:

            # face_center franka pose
            endpoint_pose_franka = arm.endpoint_pose()

            # face_center list
            endpoint_pose_list = franka_helper.franka_pose2list(endpoint_pose_franka)

            contact_pose_stamped = ros_helper.list2pose_stamped(endpoint_pose_list)
            contact_pose_homog = ros_helper.matrix_from_pose(contact_pose_stamped)

            hand_angle = get_hand_orientation_in_base(contact_pose_homog)

            # append if empty
            if num_data_points==0:
                num_data_points+=1
                hand_angle_at_recording=hand_angle
                M1_update, M2_update = generate_update_matrices(endpoint_pose_list)
                M1 +=M1_update
                M2 +=M2_update

            #if not hand_angle_all:
            #    hand_angle_all.append(hand_angle)

            hand_angle_old = hand_angle_at_recording
            diff_angle = np.abs(hand_angle-hand_angle_old)
            while diff_angle>= 2*np.pi:
                diff_angle-= 2*np.pi

            # pivot is sliding
            if pivot_sliding_commanded_boolean or sliding_measured_boolean:
                if pivot_pose is not None and torque_boundary_boolean:

                    previous_loop_sliding = True

                    # update maker for center of rotation
                    pivot_xyz = update_center_of_rotation_estimate_sliding(
                        endpoint_pose_list, pivot_xyz, d)
                    pivot_pose = ros_helper.list2pose_stamped(
                        pivot_xyz + [0,0,0,1])

                else:
                    if pivot_pose is None:
                        x0=None
                        z0=None
                        pivot_xyz=None
                        pivot_xyz_sticking = None
                        pivot_pose=None
                        d=None
                        hand_angle_at_recording=None
                        num_data_points = 0

                        M1 = np.zeros([3,3])
                        M2 = np.zeros([1,3])

            else:

                if previous_loop_sliding and (pivot_xyz_sticking is not None):

                    M2 += np.dot(M1,[pivot_xyz[0] - pivot_xyz_sticking[0],0,0])
                    #for endpoint_pose in endpoint_pose_all:
                    #    endpoint_pose[0] += pivot_xyz[0] - pivot_xyz_sticking[0]


                previous_loop_sliding = False

                # if measured pose is new, and the measurement is not at wrench cone boundary
                if np.abs(diff_angle
                    ) > ANG_DIFF_THRESH and torque_boundary_boolean:
                    
                    # append to list
                    #endpoint_pose_all.append(endpoint_pose_list)

                    num_data_points+=1
                    hand_angle_at_recording=hand_angle
                    M1_update, M2_update = generate_update_matrices(endpoint_pose_list)
                    M1 +=M1_update
                    M2 +=M2_update

                    #hand_angle_all.append(hand_angle)

                    # make list a FIFO buffer of length NBATCH
                    #if len(endpoint_pose_all) > NBATCH:
                    #    endpoint_pose_all.pop(0)
                    #    hand_angle_all.pop(0)

                    # and update
                    if num_data_points > UPDATE_LENGTH:

                        # update center of rotation estimate
                        #x0, z0, d = update_center_of_rotation_estimate(
                        #    endpoint_pose_all)

                        Coeff_Vec = np.linalg.solve(M1,M2[0])
                        x0=Coeff_Vec[0]
                        z0=Coeff_Vec[1]
                        d =Coeff_Vec[2]
                        

                        pivot_xyz = [x0,endpoint_pose_list[1],z0]

                        # update maker for center of rotation
                        pivot_pose = ros_helper.list2pose_stamped(
                            pivot_xyz + [0,0,0,1])

                        # last sticking pivot_pose
                        pivot_xyz_sticking = copy.deepcopy(pivot_xyz)


            # only publish after estimate has settled
            if num_data_points > UPDATE_LENGTH:
                update_frame_translation(pivot_xyz, frame_message)
                update_marker_pose(pivot_pose, marker_message)
                pivot_xyz_pub.publish(frame_message)
                pivot_frame_broadcaster.sendTransform(frame_message)
                pivot_marker_pub.publish(marker_message)
        
        rate.sleep()    



    