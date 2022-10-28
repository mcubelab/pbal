import numpy as np
import time
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose2D, WrenchStamped, PointStamped

def list2pose_stamped(pose, frame_id="world"):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]
    msg.pose.orientation.x = pose[3]
    msg.pose.orientation.y = pose[4]
    msg.pose.orientation.z = pose[5]
    msg.pose.orientation.w = pose[6]
    return msg

def pose_stamped2list(msg):
    return [float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w)]

def list2transform_stamped(transform, header_frame_id = 'base', child_frame_id = 'hand_estimate'):
    msg = TransformStamped()
    msg.header.frame_id = header_frame_id
    msg.child_frame_id = child_frame_id
    msg.transform.translation.x = transform[0]
    msg.transform.translation.y = transform[1]
    msg.transform.translation.z = transform[2]
    msg.transform.rotation.x = transform[3]
    msg.transform.rotation.y = transform[4]
    msg.transform.rotation.z = transform[5]
    msg.transform.rotation.w = transform[6]
    return msg

def transform_stamped2list(msg):
    return [float(msg.transform.translation.x),
            float(msg.transform.translation.y),
            float(msg.transform.translation.z),
            float(msg.transform.rotation.x),
            float(msg.transform.rotation.y),
            float(msg.transform.rotation.z),
            float(msg.transform.rotation.w)]

def list2wrench_stamped(wrench, frame_id='base'):
    msg = WrenchStamped()
    msg.header.frame_id = frame_id
    msg.wrench.force.x = wrench[0]
    msg.wrench.force.y = wrench[1]
    msg.wrench.force.z = wrench[2]
    msg.wrench.torque.x = wrench[3]
    msg.wrench.torque.y = wrench[4]
    msg.wrench.torque.z = wrench[5]
    return msg

def wrench_stamped2list(msg):
    return [float(msg.wrench.force.x), 
            float(msg.wrench.force.y), 
            float(msg.wrench.force.z), 
            float(msg.wrench.torque.x), 
            float(msg.wrench.torque.y), 
            float(msg.wrench.torque.z)]

def wrenchstamped_2FT(msg):
    force = [float(msg.wrench.force.x),
             float(msg.wrench.force.y),
             float(msg.wrench.force.z)]

    torque = [float(msg.wrench.torque.x),
              float(msg.wrench.torque.y),
              float(msg.wrench.torque.z)]

    return force, torque

def list2point_stamped(xyz):
    msg = PointStamped()
    msg.point.x = xyz[0]
    msg.point.y = xyz[1]
    msg.point.z = xyz[2]
    return msg

def point_stamped2list(msg):
    return [float(msg.point.x), float(msg.point.y), float(msg.point.z)]

def list2pose_twod(pose):
    msg = Pose2D()
    msg.x = pose[0]
    msg.y = pose[1]
    msg.theta = pose[2]
    return msg

def pose_twod2list(msg):
    return [float(msg.x), float(msg.y), float(msg.theta)]

def quat2list(quat):
    return [float(quat.x), float(quat.y), float(quat.z), float(quat.w)]

def unit_pose():
    return list2pose_stamped([0.0,0.0,0.0,0.0,0.0,0.0,1.0])

def pose_from_matrix(matrix, frame_id="world"):
    return list2pose_stamped(pose_list_from_matrix(matrix), frame_id=frame_id)

def matrix_from_pose(pose):
    return matrix_from_pose_list(pose_stamped2list(pose))

def matrix_from_transform(transform):
    return matrix_from_pose_list(transform_stamped2list(transform))


def quat_from_matrix(M):
    r00 = M[0,0]
    r01 = M[0,1]
    r02 = M[0,2]

    r10 = M[1,0]
    r11 = M[1,1]
    r12 = M[1,2]

    r20 = M[2,0]
    r21 = M[2,1]
    r22 = M[2,2]

    a = 1.0+r00+r11+r22
    b = 1.0+r00-r11-r22
    c = 1.0-r00+r11-r22
    d = 1.0-r00-r11+r22

    max_index  = np.argmax( [a,b,c,d] )

    if max_index == 0:
        qw = np.sqrt(a)/2.0
        qx = (r21-r12 )/(4.0*qw)
        qy = (r02-r20 )/(4.0*qw)
        qz = (r10-r01 )/(4.0*qw)

    if max_index == 1:
        qx = np.sqrt(b)/2.0
        qw = (r21-r12 )/(4.0*qx)
        qy = (r01+r10 )/(4.0*qx)
        qz = (r20+r02 )/(4.0*qx)

    if max_index == 2:
        qy = np.sqrt(c)/2.0
        qw = (r02-r20 )/(4.0*qy)
        qx = (r01+r10 )/(4.0*qy)
        qz = (r12+r21 )/(4.0*qy)

    if max_index == 3:
        qz = np.sqrt(d)/2.0
        qw = (r10-r01 )/(4.0*qz)
        qx = (r20+r02 )/(4.0*qz)
        qy = (r12+r21 )/(4.0*qz)

    return [qx,qy,qz,qw]

def trans_and_quat_from_matrix(M):
    return [M[0,3],M[1,3],M[2,3]], quat_from_matrix(M)

def pose_list_from_matrix(M):
    return [M[0,3],M[1,3],M[2,3]] + quat_from_matrix(M)

def matrix_from_trans_and_quat(trans=None,quat=None):

    tx = 0.0
    ty = 0.0
    tz = 0.0

    qx = 0.0
    qy = 0.0
    qz = 0.0
    qw = 1.0

    if trans is not None:
        tx = trans[0]
        ty = trans[1]
        tz = trans[2]

    if quat is not None:    
        qx = quat[0]
        qy = quat[1]
        qz = quat[2]
        qw = quat[3]

    r00 = 2*(qw*qw+qx*qx)-1
    r01 = 2*(qx*qy-qw*qz)
    r02 = 2*(qx*qz+qw*qy)

    r10 = 2*(qx*qy+qw*qz)
    r11 = 2*(qw*qw+qy*qy)-1
    r12 = 2*(qy*qz-qw*qx)

    r20 = 2*(qx*qz-qw*qy)
    r21 = 2*(qy*qz+qw*qx)
    r22 = 2*(qw*qw+qz*qz)-1

    return np.array([[r00, r01, r02,  tx],
                     [r10, r11, r12,  ty],
                     [r20, r21, r22,  tz],
                     [0.0, 0.0, 0.0, 1.0]])

def matrix_from_pose_list(pose_list):
    return matrix_from_trans_and_quat(pose_list[0:3],pose_list[3:7])


def lookupTransform(homeFrame, targetFrame, listener):
    ntfretry = 100
    retryTime = .05
    for i in range(ntfretry):
        try:
            (trans, rot) = listener.lookupTransform(targetFrame, homeFrame, listener.getLatestCommonTime(targetFrame, homeFrame))
            return (trans, rot)
        except:  
            print('[lookupTransform] failed to transform')
            print('[lookupTransform] targetFrame %s homeFrame %s, retry %d' %
                  (targetFrame, homeFrame, i))
            time.sleep(retryTime)

    return None, None

def lookupTransform_homog(homeFrame, targetFrame, listener):
    trans,rot = lookupTransform(homeFrame, targetFrame, listener)

    if trans is not None and rot is not None:
        return matrix_from_trans_and_quat(trans,rot)
    else:
        return None

def rotate_wrench(wrench_source, pose_transform):
    force_source, torque_source = wrenchstamped_2FT(wrench_source)
    T_transform_source = matrix_from_pose(pose_transform)[:3, :3]
    force_transformed = np.matmul(T_transform_source, np.array(force_source))
    torque_transformed = np.matmul(T_transform_source, np.array(torque_source))

    return list2wrench_stamped(force_transformed.tolist() + torque_transformed.tolist())

def wrench_reference_point_change(wrench_source, vector_from_new_ref):
    # wrench source and vector_from_new_ref need in the same frame
    force_source, torque_source = wrenchstamped_2FT(wrench_source)
    torque_transformed = np.array(torque_source) + np.cross(np.array(vector_from_new_ref), np.array(force_source))

    return list2wrench_stamped(force_source + torque_transformed.tolist())


def transform_pose(pose_source, pose_transform):
    T_pose_source = matrix_from_pose(pose_source)
    T_transform_source = matrix_from_pose(pose_transform)
    T_pose_final_source = np.matmul(T_transform_source, T_pose_source)
    pose_final_source = pose_from_matrix(T_pose_final_source, frame_id=pose_source.header.frame_id)
    return pose_final_source

def transform_pose_intermediate_frame(pose_source_frameA, pose_frameB, pose_transform_frameB):
    pose_source_frameB = convert_reference_frame(pose_source_frameA,
                                                  pose_frameB,
                                                  unit_pose(),
                                                  frame_id="frameB")
    pose_source_transformed_frameB = transform_pose(pose_source_frameB,
                                                    pose_transform_frameB)
    pose_source_transformed_frameA = convert_reference_frame(pose_source_transformed_frameB,
                                                  unit_pose(),
                                                  pose_frameB,
                                                  frame_id="frameB")
    return pose_source_transformed_frameA

def transform_body(pose_source_world, pose_transform_target_body):
    #convert source to target frame
    pose_source_body = convert_reference_frame(pose_source_world,
                                                 pose_source_world,
                                                 unit_pose(),
                                                 frame_id="body_frame")
    #perform transformation in body frame
    pose_source_rotated_body = transform_pose(pose_source_body,
                                              pose_transform_target_body)
    # rotate back
    pose_source_rotated_world = convert_reference_frame(pose_source_rotated_body,
                                                         unit_pose(),
                                                         pose_source_world,
                                                         frame_id="yumi_body")
    return pose_source_rotated_world



def convert_reference_frame(pose_source, pose_frame_target, pose_frame_source, frame_id = "work_obj"):
    T_pose_source = matrix_from_pose(pose_source)
    pose_transform_target2source = get_transform(pose_frame_source, pose_frame_target)
    T_pose_transform_target2source = matrix_from_pose(pose_transform_target2source)
    T_pose_target = np.matmul(T_pose_transform_target2source, T_pose_source)
    pose_target = pose_from_matrix(T_pose_target, frame_id=frame_id)
    return pose_target

def get_pose_from_tf_frame(listener, target_frame, source_name):
    trans, quat = listener.lookupTransform(source_name, target_frame, listener.getLatestCommonTime(source_name, target_frame))
    pose_target_source = list2pose_stamped(trans+quat, frame_id=source_name)
    return pose_target_source

def get_transform(pose_frame_target, pose_frame_source):
    """
    Find transform that transforms pose source to pose target
    :param pose_frame_target:
    :param pose_frame_source:
    :return:
    """
    #both poses must be expressed in same reference frame
    T_target_world = matrix_from_pose(pose_frame_target)
    T_source_world = matrix_from_pose(pose_frame_source)
    T_relative_world = np.matmul(T_target_world, np.linalg.inv(T_source_world))
    pose_relative_world = pose_from_matrix(T_relative_world, frame_id=pose_frame_source.header.frame_id)
    return pose_relative_world



def initialize_rosbag(topics, dir_save_bagfile, exp_name='test'):
    import datetime
    import subprocess
    #Saving rosbag options
    name_of_bag  = str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")) + '-' + exp_name
    rosbag_proc = subprocess.Popen('rosbag record -q -O %s %s' % (name_of_bag, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)

def terminate_rosbag():
    terminate_ros_node('/record')

def terminate_ros_node(s):
    import os
    import subprocess
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for term in list_output.decode('utf8').split("\n"):
        if (term.startswith(s)):
            os.system("rosnode kill " + term)
            print("rosnode kill " + term)

def compute_tip2tcp_offset(listener, pose_tip_push_start, tip_name='/apriltag_tip'):
    # pose_tip_start = list2pose_stamped(pose_list, frame_id='push_start')
    #1. get transforms from tf
    pose_push_start_map = get_pose_from_tf_frame(listener=listener,
                                            target_frame='/push_start',
                                            source_name='/map')

    pose_link_6_tip = get_pose_from_tf_frame(listener=listener,
                                            target_frame='/link_6',
                                            source_name=tip_name)

    #2. Find tip pose in map frame (from push start frame)
    pose_tip_map = convert_reference_frame(pose_source= pose_tip_push_start,
                                         pose_frame_target=unit_pose(),
                                         pose_frame_source=pose_push_start_map,
                                         frame_id='map')

    #3. Find tcp frame (in map) from tip frame (in map)
    pose_tcp_map = convert_reference_frame(pose_source=pose_link_6_tip,
                                                         pose_frame_target=unit_pose(),
                                                         pose_frame_source=pose_tip_map,
                                                         frame_id='map')
    return pose_tcp_map


def transform_tip2tcp(listener,  pose_tip_push_start, tip_name='/apriltag_tip'):
    pose_tcp_map = compute_tip2tcp_offset(listener,  pose_tip_push_start, tip_name)
    pose_map_list = convert_ros2abb(pose_tcp_map)
    return pose_map_list
  
def quatlist_to_theta(quat_list):
    #converts fraka quaternion to sagittal plane angle
    pose_homog = matrix_from_trans_and_quat(quat = quat_list)
    return np.arctan2(-pose_homog[1,0], -pose_homog[0,0])   

def theta_to_quatlist(theta):
    #converts sagittal <now in world manipulation frame, which is right-handed, with z axes aligned!! -Orion> plane angle to franka quaternion
    #note that the quaternion is defined in the world manipulation frame, so it will need to be converted to the base frame in order to be
    #sent to the robot as an impedance command
    return quat_from_matrix( np.array([[-np.cos(theta),  np.sin(theta), 0.0], 
                                      [ -np.sin(theta), -np.cos(theta), 0.0], 
                                      [            0.0,            0.0, 1.0]]))
