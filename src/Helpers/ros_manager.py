#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from apriltag_ros.msg import AprilTagDetectionArray

from pbal.msg import   (SlidingStateStamped, 
						FrictionParamsStamped, 
						ControlCommandStamped, 
						QPDebugStamped,
						TorqueConeBoundaryFlagStamped,
						PivotSlidingCommandedFlagStamped,
						TorqueConeBoundaryTestStamped,
 						TorqueBoundsStamped, 
 						GeneralizedPositionsStamped)
import Helpers.ros_helper as rh
import Helpers.pbal_msg_helper as pmh
from Helpers.pickle_manager import pickle_manager
from Helpers.time_logger import time_logger
import Helpers.impedance_mode_helper as IMH
import tf
import time
import numpy as np
from Estimation import friction_reasoning
import tf2_ros
from cv_bridge import CvBridge
import cv2


class ros_manager(object):
	def __init__(self,record_mode=False,path=None,experiment_label=None,load_mode=False,fname=None):
		self.data_available = []
		self.available_mask = []
		self.unpack_functions = []
		self.topic_list = []
		self.subscriber_dict = {}
		self.message_type_dict = {}
		self.buffer_dict = {}

		self.record_mode = record_mode
		self.path = path
		self.experiment_label=experiment_label
		self.rate = None

		self.tl = None
		self.qp_time_on=False

		self.my_impedance_mode_helper=None

		self.listener = None
		
		if self.record_mode:
			self.max_queue_size=np.inf
			self.subscriber_queue_size = 100
			self.pkm = pickle_manager(self.path)
			self.fname = self.pkm.generate_experiment_name(experiment_label=self.experiment_label)
		else: 
			self.max_queue_size=1
			self.subscriber_queue_size = 1

		self.load_mode = load_mode

		if self.load_mode:
			if fname is not None:
				self.fname_load = fname.split('.')[0]
			else:
				print('Error! No fname given!')
			

	def init_node(self,node_name):
		self.node_name = node_name
		rospy.init_node(self.node_name)
		time.sleep(.25)

	def is_shutdown(self):
		return rospy.is_shutdown()

	def activate_record_mode(self,path,experiment_label=None):
		self.record_mode = True
		self.path = path
		self.pkm = pickle_manager(self.path)
		self.experiment_label = experiment_label
		self.fname = self.pkm.generate_experiment_name(experiment_label=self.experiment_label)
		self.max_queue_size=np.inf
		self.subscriber_queue_size = 100

	def activate_load_mode(self,path,fname):
		self.load_mode = True
		self.path = path
		self.fname_load = fname.split('.')[0]

	def store_in_pickle(self):
		self.pkm.store_in_pickle(self.topic_list,self.buffer_dict,self.message_type_dict,experiment_label=self.experiment_label)

	def wait_for_necessary_data(self):
		print("Waiting to hear from essential subscribers")
		can_proceed = False

		while not can_proceed:
			time.sleep(.1)

			can_proceed = True
			for i in range(len(self.data_available)):
				can_proceed = can_proceed and (self.data_available[i] or self.available_mask[i])

	def unpack_all(self):
		for my_func in self.unpack_functions:
			my_func()

	def subscribe_to_list(self,list_in, isNecessary = True):
		for topic in list_in:
			self.subscribe(topic,isNecessary)

	def spawn_publisher_list(self,list_in):
		for topic in list_in:
			self.spawn_publisher(topic)

	def unregister_all(self):
		for topic in self.topic_list:
			if topic in self.subscriber_dict and self.subscriber_dict[topic] is not None:
				self.subscriber_dict[topic].unregister()

		if '/far_cam/color/image_raw' in self.subscriber_dict and self.far_came_video_writer is not None:
			time.sleep(.3)
			self.far_came_video_writer.release()

		if '/near_cam/color/image_raw' in self.subscriber_dict and self.near_came_video_writer is not None:
			time.sleep(.3)
			self.near_came_video_writer.release()

	def setRate(self,RATE):
		self.rate = rospy.Rate(RATE)
				
	def sleep(self):
		if self.rate is not None:
			self.rate.sleep()

	def init_time_logger(self):
		self.tl = time_logger(self.node_name)

	def init_qp_time(self):
		if self.tl is not None:
			self.tl.init_qp_time()
			self.qp_time_on=True

	def tl_reset(self):
		if self.tl is not None:
			self.tl.reset()

	def log_time(self):
		if self.tl is not None:
			self.tl.log_time()

	def log_qp_time(self,solve_time):
		if self.tl is not None and self.qp_time_on:
			self.tl.log_qp_time(solve_time)

	def impedance_mode_helper(self):
		self.my_impedance_mode_helper = IMH.impedance_mode_helper()

	def set_matrices_pbal_mode(self,TIPI,TOOPI,RIPI,ROOPI,rot_mat):
		if self.my_impedance_mode_helper is not None:
			self.my_impedance_mode_helper.set_matrices_pbal_mode(TIPI,TOOPI,RIPI,ROOPI,rot_mat)

	def set_cart_impedance_pose(self, pose):
		if self.my_impedance_mode_helper is not None:
			self.my_impedance_mode_helper.set_cart_impedance_pose(pose)

	def spawn_transform_listener(self):
		self.listener = tf.TransformListener()

	# calls the ROS service that tares (zeroes) the force-torque sensor
	def zero_ft_sensor(self):
		import netft_rdt_driver.srv as srv
		rospy.wait_for_service('/netft/zero', timeout=0.5)
		zero_ft = rospy.ServiceProxy('/netft/zero', srv.Zero)
		zero_ft()

	def lookupTransform(self, homeFrame, targetFrame):
		if self.listener is not None:
			ntfretry = 100
			retryTime = .05
			for i in range(ntfretry):
				try:
					(trans, rot) = self.listener.lookupTransform(targetFrame, homeFrame, self.listener.getLatestCommonTime(targetFrame, homeFrame))
					return (trans, rot)
				except:  
					print('[lookupTransform] failed to transform')
					print('[lookupTransform] targetFrame %s homeFrame %s, retry %d' %
						(targetFrame, homeFrame, i))
					time.sleep(retryTime)

			return (None, None)
		else:
			return (None, None)

	def subscribe(self,topic, isNecessary = True):
		self.available_mask.append(not isNecessary)
		self.data_available.append(False)
		self.topic_list.append(topic)

		if   topic ==  '/netft/netft_data':
			self.unpack_functions.append(self.force_unpack)
			self.ft_wrench_in_ft_sensor_buffer = []
			self.ft_wrench_in_ft_sensor_available_index = len(self.data_available)-1
			self.force_has_new = False

			self.ft_wrench_in_ft_sensor = None
			self.ft_wrench_in_ft_sensor_list = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				WrenchStamped, 
				self.force_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'WrenchStamped'
			self.buffer_dict[topic] = self.ft_wrench_in_ft_sensor_buffer

		elif topic == '/ee_pose_in_world_manipulation_from_franka_publisher':
			self.unpack_functions.append(self.ee_pose_in_world_manipulation_unpack)
			self.ee_pose_in_world_manipulation_buffer = []
			self.ee_pose_in_world_manipulation_available_index = len(self.data_available)-1
			self.ee_pose_in_world_manipulation_has_new = False

			self.ee_pose_in_world_manipulation = None
			self.ee_pose_in_world_manipulation_list = None
			self.ee_pose_in_world_manipulation_homog = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				PoseStamped, 
				self.ee_pose_in_world_manipulation_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'PoseStamped'
			self.buffer_dict[topic] = self.ee_pose_in_world_manipulation_buffer

		elif topic == '/ee_pose_in_base_from_franka_publisher':
			self.unpack_functions.append(self.ee_pose_in_base_unpack)
			self.ee_pose_in_base_buffer = []
			self.ee_pose_in_base_available_index = len(self.data_available)-1
			self.ee_pose_in_base_has_new = False

			self.ee_pose_in_base = None
			self.ee_pose_in_base_list = None
			self.ee_pose_in_base_homog = None

			self.base_z_in_ee_frame = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				PoseStamped, 
				self.ee_pose_in_base_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'PoseStamped'
			self.buffer_dict[topic] = self.ee_pose_in_base_buffer

		elif topic == '/end_effector_sensor_in_end_effector_frame':
			self.unpack_functions.append(self.end_effector_wrench_unpack)
			self.measured_contact_wrench_buffer = []
			self.measured_contact_wrench_available_index = len(self.data_available)-1
			self.end_effector_wrench_has_new = False

			self.measured_contact_wrench = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				WrenchStamped,  
				self.end_effector_wrench_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'WrenchStamped'
			self.buffer_dict[topic] = self.measured_contact_wrench_buffer

		elif topic == '/end_effector_sensor_in_world_manipulation_frame':
			self.unpack_functions.append(self.end_effector_wrench_world_manipulation_frame_unpack)
			self.measured_world_manipulation_wrench_buffer = []
			self.measured_world_manipulation_wrench_available_index = len(self.data_available)-1
			self.end_effector_wrench_world_manipulation_frame_has_new = False

			self.measured_world_manipulation_wrench = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				WrenchStamped,  
				self.end_effector_wrench_world_manipulation_frame_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'WrenchStamped'
			self.buffer_dict[topic] = self.measured_world_manipulation_wrench_buffer

		elif topic == '/torque_cone_boundary_test':
			self.unpack_functions.append(self.torque_cone_boundary_test_unpack)
			self.torque_cone_boundary_test_buffer = []
			self.torque_cone_boundary_test_available_index = len(self.data_available)-1
			self.torque_cone_boundary_test_has_new = False

			self.torque_cone_boundary_test = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				TorqueConeBoundaryTestStamped,  
				self.torque_cone_boundary_test_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'TorqueConeBoundaryTestStamped'
			self.buffer_dict[topic] = self.torque_cone_boundary_test_buffer

		elif topic == '/torque_cone_boundary_flag':
			self.unpack_functions.append(self.torque_cone_boundary_flag_unpack)
			self.torque_cone_boundary_flag_buffer = []
			self.torque_cone_boundary_flag_available_index = len(self.data_available)-1
			self.torque_cone_boundary_flag_has_new = False

			self.torque_cone_boundary_flag = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				TorqueConeBoundaryFlagStamped,  
				self.torque_cone_boundary_flag_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'TorqueConeBoundaryFlagStamped'
			self.buffer_dict[topic] = self.torque_cone_boundary_flag_buffer


		elif topic == '/friction_parameters':
			self.unpack_functions.append(self.friction_parameter_unpack)
			self.friction_parameter_buffer = []
			self.friction_parameter_available_index = len(self.data_available)-1
			self.friction_parameter_has_new = False

			self.friction_parameter_dict, dummy1 ,dummy2 = friction_reasoning.initialize_friction_dictionaries()

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				FrictionParamsStamped, 
				self.friction_parameter_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'FrictionParamsStamped'
			self.buffer_dict[topic] = self.friction_parameter_buffer

		elif topic == '/pivot_frame_realsense':
			self.unpack_functions.append(self.pivot_xyz_realsense_unpack)
			self.pivot_xyz_realsense_buffer = []
			self.pivot_xyz_realsense_available_index = len(self.data_available)-1
			self.pivot_xyz_realsense_has_new = False

			self.pivot_xyz = None
			self.pivot_xyz_realsense = None
			self.pivot_xyz_estimated = None
			self.pivot_message_realsense_time = None
			self.pivot_message_estimated_time = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				TransformStamped, 
				self.pivot_xyz_realsense_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'TransformStamped'
			self.buffer_dict[topic] = self.pivot_xyz_realsense_buffer

		elif topic == '/pivot_frame_estimated':
			self.unpack_functions.append(self.pivot_xyz_estimated_unpack)
			self.pivot_xyz_estimated_buffer = []
			self.pivot_xyz_estimated_available_index = len(self.data_available)-1
			self.pivot_xyz_estimated_has_new = False

			self.pivot_xyz = None
			self.pivot_xyz_realsense = None
			self.pivot_xyz_estimated = None
			self.pivot_message_realsense_time = None
			self.pivot_message_estimated_time = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				TransformStamped, 
				self.pivot_xyz_estimated_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'TransformStamped'
			self.buffer_dict[topic] = self.pivot_xyz_estimated_buffer

		elif topic == '/generalized_positions':
			self.unpack_functions.append(self.generalized_positions_unpack)
			self.generalized_positions_buffer = []
			self.generalized_positions_available_index = len(self.data_available)-1
			self.generalized_positions_has_new = False

			self.generalized_positions = None
			self.l_hand     = None
			self.s_hand     = None
			self.theta_hand = None
			
			self.state_not_exists_bool = True

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				GeneralizedPositionsStamped,  
				self.generalized_positions_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'GeneralizedPositionsStamped'
			self.buffer_dict[topic] = self.generalized_positions_buffer

		elif topic == '/barrier_func_control_command':
			self.unpack_functions.append(self.barrier_func_control_command_unpack)
			self.barrier_func_control_command_buffer = []
			self.barrier_func_control_command_available_index = len(self.data_available)-1
			self.barrier_func_control_command_has_new = False

			self.command_msg = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				ControlCommandStamped, 
				self.barrier_func_control_command_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'ControlCommandStamped'
			self.buffer_dict[topic] = self.barrier_func_control_command_buffer

		elif topic == '/torque_bound_message':
			self.unpack_functions.append(self.torque_bound_unpack)
			self.torque_bound_buffer = []
			self.torque_bound_available_index = len(self.data_available)-1
			self.torque_bound_has_new = False

			self.torque_bounds = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				TorqueBoundsStamped,  
				self.torque_bound_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'TorqueBoundsStamped'
			self.buffer_dict[topic] = self.torque_bound_buffer

		elif topic == '/sliding_state':
			self.unpack_functions.append(self.sliding_state_unpack)
			self.sliding_state_buffer = []
			self.sliding_state_available_index = len(self.data_available)-1
			self.sliding_state_has_new = False

			self.sliding_state = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				SlidingStateStamped, 
				self.sliding_state_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'SlidingStateStamped'
			self.buffer_dict[topic] = self.sliding_state_buffer

		elif topic == '/pivot_sliding_commanded_flag':
			self.unpack_functions.append(self.pivot_sliding_commanded_flag_unpack)
			self.pivot_sliding_commanded_flag_buffer = []
			self.pivot_sliding_commanded_flag_available_index = len(self.data_available)-1
			self.pivot_sliding_commanded_flag_has_new = False

			self.pivot_sliding_commanded_flag = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic, 
				PivotSlidingCommandedFlagStamped, 
				self.pivot_sliding_commanded_flag_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'PivotSlidingCommandedFlagStamped'
			self.buffer_dict[topic] = self.pivot_sliding_commanded_flag_buffer

		elif topic == '/qp_debug_message':
			self.unpack_functions.append(self.qp_debug_unpack)
			self.qp_debug_buffer = []
			self.qp_debug_available_index = len(self.data_available)-1
			self.qp_debug_has_new = False

			self.qp_debug_dict = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic,
				QPDebugStamped,
				self.qp_debug_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'QPDebugStamped'
			self.buffer_dict[topic] = self.qp_debug_buffer

		elif topic == '/target_frame':
			self.unpack_functions.append(self.target_frame_unpack)
			self.target_frame_buffer = []
			self.target_frame_available_index = len(self.data_available)-1
			self.target_frame_has_new = False

			self.target_frame = None
			self.target_frame_homog = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic,
				TransformStamped,
				self.target_frame_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'TransformStamped'
			self.buffer_dict[topic] = self.target_frame_buffer

		elif topic == '/tag_detections':
			self.unpack_functions.append(self.apriltag_unpack)
			self.apriltag_buffer = []
			self.apriltag_available_index = len(self.data_available)-1
			self.apriltag_has_new = False

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic,
				AprilTagDetectionArray,
				self.apriltag_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'AprilTagDetectionArray'
			self.buffer_dict[topic] = self.apriltag_buffer

		elif topic == '/far_cam/color/image_raw':
			self.unpack_functions.append(self.far_cam_image_raw_unpack)
			self.far_cam_image_raw_buffer = []
			self.far_cam_image_raw_available_index = len(self.data_available)-1
			self.far_cam_image_raw_has_new = False

			self.bridge = CvBridge()
			self.far_cam_image_raw = None
			self.far_came_video_writer = None

			callback = self.far_cam_image_raw_callback
			if self.record_mode:
				callback = self.far_cam_image_raw_callback_record_version

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic,
				Image,
				callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'Image'
			self.buffer_dict[topic] = self.far_cam_image_raw_buffer

		elif topic == '/near_cam/color/image_raw':
			self.unpack_functions.append(self.near_cam_image_raw_unpack)
			self.near_cam_image_raw_buffer = []
			self.near_cam_image_raw_available_index = len(self.data_available)-1
			self.near_cam_image_raw_has_new = False

			self.bridge = CvBridge()
			self.near_cam_image_raw = None
			self.near_came_video_writer = None

			callback = self.near_cam_image_raw_callback
			if self.record_mode:
				callback = self.near_cam_image_raw_callback_record_version

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic,
				Image,
				callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'Image'
			self.buffer_dict[topic] = self.near_cam_image_raw_buffer

		elif topic == '/far_cam/color/camera_info':
			self.unpack_functions.append(self.far_cam_camera_info_unpack)
			self.far_cam_camera_info_buffer = []
			self.far_cam_camera_info_available_index = len(self.data_available)-1
			self.far_cam_camera_info_has_new = False

			self.far_cam_camera_matrix = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic,
				CameraInfo,
				self.far_cam_camera_info_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'CameraInfo'
			self.buffer_dict[topic] = self.far_cam_camera_info_buffer

		elif topic == '/near_cam/color/camera_info':
			self.unpack_functions.append(self.near_cam_camera_info_unpack)
			self.near_cam_camera_info_buffer = []
			self.near_cam_camera_info_available_index = len(self.data_available)-1
			self.near_cam_camera_info_has_new = False

			self.near_cam_camera_matrix = None

			self.subscriber_dict[topic] = rospy.Subscriber(
				topic,
				CameraInfo,
				self.near_cam_camera_info_callback,
				queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'CameraInfo'
			self.buffer_dict[topic] = self.near_cam_camera_info_buffer

		else:
			self.subscriber_dict[topic] = None
			self.message_type_dict[topic] = None
			self.buffer_dict[topic] = None


	def spawn_publisher(self,topic):

		if topic == '/ee_pose_in_world_manipulation_from_franka_publisher':
			self.ee_pose_in_world_manipulation_from_franka_pub = rospy.Publisher(
				topic, 
				PoseStamped, 
				queue_size = 10)

		if topic == '/ee_pose_in_base_from_franka_publisher':
			self.ee_pose_in_base_from_franka_pub = rospy.Publisher(
				topic, 
				PoseStamped, 
				queue_size = 10)

		elif topic == '/end_effector_sensor_in_end_effector_frame':
			#wrench at the end-effector in the end effector coordinates -Neel 7/5/2022
			#wrench measured by the ATI, but the torque has been transformed using a different reference point,
			#specifically, the origin of the end-effector frame (should be palm center), and using end-effector basis
			self.end_effector_sensor_in_end_effector_frame_pub = rospy.Publisher(
				topic, 
				WrenchStamped, 
				queue_size = 10)

		elif topic == '/end_effector_sensor_in_world_manipulation_frame':
			#wrench at the end-effector in the static world manipulation frame coordinates 
			#wrench measured by the ATI, but the torque has been transformed using a different reference point,
			#specifically, the origin of the end-effector frame (should be palm center), and using BASE FRAME basis
			self.end_effector_sensor_in_world_manipulation_frame_pub = rospy.Publisher(
				topic, 
				WrenchStamped, 
				queue_size = 10)

		elif topic == '/torque_cone_boundary_test':
			self.torque_cone_boundary_test_pub = rospy.Publisher(
				topic, 
				TorqueConeBoundaryTestStamped , 
				queue_size = 10)

		elif topic == '/torque_cone_boundary_flag':
			self.torque_cone_boundary_flag_pub = rospy.Publisher(
				topic, 
				TorqueConeBoundaryFlagStamped , 
				queue_size = 10)

		elif topic == '/friction_parameters':
			self.friction_parameter_pub = rospy.Publisher(
				topic, 
				FrictionParamsStamped,
				queue_size = 10)

		elif topic == '/sliding_state':
			self.sliding_state_pub = rospy.Publisher(
				topic, 
				SlidingStateStamped,
				queue_size = 10)

		elif topic == '/pivot_sliding_commanded_flag':
			# setting up publisher for sliding
			self.pivot_sliding_commanded_flag_pub = rospy.Publisher(
				topic, 
				PivotSlidingCommandedFlagStamped, 
				queue_size=10)

		elif topic == '/qp_debug_message':
			self.qp_debug_message_pub = rospy.Publisher(
				topic, 
				QPDebugStamped,
				queue_size=10)

		elif topic == '/barrier_func_control_command':
			self.barrier_func_control_command_pub = rospy.Publisher(
				topic,
				ControlCommandStamped,
				queue_size=10)

		elif topic == '/target_frame':
			# intialize impedance target frame
			self.target_frame_pub = rospy.Publisher(
				topic, 
				TransformStamped, 
				queue_size=10) 
			# set up transform broadcaster
			self.target_frame_broadcaster = tf2_ros.TransformBroadcaster()


	def pub_ee_pose_in_world_manipulation_from_franka(self,ee_pose_in_world_manipulation_list):
		ee_pose_in_world_manipulation_pose_stamped = rh.list2pose_stamped(ee_pose_in_world_manipulation_list)
		ee_pose_in_world_manipulation_pose_stamped.header.stamp = rospy.Time.now()
		self.ee_pose_in_world_manipulation_from_franka_pub.publish(ee_pose_in_world_manipulation_pose_stamped)

	def pub_ee_pose_in_base_from_franka(self,ee_pose_in_base_list):
		ee_pose_in_base_pose_stamped = rh.list2pose_stamped(ee_pose_in_base_list)
		ee_pose_in_base_pose_stamped.header.stamp = rospy.Time.now()
		self.ee_pose_in_base_from_franka_pub.publish(ee_pose_in_base_pose_stamped)

	def pub_end_effector_sensor_in_end_effector_frame(self,end_effector_wrench_in_end_effector_list,frame_id):
		msg = rh.list2wrench_stamped(end_effector_wrench_in_end_effector_list,frame_id)
		msg.header.stamp = rospy.Time.now()
		self.end_effector_sensor_in_end_effector_frame_pub.publish(msg)

	def pub_end_effector_sensor_in_world_manipulation_frame(self,end_effector_wrench_in_world_manipulation_list,frame_id):
		msg = rh.list2wrench_stamped(end_effector_wrench_in_world_manipulation_list,frame_id)
		msg.header.stamp = rospy.Time.now()
		self.end_effector_sensor_in_world_manipulation_frame_pub.publish(msg)

	def pub_torque_cone_boundary_test(self,torque_boundary_boolean):
		torque_boundary_boolean_message = pmh.generate_torque_cone_boundary_test_stamped(torque_boundary_boolean)
		torque_boundary_boolean_message.header.stamp = rospy.Time.now()
		self.torque_cone_boundary_test_pub.publish(torque_boundary_boolean_message)

	def pub_torque_cone_boundary_flag(self,torque_boundary_flag):
		torque_boundary_flag_message = pmh.generate_torque_cone_boundary_flag_stamped(torque_boundary_flag)
		torque_boundary_flag_message.header.stamp = rospy.Time.now()
		self.torque_cone_boundary_flag_pub.publish(torque_boundary_flag_message)

	def pub_friction_parameter(self,friction_parameter_dict):
		friction_parameter_msg = pmh.friction_dict_to_friction_stamped(friction_parameter_dict)
		friction_parameter_msg.header.stamp = rospy.Time.now()
		self.friction_parameter_pub.publish(friction_parameter_msg)

	def pub_sliding_state(self,sliding_state_dict):
		sliding_state_stamped = pmh.sliding_dict_to_sliding_stamped(sliding_dict=sliding_state_dict)
		sliding_state_stamped.header.stamp = rospy.Time.now()
		self.sliding_state_pub.publish(sliding_state_stamped)

	def pub_pivot_sliding_commanded_flag(self,pivot_sliding_commanded_flag):
		pivot_sliding_commanded_flag_message = pmh.generate_pivot_sliding_commanded_flag(pivot_sliding_commanded_flag)
		pivot_sliding_commanded_flag_message.header.stamp = rospy.Time.now()
		self.pivot_sliding_commanded_flag_pub.publish(pivot_sliding_commanded_flag_message)

	def pub_barrier_func_control_command(self,command_msg_dict):
		command_msg = pmh.command_dict_to_command_stamped(command_msg_dict)
		command_msg.header.stamp = rospy.Time.now()
		self.barrier_func_control_command_pub.publish(command_msg)


	def pub_target_frame(self,waypoint_pose_list):
		waypoint_pose_list_stamped = rh.list2transform_stamped(waypoint_pose_list, header_frame_id = 'base', child_frame_id = 'hand_estimate')
		waypoint_pose_list_stamped.header.stamp = rospy.Time.now()
		self.target_frame_pub.publish(waypoint_pose_list_stamped)
		self.target_frame_broadcaster.sendTransform(waypoint_pose_list_stamped)

	def pub_qp_debug_message(self,debug_dict):
		qp_debug_stamped = pmh.qp_debug_dict_to_qp_debug_stamped(qp_debug_dict = debug_dict)
		qp_debug_stamped.header.stamp = rospy.Time.now()
		self.qp_debug_message_pub.publish(qp_debug_stamped)

	def force_callback(self,data):
		self.ft_wrench_in_ft_sensor_buffer.append(data)
		if len(self.ft_wrench_in_ft_sensor_buffer)>self.max_queue_size:
			self.ft_wrench_in_ft_sensor_buffer.pop(0)
		self.data_available[self.ft_wrench_in_ft_sensor_available_index]=True

	def force_unpack(self):
		if len(self.ft_wrench_in_ft_sensor_buffer)>0:
			self.ft_wrench_in_ft_sensor = self.ft_wrench_in_ft_sensor_buffer.pop(0)
			self.ft_wrench_in_ft_sensor_list = rh.wrench_stamped2list(self.ft_wrench_in_ft_sensor)

			self.force_has_new = True
		else:
			self.force_has_new = False	

	def ee_pose_in_world_manipulation_callback(self,data):
		self.ee_pose_in_world_manipulation_buffer.append(data)
		if len(self.ee_pose_in_world_manipulation_buffer)>self.max_queue_size:
			self.ee_pose_in_world_manipulation_buffer.pop(0)
		self.data_available[self.ee_pose_in_world_manipulation_available_index]=True
		
	def ee_pose_in_world_manipulation_unpack(self):
		if len(self.ee_pose_in_world_manipulation_buffer)>0:

			self.ee_pose_in_world_manipulation = self.ee_pose_in_world_manipulation_buffer.pop(0)
			self.ee_pose_in_world_manipulation_list = rh.pose_stamped2list(self.ee_pose_in_world_manipulation)
			self.ee_pose_in_world_manipulation_homog = rh.matrix_from_pose_list(self.ee_pose_in_world_manipulation_list)

			self.ee_pose_in_world_manipulation_has_new = True
		else:
			self.ee_pose_in_world_manipulation_has_new = False

	def ee_pose_in_base_callback(self,data):
		self.ee_pose_in_base_buffer.append(data)
		if len(self.ee_pose_in_base_buffer)>self.max_queue_size:
			self.ee_pose_in_base_buffer.pop(0)
		self.data_available[self.ee_pose_in_base_available_index]=True
		
	def ee_pose_in_base_unpack(self):
		if len(self.ee_pose_in_base_buffer)>0:

			self.ee_pose_in_base = self.ee_pose_in_base_buffer.pop(0)
			self.ee_pose_in_base_list = rh.pose_stamped2list(self.ee_pose_in_base)
			self.ee_pose_in_base_homog = rh.matrix_from_pose_list(self.ee_pose_in_base_list)

			self.base_z_in_ee_frame = self.ee_pose_in_base_homog[2, :3]

			self.ee_pose_in_base_has_new = True
		else:
			self.ee_pose_in_base_has_new = False

	def end_effector_wrench_callback(self,data):
		self.measured_contact_wrench_buffer.append(data)
		if len(self.measured_contact_wrench_buffer)>self.max_queue_size:
			self.measured_contact_wrench_buffer.pop(0)
		self.data_available[self.measured_contact_wrench_available_index]=True

	def end_effector_wrench_unpack(self):
		if  len(self.measured_contact_wrench_buffer)>0:

			end_effector_wrench = self.measured_contact_wrench_buffer.pop(0)

			measured_contact_wrench_6D = rh.wrench_stamped2list(
				end_effector_wrench)

			self.measured_contact_wrench = -np.array([
				measured_contact_wrench_6D[0], 
				measured_contact_wrench_6D[1],
				measured_contact_wrench_6D[-1]])

			self.end_effector_wrench_has_new = True
		else:
			self.end_effector_wrench_has_new = False

	def end_effector_wrench_world_manipulation_frame_callback(self,data):
		self.measured_world_manipulation_wrench_buffer.append(data)
		if len(self.measured_world_manipulation_wrench_buffer)>self.max_queue_size:
			self.measured_world_manipulation_wrench_buffer.pop(0)
		self.data_available[self.measured_world_manipulation_wrench_available_index]=True

	def end_effector_wrench_world_manipulation_frame_unpack(self):
		if len(self.measured_world_manipulation_wrench_buffer)>0:
			world_manipulation_wrench = self.measured_world_manipulation_wrench_buffer.pop(0)

			measured_world_manipulation_wrench_6D = rh.wrench_stamped2list(
				world_manipulation_wrench)

			self.measured_world_manipulation_wrench = -np.array([
				measured_world_manipulation_wrench_6D[0], 
				measured_world_manipulation_wrench_6D[1],
				measured_world_manipulation_wrench_6D[-1]])

			self.end_effector_wrench_world_manipulation_frame_has_new = True
		else:
			self.end_effector_wrench_world_manipulation_frame_has_new = False

	def torque_cone_boundary_test_callback(self,data):
		self.torque_cone_boundary_test_buffer.append(data)
		if len(self.torque_cone_boundary_test_buffer)>self.max_queue_size:
			self.torque_cone_boundary_test_buffer.pop(0)
		self.data_available[self.torque_cone_boundary_test_available_index]=True

	def torque_cone_boundary_test_unpack(self):
		if len(self.torque_cone_boundary_test_buffer)>0:
			data = self.torque_cone_boundary_test_buffer.pop(0)
			self.torque_cone_boundary_test = data.boundary_test

			self.torque_cone_boundary_test_has_new = True
		else:
			self.torque_cone_boundary_test_has_new = False

	def torque_cone_boundary_flag_callback(self,data):
		self.torque_cone_boundary_flag_buffer.append(data)
		if len(self.torque_cone_boundary_flag_buffer)>self.max_queue_size:
			self.torque_cone_boundary_flag_buffer.pop(0)
		self.data_available[self.torque_cone_boundary_flag_available_index]=True

	def torque_cone_boundary_flag_unpack(self):
		if len(self.torque_cone_boundary_flag_buffer)>0:
			data = self.torque_cone_boundary_flag_buffer.pop(0)
			self.torque_cone_boundary_flag = data.boundary_flag

			self.torque_cone_boundary_flag_has_new = True
		else:
			self.torque_cone_boundary_flag_has_new = False

	def friction_parameter_callback(self,data):
		self.friction_parameter_buffer.append(data)
		if len(self.friction_parameter_buffer)>self.max_queue_size:
			self.friction_parameter_buffer.pop(0)
		self.data_available[self.friction_parameter_available_index]=True

	def friction_parameter_unpack(self):
		if len(self.friction_parameter_buffer)>0:
			data = self.friction_parameter_buffer.pop(0)
			self.friction_parameter_dict = pmh.friction_stamped_to_friction_dict(data)
			friction_reasoning.convert_friction_param_dict_to_array(self.friction_parameter_dict)

			self.friction_parameter_has_new = True
		else:
			self.friction_parameter_has_new = False

	def pivot_xyz_realsense_callback(self,data):
		self.pivot_xyz_realsense_buffer.append([data,time.time()])
		if len(self.pivot_xyz_realsense_buffer)>self.max_queue_size:
			self.pivot_xyz_realsense_buffer.pop(0)
		self.data_available[self.pivot_xyz_realsense_available_index]=True

	def pivot_xyz_realsense_unpack(self):
		if len(self.pivot_xyz_realsense_buffer)>0:
			msg = self.pivot_xyz_realsense_buffer.pop(0)
			data = msg[0]
			self.pivot_message_realsense_time = msg[1]
			self.pivot_xyz_realsense =  [data.transform.translation.x,
				data.transform.translation.y,
				data.transform.translation.z]

			self.pivot_xyz = self.pivot_xyz_realsense

			self.pivot_xyz_realsense_has_new = True
		else:
			self.pivot_xyz_realsense_has_new = False

	def pivot_xyz_estimated_callback(self,data):
		self.pivot_xyz_estimated_buffer.append([data,time.time()])
		if len(self.pivot_xyz_estimated_buffer)>self.max_queue_size:
			self.pivot_xyz_estimated_buffer.pop(0)
		self.data_available[self.pivot_xyz_estimated_available_index]=True

	def pivot_xyz_estimated_unpack(self):
		if len(self.pivot_xyz_estimated_buffer)>0:
			msg = self.pivot_xyz_estimated_buffer.pop(0)
			data = msg[0]
			self.pivot_message_estimated_time = msg[1]
			self.pivot_xyz_estimated =  [data.transform.translation.x,
				data.transform.translation.y,
				data.transform.translation.z]

			#if there has been no message from the realsense
			if (self.pivot_xyz_realsense is None) or (self.pivot_message_estimated_time-self.pivot_message_realsense_time>.5):
				self.pivot_xyz = self.pivot_xyz_estimated

			self.pivot_xyz_estimated_has_new = True
		else:
			self.pivot_xyz_estimated_has_new = False

	def generalized_positions_callback(self,data):
		self.generalized_positions_buffer.append(data)
		if len(self.generalized_positions_buffer)>self.max_queue_size:
			self.generalized_positions_buffer.pop(0)
		self.data_available[self.generalized_positions_available_index]=True

	def generalized_positions_unpack(self):
		if len(self.generalized_positions_buffer)>0:
			data = self.generalized_positions_buffer.pop(0)
			self.generalized_positions = data.generalized_positions
			self.l_hand     = contact_pose[0]
			self.s_hand     = contact_pose[1]
			self.theta_hand = contact_pose[2]

			self.state_not_exists_bool = False

			self.generalized_positions_has_new = True
		else:
			self.generalized_positions_has_new = False

	def barrier_func_control_command_callback(self,data):
		self.barrier_func_control_command_buffer.append(data)
		if len(self.barrier_func_control_command_buffer)>self.max_queue_size:
			self.barrier_func_control_command_buffer.pop(0)
		self.data_available[self.barrier_func_control_command_available_index]=True

	def barrier_func_control_command_unpack(self):
		if len(self.barrier_func_control_command_buffer)>0:
			data = self.barrier_func_control_command_buffer.pop(0)
			self.command_msg = pmh.command_stamped_to_command_dict(data)

			self.barrier_func_control_command_has_new = True
		else:
			self.barrier_func_control_command_has_new = False

	def torque_bound_callback(self,data):
		self.torque_bound_buffer.append(data)
		if len(self.torque_bound_buffer)>self.max_queue_size:
			self.torque_bound_buffer.pop(0)
		self.data_available[self.torque_bound_available_index]=True

	def torque_bound_unpack(self):
		if len(self.torque_bound_buffer)>0:
			data = self.torque_bound_buffer.pop(0)
			self.torque_bounds = data.torque_bounds

			self.torque_bound_has_new = True
		else:
			self.torque_bound_has_new = False

	def sliding_state_callback(self,data):
		self.sliding_state_buffer.append(data)
		if len(self.sliding_state_buffer)>self.max_queue_size:
			self.sliding_state_buffer.pop(0)
		self.data_available[self.sliding_state_available_index]=True

	def sliding_state_unpack(self):
		if len(self.sliding_state_buffer)>0:
			data = self.sliding_state_buffer.pop(0)
			self.sliding_state = data.sliding_state

			self.sliding_state_has_new = True
		else:
			self.sliding_state_has_new = False

	def pivot_sliding_commanded_flag_callback(self,data):
		self.pivot_sliding_commanded_flag_buffer.append(data)
		if len(self.pivot_sliding_commanded_flag_buffer)>self.max_queue_size:
			self.pivot_sliding_commanded_flag_buffer.pop(0)
		self.data_available[self.pivot_sliding_commanded_flag_available_index]=True

	def pivot_sliding_commanded_flag_unpack(self):
		if len(self.pivot_sliding_commanded_flag_buffer)>0:
			data = self.pivot_sliding_commanded_flag_buffer.pop(0)
			self.pivot_sliding_commanded_flag = data.command_flag

			self.pivot_sliding_commanded_flag_has_new = True
		else:
			self.pivot_sliding_commanded_flag_has_new = False

	def qp_debug_callback(self,data):
		self.qp_debug_buffer.append(data)
		if len(self.qp_debug_buffer)>self.max_queue_size:
			self.qp_debug_buffer.pop(0)
		self.data_available[self.qp_debug_available_index]=True

	def qp_debug_unpack(self):
		if len(self.qp_debug_buffer)>0:
			data = self.qp_debug_buffer.pop(0)

			if data.qp_debug != '':
				self.qp_debug_dict = pmh.qp_debug_stamped_to_qp_debug_dict(data)
				self.qp_debug_has_new = True
			else:
				self.qp_debug_has_new = False
		else:
			self.qp_debug_has_new = False

	def target_frame_callback(self,data):
		self.target_frame_buffer.append(data)
		if len(self.target_frame_buffer)>self.max_queue_size:
			self.target_frame_buffer.pop(0)
		self.data_available[self.target_frame_available_index]=True

	def target_frame_unpack(self):
		if len(self.target_frame_buffer)>0:
			data = self.target_frame_buffer.pop(0)

			self.target_frame = rh.transform_stamped2list(data)
			self.target_frame_homog =  rh.matrix_from_pose_list(self.target_frame)

			self.target_frame_has_new = True
		else:
			self.target_frame_has_new = False

	def apriltag_callback(self,data):
		self.apriltag_buffer.append(data)
		if len(self.apriltag_buffer)>self.max_queue_size:
			self.apriltag_buffer.pop(0)
		self.data_available[self.apriltag_available_index]=True

	def apriltag_unpack(self):
		if len(self.apriltag_buffer)>0:
			data = self.apriltag_buffer.pop(0)

			self.apriltag_has_new = True
		else:
			self.apriltag_has_new = False

	def far_cam_image_raw_callback(self,data):
		self.far_cam_image_raw_buffer.append(data)
		if len(self.far_cam_image_raw_buffer)>self.max_queue_size:
			self.far_cam_image_raw_buffer.pop(0)
		self.data_available[self.far_cam_image_raw_available_index]=True

	def far_cam_image_raw_unpack(self):
		if len(self.far_cam_image_raw_buffer)>0:
			data = self.far_cam_image_raw_buffer.pop(0)
			self.far_cam_image_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")

			self.far_cam_image_raw_has_new = True
		else:
			self.far_cam_image_raw_has_new = False

	def near_cam_image_raw_callback(self,data):
		self.near_cam_image_raw_buffer.append(data)
		if len(self.near_cam_image_raw_buffer)>self.max_queue_size:
			self.near_cam_image_raw_buffer.pop(0)
		self.data_available[self.near_cam_image_raw_available_index]=True

	def near_cam_image_raw_unpack(self):
		if len(self.near_cam_image_raw_buffer)>0:
			data = self.near_cam_image_raw_buffer.pop(0)
			self.near_cam_image_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")

			self.near_cam_image_raw_has_new = True
		else:
			self.near_cam_image_raw_has_new = False

	def far_cam_image_raw_callback_record_version(self,data):
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

		if self.far_came_video_writer is None:
			image_height, image_width, image_layers = cv_image.shape
			image_size = (image_width, image_height)
			save_name = os.path.join(self.path,self.fname)+'_far_cam_color_image_raw.avi'
			self.far_came_video_writer = cv2.VideoWriter(save_name, cv2.VideoWriter_fourcc(*'DIVX'), 30, image_size)

		self.far_came_video_writer.write(cv_image)
		self.far_cam_image_raw_buffer.append(data.header.stamp.to_sec())

	def near_cam_image_raw_callback_record_version(self,data):
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

		if self.near_came_video_writer is None:
			image_height, image_width, image_layers = cv_image.shape
			image_size = (image_width, image_height)
			save_name = os.path.join(self.path,self.fname)+'_near_cam_color_image_raw.avi'
			self.near_came_video_writer = cv2.VideoWriter(save_name, cv2.VideoWriter_fourcc(*'DIVX'), 30, image_size)

		self.near_came_video_writer.write(cv_image)
		self.near_cam_image_raw_buffer.append(data.header.stamp.to_sec())

	def far_cam_camera_info_callback(self,data):
		self.far_cam_camera_info_buffer.append(data)
		if len(self.far_cam_camera_info_buffer)>self.max_queue_size:
			self.far_cam_camera_info_buffer.pop(0)
		self.data_available[self.far_cam_camera_info_available_index]=True

	def far_cam_camera_info_unpack(self):
		if len(self.far_cam_camera_info_buffer)>0:
			data = self.far_cam_camera_info_buffer.pop(0)
			self.far_cam_camera_matrix = np.reshape(data.P, (3, 4))

			self.far_cam_camera_info_has_new = True
		else:
			self.far_cam_camera_info_has_new = False

	def near_cam_camera_info_callback(self,data):
		self.near_cam_camera_info_buffer.append(data)
		if len(self.near_cam_camera_info_buffer)>self.max_queue_size:
			self.near_cam_camera_info_buffer.pop(0)
		self.data_available[self.near_cam_camera_info_available_index]=True

	def near_cam_camera_info_unpack(self):
		if len(self.near_cam_camera_info_buffer)>0:
			data = self.near_cam_camera_info_buffer.pop(0)
			self.near_cam_camera_matrix = np.reshape(data.P, (3, 4))

			self.near_cam_camera_info_has_new = True
		else:
			self.near_cam_camera_info_has_new = False