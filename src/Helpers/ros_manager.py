#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import time
import numpy as np
from Estimation import friction_reasoning


import cv2

import Helpers.kinematics_helper as kh
from Helpers.pickle_manager import pickle_manager
import pickle

class ros_manager(object):
	def __init__(self,record_mode=False,path=None,experiment_label=None,load_mode=False,fname=None):
		self.dt_rate = None
		self.data_available = []
		self.available_mask = []
		self.unpack_functions = []
		self.callback_dict = {}
		self.topic_list = []
		self.subscriber_topic_dict = {}
		self.publisher_topic_dict = {}
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

		self.static_transform_dict = {}

		self.load_mode = load_mode

		self.read_dict = None

		self.has_spawned_transform_listener = False

		if self.load_mode:
			if fname is not None:
				self.fname_load = fname.split('.')[0]
				self.initialize_read()

			else:
				print('Error! No fname given!')
		else:
			self.ros_imports()

		if self.record_mode:
			self.max_queue_size=np.inf
			self.subscriber_queue_size = 100
			self.pkm = pickle_manager(self.path)
			self.fname = self.pkm.generate_experiment_name(experiment_label=self.experiment_label)
		else: 
			self.max_queue_size=1
			self.subscriber_queue_size = 1

	def ros_imports(self):
		global rospy
		global PoseStamped, WrenchStamped, TransformStamped
		global CameraInfo, Image
		global AprilTagDetectionArray

		global SlidingStateStamped, FrictionParamsStamped, ControlCommandStamped, QPDebugStamped
		global TorqueConeBoundaryFlagStamped, PivotSlidingCommandedFlagStamped, TorqueConeBoundaryTestStamped
		global TorqueBoundsStamped, GeneralizedPositionsStamped, PolygonContactStateStamped

		global pmh, tf, time_logger, tf2_ros, CvBridge

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
								PolygonContactStateStamped,
								TorqueConeBoundaryTestStamped,
		 						TorqueBoundsStamped, 
		 						GeneralizedPositionsStamped)

		from cv_bridge import CvBridge
		import Helpers.pbal_msg_helper as pmh
		from Helpers.time_logger import time_logger
		import tf
		import tf2_ros
			
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

	def store_in_pickle(self):
		self.pkm.store_in_pickle(self.topic_list,self.buffer_dict,self.message_type_dict,experiment_label=self.experiment_label,transform_dict = self.static_transform_dict)

	def initialize_read(self):

		with open(self.path+self.fname_load+'.pickle', 'rb') as handle:
			self.read_dict = pickle.load(handle)

		if ('/near_cam/color/image_raw' in self.read_dict and 'time_list' in self.read_dict['/near_cam/color/image_raw'] and
			len(self.read_dict['/near_cam/color/image_raw']['time_list'])>0):
			self.vidcap_near = cv2.VideoCapture(self.path+self.fname_load+'_near_cam_color_image_raw.avi')
		if ('/far_cam/color/image_raw' in self.read_dict and 'time_list' in self.read_dict['/far_cam/color/image_raw'] and
			len(self.read_dict['/far_cam/color/image_raw']['time_list'])>0):
			self.vidcap_far = cv2.VideoCapture(self.path+self.fname_load+'_far_cam_color_image_raw.avi')

		self.static_transform_dict = self.read_dict['transform_dict']

		tmin = None
		tmax = None
		self.read_index_dict = {}

		for topic in self.read_dict:
			if 'time_list' in self.read_dict[topic]:
				if len(self.read_dict[topic]['time_list'])>0:
					if tmin is None:
						tmin = self.read_dict[topic]['time_list'][0]
					else:
						tmin = min(tmin,self.read_dict[topic]['time_list'][0])

					if tmax is None:
						tmax = self.read_dict[topic]['time_list'][-1]
					else:
						tmax = max(tmax,self.read_dict[topic]['time_list'][-1])

					self.read_index_dict[topic] = 0
		self.t_min_record = tmin
		self.t_max_record = tmax
		self.t_current_record = tmin-.1

	def read_still_running(self):
		return self.t_current_record<=self.t_max_record

	def update_callbacks(self):
		for topic in self.callback_dict:
			if topic in self.read_dict and 'time_list' in self.read_dict[topic] and len(self.read_dict[topic]['time_list'])>0:
				while (self.read_index_dict[topic]<len(self.read_dict[topic]['time_list']) and
					   self.read_dict[topic]['time_list'][self.read_index_dict[topic]]<=self.t_current_record and
					   self.callback_dict[topic] is not None):

					message_time = self.read_dict[topic]['time_list'][self.read_index_dict[topic]]

					if topic == '/near_cam/color/image_raw':
						success,image = self.vidcap_near.read()	
						if success:
							self.callback_dict[topic](image,message_time)

					elif topic == '/far_cam/color/image_raw':
						success,image = self.vidcap_far.read()	
						if success:
							self.callback_dict[topic](image,message_time)
					else:
						self.callback_dict[topic](self.read_dict[topic]['msg_list'][self.read_index_dict[topic]],message_time)

					self.read_index_dict[topic]+=1

	def wait_for_necessary_data(self, dt_load_mode = .001):
		print('Waiting to hear from essential subscribers')
		can_proceed = False

		if self.load_mode:
			while not can_proceed and self.t_current_record<=self.t_max_record:
				self.t_current_record+=dt_load_mode
				self.update_callbacks()

				can_proceed = True
				for i in range(len(self.data_available)):
					can_proceed = can_proceed and (self.data_available[i] or self.available_mask[i])

			if not can_proceed:
				print('essential subscribers unavailable')

			return can_proceed
		else:
			while not can_proceed:
				time.sleep(.1)

				can_proceed = True
				for i in range(len(self.data_available)):
					can_proceed = can_proceed and (self.data_available[i] or self.available_mask[i])

		return can_proceed
		

	def unpack_all(self):
		if self.load_mode:
			self.update_callbacks()
		for my_func in self.unpack_functions:
			if my_func is not None:
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
		self.dt_rate= 1.0/RATE
				
	def sleep(self,actually_sleep=True):
		if self.dt_rate is None:
			dt = .001
		else:
			dt = self.dt_rate

		self.t_current_record+=dt

		if actually_sleep:
			time.sleep(dt)

	def eval_current_time(self):
		if self.load_mode:
			return self.t_current_record
		else:
			return rospy.Time.now().to_sec()

	def init_time_logger(self,node_name=None):
		if node_name is not None:
			self.node_name = node_name
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
		global IMH
		import Helpers.impedance_mode_helper as IMH
		self.my_impedance_mode_helper = IMH.impedance_mode_helper()

	def set_matrices_pbal_mode(self,TIPI,TOOPI,RIPI,ROOPI,rot_mat):
		if self.my_impedance_mode_helper is not None:
			self.my_impedance_mode_helper.set_matrices_pbal_mode(TIPI,TOOPI,RIPI,ROOPI,rot_mat)

	def set_cart_impedance_pose(self, pose):
		if self.my_impedance_mode_helper is not None:
			self.my_impedance_mode_helper.set_cart_impedance_pose(pose)

	def set_cart_impedance_stiffness(self,stiffness=None, damping=None):
			if self.my_impedance_mode_helper is not None:
				self.my_impedance_mode_helper.set_cart_impedance_stiffness(stiffness=stiffness, damping=damping)

	def initialize_impedance_mode(self,torque_upper=None,force_upper=None):
		if torque_upper is None or force_upper is None:
			torque_upper = [40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0]
			force_upper = [100.0, 100.0, 100.0, 25.0, 25.0, 25.0]

		if self.my_impedance_mode_helper is not None:
			self.my_impedance_mode_helper.initialize_impedance_mode(torque_upper=torque_upper,force_upper=force_upper)

	def spawn_transform_listener(self):
		if not self.load_mode and not self.has_spawned_transform_listener:
			self.listener = tf.TransformListener()
			self.has_spawned_transform_listener = True

	# calls the ROS service that tares (zeroes) the force-torque sensor
	def zero_ft_sensor(self):
		import netft_rdt_driver.srv as srv
		rospy.wait_for_service('/netft/zero', timeout=0.5)
		zero_ft = rospy.ServiceProxy('/netft/zero', srv.Zero)
		zero_ft()

	def lookupTransform(self, homeFrame, targetFrame):
		if self.load_mode:
			if homeFrame in self.static_transform_dict and targetFrame in self.static_transform_dict[homeFrame]:
				return self.static_transform_dict[homeFrame][targetFrame]
			else:
				return (None, None)
		else:
			if self.listener is not None:
				ntfretry = 100
				retryTime = .05
				for i in range(ntfretry):
					try:
						(trans, rot) = self.listener.lookupTransform(targetFrame, homeFrame, self.listener.getLatestCommonTime(targetFrame, homeFrame))
			
						if self.record_mode:
							if homeFrame not in self.static_transform_dict:
								self.static_transform_dict[homeFrame] = {}
							self.static_transform_dict[homeFrame][targetFrame] = (trans, rot)

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
		if topic in self.subscriber_topic_dict:
			return None

		self.available_mask.append(not isNecessary)
		self.data_available.append(False)
		self.topic_list.append(topic)

		if   topic ==  '/netft/netft_data':
			self.unpack_functions.append(self.force_unpack)
			self.ft_wrench_in_ft_sensor_buffer = []
			self.ft_wrench_in_ft_sensor_available_index = len(self.data_available)-1
			self.force_has_new = False

			self.ft_wrench_time = None
			self.ft_wrench_in_ft_sensor = None
			self.ft_wrench_in_ft_sensor_list = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					WrenchStamped, 
					self.force_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'WrenchStamped'
			self.buffer_dict[topic] = self.ft_wrench_in_ft_sensor_buffer
			self.callback_dict[topic] = self.force_callback

		elif topic == '/ee_pose_in_world_manipulation_from_franka_publisher':
			self.unpack_functions.append(self.ee_pose_in_world_manipulation_unpack)
			self.ee_pose_in_world_manipulation_buffer = []
			self.ee_pose_in_world_manipulation_available_index = len(self.data_available)-1
			self.ee_pose_in_world_manipulation_has_new = False

			self.ee_pose_in_world_manipulation = None
			self.ee_pose_in_world_manipulation_time = None
			self.ee_pose_in_world_manipulation_list = None
			self.ee_pose_in_world_manipulation_homog = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					PoseStamped, 
					self.ee_pose_in_world_manipulation_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'PoseStamped'
			self.buffer_dict[topic] = self.ee_pose_in_world_manipulation_buffer
			self.callback_dict[topic] = self.ee_pose_in_world_manipulation_callback

		elif topic == '/ee_pose_in_base_from_franka_publisher':
			self.unpack_functions.append(self.ee_pose_in_base_unpack)
			self.ee_pose_in_base_buffer = []
			self.ee_pose_in_base_available_index = len(self.data_available)-1
			self.ee_pose_in_base_has_new = False

			self.ee_pose_in_base = None
			self.ee_pose_in_base_time = None
			self.ee_pose_in_base_list = None
			self.ee_pose_in_base_homog = None

			self.base_z_in_ee_frame = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					PoseStamped, 
					self.ee_pose_in_base_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'PoseStamped'
			self.buffer_dict[topic] = self.ee_pose_in_base_buffer
			self.callback_dict[topic] = self.ee_pose_in_base_callback

		elif topic == '/end_effector_sensor_in_end_effector_frame':
			self.unpack_functions.append(self.end_effector_wrench_unpack)
			self.measured_contact_wrench_buffer = []
			self.measured_contact_wrench_available_index = len(self.data_available)-1
			self.end_effector_wrench_has_new = False

			self.measured_contact_wrench_6D = None
			self.measured_contact_wrench = None
			self.measured_contact_wrench_time = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					WrenchStamped,  
					self.end_effector_wrench_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'WrenchStamped'
			self.buffer_dict[topic] = self.measured_contact_wrench_buffer
			self.callback_dict[topic] = self.end_effector_wrench_callback

		elif topic == '/end_effector_sensor_in_world_manipulation_frame':
			self.unpack_functions.append(self.end_effector_wrench_world_manipulation_frame_unpack)
			self.measured_world_manipulation_wrench_buffer = []
			self.measured_world_manipulation_wrench_available_index = len(self.data_available)-1
			self.end_effector_wrench_world_manipulation_frame_has_new = False

			self.measured_world_manipulation_wrench_6D = None
			self.measured_world_manipulation_wrench = None
			self.world_manipulation_wrench_time = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					WrenchStamped,  
					self.end_effector_wrench_world_manipulation_frame_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'WrenchStamped'
			self.buffer_dict[topic] = self.measured_world_manipulation_wrench_buffer
			self.callback_dict[topic] = self.end_effector_wrench_world_manipulation_frame_callback

		elif topic == '/torque_cone_boundary_test':
			self.unpack_functions.append(self.torque_cone_boundary_test_unpack)
			self.torque_cone_boundary_test_buffer = []
			self.torque_cone_boundary_test_available_index = len(self.data_available)-1
			self.torque_cone_boundary_test_has_new = False

			self.torque_cone_boundary_test = None
			self.torque_cone_boundary_test_time = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					TorqueConeBoundaryTestStamped,  
					self.torque_cone_boundary_test_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'TorqueConeBoundaryTestStamped'
			self.buffer_dict[topic] = self.torque_cone_boundary_test_buffer
			self.callback_dict[topic] = self.torque_cone_boundary_test_callback

		elif topic == '/torque_cone_boundary_flag':
			self.unpack_functions.append(self.torque_cone_boundary_flag_unpack)
			self.torque_cone_boundary_flag_buffer = []
			self.torque_cone_boundary_flag_available_index = len(self.data_available)-1
			self.torque_cone_boundary_flag_has_new = False

			self.torque_cone_boundary_flag = None
			self.torque_cone_boundary_flag_time = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					TorqueConeBoundaryFlagStamped,  
					self.torque_cone_boundary_flag_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'TorqueConeBoundaryFlagStamped'
			self.buffer_dict[topic] = self.torque_cone_boundary_flag_buffer
			self.callback_dict[topic] = self.torque_cone_boundary_flag_callback


		elif topic == '/friction_parameters':
			self.unpack_functions.append(self.friction_parameter_unpack)
			self.friction_parameter_buffer = []
			self.friction_parameter_available_index = len(self.data_available)-1
			self.friction_parameter_has_new = False

			self.friction_parameter_dict, dummy1 ,dummy2 = friction_reasoning.initialize_friction_dictionaries()
			self.friction_parameter_time = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					FrictionParamsStamped, 
					self.friction_parameter_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'FrictionParamsStamped'
			self.buffer_dict[topic] = self.friction_parameter_buffer
			self.callback_dict[topic] = self.friction_parameter_callback

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

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					TransformStamped, 
					self.pivot_xyz_realsense_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'TransformStamped'
			self.buffer_dict[topic] = self.pivot_xyz_realsense_buffer
			self.callback_dict[topic] = self.pivot_xyz_realsense_callback

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

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					TransformStamped, 
					self.pivot_xyz_estimated_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'TransformStamped'
			self.buffer_dict[topic] = self.pivot_xyz_estimated_buffer
			self.callback_dict[topic] = self.pivot_xyz_estimated_callback

		elif topic == '/generalized_positions':
			self.unpack_functions.append(self.generalized_positions_unpack)
			self.generalized_positions_buffer = []
			self.generalized_positions_available_index = len(self.data_available)-1
			self.generalized_positions_has_new = False
			self.generalized_positions_time = None

			self.generalized_positions = None
			self.l_hand     = None
			self.s_hand     = None
			self.theta_hand = None
			
			self.state_not_exists_bool = True

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					GeneralizedPositionsStamped,  
					self.generalized_positions_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'GeneralizedPositionsStamped'
			self.buffer_dict[topic] = self.generalized_positions_buffer
			self.callback_dict[topic] = self.generalized_positions_callback

		elif topic == '/barrier_func_control_command':
			self.unpack_functions.append(self.barrier_func_control_command_unpack)
			self.barrier_func_control_command_buffer = []
			self.barrier_func_control_command_available_index = len(self.data_available)-1
			self.barrier_func_control_command_has_new = False
			self.barrier_func_control_command_time = None

			self.command_msg = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					ControlCommandStamped, 
					self.barrier_func_control_command_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'ControlCommandStamped'
			self.buffer_dict[topic] = self.barrier_func_control_command_buffer
			self.callback_dict[topic] = self.barrier_func_control_command_callback

		elif topic == '/torque_bound_message':
			self.unpack_functions.append(self.torque_bound_unpack)
			self.torque_bound_buffer = []
			self.torque_bound_available_index = len(self.data_available)-1
			self.torque_bound_has_new = False
			self.torque_bounds_time = None

			self.torque_bounds = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					TorqueBoundsStamped,  
					self.torque_bound_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'TorqueBoundsStamped'
			self.buffer_dict[topic] = self.torque_bound_buffer
			self.callback_dict[topic] = self.torque_bound_callback

		elif topic == '/sliding_state':
			self.unpack_functions.append(self.sliding_state_unpack)
			self.sliding_state_buffer = []
			self.sliding_state_available_index = len(self.data_available)-1
			self.sliding_state_has_new = False
			self.sliding_state_time = None

			self.sliding_state = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					SlidingStateStamped, 
					self.sliding_state_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'SlidingStateStamped'
			self.buffer_dict[topic] = self.sliding_state_buffer
			self.callback_dict[topic] = self.sliding_state_callback

		elif topic == '/pivot_sliding_commanded_flag':
			self.unpack_functions.append(self.pivot_sliding_commanded_flag_unpack)
			self.pivot_sliding_commanded_flag_buffer = []
			self.pivot_sliding_commanded_flag_available_index = len(self.data_available)-1
			self.pivot_sliding_commanded_flag_has_new = False
			self.pivot_sliding_commanded_time = None

			self.pivot_sliding_commanded_flag = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic, 
					PivotSlidingCommandedFlagStamped, 
					self.pivot_sliding_commanded_flag_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'PivotSlidingCommandedFlagStamped'
			self.buffer_dict[topic] = self.pivot_sliding_commanded_flag_buffer
			self.callback_dict[topic] = self.pivot_sliding_commanded_flag_callback

		elif topic == '/qp_debug_message':
			self.unpack_functions.append(self.qp_debug_unpack)
			self.qp_debug_buffer = []
			self.qp_debug_available_index = len(self.data_available)-1
			self.qp_debug_has_new = False

			self.qp_debug_dict = None
			self.qp_debug_time = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic,
					QPDebugStamped,
					self.qp_debug_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'QPDebugStamped'
			self.buffer_dict[topic] = self.qp_debug_buffer
			self.callback_dict[topic] = self.qp_debug_callback

		elif topic == '/target_frame':
			self.unpack_functions.append(self.target_frame_unpack)
			self.target_frame_buffer = []
			self.target_frame_available_index = len(self.data_available)-1
			self.target_frame_has_new = False

			self.target_frame = None
			self.target_frame_time = None
			self.target_frame_homog = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic,
					TransformStamped,
					self.target_frame_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'TransformStamped'
			self.buffer_dict[topic] = self.target_frame_buffer
			self.callback_dict[topic] = self.target_frame_callback

		elif topic == '/tag_detections':
			self.unpack_functions.append(self.apriltag_unpack)
			self.apriltag_buffer = []
			self.apriltag_available_index = len(self.data_available)-1
			self.apriltag_has_new = False

			self.apriltag_pose_list_dict = None
			self.apriltag_pose_homog_dict = None
			self.apriltag_time = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic,
					AprilTagDetectionArray,
					self.apriltag_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'AprilTagDetectionArray'
			self.buffer_dict[topic] = self.apriltag_buffer
			self.callback_dict[topic] = self.apriltag_callback

		elif topic == '/far_cam/color/image_raw':
			self.unpack_functions.append(self.far_cam_image_raw_unpack)
			self.far_cam_image_raw_buffer = []
			self.far_cam_image_raw_available_index = len(self.data_available)-1
			self.far_cam_image_raw_has_new = False

			
			self.far_cam_image_raw = None
			self.far_cam_image_time = None
			self.far_came_video_writer = None

			callback = self.far_cam_image_raw_callback
			if self.record_mode:
				callback = self.far_cam_image_raw_callback_record_version

			if not self.load_mode:
				self.bridge = CvBridge()
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic,
					Image,
					callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'Image'
			self.buffer_dict[topic] = self.far_cam_image_raw_buffer
			self.callback_dict[topic] = callback

		elif topic == '/near_cam/color/image_raw':
			self.unpack_functions.append(self.near_cam_image_raw_unpack)
			self.near_cam_image_raw_buffer = []
			self.near_cam_image_raw_available_index = len(self.data_available)-1
			self.near_cam_image_raw_has_new = False

			
			self.near_cam_image_raw = None
			self.near_cam_image_time = None
			self.near_came_video_writer = None

			callback = self.near_cam_image_raw_callback
			if self.record_mode:
				callback = self.near_cam_image_raw_callback_record_version

			if not self.load_mode:
				self.bridge = CvBridge()
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic,
					Image,
					callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'Image'
			self.buffer_dict[topic] = self.near_cam_image_raw_buffer
			self.callback_dict[topic] = callback

		elif topic == '/far_cam/color/camera_info':
			self.unpack_functions.append(self.far_cam_camera_info_unpack)
			self.far_cam_camera_info_buffer = []
			self.far_cam_camera_info_available_index = len(self.data_available)-1
			self.far_cam_camera_info_has_new = False

			self.far_cam_camera_matrix = None
			self.far_cam_camera_info_time = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic,
					CameraInfo,
					self.far_cam_camera_info_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'CameraInfo'
			self.buffer_dict[topic] = self.far_cam_camera_info_buffer
			self.callback_dict[topic] = self.far_cam_camera_info_callback

		elif topic == '/near_cam/color/camera_info':
			self.unpack_functions.append(self.near_cam_camera_info_unpack)
			self.near_cam_camera_info_buffer = []
			self.near_cam_camera_info_available_index = len(self.data_available)-1
			self.near_cam_camera_info_has_new = False

			self.near_cam_camera_matrix = None
			self.near_cam_camera_info_time = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic,
					CameraInfo,
					self.near_cam_camera_info_callback,
					queue_size = self.subscriber_queue_size)

			self.message_type_dict[topic] = 'CameraInfo'
			self.buffer_dict[topic] = self.near_cam_camera_info_buffer
			self.callback_dict[topic] = self.near_cam_camera_info_callback

		elif topic == '/polygon_contact_estimate':
			self.unpack_functions.append(self.polygon_contact_estimate_unpack)
			self.polygon_contact_estimate_buffer = []
			self.polygon_contact_estimate_available_index = len(self.data_available)-1
			self.polygon_contact_estimate_has_new = False
			self.polygon_contact_estimate_time = None

			self.polygon_contact_estimate_dict = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic,
					PolygonContactStateStamped,
					self.polygon_contact_estimate_callback,
					queue_size = self.subscriber_queue_size)


			self.message_type_dict[topic] = 'PolygonContactStateStamped'
			self.buffer_dict[topic] = self.polygon_contact_estimate_buffer
			self.callback_dict[topic] = self.polygon_contact_estimate_callback

		elif topic == '/polygon_vision_estimate':
			self.unpack_functions.append(self.polygon_vision_estimate_unpack)
			self.polygon_vision_estimate_buffer = []
			self.polygon_vision_estimate_available_index = len(self.data_available)-1
			self.polygon_vision_estimate_has_new = False
			self.polygon_vision_estimate_time = None

			self.polygon_vision_estimate_dict = None

			if not self.load_mode:
				self.subscriber_dict[topic] = rospy.Subscriber(
					topic,
					PolygonContactStateStamped,
					self.polygon_vision_estimate_callback,
					queue_size = self.subscriber_queue_size)


			self.message_type_dict[topic] = 'PolygonContactStateStamped'
			self.buffer_dict[topic] = self.polygon_vision_estimate_buffer
			self.callback_dict[topic] = self.polygon_vision_estimate_callback

		else:
			self.unpack_functions.append(None)
			self.subscriber_dict[topic] = None
			self.message_type_dict[topic] = None
			self.buffer_dict[topic] = None
			self.callback_dict[topic] = None

		self.subscriber_topic_dict[topic]=None


	def spawn_publisher(self,topic):
		if topic in self.publisher_topic_dict:
			return None

		if topic == '/ee_pose_in_world_manipulation_from_franka_publisher':
			if self.load_mode:
				pass
			else:
				self.ee_pose_in_world_manipulation_from_franka_pub = rospy.Publisher(
					topic, 
					PoseStamped, 
					queue_size = 10)

		elif topic == '/ee_pose_in_base_from_franka_publisher':
			if self.load_mode:
				pass
			else:
				self.ee_pose_in_base_from_franka_pub = rospy.Publisher(
					topic, 
					PoseStamped, 
					queue_size = 10)

		elif topic == '/end_effector_sensor_in_end_effector_frame':
			if self.load_mode:
				pass
			else:
				#wrench at the end-effector in the end effector coordinates -Neel 7/5/2022
				#wrench measured by the ATI, but the torque has been transformed using a different reference point,
				#specifically, the origin of the end-effector frame (should be palm center), and using end-effector basis
				self.end_effector_sensor_in_end_effector_frame_pub = rospy.Publisher(
					topic, 
					WrenchStamped, 
					queue_size = 10)

		elif topic == '/end_effector_sensor_in_world_manipulation_frame':
			if self.load_mode:
				pass
			else:
				#wrench at the end-effector in the static world manipulation frame coordinates 
				#wrench measured by the ATI, but the torque has been transformed using a different reference point,
				#specifically, the origin of the end-effector frame (should be palm center), and using BASE FRAME basis
				self.end_effector_sensor_in_world_manipulation_frame_pub = rospy.Publisher(
					topic, 
					WrenchStamped, 
					queue_size = 10)

		elif topic == '/torque_cone_boundary_test':
			if self.load_mode:
				pass
			else:
				self.torque_cone_boundary_test_pub = rospy.Publisher(
					topic, 
					TorqueConeBoundaryTestStamped , 
					queue_size = 10)

		elif topic == '/torque_cone_boundary_flag':
			if self.load_mode:
				pass
			else:
				self.torque_cone_boundary_flag_pub = rospy.Publisher(
					topic, 
					TorqueConeBoundaryFlagStamped , 
					queue_size = 10)

		elif topic == '/friction_parameters':
			if self.load_mode:
				pass
			else:
				self.friction_parameter_pub = rospy.Publisher(
					topic, 
					FrictionParamsStamped,
					queue_size = 10)

		elif topic == '/sliding_state':
			if self.load_mode:
				pass
			else:
				self.sliding_state_pub = rospy.Publisher(
					topic, 
					SlidingStateStamped,
					queue_size = 10)

		elif topic == '/pivot_sliding_commanded_flag':
			if self.load_mode:
				pass
			else:
				# setting up publisher for sliding
				self.pivot_sliding_commanded_flag_pub = rospy.Publisher(
					topic, 
					PivotSlidingCommandedFlagStamped, 
					queue_size=10)

		elif topic == '/qp_debug_message':
			if self.load_mode:
				pass
			else:
				self.qp_debug_message_pub = rospy.Publisher(
					topic, 
					QPDebugStamped,
					queue_size=10)

		elif topic == '/barrier_func_control_command':
			if self.load_mode:
				pass
			else:
				self.barrier_func_control_command_pub = rospy.Publisher(
					topic,
					ControlCommandStamped,
					queue_size=10)

		elif topic == '/target_frame':
			if self.load_mode:
				pass
			else:
				# intialize impedance target frame
				self.target_frame_pub = rospy.Publisher(
					topic, 
					TransformStamped, 
					queue_size=10) 
				# set up transform broadcaster
				self.target_frame_broadcaster = tf2_ros.TransformBroadcaster()

		elif topic == '/ee_apriltag_in_world':
			if self.load_mode:
				pass
			else:
				self.ee_apriltag_in_world_frame_pub = rospy.Publisher(
					topic, 
					TransformStamped, 
					queue_size=10) 
				# set up transform broadcaster
				self.ee_apriltag_in_world_frame_broadcaster = tf2_ros.TransformBroadcaster()

		elif topic == '/pivot_frame_realsense':
			if self.load_mode:
				pass
			else:
				self.pivot_frame_realsense_pub = rospy.Publisher(
					topic, 
					TransformStamped, 
					queue_size=10) 
					# set up transform broadcaster
				self.pivot_frame_realsense_broadcaster = tf2_ros.TransformBroadcaster()

		elif topic == '/pivot_frame_estimated':
			if self.load_mode:
				pass
			else:
				self.pivot_frame_estimated_pub = rospy.Publisher(
					topic, 
					TransformStamped, 
					queue_size=10) 
					# set up transform broadcaster
				self.pivot_frame_estimated_broadcaster = tf2_ros.TransformBroadcaster()

		elif topic == '/polygon_contact_estimate':
			if self.load_mode:
				pass
			else:
				self.polygon_contact_estimate_pub = rospy.Publisher(
					topic,
					PolygonContactStateStamped,
					queue_size=10)

		elif topic == '/polygon_vision_estimate':
			if self.load_mode:
				pass
			else:
				self.polygon_vision_estimate_pub = rospy.Publisher(
					topic,
					PolygonContactStateStamped,
					queue_size=10)

		self.publisher_topic_dict[topic]=None


	def pub_ee_pose_in_world_manipulation_from_franka(self,ee_pose_in_world_manipulation_list):
		if '/ee_pose_in_world_manipulation_from_franka_publisher' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			ee_pose_in_world_manipulation_pose_stamped = pmh.list2pose_stamped(ee_pose_in_world_manipulation_list)
			ee_pose_in_world_manipulation_pose_stamped.header.stamp = rospy.Time.now()
			self.ee_pose_in_world_manipulation_from_franka_pub.publish(ee_pose_in_world_manipulation_pose_stamped)

	def pub_ee_pose_in_base_from_franka(self,ee_pose_in_base_list):
		if '/ee_pose_in_base_from_franka_publisher' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			ee_pose_in_base_pose_stamped = pmh.list2pose_stamped(ee_pose_in_base_list)
			ee_pose_in_base_pose_stamped.header.stamp = rospy.Time.now()
			self.ee_pose_in_base_from_franka_pub.publish(ee_pose_in_base_pose_stamped)

	def pub_end_effector_sensor_in_end_effector_frame(self,end_effector_wrench_in_end_effector_list,frame_id):
		if '/end_effector_sensor_in_end_effector_frame' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			msg = pmh.list2wrench_stamped(end_effector_wrench_in_end_effector_list,frame_id)
			msg.header.stamp = rospy.Time.now()
			self.end_effector_sensor_in_end_effector_frame_pub.publish(msg)

	def pub_end_effector_sensor_in_world_manipulation_frame(self,end_effector_wrench_in_world_manipulation_list,frame_id):
		if '/end_effector_sensor_in_world_manipulation_frame' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			msg = pmh.list2wrench_stamped(end_effector_wrench_in_world_manipulation_list,frame_id)
			msg.header.stamp = rospy.Time.now()
			self.end_effector_sensor_in_world_manipulation_frame_pub.publish(msg)

	def pub_torque_cone_boundary_test(self,torque_boundary_boolean):
		if '/torque_cone_boundary_test' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			torque_boundary_boolean_message = pmh.generate_torque_cone_boundary_test_stamped(torque_boundary_boolean)
			torque_boundary_boolean_message.header.stamp = rospy.Time.now()
			self.torque_cone_boundary_test_pub.publish(torque_boundary_boolean_message)

	def pub_torque_cone_boundary_flag(self,torque_boundary_flag):
		if '/torque_cone_boundary_flag' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			torque_boundary_flag_message = pmh.generate_torque_cone_boundary_flag_stamped(torque_boundary_flag)
			torque_boundary_flag_message.header.stamp = rospy.Time.now()
			self.torque_cone_boundary_flag_pub.publish(torque_boundary_flag_message)

	def pub_friction_parameter(self,friction_parameter_dict):
		if '/friction_parameters' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			friction_parameter_msg = pmh.friction_dict_to_friction_stamped(friction_parameter_dict)
			friction_parameter_msg.header.stamp = rospy.Time.now()
			self.friction_parameter_pub.publish(friction_parameter_msg)

	def pub_sliding_state(self,sliding_state_dict):
		if '/sliding_state' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			sliding_state_stamped = pmh.sliding_dict_to_sliding_stamped(sliding_dict=sliding_state_dict)
			sliding_state_stamped.header.stamp = rospy.Time.now()
			self.sliding_state_pub.publish(sliding_state_stamped)

	def pub_pivot_sliding_commanded_flag(self,pivot_sliding_commanded_flag):
		if '/pivot_sliding_commanded_flag' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			pivot_sliding_commanded_flag_message = pmh.generate_pivot_sliding_commanded_flag(pivot_sliding_commanded_flag)
			pivot_sliding_commanded_flag_message.header.stamp = rospy.Time.now()
			self.pivot_sliding_commanded_flag_pub.publish(pivot_sliding_commanded_flag_message)

	def pub_barrier_func_control_command(self,command_msg_dict):
		if '/barrier_func_control_command' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			command_msg = pmh.command_dict_to_command_stamped(command_msg_dict)
			command_msg.header.stamp = rospy.Time.now()
			self.barrier_func_control_command_pub.publish(command_msg)


	def pub_target_frame(self,waypoint_pose_list):
		if '/target_frame' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			waypoint_pose_stamped = pmh.list2transform_stamped(waypoint_pose_list, header_frame_id = 'base', child_frame_id = 'hand_estimate')
			waypoint_pose_stamped.header.stamp = rospy.Time.now()
			self.target_frame_pub.publish(waypoint_pose_stamped)
			self.target_frame_broadcaster.sendTransform(waypoint_pose_stamped)

	def pub_ee_apriltag_frame(self,ee_apriltag_in_world_pose_list):
		if '/ee_apriltag_in_world' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			ee_apriltag_in_world_pose_stamped = pmh.list2transform_stamped(ee_apriltag_in_world_pose_list, header_frame_id = 'base', child_frame_id = 'ee_apriltag_in_world')
			ee_apriltag_in_world_pose_stamped.header.stamp = rospy.Time.now()
			self.ee_apriltag_in_world_frame_pub.publish(ee_apriltag_in_world_pose_stamped)
			self.ee_apriltag_in_world_frame_broadcaster.sendTransform(ee_apriltag_in_world_pose_stamped)

	def pub_pivot_frame_realsense(self,pivot_pose_list):
		if '/pivot_frame_realsense' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			if len(pivot_pose_list)==3:
				pivot_pose_list = pivot_pose_list + [0.0,0.0,0.0,1.0]

			pivot_pose_stamped = pmh.list2transform_stamped(pivot_pose_list, header_frame_id = '/world_manipulation_frame', child_frame_id = 'pivot_frame_realsense')
			pivot_pose_stamped.header.stamp = rospy.Time.now()
			self.pivot_frame_realsense_pub.publish(pivot_pose_stamped)
			self.pivot_frame_realsense_broadcaster.sendTransform(pivot_pose_stamped)

	def pub_pivot_frame_estimated(self,pivot_pose_list):
		if '/pivot_frame_estimated' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			if len(pivot_pose_list)==3:
				pivot_pose_list = pivot_pose_list + [0.0,0.0,0.0,1.0]

			pivot_pose_stamped = pmh.list2transform_stamped(pivot_pose_list, header_frame_id = '/world_manipulation_frame', child_frame_id = 'pivot_frame_estimated')
			pivot_pose_stamped.header.stamp = rospy.Time.now()
			self.pivot_frame_estimated_pub.publish(pivot_pose_stamped)
			self.pivot_frame_estimated_broadcaster.sendTransform(pivot_pose_stamped)

	def pub_qp_debug_message(self,debug_dict):
		if '/qp_debug_message' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			qp_debug_stamped = pmh.qp_debug_dict_to_qp_debug_stamped(qp_debug_dict = debug_dict)
			qp_debug_stamped.header.stamp = rospy.Time.now()
			self.qp_debug_message_pub.publish(qp_debug_stamped)

	def pub_polygon_contact_estimate(self,vertex_array,contact_indices,mgl_cos_theta_list=None,mgl_sin_theta_list=None,
                                     hand_contact_indices = None, ground_contact_indices = None, wall_contact_indices =  None,
                                     wall_flag = None):
		if '/polygon_contact_estimate' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			polygon_contact_state_stamped = pmh.generate_polygon_contact_state_stamped(vertex_array,contact_indices,mgl_cos_theta_list,mgl_sin_theta_list,
																					   hand_contact_indices = hand_contact_indices, 
																					   ground_contact_indices = ground_contact_indices, 
																					   wall_contact_indices =  wall_contact_indices,
                                     												   wall_flag = wall_flag)
			polygon_contact_state_stamped.header.stamp = rospy.Time.now()
			self.polygon_contact_estimate_pub.publish(polygon_contact_state_stamped)

	def pub_polygon_vision_estimate(self,vertex_array,contact_indices=None,mgl_cos_theta_list=None,mgl_sin_theta_list=None):
		if contact_indices is None:
			contact_indices = []

		if '/polygon_vision_estimate' not in self.publisher_topic_dict:
			return None

		if self.load_mode:
			pass
		else:
			polygon_contact_state_stamped = pmh.generate_polygon_contact_state_stamped(vertex_array,contact_indices,mgl_cos_theta_list,mgl_sin_theta_list)
			polygon_contact_state_stamped.header.stamp = rospy.Time.now()
			self.polygon_vision_estimate_pub.publish(polygon_contact_state_stamped)

	def force_callback(self,data,message_time=None):
		self.ft_wrench_in_ft_sensor_buffer.append(data)
		if len(self.ft_wrench_in_ft_sensor_buffer)>self.max_queue_size:
			self.ft_wrench_in_ft_sensor_buffer.pop(0)
		self.data_available[self.ft_wrench_in_ft_sensor_available_index]=True

		if message_time is not None:
			self.ft_wrench_time = message_time

	def force_unpack(self):
		if len(self.ft_wrench_in_ft_sensor_buffer)>0:
			if self.load_mode:
				self.ft_wrench_in_ft_sensor_list = self.ft_wrench_in_ft_sensor_buffer.pop(0)
			else:
				self.ft_wrench_in_ft_sensor = self.ft_wrench_in_ft_sensor_buffer.pop(0)
				self.ft_wrench_time = self.ft_wrench_in_ft_sensor.header.stamp.to_sec()
				self.ft_wrench_in_ft_sensor_list = pmh.wrench_stamped2list(self.ft_wrench_in_ft_sensor)

			self.force_has_new = True
		else:
			self.force_has_new = False

	def ee_pose_in_world_manipulation_callback(self,data,message_time=None):
		self.ee_pose_in_world_manipulation_buffer.append(data)
		if len(self.ee_pose_in_world_manipulation_buffer)>self.max_queue_size:
			self.ee_pose_in_world_manipulation_buffer.pop(0)
		self.data_available[self.ee_pose_in_world_manipulation_available_index]=True

		if message_time is not None:
			self.ee_pose_in_world_manipulation_time = message_time
		
	def ee_pose_in_world_manipulation_unpack(self):
		if len(self.ee_pose_in_world_manipulation_buffer)>0:
			if self.load_mode:
				self.ee_pose_in_world_manipulation_list = self.ee_pose_in_world_manipulation_buffer.pop(0)
			else:
				self.ee_pose_in_world_manipulation = self.ee_pose_in_world_manipulation_buffer.pop(0)
				self.ee_pose_in_world_manipulation_time = self.ee_pose_in_world_manipulation.header.stamp.to_sec()
				self.ee_pose_in_world_manipulation_list = pmh.pose_stamped2list(self.ee_pose_in_world_manipulation)
			self.ee_pose_in_world_manipulation_homog = kh.matrix_from_pose_list(self.ee_pose_in_world_manipulation_list)

			self.ee_pose_in_world_manipulation_has_new = True
		else:
			self.ee_pose_in_world_manipulation_has_new = False

	def ee_pose_in_base_callback(self,data,message_time=None):
		self.ee_pose_in_base_buffer.append(data)
		if len(self.ee_pose_in_base_buffer)>self.max_queue_size:
			self.ee_pose_in_base_buffer.pop(0)
		self.data_available[self.ee_pose_in_base_available_index]=True

		if message_time is not None:
			self.ee_pose_in_base_time = message_time
		
	def ee_pose_in_base_unpack(self):
		if len(self.ee_pose_in_base_buffer)>0:
			if self.load_mode:
				self.ee_pose_in_base_list = self.ee_pose_in_base_buffer.pop(0)
			else:
				self.ee_pose_in_base = self.ee_pose_in_base_buffer.pop(0)
				self.ee_pose_in_base_time = self.ee_pose_in_base.header.stamp.to_sec()
				self.ee_pose_in_base_list = pmh.pose_stamped2list(self.ee_pose_in_base)
			self.ee_pose_in_base_homog = kh.matrix_from_pose_list(self.ee_pose_in_base_list)

			self.base_z_in_ee_frame = self.ee_pose_in_base_homog[2, :3]

			self.ee_pose_in_base_has_new = True
		else:
			self.ee_pose_in_base_has_new = False

	def end_effector_wrench_callback(self,data,message_time=None):
		self.measured_contact_wrench_buffer.append(data)
		if len(self.measured_contact_wrench_buffer)>self.max_queue_size:
			self.measured_contact_wrench_buffer.pop(0)
		self.data_available[self.measured_contact_wrench_available_index]=True

		if message_time is not None:
			self.measured_contact_wrench_time = message_time

	def end_effector_wrench_unpack(self):
		if  len(self.measured_contact_wrench_buffer)>0:
			if self.load_mode:
				self.measured_contact_wrench_6D = -np.array(self.measured_contact_wrench_buffer.pop(0))
			else:
				end_effector_wrench = self.measured_contact_wrench_buffer.pop(0)

				self.measured_contact_wrench_time = end_effector_wrench.header.stamp.to_sec()

				self.measured_contact_wrench_6D = -np.array(pmh.wrench_stamped2list(
					end_effector_wrench))

			self.measured_contact_wrench = np.array([
				self.measured_contact_wrench_6D[0], 
				self.measured_contact_wrench_6D[1],
				self.measured_contact_wrench_6D[-1]])

			self.end_effector_wrench_has_new = True
		else:
			self.end_effector_wrench_has_new = False

	def end_effector_wrench_world_manipulation_frame_callback(self,data,message_time=None):
		self.measured_world_manipulation_wrench_buffer.append(data)
		if len(self.measured_world_manipulation_wrench_buffer)>self.max_queue_size:
			self.measured_world_manipulation_wrench_buffer.pop(0)
		self.data_available[self.measured_world_manipulation_wrench_available_index]=True

		if message_time is not None:
			self.world_manipulation_wrench_time = message_time

	def end_effector_wrench_world_manipulation_frame_unpack(self):
		if len(self.measured_world_manipulation_wrench_buffer)>0:
			if self.load_mode:
				self.measured_world_manipulation_wrench_6D = -np.array(self.measured_world_manipulation_wrench_buffer.pop(0))
			else:
				world_manipulation_wrench = self.measured_world_manipulation_wrench_buffer.pop(0)

				self.world_manipulation_wrench_time = world_manipulation_wrench.header.stamp.to_sec()

				self.measured_world_manipulation_wrench_6D = -np.array(pmh.wrench_stamped2list(
					world_manipulation_wrench))

			self.measured_world_manipulation_wrench = np.array([
				self.measured_world_manipulation_wrench_6D[0], 
				self.measured_world_manipulation_wrench_6D[1],
				self.measured_world_manipulation_wrench_6D[-1]])

			self.end_effector_wrench_world_manipulation_frame_has_new = True
		else:
			self.end_effector_wrench_world_manipulation_frame_has_new = False

	def torque_cone_boundary_test_callback(self,data,message_time=None):
		self.torque_cone_boundary_test_buffer.append(data)
		if len(self.torque_cone_boundary_test_buffer)>self.max_queue_size:
			self.torque_cone_boundary_test_buffer.pop(0)
		self.data_available[self.torque_cone_boundary_test_available_index]=True

		if message_time is not None:
			self.torque_cone_boundary_test_time = message_time

	def torque_cone_boundary_test_unpack(self):
		if len(self.torque_cone_boundary_test_buffer)>0:
			if self.load_mode:
				self.torque_cone_boundary_test = self.torque_cone_boundary_test_buffer.pop(0)
			else:
				data = self.torque_cone_boundary_test_buffer.pop(0)
				self.torque_cone_boundary_test_time = data.header.stamp.to_sec()
				self.torque_cone_boundary_test = pmh.parse_torque_cone_boundary_test_stamped(data)

			self.torque_cone_boundary_test_has_new = True
		else:
			self.torque_cone_boundary_test_has_new = False

	def torque_cone_boundary_flag_callback(self,data,message_time=None):
		self.torque_cone_boundary_flag_buffer.append(data)
		if len(self.torque_cone_boundary_flag_buffer)>self.max_queue_size:
			self.torque_cone_boundary_flag_buffer.pop(0)
		self.data_available[self.torque_cone_boundary_flag_available_index]=True

		if message_time is not None:
			self.torque_cone_boundary_flag_time = message_time

	def torque_cone_boundary_flag_unpack(self):
		if len(self.torque_cone_boundary_flag_buffer)>0:
			if self.load_mode:
				self.torque_cone_boundary_flag = self.torque_cone_boundary_flag_buffer.pop(0)
			else:
				data = self.torque_cone_boundary_flag_buffer.pop(0)
				self.torque_cone_boundary_flag_time = data.header.stamp.to_sec()
				self.torque_cone_boundary_flag = pmh.parse_torque_cone_boundary_flag_stamped(data)

			self.torque_cone_boundary_flag_has_new = True
		else:
			self.torque_cone_boundary_flag_has_new = False

	def friction_parameter_callback(self,data,message_time=None):
		self.friction_parameter_buffer.append(data)
		if len(self.friction_parameter_buffer)>self.max_queue_size:
			self.friction_parameter_buffer.pop(0)
		self.data_available[self.friction_parameter_available_index]=True

		if message_time is not None:
			self.friction_parameter_time = message_time

	def friction_parameter_unpack(self):
		if len(self.friction_parameter_buffer)>0:
			if self.load_mode:
				self.friction_parameter_dict = self.friction_parameter_buffer.pop(0)
			else:
				data = self.friction_parameter_buffer.pop(0)
				self.friction_parameter_time = data.header.stamp.to_sec()
				self.friction_parameter_dict = pmh.friction_stamped_to_friction_dict(data)
			friction_reasoning.convert_friction_param_dict_to_array(self.friction_parameter_dict)

			self.friction_parameter_has_new = True
		else:
			self.friction_parameter_has_new = False

	def pivot_xyz_realsense_callback(self,data,message_time=None):
		self.pivot_xyz_realsense_buffer.append(data)
		if len(self.pivot_xyz_realsense_buffer)>self.max_queue_size:
			self.pivot_xyz_realsense_buffer.pop(0)
		self.data_available[self.pivot_xyz_realsense_available_index]=True

		if message_time is not None:
			self.pivot_message_realsense_time = message_time

	def pivot_xyz_realsense_unpack(self):
		if len(self.pivot_xyz_realsense_buffer)>0:
			if self.load_mode:
				self.pivot_xyz_realsense = self.pivot_xyz_realsense_buffer.pop(0)[0:3]
			else:
				data = self.pivot_xyz_realsense_buffer.pop(0)
				
				self.pivot_message_realsense_time = data.header.stamp.to_sec()
				self.pivot_xyz_realsense =  [data.transform.translation.x,
					data.transform.translation.y,
					data.transform.translation.z]

				self.pivot_xyz = self.pivot_xyz_realsense

			self.pivot_xyz_realsense_has_new = True
		else:
			self.pivot_xyz_realsense_has_new = False

	def pivot_xyz_estimated_callback(self,data,message_time=None):
		self.pivot_xyz_estimated_buffer.append(data)
		if len(self.pivot_xyz_estimated_buffer)>self.max_queue_size:
			self.pivot_xyz_estimated_buffer.pop(0)
		self.data_available[self.pivot_xyz_estimated_available_index]=True

		if message_time is not None:
			self.pivot_message_estimated_time = message_time

	def pivot_xyz_estimated_unpack(self):
		if len(self.pivot_xyz_estimated_buffer)>0:
			if self.load_mode:
				self.pivot_xyz_estimated = self.pivot_xyz_estimated_buffer.pop(0)[0:3]
			else:
				data = self.pivot_xyz_estimated_buffer.pop(0)
				
				self.pivot_message_estimated_time = data.header.stamp.to_sec()
				self.pivot_xyz_estimated =  [data.transform.translation.x,
					data.transform.translation.y,
					data.transform.translation.z]

				#if there has been no message from the realsense
				if (self.pivot_xyz_realsense is None) or (self.pivot_message_estimated_time-self.pivot_message_realsense_time>.5):
					self.pivot_xyz = self.pivot_xyz_estimated

			self.pivot_xyz_estimated_has_new = True
		else:
			self.pivot_xyz_estimated_has_new = False

	def generalized_positions_callback(self,data,message_time=None):
		self.generalized_positions_buffer.append(data)
		if len(self.generalized_positions_buffer)>self.max_queue_size:
			self.generalized_positions_buffer.pop(0)
		self.data_available[self.generalized_positions_available_index]=True

		if message_time is not None:
			self.generalized_positions_time = message_time

	def generalized_positions_unpack(self):
		if len(self.generalized_positions_buffer)>0:
			if self.load_mode:
				self.generalized_positions = self.generalized_positions_buffer.pop(0)
			else:
				data = self.generalized_positions_buffer.pop(0)
				self.generalized_positions_time = data.header.stamp.to_sec()
				self.generalized_positions = pmh.parse_generalized_positions_stamped(data)

			self.l_hand     = contact_pose[0]
			self.s_hand     = contact_pose[1]
			self.theta_hand = contact_pose[2]

			self.state_not_exists_bool = False

			self.generalized_positions_has_new = True
		else:
			self.generalized_positions_has_new = False

	def barrier_func_control_command_callback(self,data,message_time=None):
		self.barrier_func_control_command_buffer.append(data)
		if len(self.barrier_func_control_command_buffer)>self.max_queue_size:
			self.barrier_func_control_command_buffer.pop(0)
		self.data_available[self.barrier_func_control_command_available_index]=True

		if message_time is not None:
			self.barrier_func_control_command_time = message_time

	def barrier_func_control_command_unpack(self):
		if len(self.barrier_func_control_command_buffer)>0:
			if self.load_mode:
				self.command_msg = self.barrier_func_control_command_buffer.pop(0)
			else:
				data = self.barrier_func_control_command_buffer.pop(0)
				self.barrier_func_control_command_time = data.header.stamp.to_sec()
				self.command_msg = pmh.command_stamped_to_command_dict(data)

			self.barrier_func_control_command_has_new = True
		else:
			self.barrier_func_control_command_has_new = False

	def torque_bound_callback(self,data,message_time=None):
		self.torque_bound_buffer.append(data)
		if len(self.torque_bound_buffer)>self.max_queue_size:
			self.torque_bound_buffer.pop(0)
		self.data_available[self.torque_bound_available_index]=True

		if message_time is not None:
			self.torque_bounds_time = message_time

	def torque_bound_unpack(self):
		if len(self.torque_bound_buffer)>0:
			if self.load_mode:
				self.torque_bounds = self.torque_bound_buffer.pop(0)
			else:
				data = self.torque_bound_buffer.pop(0)
				self.torque_bounds_time = data.header.stamp.to_sec()
				self.torque_bounds = pmh.parse_torque_bounds_stamped(data)

			self.torque_bound_has_new = True
		else:
			self.torque_bound_has_new = False

	def sliding_state_callback(self,data,message_time=None):
		self.sliding_state_buffer.append(data)
		if len(self.sliding_state_buffer)>self.max_queue_size:
			self.sliding_state_buffer.pop(0)
		self.data_available[self.sliding_state_available_index]=True

		if message_time is not None:
			self.sliding_state_time = message_time

	def sliding_state_unpack(self):
		if len(self.sliding_state_buffer)>0:
			if self.load_mode:
				self.sliding_state = self.sliding_state_buffer.pop(0)
			else:
				data = self.sliding_state_buffer.pop(0)
				self.sliding_state_time = data.header.stamp.to_sec()
				self.sliding_state = pmh.sliding_stamped_to_sliding_dict(data)

			self.sliding_state_has_new = True
		else:
			self.sliding_state_has_new = False

	def pivot_sliding_commanded_flag_callback(self,data,message_time=None):
		self.pivot_sliding_commanded_flag_buffer.append(data)
		if len(self.pivot_sliding_commanded_flag_buffer)>self.max_queue_size:
			self.pivot_sliding_commanded_flag_buffer.pop(0)
		self.data_available[self.pivot_sliding_commanded_flag_available_index]=True

		if message_time is not None:
			self.pivot_sliding_commanded_time = message_time

	def pivot_sliding_commanded_flag_unpack(self):
		if len(self.pivot_sliding_commanded_flag_buffer)>0:
			if self.load_mode:
				self.pivot_sliding_commanded_flag = self.pivot_sliding_commanded_flag_buffer.pop(0)
			else:
				data = self.pivot_sliding_commanded_flag_buffer.pop(0)
				self.pivot_sliding_commanded_time = data.header.stamp.to_sec()
				self.pivot_sliding_commanded_flag = pmh.parse_pivot_sliding_commanded_flag(data)

			self.pivot_sliding_commanded_flag_has_new = True
		else:
			self.pivot_sliding_commanded_flag_has_new = False

	def qp_debug_callback(self,data,message_time=None):
		self.qp_debug_buffer.append(data)
		if len(self.qp_debug_buffer)>self.max_queue_size:
			self.qp_debug_buffer.pop(0)
		self.data_available[self.qp_debug_available_index]=True

		if message_time is not None:
			self.qp_debug_time = message_time

	def qp_debug_unpack(self):
		if len(self.qp_debug_buffer)>0:
			if self.load_mode:
				self.qp_debug_dict = self.qp_debug_buffer.pop(0)
				self.qp_debug_has_new = True
			else:
				data = self.qp_debug_buffer.pop(0)
				self.qp_debug_time = data.header.stamp.to_sec()

				if data.qp_debug != '':
					self.qp_debug_dict = pmh.qp_debug_stamped_to_qp_debug_dict(data)
					self.qp_debug_has_new = True
				else:
					self.qp_debug_has_new = False
		else:
			self.qp_debug_has_new = False

	def target_frame_callback(self,data,message_time=None):
		self.target_frame_buffer.append(data)
		if len(self.target_frame_buffer)>self.max_queue_size:
			self.target_frame_buffer.pop(0)
		self.data_available[self.target_frame_available_index]=True

		if message_time is not None:
			self.target_frame_time = message_time

	def target_frame_unpack(self):
		if len(self.target_frame_buffer)>0:
			if self.load_mode:
				self.target_frame = self.target_frame_buffer.pop(0)
			else:
				data = self.target_frame_buffer.pop(0)
				self.target_frame_time = data.header.stamp.to_sec()
				self.target_frame = pmh.transform_stamped2list(data)

			self.target_frame_homog =  kh.matrix_from_pose_list(self.target_frame)

			self.target_frame_has_new = True
		else:
			self.target_frame_has_new = False

	def apriltag_callback(self,data,message_time=None):
		self.apriltag_buffer.append(data)
		if len(self.apriltag_buffer)>self.max_queue_size:
			self.apriltag_buffer.pop(0)
		self.data_available[self.apriltag_available_index]=True

		if message_time is not None:
			self.apriltag_time = message_time

	def apriltag_unpack(self):
		if len(self.apriltag_buffer)>0:

			self.apriltag_pose_list_dict = {}
			self.apriltag_pose_homog_dict = {}

			if self.load_mode:
				detection_dict = self.apriltag_buffer.pop(0)
				if detection_dict is not None:
					for detection in detection_dict:
						pose_list = detection_dict[detection]['position']+detection_dict[detection]['orientation']
						self.apriltag_pose_list_dict[detection] = pose_list
						self.apriltag_pose_homog_dict[detection] = kh.matrix_from_pose_list(pose_list)
			else:
				apriltag_array = self.apriltag_buffer.pop(0)
				self.apriltag_time = apriltag_array.header.stamp.to_sec()

				for detection in apriltag_array.detections:
					pose_list = pmh.pose_stamped2list(detection.pose.pose)
					self.apriltag_pose_list_dict[detection.id[0]] = pose_list
					self.apriltag_pose_homog_dict[detection.id[0]] = kh.matrix_from_pose_list(pose_list)

			self.apriltag_has_new = True
		else:
			self.apriltag_has_new = False

	def far_cam_image_raw_callback(self,data,message_time=None):
		self.far_cam_image_raw_buffer.append(data)
		if len(self.far_cam_image_raw_buffer)>self.max_queue_size:
			self.far_cam_image_raw_buffer.pop(0)
		self.data_available[self.far_cam_image_raw_available_index]=True

		if message_time is not None:
			self.far_cam_image_time = message_time

	def far_cam_image_raw_unpack(self):
		if len(self.far_cam_image_raw_buffer)>0:
			if self.load_mode:
				self.far_cam_image_raw = self.far_cam_image_raw_buffer.pop(0)
			else:
				data = self.far_cam_image_raw_buffer.pop(0)
				self.far_cam_image_time = data.header.stamp.to_sec()
				self.far_cam_image_raw = self.bridge.imgmsg_to_cv2(data, 'bgr8')

			self.far_cam_image_raw_has_new = True
		else:
			self.far_cam_image_raw_has_new = False

	def near_cam_image_raw_callback(self,data,message_time=None):
		self.near_cam_image_raw_buffer.append(data)
		if len(self.near_cam_image_raw_buffer)>self.max_queue_size:
			self.near_cam_image_raw_buffer.pop(0)
		self.data_available[self.near_cam_image_raw_available_index]=True

		if message_time is not None:
			self.near_cam_image_time = message_time

	def near_cam_image_raw_unpack(self):
		if len(self.near_cam_image_raw_buffer)>0:
			if self.load_mode:
				self.near_cam_image_raw = self.near_cam_image_raw_buffer.pop(0)
			else:
				data = self.near_cam_image_raw_buffer.pop(0)
				self.near_cam_image_time = data.header.stamp.to_sec()
				self.near_cam_image_raw = self.bridge.imgmsg_to_cv2(data, 'bgr8')

			self.near_cam_image_raw_has_new = True
		else:
			self.near_cam_image_raw_has_new = False

	def far_cam_image_raw_callback_record_version(self,data,message_time=None):
		cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

		if self.far_came_video_writer is None:
			image_height, image_width, image_layers = cv_image.shape
			image_size = (image_width, image_height)
			save_name = os.path.join(self.path,self.fname)+'_far_cam_color_image_raw.avi'
			self.far_came_video_writer = cv2.VideoWriter(save_name, cv2.VideoWriter_fourcc(*'DIVX'), 30, image_size)

		self.far_came_video_writer.write(cv_image)
		self.far_cam_image_raw_buffer.append(data.header.stamp.to_sec())

		if message_time is not None:
			self.far_cam_image_time = message_time

	def near_cam_image_raw_callback_record_version(self,data,message_time=None):
		cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

		if self.near_came_video_writer is None:
			image_height, image_width, image_layers = cv_image.shape
			image_size = (image_width, image_height)
			save_name = os.path.join(self.path,self.fname)+'_near_cam_color_image_raw.avi'
			self.near_came_video_writer = cv2.VideoWriter(save_name, cv2.VideoWriter_fourcc(*'DIVX'), 30, image_size)

		self.near_came_video_writer.write(cv_image)
		self.near_cam_image_raw_buffer.append(data.header.stamp.to_sec())

		if message_time is not None:
			self.near_cam_image_time = message_time

	def far_cam_camera_info_callback(self,data,message_time=None):
		self.far_cam_camera_info_buffer.append(data)
		if len(self.far_cam_camera_info_buffer)>self.max_queue_size:
			self.far_cam_camera_info_buffer.pop(0)
		self.data_available[self.far_cam_camera_info_available_index]=True

		if message_time is not None:
			self.far_cam_camera_info_time = message_time

	def far_cam_camera_info_unpack(self):
		if len(self.far_cam_camera_info_buffer)>0:
			if self.load_mode:
				self.far_cam_camera_matrix = np.array(self.far_cam_camera_info_buffer.pop(0))
			else:
				data = self.far_cam_camera_info_buffer.pop(0)
				self.far_cam_camera_info_time = data.header.stamp.to_sec()
				self.far_cam_camera_matrix = np.reshape(data.P, (3, 4))

			self.far_cam_camera_info_has_new = True
		else:
			self.far_cam_camera_info_has_new = False

	def near_cam_camera_info_callback(self,data,message_time=None):
		self.near_cam_camera_info_buffer.append(data)
		if len(self.near_cam_camera_info_buffer)>self.max_queue_size:
			self.near_cam_camera_info_buffer.pop(0)
		self.data_available[self.near_cam_camera_info_available_index]=True

		if message_time is not None:
			self.near_cam_camera_info_time = message_time

	def near_cam_camera_info_unpack(self):
		if len(self.near_cam_camera_info_buffer)>0:
			if self.load_mode:
				self.near_cam_camera_matrix = np.array(self.near_cam_camera_info_buffer.pop(0))
			else:
				data = self.near_cam_camera_info_buffer.pop(0)
				self.near_cam_camera_info_time = data.header.stamp.to_sec()
				self.near_cam_camera_matrix = np.reshape(data.P, (3, 4))

			self.near_cam_camera_info_has_new = True
		else:
			self.near_cam_camera_info_has_new = False

	def polygon_contact_estimate_callback(self,data,message_time=None):
		self.polygon_contact_estimate_buffer.append(data)
		if len(self.polygon_contact_estimate_buffer)>self.max_queue_size:
			self.polygon_contact_estimate_buffer.pop(0)
		self.data_available[self.polygon_contact_estimate_available_index]=True

		if message_time is not None:
			self.polygon_contact_estimate_time = message_time

	def polygon_contact_estimate_unpack(self):
		if len(self.polygon_contact_estimate_buffer)>0:
			if self.load_mode:
				self.polygon_contact_estimate_dict = self.polygon_contact_estimate_buffer.pop(0)
			else:
				data = self.polygon_contact_estimate_buffer.pop(0)
				self.polygon_contact_estimate_time = data.header.stamp.to_sec()
				self.polygon_contact_estimate_dict = pmh.parse_polygon_contact_state_stamped(data)

			self.polygon_contact_estimate_has_new = True
		else:
			self.polygon_contact_estimate_has_new = False

	def polygon_vision_estimate_callback(self,data,message_time=None):
		self.polygon_vision_estimate_buffer.append(data)
		if len(self.polygon_vision_estimate_buffer)>self.max_queue_size:
			self.polygon_vision_estimate_buffer.pop(0)
		self.data_available[self.polygon_vision_estimate_available_index]=True

		if message_time is not None:
			self.polygon_vision_estimate_time = message_time

	def polygon_vision_estimate_unpack(self):
		if len(self.polygon_vision_estimate_buffer)>0:
			if self.load_mode:
				self.polygon_vision_estimate_dict = self.polygon_vision_estimate_buffer.pop(0)
			else:
				data = self.polygon_vision_estimate_buffer.pop(0)
				self.polygon_vision_estimate_time = data.header.stamp.to_sec()
				self.polygon_vision_estimate_dict = pmh.parse_polygon_contact_state_stamped(data)

			self.polygon_vision_estimate_has_new = True
		else:
			self.polygon_vision_estimate_has_new = False
