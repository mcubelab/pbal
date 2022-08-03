class ros_manager(object):
	def __init__(self):


	# '/ft_sensor_in_base_frame'
	# '/ft_sensor_in_end_effector_frame'
	# '/end_effector_sensor_in_end_effector_frame'
	# '/end_effector_sensor_in_base_frame'
	# '/torque_cone_boundary_test'
	# '/torque_cone_boundary_flag'
	# '/pivot_frame_realsense'
	# '/pivot_frame_estimated'
	# '/generalized_positions'
	# '/end_effector_sensor_in_end_effector_frame'
	# '/barrier_func_control_command'
	# '/friction_parameters'
	# '/torque_bound_message'
	# '/ee_pose_in_world_from_franka_publisher'
	# '/pivot_sliding_commanded_flag'
	# '/qp_debug_message'
	# '/target_frame'


	def publisher_stuff():
	    ft_sensor_in_base_frame_pub = rospy.Publisher('/ft_sensor_in_base_frame', 
	        WrenchStamped, queue_size = 10)
	    ft_sensor_in_end_effector_frame_pub = rospy.Publisher('/ft_sensor_in_end_effector_frame', 
	        WrenchStamped, queue_size = 10)
	    end_effector_sensor_in_end_effector_frame_pub = rospy.Publisher(
	        '/end_effector_sensor_in_end_effector_frame', WrenchStamped, queue_size = 10)
	    end_effector_sensor_in_base_frame_pub = rospy.Publisher(
	        '/end_effector_sensor_in_base_frame', WrenchStamped, queue_size = 10)
	    torque_cone_boundary_test_pub = rospy.Publisher(
	        '/torque_cone_boundary_test', Bool , queue_size = 10)
	    torque_cone_boundary_flag_pub = rospy.Publisher(
	        '/torque_cone_boundary_flag', Int32 , queue_size = 10)

        pivot_xyz_realsense_sub = rospy.Subscriber("/pivot_frame_realsense", 
        	TransformStamped, pivot_xyz_realsense_callback)

	    pivot_xyz_estimated_sub = rospy.Subscriber("/pivot_frame_estimated", 
	        TransformStamped, pivot_xyz_estimated_callback)

	    generalized_positions_sub = rospy.Subscriber("/generalized_positions", 
	        Float32MultiArray,  generalized_positions_callback)

	    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame", 
	        WrenchStamped,  end_effector_wrench_callback)


	    control_command_sub = rospy.Subscriber('/barrier_func_control_command', 
	        ControlCommandStamped, barrier_func_control_command_callback)

        friction_parameter_sub = rospy.Subscriber('/friction_parameters', 
        	FrictionParamsStamped, friction_parameter_callback)


        torque_bound_sub = rospy.Subscriber("/torque_bound_message", 
        	Float32MultiArray,  torque_bound_callback)


        panda_hand_in_base_pose_sub = rospy.Subscriber(
        	'/ee_pose_in_world_from_franka_publisher', PoseStamped, 
        	ee_pose_callback, queue_size=1)

	    # setting up publisher for sliding
	    pivot_sliding_commanded_flag_pub = rospy.Publisher(
	        '/pivot_sliding_commanded_flag', Bool, queue_size=10)
    	pivot_sliding_commanded_flag_msg = Bool()


	    qp_debug_message_pub = rospy.Publisher('/qp_debug_message', 
	        QPDebugStamped, queue_size=10)


	    # intialize impedance target frame
	    frame_message = initialize_frame()
	    target_frame_pub = rospy.Publisher('/target_frame', 
	        TransformStamped, queue_size=10) 

        control_command_pub = rospy.Publisher('/barrier_func_control_command', 
        	ControlCommandStamped, queue_size=10)



	def subscriber_stuff():

	def force_callback(self,data):
	    global ft_wrench_in_ft_sensor
	    ft_wrench_in_ft_sensor = data

	def ee_pose_callback(self,data):
	    global panda_hand_in_base_pose, base_z_in_panda_hand

	    panda_hand_in_base_pose = data
	    base_z_in_panda_hand = rh.matrix_from_pose(
	        panda_hand_in_base_pose)[2, :3]

    def pivot_xyz_realsense_callback(data):
	    global pivot_xyz
	    global pivot_xyz_realsense
	    global pivot_xyz_estimated
	    global pivot_message_realsense_time
	    global pivot_message_estimated_time

	    pivot_message_realsense_time = time.time()
	    pivot_xyz_realsense =  [data.transform.translation.x,
	        data.transform.translation.y,
	        data.transform.translation.z]

	    pivot_xyz = pivot_xyz_realsense

	def pivot_xyz_estimated_callback(data):
	    global pivot_xyz
	    global pivot_xyz_realsense
	    global pivot_xyz_estimated
	    global pivot_message_realsense_time
	    global pivot_message_estimated_time

	    pivot_message_estimated_time = time.time()
	    pivot_xyz_estimated =  [data.transform.translation.x,
	        data.transform.translation.y,
	        data.transform.translation.z]
	    

	    #if there has been no message from the realsense

	    if (pivot_xyz_realsense is None) or (time.time()-pivot_message_realsense_time>1.0):
	        pivot_xyz = pivot_xyz_estimated
        

	def generalized_positions_callback(data):
	    global generalized_positions
	    generalized_positions = data.data

	def end_effector_wrench_callback(data):
	    global end_effector_wrench
	    end_effector_wrench = data 

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

		        obj_pose_homog = rh.matrix_from_pose(obj_apriltag_list[0].pose.pose)

		    else:
		        object_detected = False

    def end_effector_wrench_callback(data):
	    global measured_contact_wrench_list
	    end_effector_wrench = data
	    measured_contact_wrench_6D = rh.wrench_stamped2list(
	            end_effector_wrench)
	    measured_contact_wrench = -np.array([
	            measured_contact_wrench_6D[0], 
	            measured_contact_wrench_6D[1],
	            measured_contact_wrench_6D[-1]])

	    measured_contact_wrench_list.append(measured_contact_wrench)
	    if len(measured_contact_wrench_list) > 100:
	       measured_contact_wrench_list.pop(0)

	def end_effector_wrench_base_frame_callback(data):
	    global measured_base_wrench_list
	    base_wrench = data
	    measured_base_wrench_6D = rh.wrench_stamped2list(
	            base_wrench)
	    measured_base_wrench = -np.array([
	            measured_base_wrench_6D[0], 
	            measured_base_wrench_6D[2],
	            measured_base_wrench_6D[-2]])

	    measured_base_wrench_list.append(measured_base_wrench)
	    if len(measured_base_wrench_list) > 100:
	       measured_base_wrench_list.pop(0)

	def friction_parameter_callback(data):
	    global friction_parameter_list
	    friction_dict = pmh.friction_stamped_to_friction_dict(
	        data)
	    # friction_dict = json.loads(data.data)
	    friction_parameter_list.append(friction_dict)
	    if len(friction_parameter_list) > 3:
	       friction_parameter_list.pop(0)

	def torque_bound_callback(data):
	    global torque_bound_list
	    torque_bound_list.append(json.loads(data.data).tolist())
	    if len(torque_bound_list) > 3:
	        torque_bound_list.pop(0)

	def barrier_func_control_command_callback(data):
	    global command_msg_queue
	    # print("hello")
	    command_msg_dict = pmh.command_stamped_to_command_dict(
	        data)
	    command_msg_queue.append(command_msg_dict)
	    # command_msg_queue.append(json.loads(data.data))
	    if len(command_msg_queue) > 10:
	        command_msg_queue.pop(0)

	def ee_pose_callback(data):
	    global panda_hand_in_base_pose
	    panda_hand_in_base_pose = data