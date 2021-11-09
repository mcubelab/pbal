#!/usr/bin/env python
import rospy
import pdb
import numpy as np
import json

from std_msgs.msg import Float32, String

def gravity_torque_callback(data):
    global mgl
    mgl = data.data

if __name__ == '__main__':

    rospy.init_node("barrier_func_commands")
    rate = rospy.Rate(0.05) # in yaml
    rospy.sleep(1.0)

    command_msg = String()

    command_msg_dict = {
        "theta" : np.pi/15,
        "command_flag" : 0,
        "mode" : -1,
        "x_pivot" : 0.0,
        "s" : 0.0,
    }    

    control_command_pub = rospy.Publisher(
        '/barrier_func_control_command', 
        String,
        queue_size=10)

    gravity_torque_sub = rospy.Subscriber("/gravity_torque", 
        Float32, gravity_torque_callback)

    mgl = None
    print("starting theta dither")
    while (mgl is None) and (not rospy.is_shutdown()):
    #while True:    
            command_msg.data = json.dumps(command_msg_dict)            
            control_command_pub.publish(command_msg)
            command_msg_dict["theta"] *= -1
            rate.sleep()

    command_msg_dict["theta"] = 0
    command_msg.data = json.dumps(command_msg_dict)            
    control_command_pub.publish(command_msg)  
    print("theta dither complete")

        


    # terminate rosbags
    # ros_helper.terminate_rosbag()

