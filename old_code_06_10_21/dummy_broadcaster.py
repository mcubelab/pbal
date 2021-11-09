#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool


if __name__ == '__main__':

    rospy.init_node("dummy_broadcaster")
    rospy.sleep(0.5)
    rate = rospy.Rate(100)

    # setting up publisher
    pivot_sliding_flag_pub = rospy.Publisher('/pivot_sliding_flag', Bool, 
        queue_size=10)
    pivot_sliding_flag_msg = Bool()


    print("publishing pivot sliding flag")
    while not rospy.is_shutdown():

        pivot_sliding_flag_msg.data = False
        pivot_sliding_flag_pub.publish(pivot_sliding_flag_msg)
        rate.sleep()    
