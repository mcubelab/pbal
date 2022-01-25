import time
import rospy

def compute_std_dev(my_deque, mean_val):
    ''' compute std deviation of a deque'''

    spread = 0

    for val in my_deque:
        spread += (val - mean_val) ** 2

    return (spread/len(my_deque)) ** (0.5)


