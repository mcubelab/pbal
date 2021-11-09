import rospy
from franka_interface import ArmInterface

if __name__ == '__main__':
    rospy.init_node("move_to_pbal_node")
    r = ArmInterface()
    r.move_to_pbal_start()