import rospy
from franka_interface import ArmInterface

pbal_start_pose = {
        "panda_joint1": -0.12285389533116273,
        "panda_joint2": 0.6495492118908847,
        "panda_joint3": -0.03179291257694759,
        "panda_joint4": -2.08736953684449,
        "panda_joint5": 1.4410093290209771,
        "panda_joint6": 1.5131599545882595,
        "panda_joint7": 0.4397166739718772}

if __name__ == '__main__':
    rospy.init_node("move_to_pbal_node")
    
    r = ArmInterface()
    r.set_joint_position_speed(speed=0.15)
    r.move_to_joint_positions(positions=pbal_start_pose,
        timeout=15.0) 
