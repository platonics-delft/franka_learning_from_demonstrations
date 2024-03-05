import rospy
from panda_ros import Panda
import sys

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <desired_vertical_height> ")
    else:
        height = float(sys.argv[1])
        print(f"Desired height is: {height}")
        rospy.init_node("homing_node")
        panda=Panda()
        panda.home(height=height)
        panda.offset_compensator(10)
