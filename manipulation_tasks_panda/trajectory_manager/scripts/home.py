#!/usr/bin/env python3
import rospy
from panda_ros import Panda
import sys

if __name__ == "__main__":

    height = rospy.get_param('/homing_node/height')
    print(f"Desired height is: {height}")
    rospy.init_node("homing_node")
    panda=Panda()
    panda.home(height=height)
    panda.offset_compensator(10)
