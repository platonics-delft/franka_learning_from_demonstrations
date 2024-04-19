#!/usr/bin/env python3
import rospy
from panda_ros import Panda
import sys

if __name__ == "__main__":

    height = rospy.get_param('/homing_node/height')
    front_offset = rospy.get_param('/homing_node/front_offset')
    side_offset = rospy.get_param('/homing_node/side_offset')
    print(f"Desired height is: {height}")
    rospy.init_node("homing_node")
    panda=Panda()
    panda.home(height=height , front_offset=front_offset, side_offset=side_offset)
    panda.offset_compensator(10)
