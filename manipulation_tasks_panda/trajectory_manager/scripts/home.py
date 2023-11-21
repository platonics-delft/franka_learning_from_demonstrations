import rospy
from panda_ros import Panda

rospy.init_node("homing_node")
panda=Panda()
panda.home()
