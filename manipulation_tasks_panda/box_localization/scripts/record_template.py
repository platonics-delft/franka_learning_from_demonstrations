from gui import Template
import sys
import rospy
from panda_ros import Panda

if __name__ == '__main__':
    rospy.init_node("homing_node")
    panda=Panda()
    panda.home()
    template = Template()
    template.record(name=sys.argv[1])
