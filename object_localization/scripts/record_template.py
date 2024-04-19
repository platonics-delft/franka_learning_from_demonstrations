#!/usr/bin/env python3
from gui import Template
import sys
import rospy

if __name__ == '__main__':
    try:
        name_template = rospy.get_param('/template_node/template_name')
        rospy.init_node("record_template_node")
        print("Recording template name: ", name_template)
        template = Template()
        template.record(name=name_template)
    except rospy.ROSInterruptException:
        pass

